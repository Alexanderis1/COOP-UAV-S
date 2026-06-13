"""MAPPO trainer for the cooperative weapon-target policy.

Shared-reward multi-agent PPO with a parameter-shared actor and a
centralised critic (CTDE). Standard, CPU-first, and deliberately compact —
for a small WTA policy the bottleneck is env stepping, so the design spends
its parallelism on processes (:class:`~coopuavs.rl.vec_env.SubprocVecEnv`),
not on a big network.

Credit model: a shared team reward drives one GAE advantage per env-timestep
(broadcast to all agents); the centralised critic predicts the team value
from the joint observation. Per-agent waste penalties (ammo, decoy shots)
are folded into the team reward, so an interceptor's wasteful shot lowers
the signal every agent learns from. Time-limit truncations bootstrap from
V(final state); raid-resolved terminations do not (M8).

Run it from the CLI: ``coopuavs train --help`` (or scripts/train_marl.py).
"""

from __future__ import annotations

import csv
import os
import time
from dataclasses import dataclass, field

import numpy as np

try:
    import torch
    import torch.nn as nn
except ImportError as exc:    # pragma: no cover - optional dependency
    raise ImportError(
        "coopuavs.rl.mappo requires PyTorch — install the training extras: "
        "pip install -e '.[train]'") from exc

from .models import ActorCritic
from .spaces import NUM_ACTIONS, OBS_DIM
from .vec_env import SubprocVecEnv, SyncVectorEnv


@dataclass
class MappoConfig:
    scenario: object = "scenarios/high_diver_raid.yaml"
    total_steps: int = 2_000_000
    n_envs: int = 8
    rollout_steps: int = 64
    horizon: int = 220
    epochs: int = 4
    minibatches: int = 4
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip: float = 0.2
    lr: float = 3e-4
    vf_coef: float = 0.5
    ent_coef: float = 0.01
    max_grad_norm: float = 0.5
    hidden: tuple = (256, 256)
    seed: int = 0
    randomize: bool = True
    reward_weights: dict = field(default_factory=dict)
    out_dir: str = "runs/marl"
    checkpoint_every: int = 20        # updates
    log_every: int = 1
    subproc: bool = True
    torch_threads: int = 1
    time_budget_s: float | None = None   # wall-clock cap; stop + checkpoint when hit


def _stack(obs_list, infos, agents):
    n, N = len(obs_list), len(agents)
    obs_b = np.zeros((n, N, OBS_DIM), dtype=np.float32)
    mask_b = np.ones((n, N, NUM_ACTIONS), dtype=bool)
    for e in range(n):
        for i, a in enumerate(agents):
            o = obs_list[e].get(a)
            if o is not None:
                obs_b[e, i] = o
            mk = infos[e].get(a, {}).get("action_mask") if a in infos[e] else None
            if mk is not None:
                mask_b[e, i] = mk
    return obs_b, mask_b


def _final_joint(infos_e, agents):
    """Joint observation from per-agent ``final_observation`` (for truncation
    bootstrap); missing agents (none, normally) zero-fill."""
    parts = []
    for a in agents:
        fo = infos_e.get(a, {}).get("final_observation")
        parts.append(fo if fo is not None else np.zeros(OBS_DIM, dtype=np.float32))
    return np.concatenate(parts).astype(np.float32)


def train(cfg: MappoConfig):
    os.environ.setdefault("OMP_NUM_THREADS", str(cfg.torch_threads))
    torch.set_num_threads(cfg.torch_threads)
    torch.manual_seed(cfg.seed)
    gen = torch.Generator().manual_seed(cfg.seed + 1)
    rng = np.random.default_rng(cfg.seed)

    os.makedirs(cfg.out_dir, exist_ok=True)
    env_kwargs = dict(horizon=cfg.horizon, gamma=cfg.gamma,
                      randomize=cfg.randomize, reward_weights=cfg.reward_weights)
    VecCls = SubprocVecEnv if cfg.subproc else SyncVectorEnv
    vec = VecCls(cfg.scenario, cfg.n_envs, env_kwargs=env_kwargs, seed=cfg.seed)
    agents = vec.possible_agents
    N = len(agents)
    n_envs, T = cfg.n_envs, cfg.rollout_steps

    ac = ActorCritic(N, OBS_DIM, NUM_ACTIONS, cfg.hidden)
    opt = torch.optim.Adam(ac.parameters(), lr=cfg.lr, eps=1e-5)

    log_path = os.path.join(cfg.out_dir, "train_log.csv")
    log_f = open(log_path, "w", newline="")
    logger = csv.writer(log_f)
    logger.writerow(["update", "env_steps", "mean_ep_return", "ep_count",
                     "policy_loss", "value_loss", "entropy", "approx_kl", "sps"])
    try:
        writer = _maybe_tensorboard(cfg.out_dir)
    except Exception:
        writer = None

    obs_list, infos = vec.reset()
    # With a wall-clock budget the step count is an upper bound only — the
    # timer governs (Colab/1-hour runs). Otherwise total_steps governs.
    updates = (10 ** 9 if cfg.time_budget_s
               else max(1, cfg.total_steps // (n_envs * T)))
    global_step = 0
    ep_returns: list[float] = []
    ep_accum = np.zeros(n_envs, dtype=np.float64)
    t_start = time.time()

    for update in range(1, updates + 1):
        if cfg.time_budget_s and (time.time() - t_start) >= cfg.time_budget_s:
            print(f"time budget {cfg.time_budget_s:.0f}s reached at update "
                  f"{update - 1}, {global_step} steps — stopping")
            break
        b_obs = np.zeros((T, n_envs, N, OBS_DIM), dtype=np.float32)
        b_mask = np.ones((T, n_envs, N, NUM_ACTIONS), dtype=bool)
        b_act = np.zeros((T, n_envs, N), dtype=np.int64)
        b_logp = np.zeros((T, n_envs, N), dtype=np.float32)
        b_rew = np.zeros((T, n_envs, N), dtype=np.float32)
        b_val = np.zeros((T, n_envs), dtype=np.float32)
        b_done = np.zeros((T, n_envs), dtype=np.float32)
        b_nextval = np.zeros((T, n_envs), dtype=np.float32)

        for t in range(T):
            obs_b, mask_b = _stack(obs_list, infos, agents)
            obs_t = torch.from_numpy(obs_b).reshape(n_envs * N, OBS_DIM)
            mask_t = torch.from_numpy(mask_b).reshape(n_envs * N, NUM_ACTIONS)
            with torch.no_grad():
                action, logp = ac.actor.act(obs_t, mask_t, generator=gen)
                value = ac.critic(torch.from_numpy(obs_b).reshape(n_envs, N * OBS_DIM))
            act_np = action.numpy().reshape(n_envs, N)
            actions = [{a: int(act_np[e, i]) for i, a in enumerate(agents)}
                       for e in range(n_envs)]
            nobs, rew, term, trunc, ninfos, dones = vec.step(actions)

            b_obs[t], b_mask[t] = obs_b, mask_b
            b_act[t] = act_np
            b_logp[t] = logp.numpy().reshape(n_envs, N)
            b_val[t] = value.numpy()
            b_done[t] = np.array(dones, dtype=np.float32)
            for e in range(n_envs):
                for i, a in enumerate(agents):
                    b_rew[t, e, i] = rew[e].get(a, 0.0)
                ep_accum[e] += float(np.mean([rew[e].get(a, 0.0) for a in agents]))
                if dones[e]:
                    terminated = any(term[e].values())
                    if terminated:
                        b_nextval[t, e] = 0.0
                    else:    # truncation: bootstrap from V(final joint obs)
                        fj = torch.from_numpy(_final_joint(ninfos[e], agents))
                        with torch.no_grad():
                            b_nextval[t, e] = float(ac.critic(fj.unsqueeze(0))[0])
                    ep_returns.append(float(ep_accum[e]))
                    ep_accum[e] = 0.0
            obs_list, infos = nobs, ninfos
            global_step += n_envs

        # bootstrap value for the step after the rollout (non-done steps)
        obs_b, _ = _stack(obs_list, infos, agents)
        with torch.no_grad():
            last_val = ac.critic(torch.from_numpy(obs_b).reshape(n_envs, N * OBS_DIM)).numpy()
        for t in range(T):
            for e in range(n_envs):
                if b_done[t, e] == 0.0:
                    b_nextval[t, e] = b_val[t + 1, e] if t + 1 < T else last_val[e]

        adv = _gae(b_rew, b_val, b_nextval, b_done, cfg.gamma, cfg.gae_lambda)
        team_ret = b_val + adv.mean(axis=2)        # critic target (team value)

        stats = _ppo_update(ac, opt, cfg, b_obs, b_mask, b_act,
                            b_logp, adv, team_ret, rng)

        if update % cfg.log_every == 0:
            mean_ret = float(np.mean(ep_returns[-50:])) if ep_returns else float("nan")
            sps = int(global_step / max(1e-6, time.time() - t_start))
            logger.writerow([update, global_step, round(mean_ret, 3),
                             len(ep_returns), round(stats["pg"], 4),
                             round(stats["vf"], 4), round(stats["ent"], 4),
                             round(stats["kl"], 5), sps])
            log_f.flush()
            print(f"upd {update}/{updates} step {global_step} "
                  f"ep_ret {mean_ret:.2f} (n={len(ep_returns)}) "
                  f"pg {stats['pg']:.3f} vf {stats['vf']:.3f} "
                  f"ent {stats['ent']:.3f} kl {stats['kl']:.4f} {sps} sps")
            if writer is not None:
                writer.add_scalar("charts/ep_return", mean_ret, global_step)
                writer.add_scalar("losses/policy", stats["pg"], global_step)
                writer.add_scalar("losses/value", stats["vf"], global_step)
                writer.add_scalar("losses/entropy", stats["ent"], global_step)

        if update % cfg.checkpoint_every == 0 or update == updates:
            ac.save(os.path.join(cfg.out_dir, "policy.pt"))

    ac.save(os.path.join(cfg.out_dir, "policy.pt"))
    log_f.close()
    if writer is not None:
        writer.close()
    vec.close()
    return ac


def _gae(rew, val, nextval, done, gamma, lam):
    T, n_envs, N = rew.shape
    adv = np.zeros_like(rew)
    lastgae = np.zeros((n_envs, N), dtype=np.float32)
    for t in reversed(range(T)):
        nonterminal = (1.0 - done[t])[:, None]      # [n_envs, 1]
        delta = rew[t] + gamma * nextval[t][:, None] - val[t][:, None]
        lastgae = delta + gamma * lam * nonterminal * lastgae
        adv[t] = lastgae
    return adv


def _ppo_update(ac, opt, cfg, b_obs, b_mask, b_act, b_logp, adv, team_ret, rng):
    T, n_envs, N = b_act.shape
    M = T * n_envs                       # env-timesteps (one joint obs each)
    obs = torch.from_numpy(b_obs.reshape(M, N, OBS_DIM))
    mask = torch.from_numpy(b_mask.reshape(M, N, NUM_ACTIONS))
    act = torch.from_numpy(b_act.reshape(M, N))
    old_logp = torch.from_numpy(b_logp.reshape(M, N))
    advantage = torch.from_numpy(adv.reshape(M, N))
    joint = obs.reshape(M, N * OBS_DIM)
    ret = torch.from_numpy(team_ret.reshape(M))
    advantage = (advantage - advantage.mean()) / (advantage.std() + 1e-8)

    idx = np.arange(M)
    mb_size = max(1, M // cfg.minibatches)
    last = {"pg": 0.0, "vf": 0.0, "ent": 0.0, "kl": 0.0}
    for _ in range(cfg.epochs):
        rng.shuffle(idx)        # seeded generator — reproducible PPO epochs
        for start in range(0, M, mb_size):
            mb = idx[start:start + mb_size]
            mb_t = torch.from_numpy(mb)
            o = obs[mb_t].reshape(-1, OBS_DIM)
            mk = mask[mb_t].reshape(-1, NUM_ACTIONS)
            a = act[mb_t].reshape(-1)
            olp = old_logp[mb_t].reshape(-1)
            ad = advantage[mb_t].reshape(-1)
            new_logp, entropy = ac.actor.evaluate(o, mk, a)
            ratio = torch.exp(new_logp - olp)
            pg1 = -ad * ratio
            pg2 = -ad * torch.clamp(ratio, 1 - cfg.clip, 1 + cfg.clip)
            pg_loss = torch.max(pg1, pg2).mean()
            value = ac.critic(joint[mb_t])
            v_loss = 0.5 * ((value - ret[mb_t]) ** 2).mean()
            ent = entropy.mean()
            loss = pg_loss + cfg.vf_coef * v_loss - cfg.ent_coef * ent
            opt.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(ac.parameters(), cfg.max_grad_norm)
            opt.step()
            with torch.no_grad():
                last = {"pg": float(pg_loss), "vf": float(v_loss),
                        "ent": float(ent), "kl": float((olp - new_logp).mean())}
    return last


def _maybe_tensorboard(out_dir):
    try:
        from torch.utils.tensorboard import SummaryWriter
    except Exception:
        return None
    return SummaryWriter(os.path.join(out_dir, "tb"))
