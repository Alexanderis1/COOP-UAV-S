"""CPU-parallel vectorisation of the WTA environment.

The battle sim is pure-Python and CPU-bound (GIL-bound), so saturating the
many-core training box means *processes*, not threads. ``SubprocVecEnv`` runs
one :class:`~coopuavs.rl.env.CoopWtaParallelEnv` per worker process over a
pipe, with gymnasium-style **autoreset**: when an episode ends a worker
immediately resets (next seed) and returns the fresh observation, stashing
the terminal observation in ``info['final_observation']`` so the trainer can
bootstrap the value of a time-limit truncation correctly.

Pin BLAS/torch to one thread per process on the remote box so N workers do
not oversubscribe the cores: ``OMP_NUM_THREADS=1`` (see docs/MARL.md).

A single-process ``SyncVectorEnv`` is provided for tests and debugging.
"""

from __future__ import annotations

import multiprocessing as mp

from .env import CoopWtaParallelEnv


def _make(cfg, env_kwargs):
    return CoopWtaParallelEnv(cfg, **(env_kwargs or {}))


def _worker(remote, parent_remote, cfg, env_kwargs, seed0):
    parent_remote.close()
    env = _make(cfg, env_kwargs)
    episode = 0
    obs, infos = env.reset(seed=seed0)
    try:
        while True:
            cmd, data = remote.recv()
            if cmd == "step":
                obs, rew, term, trunc, infos = env.step(data)
                done = (not env.agents) or any(term.values()) or any(trunc.values())
                if done:
                    final_obs = obs
                    episode += 1
                    obs, reset_infos = env.reset(seed=seed0 + episode * 100003)
                    for a in infos:
                        infos[a] = dict(infos[a])
                        infos[a]["final_observation"] = final_obs.get(a)
                    # carry fresh masks for the reset observation
                    for a, info in reset_infos.items():
                        infos.setdefault(a, {})["action_mask"] = info.get("action_mask")
                remote.send((obs, rew, term, trunc, infos, done))
            elif cmd == "reset":
                episode = 0
                obs, infos = env.reset(seed=data if data is not None else seed0)
                remote.send((obs, infos))
            elif cmd == "agents":
                remote.send(env.possible_agents)
            elif cmd == "close":
                env.close()
                remote.close()
                break
    except (KeyboardInterrupt, EOFError):
        pass


class SubprocVecEnv:
    def __init__(self, cfg, n_envs: int, env_kwargs: dict | None = None,
                 seed: int = 0, start_method: str | None = None):
        self.n_envs = int(n_envs)
        ctx = mp.get_context(start_method or ("spawn" if mp.get_start_method(
            allow_none=True) != "fork" else "fork"))
        self.remotes, self.work_remotes = zip(*[ctx.Pipe() for _ in range(n_envs)])
        self.procs = []
        for i, (wr, r) in enumerate(zip(self.work_remotes, self.remotes)):
            p = ctx.Process(target=_worker,
                            args=(wr, r, cfg, env_kwargs, seed + i * 7919),
                            daemon=True)
            p.start()
            wr.close()
            self.procs.append(p)
        self.remotes[0].send(("agents", None))
        self.possible_agents = self.remotes[0].recv()

    def reset(self):
        for r in self.remotes:
            r.send(("reset", None))
        out = [r.recv() for r in self.remotes]
        return [o[0] for o in out], [o[1] for o in out]

    def step(self, actions: list[dict]):
        for r, a in zip(self.remotes, actions):
            r.send(("step", a))
        results = [r.recv() for r in self.remotes]
        obs, rew, term, trunc, infos, dones = zip(*results)
        return list(obs), list(rew), list(term), list(trunc), list(infos), list(dones)

    def close(self):
        for r in self.remotes:
            try:
                r.send(("close", None))
            except (BrokenPipeError, OSError):
                pass
        for p in self.procs:
            p.join(timeout=5)
            if p.is_alive():
                p.terminate()


class SyncVectorEnv:
    """Single-process equivalent for tests/debugging (same autoreset)."""

    def __init__(self, cfg, n_envs: int, env_kwargs: dict | None = None, seed: int = 0):
        self.n_envs = int(n_envs)
        self.envs = [_make(cfg, env_kwargs) for _ in range(n_envs)]
        self._seed0 = [seed + i * 7919 for i in range(n_envs)]
        self._episode = [0] * n_envs
        self.possible_agents = self.envs[0].possible_agents

    def reset(self):
        outs = [e.reset(seed=s) for e, s in zip(self.envs, self._seed0)]
        return [o[0] for o in outs], [o[1] for o in outs]

    def step(self, actions: list[dict]):
        obs, rew, term, trunc, infos, dones = [], [], [], [], [], []
        for i, (env, a) in enumerate(zip(self.envs, actions)):
            o, r, te, tr, info = env.step(a)
            done = (not env.agents) or any(te.values()) or any(tr.values())
            if done:
                final = o
                self._episode[i] += 1
                o, rinfo = env.reset(seed=self._seed0[i] + self._episode[i] * 100003)
                for ag in info:
                    info[ag] = dict(info[ag])
                    info[ag]["final_observation"] = final.get(ag)
                for ag, ri in rinfo.items():
                    info.setdefault(ag, {})["action_mask"] = ri.get("action_mask")
            obs.append(o)
            rew.append(r)
            term.append(te)
            trunc.append(tr)
            infos.append(info)
            dones.append(done)
        return obs, rew, term, trunc, infos, dones

    def close(self):
        for e in self.envs:
            e.close()
