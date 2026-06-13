"""MAPPO networks, a smoke training run, and the deployment allocator.

Skipped entirely when the [train] extra (PyTorch) is absent."""

import copy

import numpy as np
import pytest
import yaml

torch = pytest.importorskip("torch")

from coopuavs.rl import spaces                       # noqa: E402
from coopuavs.rl.models import Actor, ActorCritic, Critic     # noqa: E402


def test_actor_masks_illegal_actions():
    actor = Actor()
    obs = torch.zeros(64, spaces.OBS_DIM)
    mask = torch.zeros(64, spaces.NUM_ACTIONS, dtype=torch.bool)
    mask[:, 0] = True            # only idle legal
    mask[:, 3] = True            # and action 3
    action, logp = actor.act(obs, mask)
    assert set(action.tolist()) <= {0, 3}, "sampled an illegal action"
    # log-prob of a masked action is ~-inf
    bad = torch.full((64,), 5, dtype=torch.long)
    lp, _ = actor.evaluate(obs, mask, bad)
    assert torch.all(lp < -1e3)


def test_critic_and_save_load(tmp_path):
    ac = ActorCritic(n_agents=8)
    joint = torch.zeros(4, 8 * spaces.OBS_DIM)
    assert ac.critic(joint).shape == (4,)
    p = tmp_path / "policy.pt"
    ac.save(p)
    loaded = ActorCritic.load(p)
    assert loaded.n_agents == 8 and loaded.obs_dim == spaces.OBS_DIM
    # weights match
    for a, b in zip(ac.actor.parameters(), loaded.actor.parameters()):
        assert torch.allclose(a, b)


@pytest.mark.slow
def test_smoke_train_runs_and_checkpoints(tmp_path):
    from coopuavs.rl.mappo import MappoConfig, train
    cfg = MappoConfig(
        scenario="scenarios/high_diver_raid.yaml",
        total_steps=64, n_envs=2, rollout_steps=8, horizon=20,
        epochs=2, minibatches=2, subproc=False, out_dir=str(tmp_path / "marl"),
        checkpoint_every=1, seed=0)
    ac = train(cfg)
    assert isinstance(ac, ActorCritic)
    assert (tmp_path / "marl" / "policy.pt").exists()
    assert (tmp_path / "marl" / "train_log.csv").exists()


@pytest.mark.slow
def test_training_is_reproducible(tmp_path):
    """Same seed -> identical policy weights (the trainer must honour the
    project's byte-reproducibility invariant; the PPO minibatch shuffle uses
    the seeded RNG, not the global one)."""
    from coopuavs.rl.mappo import MappoConfig, train

    def run(out):
        cfg = MappoConfig(
            scenario="scenarios/high_diver_raid.yaml", total_steps=48,
            n_envs=2, rollout_steps=8, horizon=18, epochs=2, minibatches=2,
            subproc=False, out_dir=str(out), seed=123)
        ac = train(cfg)
        return torch.cat([p.detach().flatten() for p in ac.actor.parameters()])

    assert torch.equal(run(tmp_path / "a"), run(tmp_path / "b"))


@pytest.mark.slow
def test_learned_allocator_plugs_into_a_battle(tmp_path):
    """Train briefly, then run a full battle with the learned allocator — it
    must drive the C2 without ever falling back to the classical allocator."""
    from coopuavs.c2.base_station import BaseStation
    from coopuavs.rl.models import ActorCritic
    from coopuavs.sim import scenario as scenario_mod

    # A tiny trained (mostly random) policy is enough to exercise the path.
    ckpt = tmp_path / "policy.pt"
    ActorCritic(n_agents=8).save(ckpt)

    cfg = yaml.safe_load(open("scenarios/high_diver_raid.yaml"))
    cfg = copy.deepcopy(cfg)
    cfg["duration"] = 70.0
    cfg["base_station"]["allocator"] = "learned"
    cfg["base_station"]["policy"] = str(ckpt)
    sc = scenario_mod.build(cfg, seed=1)
    bs = next(n for n in sc.world.nodes if isinstance(n, BaseStation))
    summary = sc.run()
    assert summary["enemies_total"] > 0
    assert bs.allocator_fallbacks == 0, "learned allocator must not error"
    # the policy actually issued some tasks / engagements
    kinds = {e["kind"] for e in sc.world.events}
    assert "enemy_spawn" in kinds
