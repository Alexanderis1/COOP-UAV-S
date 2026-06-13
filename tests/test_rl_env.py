"""Learned-WTA environment + reconciliation (numpy-only; no torch needed)."""

import numpy as np
import pytest

from coopuavs.core.messages import (
    Header, ThreatAssessment, Track, UavMode, UavState, ZoneClass,
)
from coopuavs.rl import reconcile, spaces
from coopuavs.rl.env import CoopWtaParallelEnv
from coopuavs.risk.zones import RiskMap


# -- reconciliation (the safety-critical B3 seam) ---------------------------

def _uav(uid, pos, eff="projectile", ammo=4):
    return UavState(header=Header(stamp=0.0), uav_id=uid,
                    position=np.asarray(pos, float), velocity=np.zeros(3),
                    mode=UavMode.IDLE, battery=1.0, ammo=ammo, max_speed=80.0,
                    kind="interceptor", effector=eff)


def _track(tid, pos, vel):
    return Track(header=Header(stamp=0.0), track_id=tid,
                 position=np.asarray(pos, float), velocity=np.asarray(vel, float))


def _assess(tid, score):
    return ThreatAssessment(header=Header(stamp=0.0), track_id=tid,
                            threat_score=score, time_to_impact=30.0,
                            impact_zone=ZoneClass.DANGEROUS)


def _rm():
    return RiskMap((-3000, -3000, 3000, 3000), cell_size=100,
                   default=ZoneClass.SAFE)


def _ctx(tracks, uavs):
    return dict(assessments={t.track_id: a for t, a in tracks},
                tracks={t.track_id: t for t, _ in tracks},
                available={u.uav_id: u for u in uavs},
                uav_speeds={u.uav_id: u.max_speed for u in uavs},
                risk_map=_rm(), t=0.0, task_ids={},
                uav_effectors={u.uav_id: u.effector for u in uavs})


def test_build_track_table_orders_and_excludes_debris():
    a = {1: _assess(1, 0.4), 2: _assess(2, 0.9), -101: _assess(-101, 0.99)}
    table = reconcile.build_track_table(a, debris_info={-101: "deb"})
    assert table == [2, 1], "ordered by threat score, debris excluded"


def test_two_shooters_one_track_yields_shooter_plus_support():
    trk, a = _track(1, [1000, 0, 800], [-50, 0, 0]), _assess(1, 0.8)
    uavs = [_uav("hawk-1", [800, 0, 400]), _uav("hawk-2", [600, 0, 400])]
    ctx = _ctx([(trk, a)], uavs)
    table = [1]
    tasks = reconcile.actions_to_tasks(
        {"hawk-1": 1, "hawk-2": 1}, table, **ctx)        # both shoot slot 0
    assert len(tasks) == 1 and tasks[0].track_id == 1
    assert tasks[0].shooter_id in ("hawk-1", "hawk-2")
    other = ({"hawk-1", "hawk-2"} - {tasks[0].shooter_id}).pop()
    assert tasks[0].support_ids == [other]


def test_block_only_track_is_promoted_to_a_shooter():
    trk, a = _track(1, [1000, 0, 800], [-50, 0, 0]), _assess(1, 0.8)
    uavs = [_uav("hawk-1", [800, 0, 400])]
    block0 = 1 + spaces.K_TRACKS                          # block slot 0
    tasks = reconcile.actions_to_tasks({"hawk-1": block0}, [1], **_ctx([(trk, a)], uavs))
    assert len(tasks) == 1 and tasks[0].shooter_id == "hawk-1"


def test_idle_and_support_cap_and_denied():
    trk, a = _track(1, [1000, 0, 800], [-50, 0, 0]), _assess(1, 0.8)
    uavs = [_uav(f"h{i}", [800 + 10 * i, 0, 400]) for i in range(4)]
    ctx = _ctx([(trk, a)], uavs)
    # all four shoot slot 0 -> 1 shooter + at most MAX_SUPPORT_PER_TASK support
    tasks = reconcile.actions_to_tasks({u.uav_id: 1 for u in uavs}, [1], **ctx)
    assert len(tasks) == 1
    assert len(tasks[0].support_ids) <= reconcile.assignment.MAX_SUPPORT_PER_TASK

    # idle -> no task
    assert reconcile.actions_to_tasks({u.uav_id: 0 for u in uavs}, [1], **_ctx([(trk, a)], uavs)) == []

    # denied track -> never committed
    ctx2 = _ctx([(trk, a)], uavs)
    tasks2 = reconcile.actions_to_tasks({"h0": 1}, [1], denied_tracks={1}, **ctx2)
    assert tasks2 == []


def test_net_cannot_shoot_debris_but_projectile_can():
    deb = _track(-101, [500, 0, 100], [0, 0, -40])
    a = _assess(-101, 0.9)
    net = _uav("net-1", [400, 0, 200], eff="net")
    proj = _uav("hawk-1", [400, 0, 200], eff="projectile")
    info = {-101: "deb-1"}
    # net commits to debris -> dropped (no kinetic kill); classical debris
    # fallback also cannot use the net, so no task.
    ctx = _ctx([(deb, a)], [net]); ctx["debris_info"] = info
    assert reconcile.actions_to_tasks({"net-1": 1}, [-101], **ctx) == []
    # projectile commits to debris -> a debris-intercept task
    ctx2 = _ctx([(deb, a)], [proj]); ctx2["debris_info"] = info
    tasks = reconcile.actions_to_tasks({"hawk-1": 1}, [-101], **ctx2)
    assert len(tasks) == 1 and tasks[0].target_kind == "debris"


# -- environment API --------------------------------------------------------

def _env(**kw):
    return CoopWtaParallelEnv("scenarios/high_diver_raid.yaml",
                              horizon=kw.pop("horizon", 25), seed=1, **kw)


def test_reset_step_shapes_and_legal_masks():
    env = _env()
    obs, infos = env.reset(seed=1)
    assert set(obs) == set(env.possible_agents)
    assert all(o.shape == (spaces.OBS_DIM,) for o in obs.values())
    for a in env.agents:
        mask = infos[a]["action_mask"]
        assert mask.shape == (spaces.NUM_ACTIONS,) and bool(mask[0])   # idle legal
    actions = {a: 0 for a in env.agents}      # all idle
    obs, rew, term, trunc, infos = env.step(actions)
    assert set(rew) == set(env.possible_agents)
    assert env._bs.allocator_fallbacks == 0


def test_random_rollout_never_falls_back():
    env = _env(horizon=40)
    obs, infos = env.reset(seed=2)
    rng = np.random.default_rng(0)
    for _ in range(40):
        if not env.agents:
            break
        acts = {a: int(rng.choice(np.flatnonzero(infos[a]["action_mask"])))
                for a in env.agents}
        obs, rew, term, trunc, infos = env.step(acts)
        if any(term.values()) or any(trunc.values()):
            break
    assert env._bs.allocator_fallbacks == 0, "reconciliation must never raise"


def test_determinism_same_seed_same_trajectory():
    def run():
        env = _env(horizon=30)
        obs, infos = env.reset(seed=9)
        rng = np.random.default_rng(7)
        rs = []
        for _ in range(30):
            if not env.agents:
                break
            acts = {a: int(rng.choice(np.flatnonzero(infos[a]["action_mask"])))
                    for a in env.agents}
            obs, rew, term, trunc, infos = env.step(acts)
            rs.append(round(sum(rew.values()), 6))
            if any(term.values()) or any(trunc.values()):
                break
        return rs
    assert run() == run()


def test_pettingzoo_parallel_env_surface():
    """The env presents the PettingZoo ParallelEnv surface: it subclasses
    ParallelEnv, exposes gymnasium per-agent spaces, and produces
    space-consistent, agent-keyed observations."""
    pz = pytest.importorskip("pettingzoo")
    gym = pytest.importorskip("gymnasium")
    env = _env(horizon=20)
    assert isinstance(env, pz.ParallelEnv)
    assert env.unwrapped is env
    a = env.possible_agents[0]
    ospace, aspace = env.observation_space(a), env.action_space(a)
    assert isinstance(ospace, gym.spaces.Box) and ospace.shape == (spaces.OBS_DIM,)
    assert isinstance(aspace, gym.spaces.Discrete) and aspace.n == spaces.NUM_ACTIONS
    obs, infos = env.reset(seed=1)
    assert set(obs) == set(env.possible_agents)
    for o in obs.values():
        assert np.all(np.isfinite(o)) and ospace.contains(o)
    obs, rew, term, trunc, infos = env.step({ag: 0 for ag in env.agents})
    keys = set(env.possible_agents)
    assert set(rew) == keys and set(term) == keys and set(trunc) == keys
