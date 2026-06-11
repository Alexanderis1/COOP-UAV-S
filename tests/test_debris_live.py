"""Live interceptable debris (SIM-DEB-001..005, PHY-GCS-006)."""

import numpy as np

from coopuavs.c2 import assignment
from coopuavs.core.messages import (
    EffectorType, Header, ThreatAssessment, Track, UavState, ZoneClass,
)
from coopuavs.risk.debris import TERMINAL_FALL_SPEED, fall_time
from coopuavs.sim.debris_objects import FallingDebris
from coopuavs.sim.environment import Environment
from coopuavs.sim.world import World

ENV_CFG = {
    "bounds": [-2000.0, -2000.0, 2000.0, 2000.0],
    "cell_size": 50.0,
    "default_zone": "SAFE",
    "zones": [{"rect": [-500, -500, 500, 500], "class": "CRITICAL"}],
}


def make_debris(pos=(0, 0, 500), vel=(20, 0, 0)):
    return FallingDebris("deb-x", "owa-1", np.array(pos, float),
                         np.array(vel, float), EffectorType.PROJECTILE,
                         track_ref=-101)


def test_fall_time_matches_integration():
    deb = make_debris(pos=(0, 0, 800.0))
    predicted = deb.time_to_impact()
    t, dt = 0.0, 0.01
    while not deb.landed:
        deb.step(dt)
        t += dt
    assert abs(t - predicted) < 0.1
    # analytic helper agrees with the v0.1 footprint law for a fresh drop
    assert abs(fall_time(800.0) - predicted) < 1e-6


def test_predicted_impact_carries_horizontal_velocity():
    deb = make_debris(pos=(0, 0, 300.0), vel=(30.0, -10.0, 0.0))
    impact = deb.predicted_impact()
    t_f = deb.time_to_impact()
    assert np.allclose(impact[:2], [30.0 * t_f, -10.0 * t_f], atol=1e-6)
    assert impact[2] == 0.0


def test_world_lands_debris_as_wreck():
    env = Environment.from_config(ENV_CFG)
    world = World(env, dt=0.05, seed=1)
    deb = make_debris(pos=(0, 0, 30.0), vel=(0.0, 0.0, 0.0))
    world.debris[deb.debris_id] = deb
    for _ in range(200):
        world.step()
        if not world.debris:
            break
    assert not world.debris
    assert len(world.wrecks) == 1
    assert world.wrecks[0]["zone"] == ZoneClass.CRITICAL
    assert any(e["kind"] == "debris_impact" for e in world.events)


def _track(track_id, pos, vel, speed=None):
    trk = Track(header=Header(stamp=0.0), track_id=track_id,
                position=np.array(pos, float), velocity=np.array(vel, float))
    return trk


def _assessment(track_id, score, zone=ZoneClass.SAFE, tti=20.0):
    return ThreatAssessment(header=Header(stamp=0.0), track_id=track_id,
                            threat_score=score, time_to_impact=tti,
                            impact_zone=zone)


def _uav(uid, pos):
    return UavState(header=Header(stamp=0.0), uav_id=uid,
                    position=np.array(pos, float), ammo=4, battery=1.0)


def test_debris_tasks_kinetic_only_and_prioritised():
    """Nets never get debris tasks; CRITICAL debris outranks a mid-score
    threat track and DANGEROUS debris (PHY-GCS-006)."""
    from coopuavs.risk.zones import RiskMap
    rm = RiskMap((-2000, -2000, 2000, 2000), cell_size=100, default=ZoneClass.SAFE)
    tracks = {
        1: _track(1, [1000, 1000, 400], [-30, 0, 0]),
        -101: _track(-101, [0, 0, 300], [10, 0, -30]),
        -102: _track(-102, [500, 0, 300], [10, 0, -30]),
    }
    assessments = [
        _assessment(1, 0.6),
        _assessment(-101, 0.90, ZoneClass.CRITICAL, 8.0),
        _assessment(-102, 0.55, ZoneClass.DANGEROUS, 8.0),
    ]
    uavs = [_uav("net-1", [0, -100, 200]), _uav("hawk-1", [0, 100, 200]),
            _uav("hawk-2", [400, 0, 200]), _uav("hawk-3", [600, 200, 200])]
    tasks = assignment.allocate(
        assessments, tracks, uavs, {u.uav_id: 80.0 for u in uavs}, rm, 0.0,
        debris_info={-101: "deb-a", -102: "deb-b"},
        uav_effectors={"net-1": "net", "hawk-1": "projectile",
                       "hawk-2": "projectile", "hawk-3": "projectile"},
    )
    by_track = {t.track_id: t for t in tasks}
    # CRITICAL debris first in priority order, then the 0.6 threat track,
    # then DANGEROUS debris
    assert tasks[0].track_id == -101
    assert tasks[0].target_kind == "debris" and tasks[0].debris_id == "deb-a"
    assert [t.track_id for t in tasks] == [-101, 1, -102]
    # both debris tasks went to projectile carriers, never the net
    for ref in (-101, -102):
        assert by_track[ref].shooter_id.startswith("hawk")
        assert not by_track[ref].support_ids        # ballistics: no blockers


def test_adjudicator_kill_spawns_debris_and_intercept_credits():
    """End-to-end through the adjudicator: a kill creates a live object;
    a debris shot removes it with credit and never spawns a new hazard."""
    from coopuavs.core.messages import FireRequest
    from coopuavs.interceptors.effectors import projectile_gun
    from coopuavs.sim.adjudicator import EngagementAdjudicator
    from coopuavs.interceptors.uav import InterceptorUav

    env = Environment.from_config(ENV_CFG)
    world = World(env, dt=0.05, seed=3)
    uav = InterceptorUav("hawk-1", world.bus, home=np.zeros(3),
                         effector=projectile_gun(), max_speed=80.0)
    uav.body.position = np.array([0.0, -60.0, 300.0])
    uav.body.velocity = np.array([0.0, 60.0, 0.0])

    class Target:
        id = "owa-1"
        position = np.array([0.0, 0.0, 300.0])
        velocity = np.array([0.0, 55.0, 0.0])
        alive = True
        from coopuavs.core.messages import ThreatClass
        threat_class = ThreatClass.OWA_STRATEGIC
        def kill(self):
            self.alive = False

    target = Target()
    world.enemies["owa-1"] = target
    adj = EngagementAdjudicator(world, {"hawk-1": uav}, {})
    # force a kill by rolling until hit (seeded; few iterations)
    for _ in range(50):
        adj._on_fire(FireRequest(header=Header(stamp=world.t), task_id=1,
                                 uav_id="hawk-1", track_id=1,
                                 effector=EffectorType.PROJECTILE,
                                 predicted_intercept=target.position.copy(),
                                 p_kill=0.5))
        if not target.alive:
            break
    assert not target.alive
    assert len(world.debris) == 1
    deb = next(iter(world.debris.values()))
    assert deb.track_ref < -100
    assert any(e["kind"] == "debris_spawn" for e in world.events)

    # now shoot the debris: keep firing until the roll connects
    uav.body.position = deb.position + np.array([0.0, -50.0, 0.0])
    uav.body.velocity = np.array([0.0, 60.0, 0.0])
    for _ in range(200):
        if not world.debris:
            break
        adj._on_fire(FireRequest(header=Header(stamp=world.t), task_id=2,
                                 uav_id="hawk-1", track_id=deb.track_ref,
                                 effector=EffectorType.PROJECTILE,
                                 predicted_intercept=deb.position.copy(),
                                 p_kill=0.5, target_kind="debris",
                                 debris_id=deb.debris_id))
        uav.body.position = deb.position + np.array([0.0, -50.0, 0.0]) \
            if world.debris else uav.body.position
    assert not world.debris                       # neutralized
    assert world.debris_intercepted               # credited
    assert not world.wrecks                       # fragments are negligible
    assert any(e["kind"] == "debris_neutralized" for e in world.events)


def test_retention_jitter_clamped_to_physical_range():
    """The kill-time retention jitter must never go negative (wreck flying
    backwards) nor past 2x the airframe's horizontal speed."""
    from coopuavs.risk.debris import retention_jitter
    rng = np.random.default_rng(0)
    j = retention_jitter(rng, size=200_000)   # ~6 raw samples fall below 0
    assert float(j.min()) >= 0.0
    assert float(j.max()) <= 2.0


def test_debris_picture_preserves_reporter_stamp():
    """The pseudo-track must carry the reporter's stamp, not planning time:
    downstream extrapolates position from the stamp, and the reported
    position is up to one reporter period old."""
    from coopuavs.c2.base_station import BaseStation
    from coopuavs.core.bus import MessageBus
    from coopuavs.core.messages import DebrisArray, DebrisState
    from coopuavs.risk.debris import DebrisModel

    bs = BaseStation(MessageBus(), Environment.from_config(ENV_CFG),
                     DebrisModel(np.random.default_rng(0)),
                     uav_speeds={"u1": 60.0})
    bs._on_debris(DebrisArray(header=Header(stamp=4.8), debris=[DebrisState(
        header=Header(stamp=4.8), debris_id="deb-x", track_ref=-101,
        position=np.array([0.0, 0.0, 300.0]),
        velocity=np.array([10.0, 0.0, -30.0]),
        predicted_impact=np.array([100.0, 0.0, 0.0]),
        impact_zone=ZoneClass.CRITICAL, t_impact=8.0,
    )]))
    tracks, _, _ = bs._debris_picture(t=5.0)
    assert tracks[-101].header.stamp == 4.8
