"""Kill adjudication and C2 kill bookkeeping.

Regression tests for the adjudication cross-wiring findings: a munition is
resolved against the aim point it was released at (not whatever enemy
happens to be nearest the shooter), and the C2 un-blacklists a "killed"
track that demonstrably keeps flying — a misattributed hit must not turn a
live armed threat into a permanent leaker.
"""

import numpy as np

from coopuavs.c2.base_station import (
    KILL_RECONFIRM_GRACE_S,
    UAV_STATE_STALE_S,
    BaseStation,
)
from coopuavs.core.bus import MessageBus
from coopuavs.core.messages import (
    EngagementResult,
    FireRequest,
    Header,
    ThreatClass,
    Track,
    TrackArray,
    UavState,
)
from coopuavs.interceptors.effectors import projectile_gun
from coopuavs.interceptors.uav import InterceptorUav
from coopuavs.risk.debris import DebrisModel
from coopuavs.sim.adjudicator import EngagementAdjudicator
from coopuavs.sim.environment import Environment
from coopuavs.sim.turret import GroundTurret
from coopuavs.sim.world import World
from coopuavs.threats.enemy_drone import EnemyDrone

ENV_CFG = {
    "bounds": [-5000.0, -5000.0, 5000.0, 5000.0],
    "default_zone": "SAFE",
    "assets": [{"name": "substation", "position": [0.0, 0.0, 0.0]}],
}


# -- adjudicator: resolve against the aim point --------------------------------


def add_enemy(world: World, drone_id: str, pos) -> EnemyDrone:
    enemy = EnemyDrone(drone_id, ThreatClass.OWA_STRATEGIC,
                       np.array(pos, dtype=float),
                       np.array([0.0, 0.0, 0.0]), np.random.default_rng(0))
    world.enemies[drone_id] = enemy
    return enemy


def test_shot_resolves_against_the_aim_point_not_the_shooter():
    world = World(Environment.from_config(ENV_CFG))
    # Fresh stream so the adjudication roll is its first draw (0.26 < Pk).
    world.rng = np.random.default_rng(2)
    uav = InterceptorUav("u1", world.bus, home=np.zeros(3),
                         effector=projectile_gun(), max_speed=80.0)
    uav.body.position = np.array([0.0, 0.0, 300.0])
    uav.body.velocity = np.array([80.0, 0.0, 0.0])
    adj = EngagementAdjudicator(world, {"u1": uav})

    # A non-engaged airframe sits behind the launch rail, nearer the shooter
    # than the aim point — under shooter-distance resolution it ate the shot.
    bystander = add_enemy(world, "bystander", [-40.0, 0.0, 300.0])
    engaged = add_enemy(world, "engaged", [80.0, 0.0, 300.0])

    results = []
    world.bus.subscribe("engagement/result", results.append)
    adj._on_fire(FireRequest(
        header=Header(stamp=0.0), task_id=1, uav_id="u1", track_id=5,
        predicted_intercept=np.array([80.0, 0.0, 300.0]), p_kill=0.5,
    ))

    assert engaged.killed
    assert bystander.alive
    assert results[0].hit and results[0].track_id == 5


# -- C2: kill claims reconciled against the track picture -----------------------


def fresh_tracks(t: float, track_id: int = 1, since: float = 0.0) -> TrackArray:
    return TrackArray(header=Header(stamp=t), tracks=[Track(
        header=Header(stamp=t), track_id=track_id,
        position=np.array([3000.0, 0.0, 800.0]),
        velocity=np.array([-55.0, 0.0, 0.0]),
        time_since_update=since,
    )])


def uav_state(t: float, uav_id: str = "u1") -> UavState:
    return UavState(header=Header(stamp=t), uav_id=uav_id,
                    position=np.zeros(3), ammo=4)


def make_base_station(bus: MessageBus) -> BaseStation:
    return BaseStation(bus, Environment.from_config(ENV_CFG),
                       DebrisModel(np.random.default_rng(0)),
                       uav_speeds={"u1": 60.0})


def test_misattributed_kill_is_unblacklisted_when_track_keeps_updating():
    bus = MessageBus()
    published = []
    bus.subscribe("engagement/tasks", published.append)
    bs = make_base_station(bus)
    bs._on_tracks(fresh_tracks(0.0))
    bs._on_result(EngagementResult(header=Header(stamp=0.0), task_id=1,
                                   track_id=1, uav_id="u1", hit=True))

    # Inside the grace window the kill claim stands (in-flight detections
    # from before the hit may still be updating the track).
    bs._on_tracks(fresh_tracks(1.0))
    bs._on_uav_state(uav_state(1.0))
    bs.update(1.0, 1.0)
    assert published[-1] == []

    # Still absorbing measurements well past the grace window: the threat
    # is demonstrably alive and must re-enter assessment and allocation.
    t = KILL_RECONFIRM_GRACE_S + 1.0
    bs._on_tracks(fresh_tracks(t))
    bs._on_uav_state(uav_state(t))
    bs.update(t, 1.0)
    assert [task.track_id for task in published[-1]] == [1]


def test_genuine_kill_stays_blacklisted_while_track_coasts():
    bus = MessageBus()
    published = []
    bus.subscribe("engagement/tasks", published.append)
    bs = make_base_station(bus)
    bs._on_result(EngagementResult(header=Header(stamp=0.0), task_id=1,
                                   track_id=1, uav_id="u1", hit=True))

    t = KILL_RECONFIRM_GRACE_S + 1.0
    # The dead airframe's track has had no measurement since the kill.
    bs._on_tracks(fresh_tracks(t, since=t))
    bs._on_uav_state(uav_state(t))
    bs.update(t, 1.0)
    assert published[-1] == []


# -- turret: same reconciliation as the C2 ---------------------------------------


def close_tracks(t: float, since: float = 0.0) -> TrackArray:
    """A track inside turret range, slow enough to clear the burst-Pk floor."""
    return TrackArray(header=Header(stamp=t), tracks=[Track(
        header=Header(stamp=t), track_id=1,
        position=np.array([500.0, 0.0, 300.0]),
        velocity=np.array([-55.0, 0.0, 0.0]),
        time_since_update=since,
    )])


def test_turret_unblacklists_a_track_that_keeps_flying():
    world = World(Environment.from_config(ENV_CFG))
    turret = GroundTurret("t1", world=world, position=np.zeros(3))
    turret._on_tracks(close_tracks(0.0))
    assert turret._select_target(0.0) is not None
    turret._on_result(EngagementResult(header=Header(stamp=0.0), task_id=1,
                                       track_id=1, uav_id="t1", hit=True))
    assert turret._select_target(1.0) is None   # claim stands inside the grace

    t = KILL_RECONFIRM_GRACE_S + 1.0
    turret._on_tracks(close_tracks(t))
    target = turret._select_target(t)
    assert target is not None and target.track_id == 1


# -- C2: silent telemetry is not allocated ---------------------------------------


def test_stale_telemetry_is_not_allocated():
    bus = MessageBus()
    published = []
    bus.subscribe("engagement/tasks", published.append)
    bs = make_base_station(bus)
    bs._on_uav_state(uav_state(0.0))

    t = UAV_STATE_STALE_S + 1.0
    bs._on_tracks(fresh_tracks(t))
    bs.update(t, 1.0)
    assert published[-1] == []                 # u1 has been silent too long

    bs._on_uav_state(uav_state(t))
    bs.update(t + 1.0, 1.0)
    assert [task.shooter_id for task in published[-1]] == ["u1"]
