"""The fire-clearance safety chain: tokens are bound to the engagement the
ROE actually costed (track id + freshness), denials expire, task ids are
stable per pairing, and threat evaluation only matches closing geometry.

These are the regression tests for the review findings where a stale or
mis-correlated clearance could release a weapon on a target whose debris
footprint was never evaluated.
"""

import numpy as np

from coopuavs.c2 import assignment, threat_evaluation
from coopuavs.c2.base_station import BaseStation
from coopuavs.c2.roe import DENIAL_TTL_S, RoeConfig, RulesOfEngagement
from coopuavs.core.bus import MessageBus
from coopuavs.core.messages import (
    EffectorType,
    EngagementDecision,
    EngagementTask,
    FireClearance,
    FireRequest,
    Header,
    ThreatAssessment,
    Track,
    TrackArray,
    UavMode,
    UavState,
    ZoneClass,
)
from coopuavs.interceptors.effectors import projectile_gun
from coopuavs.interceptors.uav import CLEARANCE_VALID_S, InterceptorUav
from coopuavs.risk.debris import DebrisModel
from coopuavs.risk.zones import RiskMap
from coopuavs.sim.environment import Environment


# -- shooter-side token correlation (the H1 interlock) ---------------------------


def make_engaged_uav(bus: MessageBus) -> InterceptorUav:
    """A shooter in parameters against track 1: next update() reaches
    ENGAGE and the clearance state machine is live."""
    uav = InterceptorUav("u1", bus, home=np.array([0.0, 0.0, 0.0]),
                         effector=projectile_gun(), max_speed=80.0)
    uav.body.position = np.array([0.0, 0.0, 300.0])
    uav.body.velocity = np.array([50.0, 0.0, 0.0])
    return uav


def task_for(track_id: int, task_id: int) -> EngagementTask:
    return EngagementTask(header=Header(stamp=0.0), task_id=task_id,
                          track_id=track_id, shooter_id="u1")


def track_msg(track_id: int, t: float = 0.0) -> TrackArray:
    return TrackArray(header=Header(stamp=t), tracks=[Track(
        header=Header(stamp=t), track_id=track_id,
        position=np.array([100.0, 0.0, 300.0]),
        velocity=np.array([10.0, 0.0, 0.0]),
    )])


def test_uav_ignores_clearance_for_another_track():
    bus = MessageBus()
    fires = []
    bus.subscribe("engagement/fire", fires.append)
    uav = make_engaged_uav(bus)
    bus.publish("engagement/tasks", [task_for(track_id=1, task_id=7)])
    bus.publish("tracks", track_msg(1))

    # An AUTHORIZED token costed for a *different* track arrives — the
    # exact retask-while-pending hazard: it must never release on track 1.
    bus.publish("engagement/clearance", FireClearance(
        header=Header(stamp=0.0), task_id=3, uav_id="u1", track_id=2,
        decision=EngagementDecision.AUTHORIZED,
    ))
    assert uav._clearance is None              # not even stored
    uav.update(0.0, 0.1)
    assert fires == []

    # The matching token releases the shot.
    bus.publish("engagement/clearance", FireClearance(
        header=Header(stamp=0.0), task_id=7, uav_id="u1", track_id=1,
        decision=EngagementDecision.AUTHORIZED,
    ))
    uav.update(0.1, 0.1)
    assert len(fires) == 1
    assert fires[0].track_id == 1


def test_uav_discards_stale_clearance_token():
    bus = MessageBus()
    fires = []
    bus.subscribe("engagement/fire", fires.append)
    uav = make_engaged_uav(bus)
    bus.publish("engagement/tasks", [task_for(track_id=1, task_id=7)])
    bus.publish("tracks", track_msg(1))

    bus.publish("engagement/clearance", FireClearance(
        header=Header(stamp=0.0), task_id=7, uav_id="u1", track_id=1,
        decision=EngagementDecision.AUTHORIZED,
    ))
    # Consumed long after issue: the costed geometry no longer exists.
    uav.update(CLEARANCE_VALID_S + 2.0, 0.1)
    assert fires == []
    assert uav._clearance is None              # discarded, not banked


def test_retasking_invalidates_clearance_state():
    bus = MessageBus()
    fires = []
    bus.subscribe("engagement/fire", fires.append)
    uav = make_engaged_uav(bus)
    bus.publish("engagement/tasks", [task_for(track_id=1, task_id=7)])
    bus.publish("tracks", track_msg(1))
    bus.publish("engagement/clearance", FireClearance(
        header=Header(stamp=0.0), task_id=7, uav_id="u1", track_id=1,
        decision=EngagementDecision.AUTHORIZED,
    ))
    assert uav._clearance is not None

    # C2 retasks the shooter to track 2 before the token is consumed.
    bus.publish("engagement/tasks", [task_for(track_id=2, task_id=8)])
    assert uav._clearance is None
    bus.publish("tracks", track_msg(2))
    uav.update(0.1, 0.1)
    assert fires == []                         # re-requests, does not fire


def test_stale_denied_does_not_clear_the_new_task():
    bus = MessageBus()
    uav = make_engaged_uav(bus)
    bus.publish("engagement/tasks", [task_for(track_id=2, task_id=8)])
    bus.publish("engagement/clearance", FireClearance(
        header=Header(stamp=0.0), task_id=7, uav_id="u1", track_id=1,
        decision=EngagementDecision.DENIED,
    ))
    assert uav._task is not None and uav._task.track_id == 2


# -- denial TTL (base station) -----------------------------------------------------


def make_base_station(bus: MessageBus) -> BaseStation:
    env = Environment.from_config({
        "bounds": [-5000.0, -5000.0, 5000.0, 5000.0],
        "default_zone": "SAFE",
        "assets": [{"name": "substation", "position": [0.0, 0.0, 0.0]}],
    })
    return BaseStation(bus, env, DebrisModel(np.random.default_rng(0)),
                       uav_speeds={"u1": 60.0})


def test_denied_track_reenters_allocation_after_ttl():
    bus = MessageBus()
    published = []
    bus.subscribe("engagement/tasks", published.append)
    bs = make_base_station(bus)
    bs._on_tracks(TrackArray(header=Header(stamp=0.0), tracks=[Track(
        header=Header(stamp=0.0), track_id=1,
        position=np.array([3000.0, 0.0, 800.0]),
        velocity=np.array([-55.0, 0.0, 0.0]),
    )]))
    bs._denied[1] = 0.0

    # Telemetry is republished fresh before each planning tick, as the
    # 10 Hz uav/state stream does — silent platforms are not allocated.
    bs._on_uav_state(UavState(header=Header(stamp=DENIAL_TTL_S - 5.0),
                              uav_id="u1", position=np.zeros(3), ammo=4))
    bs.update(DENIAL_TTL_S - 5.0, 1.0)
    assert published[-1] == []                 # still inside the TTL window
    bs._on_uav_state(UavState(header=Header(stamp=DENIAL_TTL_S + 1.0),
                              uav_id="u1", position=np.zeros(3), ammo=4))
    bs.update(DENIAL_TTL_S + 1.0, 1.0)
    assert [task.track_id for task in published[-1]] == [1]


def test_unavailable_platforms_are_not_assigned():
    """RTB / rearming / low-battery airframes ignore tasking — handing
    them the shooter slot silently un-defends the track."""
    bus = MessageBus()
    published = []
    bus.subscribe("engagement/tasks", published.append)
    bs = make_base_station(bus)
    bs._on_tracks(TrackArray(header=Header(stamp=0.0), tracks=[Track(
        header=Header(stamp=0.0), track_id=1,
        position=np.array([3000.0, 0.0, 800.0]),
        velocity=np.array([-55.0, 0.0, 0.0]),
    )]))
    for uid, kw in (("u1", dict(ammo=0)),
                    ("u2", dict(ammo=4, battery=0.05)),
                    ("u3", dict(ammo=4, mode=UavMode.REARM)),
                    ("u4", dict(ammo=4, mode=UavMode.RTB))):
        bs._on_uav_state(UavState(header=Header(stamp=0.0), uav_id=uid,
                                  position=np.zeros(3), **kw))
    bs.update(0.0, 1.0)
    assert published[-1] == []

    bs._on_uav_state(UavState(header=Header(stamp=0.0), uav_id="u5",
                              position=np.zeros(3), ammo=4))
    bs.update(1.0, 1.0)
    assert [task.shooter_id for task in published[-1]] == ["u5"]


# -- stable task ids ----------------------------------------------------------------


def test_task_id_stable_for_unchanged_pairing():
    rm = RiskMap((-5000, -5000, 5000, 5000), default=ZoneClass.SAFE)
    tracks = {1: Track(header=Header(stamp=0.0), track_id=1,
                       position=np.array([3000.0, 0.0, 1000.0]),
                       velocity=np.array([-55.0, 0.0, 0.0]))}
    assessments = [ThreatAssessment(header=Header(stamp=0.0), track_id=1,
                                    threat_score=0.8, time_to_impact=100.0,
                                    predicted_impact=np.zeros(3))]
    uavs = [UavState(header=Header(stamp=0.0), uav_id="u1",
                     position=np.zeros(3), ammo=4)]
    registry: dict[tuple[int, str], int] = {}

    first = assignment.allocate(assessments, tracks, uavs, {"u1": 60.0}, rm,
                                t=0.0, task_ids=registry)
    second = assignment.allocate(assessments, tracks, uavs, {"u1": 60.0}, rm,
                                 t=1.0, task_ids=registry)
    assert first[0].task_id == second[0].task_id


# -- threat evaluation: closing geometry only ----------------------------------------


def test_receding_track_is_not_urgent():
    env = Environment.from_config({
        "bounds": [-5000.0, -5000.0, 5000.0, 5000.0],
        "default_zone": "SAFE",
        "assets": [{"name": "substation", "position": [0.0, 0.0, 0.0]}],
    })
    inbound = Track(header=Header(stamp=0.0), track_id=1,
                    position=np.array([3000.0, 0.0, 800.0]),
                    velocity=np.array([-55.0, 0.0, 0.0]))
    # Same distance from the asset, same speed — flying *away* after a pass.
    outbound = Track(header=Header(stamp=0.0), track_id=2,
                     position=np.array([300.0, 0.0, 800.0]),
                     velocity=np.array([55.0, 0.0, 0.0]))
    a_in = threat_evaluation.assess(inbound, env, t=0.0)
    a_out = threat_evaluation.assess(outbound, env, t=0.0)
    assert a_out.time_to_impact > a_in.time_to_impact
    assert a_out.threat_score < a_in.threat_score


# -- ROE: now-or-never needs strict improvement --------------------------------------


def test_now_or_never_holds_on_flat_cost_without_time_pressure():
    """Uniform ground means every lookahead ties: that is 'holding costs
    nothing', not authority to skip the HOLD/herd loop."""
    rm = RiskMap((-5000, -5000, 5000, 5000), cell_size=100.0,
                 default=ZoneClass.DANGEROUS)
    roe = RulesOfEngagement(rm, DebrisModel(np.random.default_rng(4)), RoeConfig())
    request = FireRequest(
        header=Header(stamp=0.0), task_id=1, uav_id="u1", track_id=1,
        effector=EffectorType.NET,
        predicted_intercept=np.array([2000.0, 2000.0, 300.0]), p_kill=0.5,
    )
    assessment = ThreatAssessment(header=Header(stamp=0.0), track_id=1,
                                  threat_score=0.9, time_to_impact=120.0,
                                  predicted_impact=np.zeros(3))
    c = roe.evaluate(request, np.array([55.0, 0.0, 0.0]), EffectorType.NET,
                     assessment, t=0.0)
    assert c.decision == EngagementDecision.HOLD
    assert c.track_id == 1                     # verdicts carry their track
