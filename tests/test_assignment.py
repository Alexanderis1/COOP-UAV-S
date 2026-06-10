import numpy as np

from coopuavs.core.messages import (
    Header,
    ThreatAssessment,
    Track,
    UavState,
    ZoneClass,
)
from coopuavs.c2 import assignment
from coopuavs.risk.zones import RiskMap


def track(tid, pos, vel, p_decoy=0.0):
    return Track(
        header=Header(stamp=0.0), track_id=tid,
        position=np.asarray(pos, float), velocity=np.asarray(vel, float),
        p_decoy=p_decoy,
    )


def uav(uid, pos, ammo=4):
    return UavState(header=Header(stamp=0.0), uav_id=uid,
                    position=np.asarray(pos, float), ammo=ammo)


def assess(tid, score=0.8):
    return ThreatAssessment(header=Header(stamp=0.0), track_id=tid,
                            threat_score=score, time_to_impact=100.0,
                            predicted_impact=np.zeros(3))


def setup():
    rm = RiskMap((-5000, -5000, 5000, 5000), default=ZoneClass.SAFE)
    speeds = {"u1": 60.0, "u2": 60.0, "u3": 60.0}
    return rm, speeds


def test_each_threat_gets_a_distinct_shooter():
    rm, speeds = setup()
    tracks = {
        1: track(1, [3000, 3000, 1000], [-50, -50, 0]),
        2: track(2, [-3000, 3000, 1000], [50, -50, 0]),
    }
    uavs = [uav("u1", [2500, 0, 0]), uav("u2", [-2500, 0, 0]), uav("u3", [0, 0, 0])]
    tasks = assignment.allocate(
        [assess(1), assess(2)], tracks, uavs, speeds, rm, t=0.0
    )
    shooters = {t.track_id: t.shooter_id for t in tasks}
    assert shooters[1] == "u1" and shooters[2] == "u2"
    # The spare UAV reinforces someone.
    assert any("u3" in t.support_ids for t in tasks)


def test_high_decoy_probability_excluded():
    rm, speeds = setup()
    tracks = {1: track(1, [3000, 0, 1000], [-55, 0, 0], p_decoy=0.95)}
    tasks = assignment.allocate(
        [assess(1)], tracks, [uav("u1", [0, 0, 0])], speeds, rm, t=0.0
    )
    assert tasks == []


def test_denied_tracks_excluded():
    rm, speeds = setup()
    tracks = {1: track(1, [3000, 0, 1000], [-55, 0, 0])}
    tasks = assignment.allocate(
        [assess(1)], tracks, [uav("u1", [0, 0, 0])], speeds, rm,
        t=0.0, denied_tracks={1},
    )
    assert tasks == []


def test_fast_target_gets_support():
    rm, speeds = setup()
    # 100 m/s target vs 60 m/s interceptors: needs blockers.
    tracks = {1: track(1, [0, 5000, 2000], [0, -100, 0])}
    uavs = [uav("u1", [0, 0, 0]), uav("u2", [500, 0, 0]), uav("u3", [-500, 0, 0])]
    tasks = assignment.allocate([assess(1)], tracks, uavs, speeds, rm, t=0.0)
    assert len(tasks) == 1
    assert len(tasks[0].support_ids) == 2
