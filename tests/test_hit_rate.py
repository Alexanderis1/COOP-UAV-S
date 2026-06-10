"""Hit-rate regression floor (v0.3 fire-quality fixes).

Before the fixes the 5-seed reference raid spent ~15.8 rounds per kill
(envelope-edge shots, stale aim points, net-vs-OWA pairings, turret spray
at hopeless geometry). These tests pin the recovered behaviour.
"""

import numpy as np

from coopuavs.c2 import assignment
from coopuavs.core.messages import Header, ThreatAssessment, Track, UavState, ZoneClass
from coopuavs.risk.zones import RiskMap
from coopuavs.sim import scenario as scenario_mod


def test_five_seed_ammo_economy():
    shots = kills = 0
    for seed in range(5):
        sc = scenario_mod.load("scenarios/residential_raid.yaml", seed=seed)
        sc.run()
        eco = sc.eval_tracker.metrics()["economics"]
        shots += eco["shots"]
        kills += eco["kills"]
    assert kills >= 10, f"kill floor regressed: {kills} kills over 5 seeds"
    assert shots / kills <= 9.0, \
        f"ammo per kill regressed: {shots}/{kills} = {shots / kills:.1f}"


def test_net_never_paired_with_fast_track():
    """PHY-GCS-007: the closing-speed envelope filter keeps net carriers
    off targets they can only roll zero-Pk shots at."""
    rm = RiskMap((-2000, -2000, 2000, 2000), cell_size=100, default=ZoneClass.SAFE)
    trk = Track(header=Header(stamp=0.0), track_id=1,
                position=np.array([1000.0, 0.0, 800.0]),
                velocity=np.array([-55.0, 0.0, 0.0]))     # OWA cruise
    assessment = ThreatAssessment(header=Header(stamp=0.0), track_id=1,
                                  threat_score=0.8, time_to_impact=30.0)
    uavs = [
        UavState(header=Header(stamp=0.0), uav_id="net-1",
                 position=np.array([900.0, 0.0, 400.0]), ammo=2, battery=1.0),
        UavState(header=Header(stamp=0.0), uav_id="hawk-1",
                 position=np.array([-1500.0, 0.0, 400.0]), ammo=8, battery=1.0),
    ]
    tasks = assignment.allocate(
        [assessment], {1: trk}, uavs,
        {"net-1": 50.0, "hawk-1": 80.0}, rm, 0.0,
        uav_effectors={"net-1": "net", "hawk-1": "projectile"},
    )
    # the net carrier is closer, but 55 m/s > 0.8 x 45 m/s closing envelope
    assert tasks[0].shooter_id == "hawk-1"
