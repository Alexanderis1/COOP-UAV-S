"""Sentinel surveillance UAVs (PHY-SNT-001..003)."""

import numpy as np

from coopuavs.core.messages import UavMode
from coopuavs.sim import scenario as scenario_mod

CFG = {
    "name": "sentinel-test",
    "seed": 5,
    "dt": 0.05,
    "duration": 120.0,
    "environment": {
        "bounds": [-4000.0, -4000.0, 4000.0, 4000.0],
        "cell_size": 100.0,
        "default_zone": "SAFE",
        "assets": [{"name": "hq", "position": [0.0, 0.0, 0.0]}],
        "buildings": [
            # a concrete slab that masks the ground EO tower's look north
            {"rect": [-200.0, 300.0, 200.0, 500.0], "height": 60.0,
             "kind": "residential_high"},
        ],
        "charging_stations": [
            {"id": "cs-1", "pos": [0.0, -1500.0, 0.0], "rooftop": False},
        ],
    },
    # only a short-range ground EO sensor: the sentinel is the long eye
    "sensors": [
        {"type": "eo_ir", "name": "eo-base", "position": [0.0, -200.0, 20.0],
         "max_range": 600.0},
    ],
    "seekers": False,
    "interceptors": [],
    "sentinels": [
        {"id": "sent-1", "station": "cs-1",
         "orbit": {"center": [0.0, 1200.0], "radius": 500.0, "alt": 300.0,
                   "speed": 25.0}},
    ],
    "threats": [
        # holds far north — invisible to the 600 m ground sensor
        {"time": 1.0, "class": "OWA_STRATEGIC", "spawn": [-3500.0, 2500.0, 800.0],
         "target": [3500.0, 2500.0, 0.0]},
    ],
}


def build():
    import copy
    return scenario_mod.build(copy.deepcopy(CFG))


def test_sentinel_orbit_flown_and_state_published():
    sc = build()
    sent = sc.sentinels["sent-1"]
    states = []
    sc.world.bus.subscribe("uav/state", lambda m: states.append(m)
                           if m.uav_id == "sent-1" else None)
    sc.world.run(110.0, stop_when_clear=False)
    # reached the orbit annulus and patrols it
    r = float(np.linalg.norm(sent.position[:2] - np.array([0.0, 1200.0])))
    assert abs(r - 500.0) < 160.0
    assert sent.mode == UavMode.PATROL
    assert states and states[-1].kind == "sentinel" and states[-1].ammo == 0


def test_sentinel_detections_form_track():
    """The target flies beyond every ground sensor: only the sentinel's
    mounted payload can build the picture."""
    sc = build()
    tracks = []
    sc.world.bus.subscribe("tracks", lambda m: tracks.append(m))
    sc.world.run(110.0, stop_when_clear=False)
    assert any(m.tracks for m in tracks), "sentinel detections never fused into a track"


def test_low_battery_rtb_and_resume():
    sc = build()
    sent = sc.sentinels["sent-1"]
    sc.world.run(40.0, stop_when_clear=False)
    sent.battery = 0.10                     # force the RTB floor
    sc.world.run(60.0, stop_when_clear=False)
    assert sent.mode in (UavMode.RTB, UavMode.REARM, UavMode.PATROL, UavMode.TRANSIT)
    # after the turnaround the battery is restored and patrol resumes
    sc.world.run(120.0, stop_when_clear=False)
    assert sent.battery > 0.5
    assert sent.mode in (UavMode.PATROL, UavMode.TRANSIT)
