"""Forward high-altitude CAP sentinels: patrol geometry, start-on-station,
and the detection time-margin against the high-altitude diver (PHY-SNT-004)."""

import copy

import numpy as np
import pytest
import yaml

from coopuavs.core.messages import UavMode
from coopuavs.interceptors import patrol
from coopuavs.sim import scenario as scenario_mod


# -- patrol geometry --------------------------------------------------------

def test_circle_waypoint_matches_legacy_formula():
    """The circle pattern must reproduce the v0.1 orbit formula exactly, or
    existing sentinel scenarios (urban_raid) stop being byte-reproducible."""
    center = np.array([100.0, -200.0])
    r, alt = 800.0, 350.0
    for phase in (0.0, 0.5, 1.3, 3.0, 6.0):
        wp = patrol.orbit_waypoint("circle", center, r, alt, phase)
        ang = phase + 0.15
        legacy = np.array([center[0] + r * np.sin(ang),
                           center[1] + r * np.cos(ang), alt])
        assert np.allclose(wp, legacy)
    pos = np.array([900.0, -200.0])
    assert np.isclose(patrol.path_offset("circle", center, r, pos),
                      abs(float(np.linalg.norm(pos - center)) - r))


def test_racetrack_waypoints_lie_on_the_loop():
    center = np.array([0.0, 4000.0])
    r, alt, leg, hd = 700.0, 3900.0, 3000.0, 90.0
    for phase in np.linspace(0.0, 2 * np.pi, 25, endpoint=False):
        wp = patrol.orbit_waypoint("racetrack", center, r, alt, phase,
                                   heading_deg=hd, leg=leg, lead=0.0)
        assert abs(wp[2] - alt) < 1e-9
        off = patrol.path_offset("racetrack", center, r, wp[:2],
                                 heading_deg=hd, leg=leg)
        assert off < 1e-6, f"phase {phase}: waypoint off the loop by {off}"
    assert np.isclose(patrol.loop_length("racetrack", r, leg),
                      2 * leg + 2 * np.pi * r)


# -- start-on-station -------------------------------------------------------

def _cap_cfg(**orbit):
    base_orbit = {"center": [0.0, 3000.0], "radius": 600.0, "alt": 3500.0,
                  "speed": 35.0}
    base_orbit.update(orbit)
    return {
        "name": "cap", "seed": 1, "dt": 0.05, "duration": 30.0,
        "environment": {"bounds": [-6000.0, -6000.0, 6000.0, 6000.0],
                        "default_zone": "SAFE",
                        "assets": [{"name": "hq", "position": [0.0, 0.0, 0.0]}]},
        "sensors": [], "seekers": False, "interceptors": [],
        "sentinels": [{"id": "cap-1", "home": [0.0, -2500.0, 0.0],
                       "max_speed": 42.0, "battery_minutes": 120.0,
                       "start_on_station": True, "orbit": base_orbit,
                       "payload": [{"type": "airborne_radar"}]}],
        "threats": [],
    }


def test_start_on_station_spawns_in_patrol_on_the_path():
    sc = scenario_mod.build(_cap_cfg())
    sent = sc.sentinels["cap-1"]
    assert sent.mode == UavMode.PATROL, "should stand up already patrolling"
    off = patrol.path_offset("circle", sent.orbit_center, sent.orbit_radius,
                             sent.body.position[:2])
    assert off < 1.0 and abs(sent.body.position[2] - 3500.0) < 1.0
    # the mounted airborne radar node exists
    assert any(n.name == "aewr-cap-1" for n in sc.world.nodes)


def test_racetrack_cap_holds_station():
    sc = scenario_mod.build(_cap_cfg(pattern="racetrack", heading_deg=90.0,
                                     leg=2500.0, center=[0.0, 3500.0]))
    sent = sc.sentinels["cap-1"]
    sc.world.run(20.0, stop_when_clear=False)
    assert sent.mode == UavMode.PATROL
    off = patrol.path_offset("racetrack", sent.orbit_center, sent.orbit_radius,
                             sent.body.position[:2], heading_deg=90.0, leg=2500.0)
    assert off < 200.0, f"drifted off the barrier racetrack by {off} m"


# -- the time-margin win (A/B) ----------------------------------------------

@pytest.mark.slow
def test_cap_sentinels_cut_jet_detection_latency():
    """Forward high CAP must identify the diving jets far earlier than the
    ground sensor network — the literal objective. Robust across seeds."""
    cfg = yaml.safe_load(open("scenarios/high_diver_raid.yaml"))
    no_cap = copy.deepcopy(cfg)
    no_cap.pop("sentinels")

    def mean_jet_acq(c, seed):
        sc = scenario_mod.build(copy.deepcopy(c), seed=seed)
        sc.world.run(sc.duration)
        lats = [e.acquired_t - e.spawn_t for e in sc.world.enemies.values()
                if e.threat_class.value == "owa_jet" and e.acquired_t is not None]
        return float(np.mean(lats)) if lats else None

    seeds = range(1, 4)
    with_cap = np.mean([mean_jet_acq(cfg, s) for s in seeds])
    without = np.mean([mean_jet_acq(no_cap, s) for s in seeds])
    assert with_cap < without - 0.5, \
        f"CAP must cut jet acquisition latency (with={with_cap:.2f}s, " \
        f"without={without:.2f}s)"
