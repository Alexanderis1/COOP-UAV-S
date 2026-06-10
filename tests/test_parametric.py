"""Parametric scenario builds from ICD §3 start_run requests (SIM-THR-002)."""

import numpy as np
import pytest

from coopuavs.core.messages import ThreatClass
from coopuavs.sim import scenario as scenario_mod

PRESET = {
    "name": "preset",
    "seed": 1,
    "dt": 0.05,
    "duration": 300.0,
    "environment": {
        "bounds": [-4000.0, -4000.0, 4000.0, 4000.0],
        "default_zone": "SAFE",
        "assets": [
            {"name": "substation", "position": [500.0, 0.0, 0.0]},
            {"name": "depot", "position": [-1000.0, -500.0, 0.0]},
        ],
    },
    "sensors": [
        {"type": "radar", "name": "radar-1", "position": [0.0, 0.0, 10.0],
         "max_range": 9000.0},
    ],
    "interceptors": [
        {"id": "u1", "home": [0.0, -500.0, 0.0], "effector": "projectile"},
    ],
    "turrets": [
        {"id": "t1", "position": [0.0, 0.0, 5.0]},
    ],
    "threats": [
        {"time": 5.0, "class": "OWA_STRATEGIC",
         "spawn": [-3800.0, 3800.0, 1200.0], "target": "substation"},
    ],
}


def spawn_all(sc):
    """Force-spawn every scheduled enemy (factories run at their wave time)."""
    while sc.world._spawn_queue:
        _, factory = sc.world._spawn_queue.pop(0)
        enemy = factory()
        sc.world.enemies[enemy.id] = enemy
    return list(sc.world.enemies.values())


def test_counts_targets_axes_and_weather():
    request = {
        "threats": {
            "fpv": {"count": 3, "target": "depot", "axis_deg": 270.0,
                    "first_time": 5.0, "spacing": 4.0},
            "owa_strategic": {"count": 2, "target": "auto",
                              "axis_deg": None, "first_time": 20.0, "spacing": 8.0},
        },
        "weather": {"wind_speed": 6.0, "wind_dir_deg": 315.0, "fog": 0.3},
        "duration": 240.0,
        "speed": 2.0,
        "posture": "pre_authorized",
    }
    sc = scenario_mod.build_parametric(request, PRESET, seed=42)

    spawn_times = [t for t, _ in sc.world._spawn_queue]
    assert spawn_times == [5.0, 9.0, 13.0, 20.0, 28.0]

    enemies = spawn_all(sc)
    by_class = {}
    for e in enemies:
        by_class.setdefault(e.threat_class, []).append(e)
    assert len(by_class[ThreatClass.FPV]) == 3
    assert len(by_class[ThreatClass.OWA_STRATEGIC]) == 2

    # Explicit target honoured; "auto" round-robins over the preset assets.
    assert all(e.target_name == "depot" for e in by_class[ThreatClass.FPV])
    assert sorted(e.target_name for e in by_class[ThreatClass.OWA_STRATEGIC]) \
        == ["depot", "substation"]

    # Spawns sit outside the map on the requested bearing at class altitude.
    for e in by_class[ThreatClass.FPV]:
        assert e.position[0] < -4000.0           # approach from the west
        assert abs(e.position[2] - 80.0) < 1.0   # FPV cruise altitude
    for e in by_class[ThreatClass.OWA_STRATEGIC]:
        assert abs(e.position[2] - 1500.0) < 1.0

    # Weather override and run meta land on the scenario.
    assert sc.world.weather.wind_speed == 6.0
    assert sc.world.weather.fog == 0.3
    assert sc.duration == 240.0
    assert sc.meta["speed"] == 2.0
    assert sc.meta["posture"] == "pre_authorized"
    assert sc.meta["seed"] == 42
    assert "t1" in sc.turrets


def test_parametric_is_deterministic_for_seed():
    request = {"threats": {"loitering": {"count": 2, "target": "auto",
                                         "axis_deg": None}}}
    a = spawn_all(scenario_mod.build_parametric(request, PRESET, seed=7))
    b = spawn_all(scenario_mod.build_parametric(request, PRESET, seed=7))
    assert all(np.array_equal(x.position, y.position) for x, y in zip(a, b))


def test_unknown_asset_rejected_with_clear_message():
    request = {"threats": {"fpv": {"count": 1, "target": "power-plant"}}}
    with pytest.raises(ValueError, match="power-plant.*substation"):
        scenario_mod.build_parametric(request, PRESET, seed=1)


def test_unknown_class_rejected():
    request = {"threats": {"ballistic": {"count": 1, "target": "auto"}}}
    with pytest.raises(ValueError, match="ballistic"):
        scenario_mod.build_parametric(request, PRESET, seed=1)


def test_oversized_group_count_rejected():
    request = {"threats": {"fpv": {"count": scenario_mod.MAX_GROUP_COUNT + 1}}}
    with pytest.raises(ValueError, match="per-class maximum"):
        scenario_mod.build_parametric(request, PRESET, seed=1)


def test_oversized_total_threats_rejected():
    # Each group within the per-class cap, but the raid total over the limit.
    per_class = scenario_mod.MAX_GROUP_COUNT
    request = {"threats": {
        cls: {"count": per_class} for cls in ("fpv", "owa_strategic", "loitering")
    }}
    assert 3 * per_class > scenario_mod.MAX_TOTAL_THREATS
    with pytest.raises(ValueError, match="exceeding the"):
        scenario_mod.build_parametric(request, PRESET, seed=1)


def test_excessive_or_non_finite_duration_rejected():
    request = {"threats": {"fpv": {"count": 1}},
               "duration": scenario_mod.MAX_DURATION_S + 1.0}
    with pytest.raises(ValueError, match="duration"):
        scenario_mod.build_parametric(request, PRESET, seed=1)
    request["duration"] = float("nan")
    with pytest.raises(ValueError, match="duration"):
        scenario_mod.build_parametric(request, PRESET, seed=1)


def test_capped_request_still_builds():
    request = {"threats": {"fpv": {"count": scenario_mod.MAX_GROUP_COUNT}},
               "duration": scenario_mod.MAX_DURATION_S}
    sc = scenario_mod.build_parametric(request, PRESET, seed=1)
    assert len(sc.world._spawn_queue) == scenario_mod.MAX_GROUP_COUNT
    assert sc.duration == scenario_mod.MAX_DURATION_S


def test_yaml_build_path_unchanged():
    sc = scenario_mod.build(PRESET)
    assert len(sc.world._spawn_queue) == 1
    assert sc.meta["seed"] == 1
