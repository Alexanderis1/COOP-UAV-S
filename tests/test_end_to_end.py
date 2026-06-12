"""End-to-end smoke test: a small raid against a small defended area.

Asserts the full pipeline closes the loop — sensing, fusion, threat
evaluation, assignment, pursuit, ROE-cleared fire, adjudication — by
requiring at least one kill, and that the safety layer keeps wrecks out of
the CRITICAL zone.
"""

import copy

from coopuavs.sim import scenario as scenario_mod

SMALL_SCENARIO = {
    "name": "smoke",
    "seed": 11,
    "dt": 0.05,
    "duration": 240.0,
    "environment": {
        "bounds": [-4000.0, -4000.0, 4000.0, 4000.0],
        "cell_size": 100.0,
        "default_zone": "SAFE",
        "zones": [
            {"rect": [-800, -800, 800, 800], "class": "DANGEROUS"},
            {"rect": [-300, -300, 300, 300], "class": "CRITICAL"},
        ],
        "assets": [
            {"name": "substation", "position": [0.0, 0.0, 0.0], "value": 1.0}
        ],
    },
    "base_station": {"rate_hz": 1.0},
    "sensors": [
        {"type": "radar", "name": "radar-1", "position": [0.0, -1000.0, 10.0],
         "max_range": 9000.0},
        {"type": "eo_ir", "name": "eo-1", "position": [0.0, 0.0, 20.0]},
    ],
    "interceptors": [
        {"id": "u1", "home": [-200.0, -1000.0, 0.0], "effector": "projectile",
         "max_speed": 80.0},
        {"id": "u2", "home": [200.0, -1000.0, 0.0], "effector": "projectile",
         "max_speed": 80.0},
    ],
    "threats": [
        {"time": 5.0, "class": "OWA_STRATEGIC",
         "spawn": [-3800.0, 3800.0, 1200.0], "target": "substation"},
        {"time": 15.0, "class": "OWA_STRATEGIC",
         "spawn": [-3500.0, 3800.0, 1300.0], "target": "substation"},
    ],
}


def test_raid_is_engaged_and_safely():
    sc = scenario_mod.build(SMALL_SCENARIO)
    summary = sc.run()

    assert summary["enemies_total"] == 2
    # The defence must defeat at least one OWA on this easy geometry.
    assert summary["kills"] >= 1
    # Safety invariant: no wreck on CRITICAL ground.
    assert summary["wrecks_by_zone"].get("CRITICAL", 0) == 0
    # The loop produced the expected event types.
    kinds = {e["kind"] for e in sc.world.events}
    assert "enemy_spawn" in kinds and "kill" in kinds


def test_deterministic_given_seed():
    s1 = scenario_mod.build(SMALL_SCENARIO).run()
    s2 = scenario_mod.build(SMALL_SCENARIO).run()
    assert s1 == s2


def test_raid_load_factor_airframe():
    """The same raid on the 3-DOF load-factor airframe with terminal PN:
    the kill chain and the safety invariant must hold, the run must stay
    deterministic, and it must complete in the same number of world steps
    (the fidelity bump is not allowed to cost Monte-Carlo throughput)."""
    cfg = copy.deepcopy(SMALL_SCENARIO)
    for u in cfg["interceptors"]:
        u["airframe"] = "load_factor"
        u["n_max"] = 4.0
    sc = scenario_mod.build(cfg)
    summary = sc.run()

    assert summary["kills"] >= 1
    assert summary["wrecks_by_zone"].get("CRITICAL", 0) == 0
    assert scenario_mod.build(cfg).run() == summary
    assert sc.world.dt == SMALL_SCENARIO["dt"]   # same tick, same cost
