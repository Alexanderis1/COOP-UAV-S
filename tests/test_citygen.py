"""Deterministic city generator (SIM-ENV-006)."""

import copy

from coopuavs.core.messages import ZoneClass
from coopuavs.sim import scenario as scenario_mod
from coopuavs.sim.citygen import generate
from coopuavs.sim.environment import Environment


def test_same_seed_same_city():
    assert generate(7) == generate(7)


def test_different_seed_different_city():
    a, b = generate(7), generate(8)
    assert a["environment"]["buildings"] != b["environment"]["buildings"]


def test_city_composition():
    cfg = generate(7)
    kinds = {}
    for b in cfg["environment"]["buildings"]:
        kinds[b["kind"]] = kinds.get(b["kind"], 0) + 1
    assert kinds.get("hospital") == 1
    assert kinds.get("school") == 2
    assert kinds.get("water") == 1
    assert kinds.get("park", 0) >= 2
    assert kinds.get("residential_high", 0) > 10
    assert kinds.get("residential_low", 0) > 10
    assert kinds.get("industrial", 0) >= 8
    assert len(cfg["interceptors"]) == 20
    assert len(cfg["sentinels"]) == 10
    assert len(cfg["environment"]["charging_stations"]) == 6
    assert sum(1 for i in cfg["interceptors"] if i["effector"] == "projectile") == 14


def test_derived_zones_show_all_three_colours():
    env = Environment.from_config(generate(7)["environment"])
    grid = env.risk_map.grid
    for zone in (ZoneClass.SAFE, ZoneClass.DANGEROUS, ZoneClass.CRITICAL):
        assert (grid == int(zone)).any()


def test_checked_in_urban_raid_matches_generator():
    """The committed scenario claims `citygen --seed 7` provenance: pin it
    to the generator so the two cannot drift silently after tuning."""
    import yaml
    with open("scenarios/urban_raid.yaml", encoding="utf-8") as f:
        assert yaml.safe_load(f) == generate(7)


def test_generated_scenario_builds_and_runs():
    sc = scenario_mod.build(copy.deepcopy(generate(7)))
    assert len(sc.uavs) == 20 and len(sc.sentinels) == 10 and len(sc.turrets) == 3
    sc.world.run(5.0, stop_when_clear=False)
    assert sc.world.t >= 5.0
