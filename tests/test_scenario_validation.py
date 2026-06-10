"""Scenario sanity: threat ids must be unique — ``world.enemies`` is keyed
by id, so a duplicate would silently replace an in-flight enemy at spawn."""

import copy

import pytest

from coopuavs.sim import scenario as scenario_mod

BASE = {
    "name": "ids",
    "duration": 10.0,
    "environment": {
        "bounds": [-2000.0, -2000.0, 2000.0, 2000.0],
        "default_zone": "SAFE",
        "assets": [{"name": "substation", "position": [0.0, 0.0, 0.0]}],
    },
    "threats": [
        {"time": 1.0, "class": "FPV", "id": "fpv-1",
         "spawn": [-1800.0, 0.0, 80.0], "target": "substation"},
        {"time": 2.0, "class": "FPV", "id": "fpv-1",
         "spawn": [-1800.0, 500.0, 80.0], "target": "substation"},
    ],
}


def test_duplicate_threat_id_is_rejected():
    with pytest.raises(ValueError, match="duplicate threat id"):
        scenario_mod.build(copy.deepcopy(BASE))


def test_auto_ids_skip_an_explicit_collision():
    cfg = copy.deepcopy(BASE)
    cfg["threats"][1].pop("id")        # auto id would be fpv-1: must skip ahead
    sc = scenario_mod.build(cfg)
    ids = [factory().id for _, factory in sc.world._spawn_queue]
    assert ids == ["fpv-1", "fpv-2"]
