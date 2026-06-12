"""P4-7 recorder/ICD additive fields (ICD_RUNTIME v0.4).

Sitl runs MAY add `att`/`nav_q` (and `health`, from P5) to `uavs[]`
frame entries; the keys appear exactly when the platform reports them.
The binding claim is backwards compatibility: a pointmass recording
carries NONE of the new keys — its uav entries keep the exact v0.3 key
set — while a sitl recording carries estimate-domain attitude and nav
quality once telemetry flows.
"""

from __future__ import annotations

import copy

import numpy as np

from coopuavs.sim import scenario as scenario_mod
from test_end_to_end import SMALL_SCENARIO
from test_sitl_stage1 import SITL_SMALL_SCENARIO

V03_UAV_KEYS = {"id", "pos", "vel", "mode", "ammo", "battery", "task_id",
                "link", "kind", "effector"}


def test_pointmass_recording_keeps_v03_schema_exactly():
    sc = scenario_mod.build(copy.deepcopy(SMALL_SCENARIO))
    sc.world.run(8.0, stop_when_clear=False)
    frames = sc.recorder.frames
    entries = [u for f in frames for u in f["uavs"]]
    assert entries, "no uav entries recorded"
    for u in entries:
        assert set(u) == V03_UAV_KEYS, set(u) ^ V03_UAV_KEYS


def test_sitl_recording_carries_attitude_and_nav_quality():
    sc = scenario_mod.build(copy.deepcopy(SITL_SMALL_SCENARIO))
    sc.world.run(8.0, stop_when_clear=False)   # past boot + first telemetry
    last = sc.recorder.frames[-1]
    by_id = {u["id"]: u for u in last["uavs"]}
    assert "u1" in by_id
    u = by_id["u1"]
    att = u["att"]
    assert len(att) == 4
    assert abs(float(np.linalg.norm(att)) - 1.0) < 0.01   # unit quaternion
    assert 0.0 < u["nav_q"] < 10.0                        # sigma_pos_h, m
    # P5-4: the UavHealth digest rides every sitl entry (ICD v0.5);
    # a healthy boot reports a clean word.
    assert u["health"]["faults"] == 0 and u["health"]["codes"] == []
    # the replay file serialises (json round-trip safety)
    import json
    json.dumps(last)