"""P5-4: UavHealth northbound — the CBIT digest rides UavState >= 1 Hz
(PHY-UAV-013/033) through the same comms layer as everything else, into
the recorder (ICD v0.5 additive).

The digest merges two health sources: the FCU's HEALTH wire word and
the MC's own CBIT engine (the C2 radio is the MC's — LINK_C2_LOSS is
invisible to the FCU). C2 loss inhibits release even with a token in
hand: a clearance nobody can re-confirm authorizes a stale geometry.
"""

from __future__ import annotations

import copy
import json
import sys

sys.path.insert(0, "tests")

from coopuavs.coopfc.cbit import FAULTS
from coopuavs.coopfc.cbit.dictionary import word_names
from coopuavs.core.messages import EngagementDecision
from test_sitl_stage2 import _AppHost, _clr, _hosted_engine, _run, _task, _tracks


def _last_state(mcu):
    states = mcu.ports.box("uav_state").drain()
    assert states, "no telemetry published"
    return states[-1]


# ----------------------------------------------------------- the digest

def test_health_rides_uav_state_healthy_then_faulted():
    eng, mcu = _hosted_engine()
    t = _run(eng, 0.0, 6.0)
    h = _last_state(mcu).health
    assert h == {"faults": 0, "codes": [], "inhibit_fire": False,
                 "inhibit_arming": False, "degraded": ""}

    eng.fcus[0].params._values["fcu.pos_kp"] = 999.0   # bit-rot
    t = _run(eng, t, 3.0)                              # raise + HEALTH 1 Hz
    h = _last_state(mcu).health
    assert h["faults"] & (1 << FAULTS["PARAM_CRC"].bit)
    assert "PARAM_CRC" in h["codes"]
    assert h["inhibit_fire"] and h["inhibit_arming"]


def test_word_names_round_trip():
    word = (1 << FAULTS["EKF_DIVERGED"].bit) | (1 << FAULTS["ESC_STALE"].bit)
    assert word_names(word) == ["EKF_DIVERGED", "ESC_STALE"]
    assert word_names(0) == []


# ------------------------------------------------- C2 loss, MC-side fault

def test_c2_loss_is_an_mc_side_fault_and_vetoes_release():
    host = _AppHost()
    host.post("tasks", [_task(track_id=1, task_id=7)])
    host.post("tracks", _tracks(1))
    host.post("clearance", _clr(track_id=1, task_id=7,
                                decision=EngagementDecision.AUTHORIZED))
    host.post("link_quality", 0.05)             # jammed C2
    host.step(0.0)
    host.step(0.1)
    assert host.fires == []                     # token in hand, no re-confirm
    states = host.ports.box("uav_state").drain()
    h = states[-1].health
    assert "LINK_C2_LOSS" in h["codes"] and h["inhibit_fire"]

    host.post("link_quality", 1.0)              # C2 back, token still fresh
    host.step(0.3)
    assert len(host.fires) == 1                 # release resumes
    h = host.ports.box("uav_state").drain()[-1].health
    assert h["codes"] == [] and not h["inhibit_fire"]


# -------------------------------------------------------------- recorder

def test_recorded_frames_carry_health_at_rate():
    from coopuavs.sim import scenario as scenario_mod
    from test_sitl_stage1 import SITL_SMALL_SCENARIO

    cfg = copy.deepcopy(SITL_SMALL_SCENARIO)
    cfg["duration"] = 10.0
    cfg["faults"] = [{"t": 5.0, "uav": "u1", "kind": "gyro_stuck"}]
    sc = scenario_mod.build(cfg, seed=2)
    sc.run()
    frames = sc.recorder.frames
    with_health = [f for f in frames
                   if any("health" in u for u in f["uavs"])]
    assert len(with_health) >= 10               # >= 1 Hz over 10 s
    last = json.loads(json.dumps(with_health[-1]))   # ICD: json round-trip
    u1 = next(u for u in last["uavs"] if u["id"] == "u1")
    assert "GYRO_STUCK" in u1["health"]["codes"]
    assert u1["health"]["degraded"] == "LAND"
    # sentinel-free scenario: every sitl uav reports a digest
    assert all("health" in u for u in last["uavs"])


def test_pointmass_recording_keeps_the_exact_v03_key_set():
    from coopuavs.sim import scenario as scenario_mod
    from test_sitl_stage1 import SITL_SMALL_SCENARIO

    cfg = copy.deepcopy(SITL_SMALL_SCENARIO)
    del cfg["fidelity"]
    del cfg["sitl"]
    cfg["duration"] = 4.0
    sc = scenario_mod.build(cfg, seed=2)
    sc.run()
    for f in sc.recorder.frames:
        for u in f["uavs"]:
            assert "health" not in u and "att" not in u and "nav_q" not in u
