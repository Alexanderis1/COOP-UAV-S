"""P5-3 degraded-mode scenarios (SIM-SIL-003 -> PHY-UAV-011/033).

Three claims, each end-to-end through the scenario/engine stack:

1. motor degradation mid-raid -> the vehicle detects, LANDs under
   control, stays grounded (arming inhibited by the latched fault), and
   the mission's no-CRITICAL-wreck invariant holds;
2. GNSS denial -> honest dead-reckoning (claimed sigma covers the true
   error), the DR budget commands RTB long before the estimate is
   operationally blind, and the vehicle physically comes home on the
   drifting estimate — sustained for the literal 5-minute PHY-UAV-011
   window in the @slow variant;
3. the fire interlock holds under EVERY injected fault kind: no token,
   no release, no matter what is failing (never-self-authorize,
   PHY-UAV-033; the token-in-hand veto chain is test_coopfc_cbit_actions).
"""

from __future__ import annotations

import copy
import sys

import numpy as np
import pytest

sys.path.insert(0, "tests")

from coopuavs.coopfc.fcu import ARMED, LAND, RTL, STANDBY
from test_sitl_faults import _hover, _run

DT = 0.05


# ------------------------------------------------- motor-out, no wreck

def test_motor_degraded_scenario_lands_no_critical_wreck():
    from coopuavs.sim import scenario as scenario_mod
    from test_sitl_stage1 import SITL_SMALL_SCENARIO

    cfg = copy.deepcopy(SITL_SMALL_SCENARIO)
    cfg["faults"] = [{"t": 10.0, "uav": "u1", "kind": "motor",
                      "rotor": 1, "scale": 0.775}]
    sc = scenario_mod.build(cfg, seed=2)
    summary = sc.run()
    eng = sc.world.micro
    fcu = eng.fcus[eng.index["u1"]]
    assert fcu.cbit.raised("MOTOR_RESPONSE")          # detected...
    assert fcu.failsafe == "MOTOR_RESPONSE"           # ...owned the response
    # controlled descent completed: grounded via the touchdown latch,
    # and the latched fault holds the vehicle down (arming inhibited)
    assert fcu.state == STANDBY and fcu.touchdown
    assert float(eng.state[eng.index["u1"], 2]) < 5.0
    ok, why = fcu.cmd_arm()
    assert not ok and "MOTOR_RESPONSE" in why
    # the raid invariant survives the casualty
    assert summary["wrecks_by_zone"].get("CRITICAL", 0) == 0


# ---------------------------------------------------- GNSS denial / DR

def _denied_rtb(t_total: float) -> None:
    """Shared body: deny GPS on a vehicle 300 m from home; the DR budget
    must order RTB and the truth must come home on the drifting estimate,
    with the claimed sigma honestly covering the true error throughout."""
    eng, t = _hover()
    fcu = eng.fcus[0]
    fcu.cmd_set_home((-300.0, 0.0, 0.0))
    eng.fault_gps_denied("u1")
    t_end = t + t_total
    worst_ratio = 0.0
    seen_rtl = False
    while t < t_end:
        t = _run(eng, t, 1.0)
        nav = fcu.nav
        if fcu.state == ARMED and nav is not None:
            err = float(np.linalg.norm(
                np.asarray(nav.pos) - eng.state[0, 0:3]))
            worst_ratio = max(worst_ratio,
                              err / max(nav.sigma_pos_h, 1.0))
            if fcu.cbit.raised("DR_BUDGET_LOW"):
                # past the budget the FCU may only be coming home
                assert fcu.mode in (RTL, LAND), fcu.mode
                seen_rtl = True
    assert seen_rtl, "DR budget never ordered RTB"
    assert fcu.failsafe == "DR_BUDGET_LOW"
    assert worst_ratio < 4.0, (
        f"DR claim dishonest: |est-truth| reached {worst_ratio:.1f}x "
        "the claimed sigma")
    # truth physically came home (the drifting estimate is the only map:
    # nav-error-class offset from the pad is the honest outcome)
    home_err = float(np.linalg.norm(eng.state[0, 0:2] - (-300.0, 0.0)))
    assert home_err < 60.0, f"truth ended {home_err:.0f} m from home"
    assert eng.state[0, 2] < 5.0                       # landed


def test_gps_denied_dr_budget_brings_it_home():
    _denied_rtb(120.0)


@pytest.mark.slow
def test_gps_denied_five_minutes_sustained():
    # PHY-UAV-011's literal window: five denied minutes end grounded at
    # home, never airborne past the DR budget.
    _denied_rtb(300.0)


# ------------------------------------------- interlock under every fault

_FAULT_MATRIX = [
    ("gps_denial", {}),
    ("gps_degraded", {"scale": 15.0}),
    ("sensor_dropout", {"sensor": "mag"}),
    ("imu_noise", {"scale": 25.0}),
    ("gyro_stuck", {}),
    ("motor", {"rotor": 1, "scale": 0.775}),
    ("mc_link_jam", {}),
    ("cell_imbalance", {"delta": 0.3}),
    ("batt_r0_scale", {"scale": 3.0}),
]


@pytest.mark.parametrize("kind,params",
                         _FAULT_MATRIX, ids=[k for k, _ in _FAULT_MATRIX])
def test_interlock_holds_under_fault(kind, params):
    """A shooter mid-engagement (in envelope, requesting) under each
    fault kind, with NO clearance ever delivered: zero releases. The
    safety chain is in the messages, not in the vehicle's health."""
    from coopuavs.core.messages import Header, Track, TrackArray
    from test_sitl_stage2 import _hosted_engine, _task
    from test_sitl_stage2 import _run as _run2

    eng, mcu = _hosted_engine()
    t = _run2(eng, 0.0, 6.0)
    assert eng.fcus[0].state == ARMED
    own = eng.state[0, 0:3]
    mcu.ports.box("tasks").post([_task(track_id=1, task_id=1)])
    mcu.ports.box("tracks").post(TrackArray(header=Header(stamp=t), tracks=[
        Track(header=Header(stamp=t), track_id=1,
              position=own + np.array([60.0, 0.0, 0.0]),
              velocity=np.array([-5.0, 0.0, 0.0]))]))
    t = _run2(eng, t, 1.0)
    eng.schedule_fault(t, "u1", kind, **params)
    _run2(eng, t, 5.0)
    assert mcu.ports.box("fire").drain() == [], kind   # never self-authorized
