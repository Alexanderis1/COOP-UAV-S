"""CBIT fault dictionary: the fault matrix (PLAN_PROBLEM1 P5, PHY-UAV-033).

One row per fault code, table-driven on purpose — the P5 gate demands
every row test-covered, and the engine (engine.py) takes ALL behavior
(debounce, latch, inhibits, degraded response) from this table so a new
fault is one row + one monitor, never new engine logic.

Field semantics:

- ``bit`` — position in the u32 HEALTH fault word on the coop-link
  (P5-1c) and in the northbound UavHealth (P5-4). ICD-grade: pinned
  literally by test_coopfc_cbit.py, renumbering is a contract break.
- ``severity`` — WARN or CRIT. Dictionary invariant (tested): every
  CRIT fault inhibits both arming and fire.
- ``latching`` — a raised latching fault survives the condition
  clearing and needs an explicit ground/maintenance ``clear()`` (the
  P4-4 pad cycle clears the battery family on pack swap). Faults whose
  SOURCE already latches (BatteryMonitor, ``Ekf.diverged``, scheduler
  task faults) are non-latching mirrors of that latch — double-latching
  would deadlock recoveries the source machinery already handles (e.g.
  post-touchdown realignment clearing ``ekf.diverged``).
- ``debounce_s`` — the condition must hold continuously this long
  (monitor time) before the fault raises: detection latency is
  ``debounce_s`` at the monitor cadence, pinned per row in tests.
- ``mirror`` — the legacy ``Fcu._failsafes`` chain owns this fault's
  flight response (BATT_LOW/BATT_CRIT/LINK_MC_LOSS — P3 contract,
  first-reason latch); the row documents the response but
  ``CbitEngine.degraded_mode()`` excludes mirror rows so no-fault and
  legacy-failsafe runs stay bit-identical to P4.
- ``degraded_mode`` — CBIT-commanded response (P5-1c wiring):
  FAILSAFE_ATT (nav-loss class: attitude-only descent — position and
  velocity control are meaningless on a diverged estimate), LAND
  (controlled descent now), RTL, or none.

Notes per row group:

- IMU_STALE/GYRO_STUCK degraded LAND is best-effort and documented as
  such: with the rate-loop feedback dead the descent is not guaranteed
  stable — the rows pin detection + inhibits, not survival (quad
  physics; PX4 parity).
- GPS_LOSS commands nothing by itself: the EKF dead-reckons inherently,
  and DR_BUDGET_LOW (sigma over budget) owns the RTL decision.
- MOTOR_RESPONSE is one bit; the rotor index travels in the snapshot
  ``detail`` (user decision 2026-06-12: partial-degradation faults are
  the flyable, detectable class; full motor-out on a quad is
  unrecoverable and excluded from the no-wreck gate).
- BATT_SAG_ANOM/CELL_IMBALANCE monitors arrive with the P5-1f SOC work;
  the rows exist now so the wire layout never moves.
- LINK_C2_LOSS is monitored MC-side (the app owns C2 link knowledge);
  its never-self-authorize response is structural in FireControl.
"""

from __future__ import annotations

from typing import NamedTuple

WARN = "WARN"
CRIT = "CRIT"

ACT_NONE = ""
ACT_RTL = "RTL"
ACT_LAND = "LAND"
ACT_FAILSAFE_ATT = "FAILSAFE_ATT"

# CBIT response priority (P5-1c failsafe-chain slots, pinned in tests):
# nav-loss attitude fallback beats everything (LAND/RTL need a usable
# position estimate), then get-down-now, then come-home.
_ACT_RANK = {ACT_FAILSAFE_ATT: 3, ACT_LAND: 2, ACT_RTL: 1, ACT_NONE: 0}


class FaultSpec(NamedTuple):
    code: str
    bit: int
    severity: str
    latching: bool
    debounce_s: float
    inhibit_arming: bool
    inhibit_fire: bool
    degraded_mode: str
    mirror: bool = False


_ROWS = (
    #         code            bit  sev   latch  deb   arm    fire   degraded         mirror
    FaultSpec("IMU_STALE",      0, CRIT, False, 0.10, True,  True,  ACT_LAND),
    FaultSpec("IMU_RANGE",      1, WARN, False, 0.10, False, True,  ACT_NONE),
    FaultSpec("IMU_NOISE",      2, WARN, False, 0.50, False, True,  ACT_NONE),
    FaultSpec("GYRO_STUCK",     3, CRIT, True,  0.10, True,  True,  ACT_LAND),
    FaultSpec("GPS_LOSS",       4, WARN, False, 1.00, True,  True,  ACT_NONE),
    FaultSpec("GPS_DEGRADED",   5, WARN, False, 1.00, False, True,  ACT_NONE),
    FaultSpec("BARO_FAULT",     6, WARN, False, 0.50, False, False, ACT_NONE),
    FaultSpec("MAG_FAULT",      7, WARN, True,  1.00, False, False, ACT_NONE),
    FaultSpec("EKF_INNOV",      8, WARN, False, 1.00, False, True,  ACT_NONE),
    FaultSpec("EKF_DIVERGED",   9, CRIT, False, 0.00, True,  True,  ACT_FAILSAFE_ATT),
    FaultSpec("DR_BUDGET_LOW", 10, WARN, False, 0.50, False, True,  ACT_RTL),
    FaultSpec("MOTOR_RESPONSE", 11, CRIT, True, 0.50, True,  True,  ACT_LAND),
    # 3 s: a max-performance accel transient legitimately rails the
    # motors for ~1-2 s; only demand the airframe cannot leave is a fault.
    FaultSpec("SAT_PERSIST",   12, WARN, False, 3.00, False, False, ACT_NONE),
    FaultSpec("BATT_LOW",      13, WARN, False, 0.00, True,  False, ACT_RTL,  mirror=True),
    FaultSpec("BATT_CRIT",     14, CRIT, False, 0.00, True,  True,  ACT_LAND, mirror=True),
    FaultSpec("BATT_SAG_ANOM", 15, WARN, True,  2.00, False, False, ACT_NONE),
    FaultSpec("CELL_IMBALANCE", 16, WARN, True, 2.00, True,  False, ACT_RTL),
    FaultSpec("LINK_MC_LOSS",  17, WARN, False, 0.00, False, True,  ACT_RTL,  mirror=True),
    FaultSpec("LINK_C2_LOSS",  18, WARN, False, 0.00, False, True,  ACT_NONE),
    FaultSpec("SCHED_OVERRUN", 19, WARN, False, 0.00, True,  False, ACT_NONE),
    FaultSpec("PARAM_CRC",     20, CRIT, True,  0.00, True,  True,  ACT_NONE),
    FaultSpec("ALIGN_FAIL",    21, WARN, False, 0.00, True,  False, ACT_NONE),
    FaultSpec("WDOG_MISS",     22, WARN, True,  0.00, True,  False, ACT_NONE),
    # Added during P5-2a: a dead ESC-telemetry bus blinds the battery
    # monitor AND the SOC counter mid-flight (the plan's ~22-code list
    # had no row for it; the dropout injection matrix exposed the gap).
    FaultSpec("ESC_STALE",     23, WARN, False, 0.50, True,  False, ACT_NONE),
)

FAULTS: dict[str, FaultSpec] = {row.code: row for row in _ROWS}

if len(FAULTS) != len(_ROWS):                          # pragma: no cover
    raise AssertionError("duplicate fault codes in dictionary")


def act_rank(action: str) -> int:
    """Priority rank of a degraded-mode action (higher wins)."""
    return _ACT_RANK[action]
