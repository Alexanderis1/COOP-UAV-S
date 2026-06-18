"""P5-1a CBIT dictionary + engine core: table-driven behavior per fault.

The dictionary is the fault matrix the P5 gate demands 100% coverage of;
these tests drive EVERY row through raise/debounce/latch/clear and pin
the wire bit layout (the HEALTH message fault word, P5-1c) so a refactor
cannot silently renumber what the MC and recorder already decode.

Engine semantics under test:
- ``report(code, active, now)`` with debounce on raise (condition must
  hold continuously for ``debounce_s`` of monitor time), immediate clear
  for non-latching faults, explicit ``clear()`` (ground op) for latched;
- inhibit aggregation: any raised fault with the flag inhibits;
- ``degraded_mode()``: highest-priority action among raised NON-mirror
  faults (FAILSAFE_ATT > LAND > RTL); mirror rows (BATT_LOW/BATT_CRIT/
  LINK_MC_LOSS) document their response but the legacy ``_failsafes``
  chain keeps command authority — no-fault runs stay bit-identical.
"""

import pytest

from coopuavs.coopfc.cbit import (
    ACT_FAILSAFE_ATT,
    ACT_LAND,
    ACT_NONE,
    ACT_RTL,
    CRIT,
    FAULTS,
    WARN,
    CbitEngine,
)

PERIOD = 0.02   # the 50 Hz fast-monitor cadence the FCU will use

# Golden wire-bit map: the HEALTH fault word is decoded by the MC app and
# recorded northbound — bit positions are an ICD-grade contract. Literal
# on purpose: a dictionary edit must touch this test to renumber.
EXPECTED_BITS = {
    "IMU_STALE": 0,
    "IMU_RANGE": 1,
    "IMU_NOISE": 2,
    "GYRO_STUCK": 3,
    "GPS_LOSS": 4,
    "GPS_DEGRADED": 5,
    "BARO_FAULT": 6,
    "MAG_FAULT": 7,
    "EKF_INNOV": 8,
    "EKF_DIVERGED": 9,
    "DR_BUDGET_LOW": 10,
    "MOTOR_RESPONSE": 11,
    "SAT_PERSIST": 12,
    "BATT_LOW": 13,
    "BATT_CRIT": 14,
    "BATT_SAG_ANOM": 15,
    "CELL_IMBALANCE": 16,
    "LINK_MC_LOSS": 17,
    "LINK_C2_LOSS": 18,
    "SCHED_OVERRUN": 19,
    "PARAM_CRC": 20,
    "ALIGN_FAIL": 21,
    "WDOG_MISS": 22,
    "ESC_STALE": 23,     # added P5-2a: dead telemetry bus blinds the
                         # battery monitor + SOC counter
}


def raise_fault(eng: CbitEngine, code: str, t0: float = 0.0,
                detail: str = "") -> float:
    """Drive one fault through its debounce at the 50 Hz cadence;
    returns the time of the report that raised it."""
    spec = FAULTS[code]
    t = t0
    eng.report(code, True, t, detail=detail)
    while not eng.raised(code):
        t += PERIOD
        eng.report(code, True, t, detail=detail)
        assert t - t0 <= spec.debounce_s + PERIOD + 1e-9, (
            f"{code} not raised within debounce window")
    return t


# ------------------------------------------------------------- dictionary

def test_dictionary_bits_pinned():
    assert {c: s.bit for c, s in FAULTS.items()} == EXPECTED_BITS


def test_dictionary_rows_well_formed():
    bits = [s.bit for s in FAULTS.values()]
    assert sorted(bits) == list(range(len(FAULTS)))      # unique, dense
    assert all(0 <= b < 32 for b in bits)                # u32 fault word
    for code, s in FAULTS.items():
        assert s.code == code
        assert s.severity in (WARN, CRIT)
        assert s.degraded_mode in (ACT_NONE, ACT_RTL, ACT_LAND,
                                   ACT_FAILSAFE_ATT)
        assert s.debounce_s >= 0.0


def test_dictionary_crit_always_inhibits():
    # A CRIT fault that still allowed arming or release would be a
    # self-contradiction in the safety table.
    for code, s in FAULTS.items():
        if s.severity == CRIT:
            assert s.inhibit_arming and s.inhibit_fire, code


def test_dictionary_mirror_rows():
    # Exactly the rows whose degraded response the legacy _failsafes
    # chain already owns (bit-identity of no-fault runs depends on CBIT
    # never double-commanding these).
    mirrors = {c for c, s in FAULTS.items() if s.mirror}
    assert mirrors == {"BATT_LOW", "BATT_CRIT", "LINK_MC_LOSS"}
    for code in mirrors:
        assert FAULTS[code].degraded_mode != ACT_NONE   # documented response


# ------------------------------------------------------------ raise/clear

def test_zero_debounce_raises_on_first_report():
    eng = CbitEngine()
    eng.report("EKF_DIVERGED", True, 1.0)
    assert eng.raised("EKF_DIVERGED")


def test_debounce_detection_latency():
    # debounce_s of continuous truth at the 50 Hz cadence: raised at the
    # first report with now - since >= debounce_s, not one tick earlier.
    eng = CbitEngine()
    spec = FAULTS["MOTOR_RESPONSE"]
    n_hold = round(spec.debounce_s / PERIOD)
    for k in range(n_hold):
        eng.report("MOTOR_RESPONSE", True, k * PERIOD)
        assert not eng.raised("MOTOR_RESPONSE"), f"early raise at {k}"
    eng.report("MOTOR_RESPONSE", True, n_hold * PERIOD)
    assert eng.raised("MOTOR_RESPONSE")


def test_flicker_resets_debounce():
    eng = CbitEngine()
    spec = FAULTS["GPS_LOSS"]
    t = 0.0
    n_almost = round(spec.debounce_s / PERIOD) - 1
    for k in range(n_almost):
        eng.report("GPS_LOSS", True, t)
        t += PERIOD
    eng.report("GPS_LOSS", False, t)     # one healthy sample
    t += PERIOD
    for k in range(n_almost):
        eng.report("GPS_LOSS", True, t)
        t += PERIOD
        assert not eng.raised("GPS_LOSS")


def test_nonlatching_clears_with_condition():
    eng = CbitEngine()
    t = raise_fault(eng, "GPS_LOSS")
    eng.report("GPS_LOSS", False, t + PERIOD)
    assert not eng.raised("GPS_LOSS")
    assert not eng.inhibit_fire


def test_latching_persists_until_ground_clear():
    eng = CbitEngine()
    t = raise_fault(eng, "MOTOR_RESPONSE", detail="rotor 2")
    eng.report("MOTOR_RESPONSE", False, t + PERIOD)
    assert eng.raised("MOTOR_RESPONSE")          # condition gone, latch holds
    eng.clear("MOTOR_RESPONSE")                  # maintenance/ground op
    assert not eng.raised("MOTOR_RESPONSE")
    eng.clear("MOTOR_RESPONSE")                  # idempotent on clear faults


def test_unknown_code_is_loud():
    eng = CbitEngine()
    with pytest.raises(KeyError):
        eng.report("NO_SUCH_FAULT", True, 0.0)
    with pytest.raises(KeyError):
        eng.clear("NO_SUCH_FAULT")


# ----------------------------------------------------------- aggregation

def test_inhibit_aggregation():
    eng = CbitEngine()
    assert not eng.inhibit_arming and not eng.inhibit_fire
    t = raise_fault(eng, "IMU_RANGE")            # WARN: fire only
    assert eng.inhibit_fire and not eng.inhibit_arming
    raise_fault(eng, "WDOG_MISS", t0=t)          # WARN: arming only
    assert eng.inhibit_fire and eng.inhibit_arming
    eng.report("IMU_RANGE", False, t + 10.0)
    assert not eng.inhibit_fire                  # WDOG_MISS doesn't inhibit fire
    assert eng.inhibit_arming                    # latched


def test_word_one_bit_per_fault():
    for code, spec in FAULTS.items():
        eng = CbitEngine()
        raise_fault(eng, code)
        assert eng.word() == 1 << spec.bit, code


def test_word_accumulates_and_faults_in_bit_order():
    eng = CbitEngine()
    eng.report("EKF_DIVERGED", True, 0.0)
    eng.report("PARAM_CRC", True, 0.0)
    eng.report("SCHED_OVERRUN", True, 0.0)
    assert eng.word() == (1 << 9) | (1 << 19) | (1 << 20)
    assert eng.faults() == ["EKF_DIVERGED", "SCHED_OVERRUN", "PARAM_CRC"]


def test_degraded_mode_priority():
    eng = CbitEngine()
    assert eng.degraded_mode() == ACT_NONE
    raise_fault(eng, "DR_BUDGET_LOW")            # RTL class
    assert eng.degraded_mode() == ACT_RTL
    raise_fault(eng, "MOTOR_RESPONSE")           # LAND class outranks RTL
    assert eng.degraded_mode() == ACT_LAND
    eng.report("EKF_DIVERGED", True, 100.0)      # nav-loss outranks all
    assert eng.degraded_mode() == ACT_FAILSAFE_ATT


def test_mirror_faults_never_command():
    # The legacy failsafe chain owns these responses; CBIT reporting them
    # must not add a second commander (bit-identity of P3/P4 behavior).
    eng = CbitEngine()
    eng.report("BATT_CRIT", True, 0.0)
    eng.report("LINK_MC_LOSS", True, 0.0)
    assert eng.raised("BATT_CRIT") and eng.raised("LINK_MC_LOSS")
    assert eng.degraded_mode() == ACT_NONE
    assert eng.inhibit_fire                      # inhibits still aggregate


# -------------------------------------------------------------- snapshot

def test_snapshot_since_and_detail():
    eng = CbitEngine()
    raise_fault(eng, "MOTOR_RESPONSE", t0=2.0, detail="rotor 2")
    snap = eng.snapshot()
    assert set(snap) == {"MOTOR_RESPONSE"}
    entry = snap["MOTOR_RESPONSE"]
    assert entry["since"] == pytest.approx(2.0)  # condition onset, not raise
    assert entry["detail"] == "rotor 2"
    assert entry["latched"] is True


# ------------------------------------------- table-driven full matrix row

@pytest.mark.parametrize("code", sorted(FAULTS))
def test_matrix_row_behavior(code):
    """Every dictionary row: detection latency = debounce at the 50 Hz
    cadence, inhibit flags exactly per spec, latch semantics per spec."""
    spec = FAULTS[code]
    eng = CbitEngine()
    t_raise = raise_fault(eng, code)
    assert t_raise == pytest.approx(
        round(spec.debounce_s / PERIOD) * PERIOD)
    assert eng.inhibit_arming == spec.inhibit_arming
    assert eng.inhibit_fire == spec.inhibit_fire
    if spec.mirror:
        assert eng.degraded_mode() == ACT_NONE
    else:
        assert eng.degraded_mode() == spec.degraded_mode
    eng.report(code, False, t_raise + PERIOD)
    assert eng.raised(code) == spec.latching
    eng.clear(code)
    assert not eng.raised(code)
    assert eng.word() == 0


# ------------------------------------------- debounce continuity (review)

def test_parked_debounce_does_not_complete_across_a_reporting_gap():
    """A monitor that went silent (disarm, dropout, realignment) broke
    the continuous-hold claim: one blip after the gap must restart the
    debounce, not complete it instantly."""
    from coopuavs.coopfc.cbit.engine import REPORT_GAP_S
    eng = CbitEngine()
    eng.report("MOTOR_RESPONSE", True, 0.0)        # pending (deb 0.5 s)
    eng.report("MOTOR_RESPONSE", True, 0.2)        # still pending
    t = 0.2 + REPORT_GAP_S + 1.0                   # monitor silent
    eng.report("MOTOR_RESPONSE", True, t)          # restart, NOT raise
    assert not eng.raised("MOTOR_RESPONSE")
    # held continuously after the gap -> raises on its own debounce
    eng.report("MOTOR_RESPONSE", True, t + 0.5)
    assert eng.raised("MOTOR_RESPONSE")


def test_slow_cadence_reports_are_gap_free():
    # 1 Hz monitor cadence sits under REPORT_GAP_S: normal slow-task
    # debounces are unaffected by the continuity rule.
    eng = CbitEngine()
    eng.report("CELL_IMBALANCE", True, 0.0)        # deb 2.0 s
    eng.report("CELL_IMBALANCE", True, 1.0)
    eng.report("CELL_IMBALANCE", True, 2.0)
    assert eng.raised("CELL_IMBALANCE")


def test_snapshot_since_survives_condition_flicker_on_latched_fault():
    eng = CbitEngine()
    eng.report("GYRO_STUCK", True, 1.0)
    eng.report("GYRO_STUCK", True, 1.2)            # raised (deb 0.1, latch)
    assert eng.raised("GYRO_STUCK")
    eng.report("GYRO_STUCK", False, 1.4)           # condition flickers off
    assert eng.raised("GYRO_STUCK")                # latched holds
    assert eng.snapshot()["GYRO_STUCK"]["since"] == 1.0   # onset kept
