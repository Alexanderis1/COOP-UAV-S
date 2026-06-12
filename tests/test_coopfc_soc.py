"""P5-1f(ii): FCU SOC estimator + battery-family CBIT monitors.

The estimator is OCV-seeded coulomb counting (coopfc/soc.py); the
calibration table is a flight-software COPY of the physics pack curve —
the first test pins the two equal so they cannot drift apart silently.
Observation-only step: the failsafe arbitration (voltage AND SOC) is
P5-1f(iii), so nothing here changes flight behavior yet.
"""

from __future__ import annotations

import numpy as np

from coopuavs.coopfc.soc import (
    OCV_SOC, OCV_V, SocEstimator, SocParams, ocv_v_cell,
    soc_from_rest_v_cell,
)
from coopuavs.physics import battery as phys_batt
from coopuavs.sil.bench import Bench

from test_coopfc_cbit_monitors import CbitHost, armed_host


# ---------------------------------------------------------------- tables

def test_calibration_table_matches_the_physics_pack():
    np.testing.assert_array_equal(np.asarray(OCV_SOC), phys_batt._OCV_SOC)
    np.testing.assert_array_equal(np.asarray(OCV_V), phys_batt._OCV_V)


def test_ocv_inversion_round_trip_and_clamps():
    for soc in (0.0, 0.07, 0.33, 0.5, 0.72, 0.93, 1.0):
        assert abs(soc_from_rest_v_cell(ocv_v_cell(soc)) - soc) < 1e-12
    assert soc_from_rest_v_cell(3.0) == 0.0
    assert soc_from_rest_v_cell(4.5) == 1.0
    assert ocv_v_cell(-0.1) == OCV_V[0]
    assert ocv_v_cell(1.1) == OCV_V[-1]


# ------------------------------------------------------------- estimator

def test_rest_seed_then_coulomb_count():
    est = SocEstimator(SocParams(capacity_ah=16.0, cells=12))
    v_rest = ocv_v_cell(0.7) * 12
    t = 0.0
    for _ in range(5):                          # rest window
        est.update(t, v_rest, 0.5)
        t += 0.1
    assert est.soc is not None
    assert abs(est.soc - 0.7) < 1e-9
    for _ in range(3600):                       # 360 s at 16 A
        est.update(t, 40.0, 16.0)               # sagged volts: irrelevant now
        t += 0.1
    assert abs(est.soc - 0.6) < 1e-6            # exactly C/10 consumed


def test_no_seed_under_load_and_reset():
    est = SocEstimator(SocParams())
    t = 0.0
    for _ in range(50):
        est.update(t, 40.0, 120.0)              # sagged + loaded
        t += 0.1
    assert est.soc is None                      # never guesses from sag
    for _ in range(5):
        est.update(t, ocv_v_cell(0.9) * 12, 0.0)
        t += 0.1
    assert abs(est.soc - 0.9) < 1e-9
    est.reset()
    assert est.soc is None


# ------------------------------------------------------------- monitors

def rested_armed_host() -> CbitHost:
    """Boot at true rest (esc_i 0): the SOC estimator seeds from OCV."""
    h = CbitHost()
    h.esc_i = 0.0
    h.boot_and_arm()
    assert h.fcu.soc_est.soc is not None
    return h


def test_cell_imbalance_raises_on_tap_spread():
    h = armed_host()
    cells = [4.0] * 12
    cells[5] = 3.85                             # 150 mV spread
    h.esc_cells = tuple(cells)
    h.run(3.0)                                  # 2 s dictionary debounce
    assert h.fcu.cbit.raised("CELL_IMBALANCE")
    assert "spread" in h.fcu.cbit.snapshot()["CELL_IMBALANCE"]["detail"]
    assert h.fcu.cbit.inhibit_arming            # WARN + arm inhibit + RTL class


def test_batt_sag_anomaly_against_soc_expectation():
    h = rested_armed_host()                     # SOC seeded ~0.92
    h.esc_i = 5.0                               # off the rest window
    h.run(1.0)
    h.v_cell = 3.60                             # sag 5 A cannot explain
    h.run(3.0)
    assert h.fcu.cbit.raised("BATT_SAG_ANOM")
    assert not h.fcu.cbit.raised("BATT_LOW")    # 3.6 > 3.5: voltage monitor quiet


def test_batt_reset_reseeds_soc_and_clears_pack_latches():
    h = CbitHost()
    h.esc_i = 0.0                               # resting on the stand
    h.run(2.6)
    cells = [4.0] * 12
    cells[2] = 3.80
    h.esc_cells = tuple(cells)
    h.run(3.0)
    assert h.fcu.cbit.raised("CELL_IMBALANCE")  # latched (pack suspect)
    h.esc_cells = None                          # pack swapped on the pad
    ok, why = h.fcu.cmd_batt_reset()
    assert ok, why
    assert h.fcu.soc_est.soc is None            # re-seeding from rest
    assert not h.fcu.cbit.raised("CELL_IMBALANCE")
    h.run(1.0)
    assert h.fcu.soc_est.soc is not None        # new pack seeded


# ----------------------------------------------- arbitration (P5-1f iii)

def test_arbitration_vetoes_loaded_sag_on_a_charged_pack():
    from coopuavs.coopfc.battery_monitor import LOW, NORMAL, BatteryMonitor
    sagged = 3.45 * 12
    m = BatteryMonitor()
    for k in range(30):                         # 3 s, way past debounce
        m.update(k * 0.1, sagged, soc=0.9, i_bus=150.0)
    assert m.state == NORMAL                    # I*R sag, not charge state

    m = BatteryMonitor()                        # same volts AT REST: real
    for k in range(30):
        m.update(k * 0.1, sagged, soc=0.9, i_bus=0.0)
    assert m.state == LOW

    m = BatteryMonitor()                        # loaded but discharged: real
    for k in range(30):
        m.update(k * 0.1, sagged, soc=0.4, i_bus=150.0)
    assert m.state == LOW


def test_soc_thresholds_latch_upward():
    from coopuavs.coopfc.battery_monitor import (
        CRITICAL, LOW, BatteryMonitor,
    )
    m = BatteryMonitor()
    good = 3.80 * 12                            # sag-free volts throughout
    assert m.update(0.0, good, soc=0.20, i_bus=50.0) == LOW
    assert m.update(0.1, good, soc=0.05, i_bus=50.0) == CRITICAL
    assert m.update(0.2, good, soc=0.95, i_bus=50.0) == CRITICAL  # latched


def test_no_soc_is_exactly_voltage_only():
    from coopuavs.coopfc.battery_monitor import LOW, BatteryMonitor
    m = BatteryMonitor()
    for k in range(30):
        m.update(k * 0.1, 3.45 * 12, soc=None, i_bus=400.0)
    assert m.state == LOW                       # the P4 behavior, untouched


def test_climb_sag_scenario_flies_through():
    # The P4-R residual this decision owns: a charged pack sagging under
    # full power must NOT trip the failsafe; the calibration explains
    # the sag (no SAG_ANOM either), and telemetry reports real SOC.
    h = rested_armed_host()                     # SOC seeded ~0.92
    h.run(1.0)
    h.v_cell = 3.45                             # deep sag...
    h.esc_i = 150.0                             # ...under load
    h.run(3.0)
    assert h.fcu.failsafe == ""                 # no BATT_LOW
    assert h.fcu.batt.state == "NORMAL"
    assert not h.fcu.cbit.raised("BATT_SAG_ANOM")   # I*(r0+r1) explains it
    assert h.fcu.battery_fraction() > 0.8       # telemetry = real SOC
    h.esc_i = 0.0                               # load gone, volts STILL low:
    h.run(2.0)                                  # that is a real dying pack
    # rest recal walks SOC down to the ~0.04 the rest volts imply, so
    # the latch escalates LOW -> CRITICAL; the first reason sticks.
    assert h.fcu.batt.state in ("LOW", "CRITICAL")
    assert h.fcu.failsafe in ("BATT_LOW", "BATT_CRIT")


# ------------------------------------------------------------ integration

def test_bench_soc_tracks_the_real_pack():
    b = Bench(seed=5)
    b.boot_and_arm()
    assert b.fcu.soc_est.soc is not None        # seeded on the stand
    assert abs(b.fcu.soc_est.soc - float(b.pt.battery.soc[0])) < 0.05
    b.run(20.0)                                 # hover burns real charge
    truth = float(b.pt.battery.soc[0])
    assert truth < 0.999                        # something was consumed
    assert abs(b.fcu.soc_est.soc - truth) < 0.02
    assert not b.fcu.cbit.raised("BATT_SAG_ANOM")   # healthy pack, no anomaly
    assert not b.fcu.cbit.raised("CELL_IMBALANCE")


def test_engine_configures_fcu_per_airframe_pack():
    from coopuavs.core.rng import RngRegistry
    from coopuavs.sil.fleet import SitlEngine

    eng = SitlEngine([("i1", (0.0, 0.0, 50.0)),
                      ("s1", (50.0, 0.0, 50.0), "sentinel_quad")],
                     RngRegistry(3))
    by_id = {uid: eng.fcus[i] for uid, i in eng.index.items()}
    assert by_id["i1"].params.get("fcu.batt_capacity_ah") == 16.0
    assert by_id["s1"].params.get("fcu.batt_capacity_ah") == 24.0
    # explicit scenario overlay wins over the airframe calibration
    eng2 = SitlEngine([("i1", (0.0, 0.0, 50.0))], RngRegistry(3),
                      fcu_overlay={"fcu.batt_capacity_ah": 99.0})
    assert eng2.fcus[0].params.get("fcu.batt_capacity_ah") == 99.0
