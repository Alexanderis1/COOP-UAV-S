"""P5-2a fault-injection seams (SIM-SIL-003): hw/link-level injection,
end-to-end CBIT detection, and the no-fault bit-identity contract.

Injection philosophy (pinned here): faults mask or transform the
EXISTING stochastic streams — device banks keep drawing for every
vehicle, faulted or not, so no fault ever consumes RNG and draw
histories never move. Denial is not a dead wire (frames flow with
FIX_NONE; the driver stays fresh while fusion starves); a dead wire is
``fault_sensor_dropout`` (the driver goes stale); a stuck sensor is
fresh frames with frozen values.
"""

from __future__ import annotations

import numpy as np

from coopuavs.coopfc.fcu import ARMED, LAND, RTL
from coopuavs.core.rng import RngRegistry
from coopuavs.sil.fleet import SitlEngine

DT = 0.05


def _engine(seed: int = 11) -> SitlEngine:
    return SitlEngine([("u1", (0.0, 0.0, 60.0))], RngRegistry(seed),
                      world_dt=DT)


def _run(eng: SitlEngine, t: float, t_span: float) -> float:
    for m in range(round(t_span / DT)):
        eng.run_macro_step(t + m * DT, DT)
    return t + round(t_span / DT) * DT


def _hover(seed: int = 11) -> tuple[SitlEngine, float]:
    """Booted, armed, holding position (the engine heartbeat
    placeholder keeps the unlinked FCU's link-loss clock fed)."""
    eng = _engine(seed)
    t = _run(eng, 0.0, 4.0)
    fcu = eng.fcus[0]
    assert fcu.pbit_ok, fcu.pbit_reasons
    ok, why = fcu.cmd_arm()
    assert ok, why
    # Ground datum below the vehicle: home defaults to the ARMING point
    # (z=60), where a LAND response would latch touchdown instantly
    # (the SynthHost convention).
    fcu.cmd_set_home((0.0, 0.0, 0.0))
    t = _run(eng, t, 2.0)
    assert fcu.state == ARMED and fcu.cbit.word() == 0
    return eng, t


# ------------------------------------------------------------ GPS family

def test_gps_denial_starves_fusion_not_the_driver():
    eng, t = _hover()
    fcu = eng.fcus[0]
    eng.fault_gps_denied("u1")
    t = _run(eng, t, 3.0)
    assert fcu.cbit.raised("GPS_LOSS")
    assert not fcu.gps_drv.stale          # frames flow: denial != dead wire
    eng.fault_gps_denied("u1", on=False)
    t = _run(eng, t, 2.0)
    assert not fcu.cbit.raised("GPS_LOSS")


def test_gps_degraded_scale_trips_the_reject_window():
    eng, t = _hover()
    fcu = eng.fcus[0]
    eng.fault_gps_degraded("u1", 15.0)
    t = _run(eng, t, 5.0)
    assert fcu.cbit.raised("GPS_DEGRADED")
    eng.fault_gps_degraded("u1", 1.0)
    t = _run(eng, t, 4.0)
    assert not fcu.cbit.raised("GPS_DEGRADED")


# -------------------------------------------------------------- dropouts

def test_sensor_dropouts_stale_their_drivers():
    cases = (("imu", "IMU_STALE"), ("baro", "BARO_FAULT"),
             ("mag", "MAG_FAULT"), ("gps", "GPS_LOSS"),
             ("esc", "ESC_STALE"))
    for sensor, code in cases:
        eng, t = _hover()
        eng.fault_sensor_dropout("u1", sensor)
        _run(eng, t, 4.0)
        assert eng.fcus[0].cbit.raised(code), (sensor, code)


def test_gyro_stuck_injection_lands_the_vehicle():
    eng, t = _hover()
    fcu = eng.fcus[0]
    eng.fault_gyro_stuck("u1")
    t = _run(eng, t, 2.0)
    assert fcu.cbit.raised("GYRO_STUCK")
    assert not fcu.cbit.raised("IMU_STALE")    # fresh frames, frozen values
    assert fcu.mode == LAND                    # best-effort get-down
    assert fcu.failsafe == "GYRO_STUCK"


def test_imu_noise_inflation():
    eng, t = _hover()
    eng.fault_imu_noise("u1", 25.0)
    t = _run(eng, t, 2.0)
    assert eng.fcus[0].cbit.raised("IMU_NOISE")
    eng.fault_imu_noise("u1", 1.0)
    t = _run(eng, t, 2.0)
    assert not eng.fcus[0].cbit.raised("IMU_NOISE")


# ---------------------------------------------------------------- motor

def test_motor_esc_gain_fault_detected_and_landed():
    eng, t = _hover()
    fcu = eng.fcus[0]
    eng.fault_motor("u1", 1, 0.775)            # the flyable 40% class
    t = _run(eng, t, 4.0)
    assert fcu.cbit.raised("MOTOR_RESPONSE")
    assert fcu.cbit.snapshot()["MOTOR_RESPONSE"]["detail"] == "rotor 1"
    assert fcu.mode == LAND
    assert fcu.failsafe == "MOTOR_RESPONSE"
    # flyable: the descent is controlled, not a tumble
    assert abs(float(eng.state[0, 10])) < 2.0  # roll rate bounded


# ------------------------------------------------------------------ link

def test_mc_link_jam_starves_the_fcu_home():
    import sys
    sys.path.insert(0, "tests")
    from test_sitl_stage2 import _hosted_engine
    from test_sitl_stage2 import _run as _run2

    eng, mcu = _hosted_engine()
    t = _run2(eng, 0.0, 6.0)
    fcu = eng.fcus[0]
    assert fcu.state == ARMED and fcu.failsafe == ""
    eng.fault_mc_link_jam("u1")
    t = _run2(eng, t, 3.0)
    assert fcu.cbit.raised("LINK_MC_LOSS")
    # P3/P4 first-reason contract: wire silence latches OFFBOARD_TIMEOUT
    # (0.5 s) before LINK_LOSS escalates the MODE to RTL at 2 s.
    assert fcu.failsafe == "OFFBOARD_TIMEOUT"
    assert fcu.mode in (RTL, LAND)
    eng.fault_mc_link_jam("u1", on=False)
    t = _run2(eng, t, 1.0)
    assert fcu.failsafe == "OFFBOARD_TIMEOUT"  # latched for the flight


# ------------------------------------------------- P5-2b faults: schedule

def test_schedule_window_applies_and_clears():
    eng, t = _hover()
    eng.schedule_fault(t + 1.0, "u1", "sensor_dropout", sensor="baro",
                       until=t + 6.0)
    t = _run(eng, t, 5.0)                      # inside the window (the
    assert eng.fcus[0].cbit.raised("BARO_FAULT")   # monitor is a 1 Hz task)
    t = _run(eng, t, 4.0)                      # window closed + recovery
    assert not eng.fcus[0].cbit.raised("BARO_FAULT")


def test_schedule_validation_is_loud():
    eng = _engine()
    for bad in (
        dict(t=1.0, uid="u1", kind="no_such_fault"),
        dict(t=1.0, uid="ghost", kind="gps_denial"),
        dict(t=1.0, uid="u1", kind="gps_degraded"),           # missing scale
        dict(t=1.0, uid="u1", kind="gps_denial", scale=2.0),  # extra param
        dict(t=1.0, uid="u1", kind="sensor_dropout", sensor="lidar"),
        dict(t=1.0, uid="u1", kind="motor", rotor=9, scale=0.5),
        dict(t=5.0, uid="u1", kind="gps_denial", until=4.0),
    ):
        try:
            eng.schedule_fault(bad.pop("t"), bad.pop("uid"), bad.pop("kind"),
                               until=bad.pop("until", None), **bad)
        except ValueError:
            continue
        raise AssertionError(f"accepted bad schedule {bad}")


def test_scenario_faults_block_parsing_is_loud():
    from coopuavs.sim.scenario import _parse_faults

    ids = {"u1", "u2"}
    ok = _parse_faults([{"t": 4.0, "uav": "u1", "kind": "gps_denial",
                         "until": 9.0}], ids)
    assert ok[0]["kind"] == "gps_denial"
    for bad in (
        [{"uav": "u1", "kind": "gps_denial"}],                 # no t
        [{"t": 1.0, "uav": "u3", "kind": "gps_denial"}],       # unknown uav
        [{"t": 1.0, "uav": "u1", "kind": "warp_core"}],        # unknown kind
        [{"t": 1.0, "uav": "u1", "kind": "gps_denial",
          "typo": 1}],                                         # unknown key
        [{"t": 1.0, "uav": "u1", "kind": "motor", "rotor": 1}],  # missing scale
    ):
        try:
            _parse_faults(bad, ids)
        except ValueError:
            continue
        raise AssertionError(f"accepted bad faults block {bad}")


def test_faults_block_requires_sitl_fidelity():
    import copy
    import sys
    sys.path.insert(0, "tests")
    from coopuavs.sim import scenario as scenario_mod
    from test_sitl_stage1 import SITL_SMALL_SCENARIO

    cfg = copy.deepcopy(SITL_SMALL_SCENARIO)
    del cfg["fidelity"]
    del cfg["sitl"]
    cfg["faults"] = [{"t": 1.0, "uav": "u1", "kind": "gps_denial"}]
    try:
        scenario_mod.build(cfg)
    except ValueError as e:
        assert "fidelity.fleet=sitl" in str(e)
    else:
        raise AssertionError("pointmass scenario accepted a faults block")


def test_scenario_faults_block_end_to_end():
    import copy
    import sys
    sys.path.insert(0, "tests")
    from coopuavs.sim import scenario as scenario_mod
    from test_sitl_stage1 import SITL_SMALL_SCENARIO

    cfg = copy.deepcopy(SITL_SMALL_SCENARIO)
    cfg["duration"] = 12.0
    cfg["faults"] = [{"t": 6.0, "uav": "u1", "kind": "gps_denial"}]
    sc = scenario_mod.build(cfg, seed=2)
    sc.run()
    eng = sc.world.micro
    assert eng.fcus[eng.index["u1"]].cbit.raised("GPS_LOSS")
    assert not eng.fcus[eng.index["u2"]].cbit.raised("GPS_LOSS")


# ---------------------------------------------------------- determinism

def test_explicit_noop_faults_are_bitwise_invisible():
    a = _engine(seed=23)
    b = _engine(seed=23)
    b.fault_gps_degraded("u1", 1.0)            # explicit no-ops: the lazy
    b.fault_imu_noise("u1", 1.0)               # arrays exist, values 1.0
    b.fault_motor("u1", 0, 1.0)
    t = 0.0
    for m in range(round(4.0 / DT)):
        a.run_macro_step(t, DT)
        b.run_macro_step(t, DT)
        t += DT
    np.testing.assert_array_equal(a.state, b.state)
    assert a.fcus[0].nav == b.fcus[0].nav


def test_faulted_run_twice_is_bitwise():
    def run(seed):
        eng = _engine(seed)
        t = _run(eng, 0.0, 2.0)
        eng.fault_gps_denied("u1")
        eng.fault_motor("u1", 2, 0.8)
        t = _run(eng, t, 2.0)
        eng.fault_gps_denied("u1", on=False)
        _run(eng, t, 1.0)
        return eng.state.copy()

    np.testing.assert_array_equal(run(31), run(31))
