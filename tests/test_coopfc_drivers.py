"""P3-3: coopfc/hal + coopfc/drivers — HAL seam, staleness, unit round-trips.

The HAL is the MCU-portable boundary: the host (VirtualMCU, P4; bench,
P3-8) writes raw frames into named seq-stamped ports, drivers poll them
at their own task rate, convert device units to SI, publish typed
messages, and count staleness deterministically (no new seq = one stale
tick). Drivers import nothing outside coopfc — these tests feed
synthetic frames and cross-check conversions against the hw/ models
test-side (baro ISA inverse must match hw.baro.altitude_from_pressure
bit-near; esc rpm->rad/s must invert the encoding).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from coopuavs.coopfc.core.topics import TopicStore
from coopuavs.coopfc.drivers.baro import BaroDriver, pressure_to_altitude
from coopuavs.coopfc.drivers.esc import EscDriver
from coopuavs.coopfc.drivers.gps import GpsDriver
from coopuavs.coopfc.drivers.imu import ImuDriver
from coopuavs.coopfc.drivers.mag import MagDriver
from coopuavs.coopfc.hal import HalIO
from coopuavs.hw import baro as hw_baro
from coopuavs.physics import atmosphere

# ------------------------------------------------------------------------ hal


def test_hal_port_seq_and_latest_value():
    hal = HalIO()
    port = hal.port("imu")
    assert port.read() == (0, None)
    port.write("frame1")
    port.write("frame2")
    assert port.read() == (2, "frame2")
    assert hal.port("imu") is port  # same name -> same port


def test_hal_ports_are_independent():
    hal = HalIO()
    hal.port("imu").write("x")
    assert hal.port("gps").read() == (0, None)


# ------------------------------------------------------------------------ imu


def make_imu(stale_after=2):
    hal, topics = HalIO(), TopicStore()
    drv = ImuDriver(hal.port("imu"), topics, stale_after=stale_after)
    sub = topics.subscribe("imu_raw")
    return hal, drv, sub


def test_imu_driver_publishes_si_passthrough():
    hal, drv, sub = make_imu()
    hal.port("imu").write(((0.1, -0.2, 0.3), (0.0, 0.1, 9.7)))
    drv.tick(0.0025)
    msg = sub.read()
    assert msg.stamp == 0.0025
    assert msg.gyro == (0.1, -0.2, 0.3)
    assert msg.accel == (0.0, 0.1, 9.7)


def test_imu_staleness_counts_and_recovers():
    hal, drv, sub = make_imu(stale_after=2)
    hal.port("imu").write(((0.0, 0.0, 0.0), (0.0, 0.0, 9.81)))
    drv.tick(0.0)
    assert sub.updated and sub.read() is not None
    assert drv.stale is False

    drv.tick(0.0025)  # no new frame
    assert sub.updated is False
    assert drv.stale_ticks == 1 and drv.stale is False
    drv.tick(0.005)
    assert drv.stale_ticks == 2 and drv.stale is True

    hal.port("imu").write(((1.0, 0.0, 0.0), (0.0, 0.0, 9.81)))
    drv.tick(0.0075)
    assert drv.stale_ticks == 0 and drv.stale is False
    assert sub.read().gyro == (1.0, 0.0, 0.0)


# ------------------------------------------------------------------------ gps


def test_gps_driver_carries_measurement_stamp():
    hal, topics = HalIO(), TopicStore()
    drv = GpsDriver(hal.port("gps"), topics)
    sub = topics.subscribe("gps_fix")
    # Frame: (pos, vel, fix_type, fix_stamp) — measured at 0.0, delivered
    # 120 ms later (the hw latency contract); EKF OOSM keys on fix_stamp.
    hal.port("gps").write(((10.0, 20.0, 30.0), (1.0, 0.0, -0.5), 3, 0.0))
    drv.tick(0.12)
    msg = sub.read()
    assert msg.stamp == 0.12
    assert msg.fix_stamp == 0.0
    assert msg.pos == (10.0, 20.0, 30.0)
    assert msg.vel == (1.0, 0.0, -0.5)
    assert msg.fix_type == 3


def test_gps_driver_stale_until_first_fix():
    hal, topics = HalIO(), TopicStore()
    drv = GpsDriver(hal.port("gps"), topics, stale_after=3)
    sub = topics.subscribe("gps_fix")
    for i in range(3):
        drv.tick(i * 0.1)
    assert sub.updated is False
    assert drv.stale is True


# ----------------------------------------------------------------------- baro


def test_pressure_to_altitude_matches_hw_inverse():
    # coopfc owns its ISA constants (import fence); they must agree with
    # the hw/ inverse to float precision across the flight envelope.
    alts = np.linspace(0.0, 10000.0, 101)
    p = atmosphere.pressure(alts)
    ours = np.array([pressure_to_altitude(float(pi)) for pi in p])
    ref = hw_baro.altitude_from_pressure(p)
    assert np.max(np.abs(ours - ref)) < 1e-9


def test_baro_driver_round_trip():
    hal, topics = HalIO(), TopicStore()
    drv = BaroDriver(hal.port("baro"), topics)
    sub = topics.subscribe("baro_alt")
    p = float(atmosphere.pressure(123.4))
    hal.port("baro").write(p)
    drv.tick(0.02)
    msg = sub.read()
    assert msg.pressure_pa == p
    assert msg.alt_m == pytest.approx(123.4, abs=1e-9)


def test_baro_driver_rejects_garbage_without_crashing():
    hal, topics = HalIO(), TopicStore()
    drv = BaroDriver(hal.port("baro"), topics)
    sub = topics.subscribe("baro_alt")
    for bad in (0.0, -5.0, math.nan, math.inf):
        hal.port("baro").write(bad)
        drv.tick(0.02)
    assert sub.updated is False  # nothing published
    assert drv.bad_frames == 4


# ------------------------------------------------------------------------ mag


def test_mag_driver_passthrough_ut():
    hal, topics = HalIO(), TopicStore()
    drv = MagDriver(hal.port("mag"), topics)
    sub = topics.subscribe("mag_body")
    hal.port("mag").write((21.0, 1.5, -43.0))
    drv.tick(0.02)
    assert sub.read().field_ut == (21.0, 1.5, -43.0)


# ------------------------------------------------------------------------ esc


def test_esc_driver_rpm_to_rad_s_round_trip():
    hal, topics = HalIO(), TopicStore()
    drv = EscDriver(hal.port("esc"), topics)
    sub = topics.subscribe("esc_status")
    omega = (800.0, 810.0, 790.0, 805.0)  # rad/s, mechanical shaft
    rpm = tuple(w * 60.0 / math.tau for w in omega)  # hw encoding
    hal.port("esc").write((rpm, 44.4, 95.0))
    drv.tick(0.1)
    msg = sub.read()
    assert msg.omega == pytest.approx(omega, abs=1e-9)
    assert msg.rpm == rpm
    assert msg.v_bus == 44.4
    assert msg.i_bus == 95.0


# ------------------------------------------------------- scheduler integration


def test_drivers_run_under_scheduler():
    from coopuavs.coopfc.sched import Scheduler

    hal, topics = HalIO(), TopicStore()
    imu = ImuDriver(hal.port("imu"), topics)
    baro = BaroDriver(hal.port("baro"), topics)
    sched = Scheduler(800)
    sched.add("imu_drv", 400, imu.tick)
    sched.add("baro_drv", 50, baro.tick)
    imu_sub = topics.subscribe("imu_raw")

    p0 = float(atmosphere.pressure(0.0))
    for _ in range(800):
        hal.port("imu").write(((0.0, 0.0, 0.0), (0.0, 0.0, 9.81)))
        hal.port("baro").write(p0)
        sched.run_tick()
    assert sched.stats("imu_drv").fires == 400
    assert imu_sub.read().stamp == 798 / 800  # last 400 Hz fire, derived time
    assert imu.stale is False and baro.stale is False
