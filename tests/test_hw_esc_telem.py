"""P2-5: hw/esc_telem.py — per-ESC telemetry frames (PHY-UAV-013)."""

import numpy as np
import pytest

from coopuavs.hw.esc_telem import EscTelem, EscTelemParams
from coopuavs.hw.params import load_devices
from coopuavs.physics.battery import BatteryEcm
from coopuavs.physics.motor import MotorEsc
from coopuavs.physics.multirotor import MultirotorParams
from coopuavs.physics.params import load_airframe
from coopuavs.physics.powertrain import Powertrain

RPM_PER_RAD_S = 60.0 / (2.0 * np.pi)


def _params(**over) -> EscTelemParams:
    base = dict(rate_hz=10.0, sigma_rpm=0.0, sigma_v=0.0, sigma_i=0.0,
                rpm_lsb=0.0, v_lsb=0.0, i_lsb=0.0)
    base.update(over)
    return EscTelemParams(**base)


def test_quiet_telemetry_reads_truth_with_exact_rpm_conversion():
    telem = EscTelem(_params(), 3, 4, np.random.default_rng(1))
    omega = np.full((3, 4), 1200.0)
    v_bus = np.array([44.4, 40.0, 38.5])
    i_bus = np.array([120.0, 80.0, 12.5])
    frame = telem.sample(omega, v_bus, i_bus)
    np.testing.assert_array_equal(frame.rpm, omega * RPM_PER_RAD_S)
    np.testing.assert_array_equal(frame.voltage, v_bus)
    np.testing.assert_array_equal(frame.current, i_bus)


def test_quantization_grids():
    telem = EscTelem(_params(rpm_lsb=10.0, v_lsb=0.01, i_lsb=0.1), 2, 4,
                     np.random.default_rng(2))
    frame = telem.sample(np.full((2, 4), 1234.567), np.array([44.123, 39.9876]),
                         np.array([123.456, 7.89]))
    for arr, lsb in ((frame.rpm, 10.0), (frame.voltage, 0.01),
                     (frame.current, 0.1)):
        counts = arr / lsb
        np.testing.assert_allclose(counts, np.round(counts), atol=1e-9)
    assert np.abs(frame.rpm - 1234.567 * RPM_PER_RAD_S).max() <= 5.0


def test_noise_stds():
    telem = EscTelem(_params(sigma_rpm=5.0, sigma_v=0.02, sigma_i=0.1),
                     8192, 4, np.random.default_rng(3))
    frame = telem.sample(np.zeros((8192, 4)), np.zeros(8192), np.zeros(8192))
    assert abs(frame.rpm.std() - 5.0) / 5.0 < 0.05
    assert abs(frame.voltage.std() - 0.02) / 0.02 < 0.05
    assert abs(frame.current.std() - 0.1) / 0.1 < 0.05


def test_determinism_and_fleet_growth():
    p = _params(sigma_rpm=5.0, sigma_v=0.02, sigma_i=0.1)

    def run(seed, n):
        telem = EscTelem(p, n, 4, np.random.default_rng(seed))
        return np.stack([telem.sample(np.zeros((n, 4)), np.zeros(n),
                                      np.zeros(n)).rpm for _ in range(30)])

    np.testing.assert_array_equal(run(7, 3), run(7, 3))
    assert np.abs(run(7, 3) - run(8, 3)).max() > 0.0
    np.testing.assert_array_equal(run(9, 3), run(9, 6)[:, :3, :])


def test_telemetry_of_a_running_powertrain_stays_in_envelope():
    cfg = MultirotorParams.from_dict(load_airframe("interceptor_quad"))
    n = 2
    motor = MotorEsc(n, cfg.n_rotors, **cfg.motor)
    battery = BatteryEcm(n, **cfg.battery)
    pt = Powertrain(motor, battery, i_bus_max_a=350.0)
    telem = EscTelem(_params(rpm_lsb=10.0, v_lsb=0.01, i_lsb=0.1), n,
                     cfg.n_rotors, np.random.default_rng(4))
    throttle = np.full((n, cfg.n_rotors), 0.6)
    for _ in range(400):                       # 0.5 s at 800 Hz
        omega, v_bus, i_bus = pt.step(1.0 / 800.0, throttle)
    frame = telem.sample(omega, v_bus, i_bus)
    assert np.all(frame.rpm > 0.0) and np.all(frame.rpm < 14_000.0)
    assert np.all(frame.voltage >= 36.0) and np.all(frame.voltage <= 50.4)
    assert np.all(frame.current > 0.0) and np.all(frame.current <= 350.0)


def test_params_load_from_yaml_and_validation():
    cfg = load_devices("interceptor_devices")
    p = EscTelemParams.from_dict(cfg["esc_telem"])
    assert p.rate_hz >= 1.0                    # PHY-UAV-013: >= 1 Hz health
    EscTelem(p, 2, 4, np.random.default_rng(0))
    with pytest.raises(ValueError):
        _params(rate_hz=0.0)
    with pytest.raises(ValueError):
        _params(sigma_rpm=-1.0)
    with pytest.raises(ValueError):
        _params(v_lsb=np.nan)
    with pytest.raises(ValueError):
        EscTelem(_params(), 0, 4, np.random.default_rng(1))
    with pytest.raises(ValueError):
        EscTelem(_params(), 2, 0, np.random.default_rng(1))
