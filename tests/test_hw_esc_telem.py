"""P2-5: hw/esc_telem.py — per-ESC telemetry frames (PHY-UAV-013).

P5-1f adds the BMS cell-tap channels (``cells``): the frame carries
per-series-cell voltages with their own noise/quantization, sourced
from ``BatteryEcm.cell_voltages``. The per-sample draw layout is now
[rpm x rotors, v, i, cell x cells] — a documented draw-layout change
(test_hw_draw_layout pins it).
"""

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
                rpm_lsb=0.0, v_lsb=0.0, i_lsb=0.0,
                sigma_v_cell=0.0, v_cell_lsb=0.0)
    base.update(over)
    return EscTelemParams(**base)


def _cells(v_bus: np.ndarray, cells: int = 12) -> np.ndarray:
    return np.repeat(v_bus[:, None] / cells, cells, axis=1)


def test_quiet_telemetry_reads_truth_with_exact_rpm_conversion():
    telem = EscTelem(_params(), 3, 4, np.random.default_rng(1))
    omega = np.full((3, 4), 1200.0)
    v_bus = np.array([44.4, 40.0, 38.5])
    i_bus = np.array([120.0, 80.0, 12.5])
    frame = telem.sample(omega, v_bus, i_bus, _cells(v_bus))
    np.testing.assert_array_equal(frame.rpm, omega * RPM_PER_RAD_S)
    np.testing.assert_array_equal(frame.voltage, v_bus)
    np.testing.assert_array_equal(frame.current, i_bus)
    np.testing.assert_array_equal(frame.cells, _cells(v_bus))


def test_quantization_grids():
    telem = EscTelem(_params(rpm_lsb=10.0, v_lsb=0.01, i_lsb=0.1,
                             v_cell_lsb=0.01), 2, 4,
                     np.random.default_rng(2))
    v_bus = np.array([44.123, 39.9876])
    frame = telem.sample(np.full((2, 4), 1234.567), v_bus,
                         np.array([123.456, 7.89]), _cells(v_bus))
    for arr, lsb in ((frame.rpm, 10.0), (frame.voltage, 0.01),
                     (frame.current, 0.1), (frame.cells, 0.01)):
        counts = arr / lsb
        np.testing.assert_allclose(counts, np.round(counts), atol=1e-9)
    assert np.abs(frame.rpm - 1234.567 * RPM_PER_RAD_S).max() <= 5.0


def test_noise_stds():
    telem = EscTelem(_params(sigma_rpm=5.0, sigma_v=0.02, sigma_i=0.1,
                             sigma_v_cell=0.005),
                     8192, 4, np.random.default_rng(3))
    z = np.zeros(8192)
    frame = telem.sample(np.zeros((8192, 4)), z, z, np.zeros((8192, 12)))
    assert abs(frame.rpm.std() - 5.0) / 5.0 < 0.05
    assert abs(frame.voltage.std() - 0.02) / 0.02 < 0.05
    assert abs(frame.current.std() - 0.1) / 0.1 < 0.05
    assert abs(frame.cells.std() - 0.005) / 0.005 < 0.05


def test_determinism_and_fleet_growth():
    p = _params(sigma_rpm=5.0, sigma_v=0.02, sigma_i=0.1, sigma_v_cell=0.005)

    def run(seed, n):
        telem = EscTelem(p, n, 4, np.random.default_rng(seed))
        return np.stack([telem.sample(np.zeros((n, 4)), np.zeros(n),
                                      np.zeros(n), np.zeros((n, 12))).rpm
                         for _ in range(30)])

    np.testing.assert_array_equal(run(7, 3), run(7, 3))
    assert np.abs(run(7, 3) - run(8, 3)).max() > 0.0
    np.testing.assert_array_equal(run(9, 3), run(9, 6)[:, :3, :])


def test_telemetry_of_a_running_powertrain_stays_in_envelope():
    cfg = MultirotorParams.from_dict(load_airframe("interceptor_quad"))
    n = 2
    motor = MotorEsc(n, cfg.n_rotors, **cfg.motor)
    battery = BatteryEcm(n, **cfg.battery)
    pt = Powertrain(motor, battery, i_bus_max_a=350.0)
    telem = EscTelem(_params(rpm_lsb=10.0, v_lsb=0.01, i_lsb=0.1,
                             v_cell_lsb=0.01), n,
                     cfg.n_rotors, np.random.default_rng(4),
                     cells=battery.n_series)
    throttle = np.full((n, cfg.n_rotors), 0.6)
    for _ in range(400):                       # 0.5 s at 800 Hz
        omega, v_bus, i_bus = pt.step(1.0 / 800.0, throttle)
    frame = telem.sample(omega, v_bus, i_bus, battery.cell_voltages(v_bus))
    assert np.all(frame.rpm > 0.0) and np.all(frame.rpm < 14_000.0)
    assert np.all(frame.voltage >= 36.0) and np.all(frame.voltage <= 50.4)
    assert np.all(frame.current > 0.0) and np.all(frame.current <= 350.0)
    # balanced pack: cell taps agree with the equal split of the bus read
    np.testing.assert_allclose(frame.cells.sum(axis=1), frame.voltage,
                               atol=12 * 0.01 + 0.02)


def test_injected_imbalance_is_visible_in_the_taps():
    # The CELL_IMBALANCE fault chain (P5-2a injects, the FCU monitor
    # consumes): a weak cell shows up as a tap spread; the pack stays
    # voltage-consistent (cells sum to the bus).
    cfg = MultirotorParams.from_dict(load_airframe("interceptor_quad"))
    battery = BatteryEcm(1, **cfg.battery)
    deltas = np.zeros(battery.n_series)
    deltas[3] = -0.25                          # cell 3 sits 25% SOC low
    deltas -= deltas.mean()                    # spread, not capacity edit
    battery.inject_cell_imbalance(0, deltas)
    v_bus = battery.step(1e-3, np.array([50.0]))
    cells = battery.cell_voltages(v_bus)
    np.testing.assert_allclose(cells.sum(axis=1), v_bus, atol=1e-9)
    spread = cells.max() - cells.min()
    assert spread > 0.05                       # well above tap noise
    assert cells[0].argmin() == 3
    telem = EscTelem(_params(), 1, 4, np.random.default_rng(5))
    frame = telem.sample(np.zeros((1, 4)), v_bus, np.array([50.0]), cells)
    np.testing.assert_array_equal(frame.cells, cells)


def test_unfaulted_battery_is_bitwise_pre_p5():
    # The fault seams default to the exact pre-P5 arithmetic paths.
    cfg = MultirotorParams.from_dict(load_airframe("interceptor_quad"))
    a = BatteryEcm(2, **cfg.battery)
    b = BatteryEcm(2, **cfg.battery)
    b.cell_delta_soc = None                    # explicit: no injection
    i = np.array([120.0, 35.0])
    for _ in range(200):
        va = a.step(1e-3, i)
        vb = b.step(1e-3, i)
    np.testing.assert_array_equal(va, vb)
    np.testing.assert_array_equal(a.soc, b.soc)
    np.testing.assert_array_equal(a.cell_voltages(va),
                                  np.repeat(va[:, None] / a.n_series,
                                            a.n_series, axis=1))


def test_params_load_from_yaml_and_validation():
    cfg = load_devices("interceptor_devices")
    p = EscTelemParams.from_dict(cfg["esc_telem"])
    assert p.rate_hz >= 1.0                    # PHY-UAV-013: >= 1 Hz health
    assert p.sigma_v_cell > 0.0 and p.v_cell_lsb > 0.0
    EscTelem(p, 2, 4, np.random.default_rng(0))
    with pytest.raises(ValueError):
        _params(rate_hz=0.0)
    with pytest.raises(ValueError):
        _params(sigma_rpm=-1.0)
    with pytest.raises(ValueError):
        _params(v_lsb=np.nan)
    with pytest.raises(ValueError):
        _params(sigma_v_cell=-0.1)
    with pytest.raises(ValueError):
        EscTelem(_params(), 0, 4, np.random.default_rng(1))
    with pytest.raises(ValueError):
        EscTelem(_params(), 2, 0, np.random.default_rng(1))
    with pytest.raises(ValueError):
        EscTelem(_params(), 2, 4, np.random.default_rng(1), cells=0)
