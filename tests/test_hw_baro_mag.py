"""P2-3: hw/baro.py (ISA round-trip + drift) and hw/mag.py (theater field
vector + Gauss-Markov bias + hard iron)."""

import numpy as np
import pytest

from coopuavs.hw.baro import Baro, BaroParams, altitude_from_pressure
from coopuavs.hw.mag import Mag, MagParams, theater_field_enu
from coopuavs.hw.params import load_devices
from coopuavs.physics import atmosphere
from coopuavs.physics import rigid_body as rb


def _baro(**over) -> BaroParams:
    base = dict(rate_hz=50.0, sigma_pa=0.0, gm_sigma_pa=0.0,
                gm_tau_s=600.0, lsb_pa=0.0)
    base.update(over)
    return BaroParams(**base)


def _mag(**over) -> MagParams:
    base = dict(rate_hz=50.0, magnitude_ut=50.0, declination_deg=4.0,
                inclination_deg=63.0, sigma_ut=0.0, gm_sigma_ut=0.0,
                gm_tau_s=300.0, hard_iron_sigma_ut=0.0, lsb_ut=0.0)
    base.update(over)
    return MagParams(**base)


# ------------------------------------------------------------------- baro

def test_isa_inverse_is_exact_round_trip():
    alt = np.array([-100.0, 0.0, 35.0, 200.0, 1500.0, 8000.0])
    np.testing.assert_allclose(
        altitude_from_pressure(atmosphere.pressure(alt)), alt, atol=1e-9)


def test_quiet_baro_reads_isa_pressure_exactly():
    baro = Baro(_baro(), 3, np.random.default_rng(1))
    alt = np.array([10.0, 120.0, 900.0])
    np.testing.assert_array_equal(baro.sample(alt), atmosphere.pressure(alt))


def test_noisy_baro_altitude_error_matches_hydrostatic_scaling():
    sigma_pa = 3.0
    baro = Baro(_baro(sigma_pa=sigma_pa), 8192, np.random.default_rng(2))
    alt = np.full(8192, 150.0)
    h_meas = altitude_from_pressure(baro.sample(alt))
    sigma_h = sigma_pa / (atmosphere.density(150.0) * atmosphere.G0)
    err = h_meas - 150.0
    assert abs(err.std() - sigma_h) / sigma_h < 0.05
    assert abs(err.mean()) < 5.0 * sigma_h / np.sqrt(8192)


def test_baro_drift_is_stationary_gm_with_configured_correlation():
    tau = 2.0                                    # 50 Hz -> lag-25 rho e^-0.25
    baro = Baro(_baro(gm_sigma_pa=15.0, gm_tau_s=tau), 4096,
                np.random.default_rng(3))
    alt = np.zeros(4096)
    p0 = float(atmosphere.pressure(0.0))
    series = np.stack([baro.sample(alt) - p0 for _ in range(200)])
    assert abs(series.var() - 15.0**2) / 15.0**2 < 0.05
    lag = 25
    rho = np.mean(series[:-lag] * series[lag:]) / series.var()
    assert abs(rho - np.exp(-lag / (50.0 * tau))) < 0.05


def test_baro_quantization_grid():
    baro = Baro(_baro(lsb_pa=1.2), 2, np.random.default_rng(4))
    p = baro.sample(np.array([77.0, 433.0]))
    counts = p / 1.2
    np.testing.assert_allclose(counts, np.round(counts), atol=1e-9)


def test_baro_rejects_invalid_altitude():
    baro = Baro(_baro(), 1, np.random.default_rng(5))
    with pytest.raises(ValueError):
        baro.sample(np.array([12_000.0]))        # above ISA troposphere model
    with pytest.raises(ValueError):
        baro.sample(np.array([np.nan]))


def test_baro_determinism_and_fleet_growth():
    p = _baro(sigma_pa=3.0, gm_sigma_pa=15.0, gm_tau_s=600.0)

    def run(seed, n):
        baro = Baro(p, n, np.random.default_rng(seed))
        alt = np.zeros(n)
        return np.stack([baro.sample(alt) for _ in range(50)])

    np.testing.assert_array_equal(run(7, 3), run(7, 3))
    assert np.abs(run(7, 3) - run(8, 3)).max() > 0.0
    np.testing.assert_array_equal(run(9, 3), run(9, 6)[:, :3])


# -------------------------------------------------------------------- mag

def test_theater_field_geometry():
    b = theater_field_enu(50.0, declination_deg=4.0, inclination_deg=63.0)
    assert b.shape == (3,)
    np.testing.assert_allclose(np.linalg.norm(b), 50.0, rtol=1e-12)
    # declination: horizontal field bearing east of true north
    np.testing.assert_allclose(np.degrees(np.arctan2(b[0], b[1])), 4.0)
    # inclination: dip below horizontal (northern hemisphere -> B_up < 0)
    horiz = np.hypot(b[0], b[1])
    np.testing.assert_allclose(np.degrees(np.arctan2(-b[2], horiz)), 63.0)


def test_quiet_mag_at_identity_reads_theater_field():
    mag = Mag(_mag(), 2, np.random.default_rng(10))
    quat = np.zeros((2, 4))
    quat[:, 0] = 1.0
    expect = theater_field_enu(50.0, 4.0, 63.0)
    np.testing.assert_allclose(mag.sample(quat), [expect, expect], atol=1e-12)


def test_quiet_mag_matches_rotation_matrix_and_preserves_norm():
    rng = np.random.default_rng(11)
    n = 6
    axis = rng.standard_normal((n, 3))
    axis /= np.linalg.norm(axis, axis=1, keepdims=True)
    quat = rb.quat_from_axis_angle(axis, rng.uniform(-np.pi, np.pi, n))
    mag = Mag(_mag(), n, np.random.default_rng(12))
    meas = mag.sample(quat)
    b_world = theater_field_enu(50.0, 4.0, 63.0)
    expect = np.einsum("nij,j->ni", rb.quat_to_rotmat(quat).transpose(0, 2, 1),
                       b_world)
    np.testing.assert_allclose(meas, expect, atol=1e-10)
    np.testing.assert_allclose(np.linalg.norm(meas, axis=1), 50.0, rtol=1e-10)


def test_mag_yaw_rotation_moves_heading_components():
    yaw = np.radians(30.0)
    quat = rb.quat_from_axis_angle(np.array([[0.0, 0.0, 1.0]]), np.array([yaw]))
    mag = Mag(_mag(declination_deg=0.0), 1, np.random.default_rng(13))
    meas = mag.sample(quat)[0]
    b = theater_field_enu(50.0, 0.0, 63.0)
    horiz = np.hypot(b[0], b[1])
    # FLU body x = forward; at yaw psi (CCW from north? ENU yaw from east),
    # the horizontal field swings by -psi in the body frame.
    measured_bearing = np.arctan2(meas[1], meas[0])
    truth_bearing = np.arctan2(b[1], b[0])
    np.testing.assert_allclose(measured_bearing, truth_bearing - yaw, atol=1e-12)
    np.testing.assert_allclose(np.hypot(meas[0], meas[1]), horiz, rtol=1e-12)


def test_mag_hard_iron_repeats_per_seed_and_scales():
    p = _mag(hard_iron_sigma_ut=2.0)
    quat = np.zeros((4096, 4))
    quat[:, 0] = 1.0
    expect = theater_field_enu(50.0, 4.0, 63.0)

    def bias(seed):
        return Mag(p, 4096, np.random.default_rng(seed)).sample(quat) - expect

    b1, b2, b3 = bias(20), bias(20), bias(21)
    np.testing.assert_array_equal(b1, b2)
    assert np.abs(b1 - b3).max() > 0.0
    assert abs(b1.std() - 2.0) / 2.0 < 0.05


def test_mag_gm_bias_and_white_noise_statistics():
    tau = 1.0
    p = _mag(gm_sigma_ut=0.5, gm_tau_s=tau)
    quat = np.zeros((4096, 4))
    quat[:, 0] = 1.0
    expect = theater_field_enu(50.0, 4.0, 63.0)
    mag = Mag(p, 4096, np.random.default_rng(22))
    series = np.stack([mag.sample(quat) - expect for _ in range(150)])
    assert abs(series.var() - 0.5**2) / 0.5**2 < 0.05
    lag = 50                                     # 50 Hz -> rho e^-1
    rho = np.mean(series[:-lag] * series[lag:]) / series.var()
    assert abs(rho - np.exp(-1.0)) < 0.05

    white = Mag(_mag(sigma_ut=0.3), 4096, np.random.default_rng(23))
    w = white.sample(quat) - expect
    assert abs(w.std() - 0.3) / 0.3 < 0.05


def test_mag_quantization_and_determinism_and_fleet_growth():
    p = _mag(sigma_ut=0.3, gm_sigma_ut=0.5, hard_iron_sigma_ut=2.0, lsb_ut=0.3)

    def run(seed, n):
        mag = Mag(p, n, np.random.default_rng(seed))
        quat = np.zeros((n, 4))
        quat[:, 0] = 1.0
        return np.stack([mag.sample(quat) for _ in range(40)])

    a = run(30, 3)
    counts = a / 0.3
    np.testing.assert_allclose(counts, np.round(counts), atol=1e-9)
    np.testing.assert_array_equal(a, run(30, 3))
    assert np.abs(a - run(31, 3)).max() > 0.0
    np.testing.assert_array_equal(a, run(30, 6)[:, :3, :])


# ------------------------------------------------------------- parameters

def test_params_load_from_yaml():
    cfg = load_devices("interceptor_devices")
    bp = BaroParams.from_dict(cfg["baro"])
    mp = MagParams.from_dict(cfg["mag"])
    assert bp.rate_hz == 50.0 and mp.rate_hz == 50.0
    assert mp.magnitude_ut > 0.0
    Baro(bp, 2, np.random.default_rng(0))
    Mag(mp, 2, np.random.default_rng(0))


def test_params_validation_rejects_bad_values():
    with pytest.raises(ValueError):
        _baro(rate_hz=0.0)
    with pytest.raises(ValueError):
        _baro(sigma_pa=-1.0)
    with pytest.raises(ValueError):
        _baro(gm_tau_s=0.0)
    with pytest.raises(ValueError):
        _mag(magnitude_ut=-1.0)
    with pytest.raises(ValueError):
        _mag(inclination_deg=120.0)
    with pytest.raises(ValueError):
        _mag(sigma_ut=np.nan)
    with pytest.raises(ValueError):
        Baro(_baro(), 0, np.random.default_rng(1))
    with pytest.raises(ValueError):
        Mag(_mag(), 0, np.random.default_rng(1))
