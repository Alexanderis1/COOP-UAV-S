"""P2-1: hw/imu.py — Kalibr/PX4-style stochastic IMU.

Measurement model (per axis, body FLU):

    gyro  = omega_body  + b0 + b_rw + b_gm + n_white   -> clip(range) -> quantize
    accel = f_body      + b0 + b_rw + b_gm + n_white   -> clip(range) -> quantize

with f_body the specific force q^-1 (a_world - g_world). The @slow Allan
suite is the headline pin: the configured noise density N, Gauss-Markov
bias (B proxy) and bias random walk K are recovered from the generated
stream within +-10% each (the P2-1 gate criterion).
"""

import numpy as np
import pytest
from allan_util import oadev

from coopuavs.hw import stoch
from coopuavs.hw.imu import Imu, ImuParams
from coopuavs.hw.params import load_devices
from coopuavs.physics import GRAVITY
from coopuavs.physics import rigid_body as rb

G_WORLD = np.array([0.0, 0.0, -GRAVITY])


def _params(**over) -> ImuParams:
    """All-zero noise baseline; tests switch on one effect at a time."""
    base = dict(
        rate_hz=400.0,
        gyro_noise_density=0.0, gyro_gm_sigma=0.0, gyro_gm_tau_s=100.0,
        gyro_rw_sigma=0.0, gyro_turn_on_sigma=0.0, gyro_lsb=0.0,
        gyro_range=np.inf,
        accel_noise_density=0.0, accel_gm_sigma=0.0, accel_gm_tau_s=100.0,
        accel_rw_sigma=0.0, accel_turn_on_sigma=0.0, accel_lsb=0.0,
        accel_range=np.inf,
        fifo_depth=8,
    )
    base.update(over)
    return ImuParams(**base)


def _noisy(**over) -> ImuParams:
    return _params(
        gyro_noise_density=1e-3, gyro_gm_sigma=4e-3, gyro_gm_tau_s=20.0,
        gyro_rw_sigma=2e-4, gyro_turn_on_sigma=3e-3,
        accel_noise_density=2e-3, accel_gm_sigma=8e-3, accel_gm_tau_s=20.0,
        accel_rw_sigma=4e-4, accel_turn_on_sigma=2e-1,
        **over)


def _hover_truth(n):
    quat = np.zeros((n, 4))
    quat[:, 0] = 1.0
    return quat, np.zeros((n, 3)), np.zeros((n, 3))


# ----------------------------------------------------------- deterministic

def test_quiet_imu_reads_rates_and_specific_force_exactly():
    rng = np.random.default_rng(1)
    n = 5
    imu = Imu(_params(), n, np.random.default_rng(2))
    axis = rng.standard_normal((n, 3))
    axis /= np.linalg.norm(axis, axis=1, keepdims=True)
    quat = rb.quat_from_axis_angle(axis, rng.uniform(-np.pi, np.pi, n))
    omega = rng.standard_normal((n, 3))
    a_world = rng.standard_normal((n, 3))
    gyro, accel = imu.sample(quat, omega, a_world)
    np.testing.assert_array_equal(gyro, omega)
    np.testing.assert_allclose(
        accel, rb.quat_rotate_inv(quat, a_world - G_WORLD), atol=1e-12)


def test_hover_specific_force_is_plus_g_up_and_freefall_is_zero():
    imu = Imu(_params(), 2, np.random.default_rng(3))
    quat, omega, _ = _hover_truth(2)
    _, accel = imu.sample(quat, omega, np.zeros((2, 3)))
    np.testing.assert_allclose(accel, [[0, 0, GRAVITY]] * 2, atol=1e-12)
    _, accel = imu.sample(quat, omega, np.broadcast_to(G_WORLD, (2, 3)))
    np.testing.assert_allclose(accel, np.zeros((2, 3)), atol=1e-12)


def test_quantization_lands_on_grid():
    lsb_g, lsb_a = 1.5e-3, 5e-3
    imu = Imu(_params(gyro_lsb=lsb_g, accel_lsb=lsb_a), 3, np.random.default_rng(4))
    quat, _, _ = _hover_truth(3)
    omega = np.full((3, 3), 0.123456)
    gyro, accel = imu.sample(quat, omega, np.zeros((3, 3)))
    np.testing.assert_array_equal(gyro, stoch.quantize(omega, lsb_g))
    f = np.broadcast_to([0.0, 0.0, GRAVITY], (3, 3))
    np.testing.assert_array_equal(accel, stoch.quantize(f, lsb_a))


def test_saturation_clips_to_range_before_quantization():
    imu = Imu(_params(gyro_range=10.0, gyro_lsb=0.5,
                      accel_range=20.0, accel_lsb=0.5),
              1, np.random.default_rng(5))
    quat, _, _ = _hover_truth(1)
    gyro, accel = imu.sample(quat, np.array([[99.0, -99.0, 3.14]]),
                             np.array([[0.0, 0.0, 1e4]]))
    np.testing.assert_array_equal(gyro[0], [10.0, -10.0, 3.0])
    assert accel[0, 2] == 20.0


# -------------------------------------------------------------- stochastic

def test_turn_on_bias_repeats_per_seed_and_differs_across_seeds_and_vehicles():
    p = _params(gyro_turn_on_sigma=1e-2, accel_turn_on_sigma=1e-1)
    quat, omega, a_world = _hover_truth(4)

    def biases(seed):
        imu = Imu(p, 4, np.random.default_rng(seed))
        gyro, accel = imu.sample(quat, omega, a_world)
        return gyro, accel - [0.0, 0.0, GRAVITY]

    g1, a1 = biases(42)
    g2, a2 = biases(42)
    g3, _ = biases(43)
    np.testing.assert_array_equal(g1, g2)
    np.testing.assert_array_equal(a1, a2)
    assert np.abs(g1 - g3).max() > 0.0
    assert np.abs(g1).max() > 0.0
    # per-vehicle draws are independent: no two rows equal
    assert np.abs(g1[0] - g1[1]).max() > 0.0


def test_turn_on_bias_ensemble_std_matches_sigma():
    sigma = 5e-3
    imu = Imu(_params(gyro_turn_on_sigma=sigma), 4096, np.random.default_rng(6))
    quat, omega, a_world = _hover_truth(4096)
    gyro, _ = imu.sample(quat, omega, a_world)
    assert abs(gyro.std() - sigma) / sigma < 0.05


def test_white_noise_std_matches_noise_density():
    nd, rate = 2e-3, 400.0
    imu = Imu(_params(gyro_noise_density=nd, rate_hz=rate), 4,
              np.random.default_rng(7))
    noise = imu.generate(20_000)        # (steps, n, 6)
    gyro = noise[:, :, :3]
    expect = nd * np.sqrt(rate)
    assert abs(gyro.std() - expect) / expect < 0.05
    assert abs(gyro.mean()) < 5 * expect / np.sqrt(gyro.size)


def test_gm_bias_stationary_variance_and_decay():
    sigma, tau, rate = 3e-3, 5.0, 100.0
    imu = Imu(_params(gyro_gm_sigma=sigma, gyro_gm_tau_s=tau, rate_hz=rate),
              512, np.random.default_rng(8))
    noise = imu.generate(4_000)
    gyro = noise[:, :, 0]                       # (steps, n)
    assert abs(gyro.var() - sigma**2) / sigma**2 < 0.10
    lag = 100                                   # exp(-lag/(rate*tau)) = e^-0.2
    rho = np.mean(gyro[:-lag] * gyro[lag:]) / gyro.var()
    assert abs(rho - np.exp(-lag / (rate * tau))) < 0.05


def test_bias_random_walk_variance_grows():
    k, rate, steps = 5e-4, 100.0, 4_000
    imu = Imu(_params(gyro_rw_sigma=k, rate_hz=rate), 2048,
              np.random.default_rng(9))
    noise = imu.generate(steps)
    last = noise[-1, :, :3]
    expect = k**2 * steps / rate
    assert abs(last.var() - expect) / expect < 0.10


# ------------------------------------------------ vectorized == per-tick

def test_generate_is_bitexact_with_sample_loop():
    n, steps = 3, 1500
    imu_a = Imu(_noisy(), n, np.random.default_rng(10))
    imu_b = Imu(_noisy(), n, np.random.default_rng(10))
    quat, omega, _ = _hover_truth(n)
    a_world = np.broadcast_to(G_WORLD, (n, 3))    # zero specific force
    stepped = np.empty((steps, n, 6))
    for kk in range(steps):
        gyro, accel = imu_a.sample(quat, omega, a_world)
        stepped[kk, :, :3] = gyro
        stepped[kk, :, 3:] = accel
    gen = np.concatenate(
        [imu_b.generate(700), imu_b.generate(steps - 700)], axis=0)
    np.testing.assert_array_equal(stepped, gen)


# ------------------------------------------------------------------- FIFO

def test_fifo_returns_pushed_samples_in_order_and_clears():
    n = 2
    imu = Imu(_params(fifo_depth=8), n, np.random.default_rng(11))
    quat, _, a_world = _hover_truth(n)
    expect = []
    for k in range(5):
        omega = np.full((n, 3), float(k))
        gyro, accel = imu.sample(quat, omega, a_world)
        expect.append(np.concatenate([gyro, accel], axis=1))
    frames, overflowed = imu.fifo_read()
    assert not overflowed
    np.testing.assert_array_equal(frames, np.stack(expect))
    frames, overflowed = imu.fifo_read()
    assert frames.shape == (0, n, 6) and not overflowed


def test_fifo_overflow_drops_oldest_and_flags():
    n = 1
    imu = Imu(_params(fifo_depth=4), n, np.random.default_rng(12))
    quat, _, a_world = _hover_truth(n)
    for k in range(7):
        imu.sample(quat, np.full((n, 3), float(k)), a_world)
    frames, overflowed = imu.fifo_read()
    assert overflowed
    np.testing.assert_array_equal(frames[:, 0, 0], [3.0, 4.0, 5.0, 6.0])
    _, overflowed = imu.fifo_read()
    assert not overflowed                      # flag cleared by the read


# ----------------------------------------------------------- determinism

def test_run_twice_is_identical():
    def run(seed):
        imu = Imu(_noisy(), 4, np.random.default_rng(seed))
        quat, omega, a_world = _hover_truth(4)
        return np.stack([np.concatenate(imu.sample(quat, omega, a_world), axis=1)
                         for _ in range(50)])
    np.testing.assert_array_equal(run(99), run(99))
    assert np.abs(run(99) - run(100)).max() > 0.0


def test_fleet_growth_leaves_existing_vehicles_streams_identical():
    # P0 contract (Dryden precedent): vehicle i's draws come from spawn child
    # i of the injected parent, so growing the fleet cannot shift them.
    small = Imu(_noisy(), 3, np.random.default_rng(77)).generate(200)
    big = Imu(_noisy(), 6, np.random.default_rng(77)).generate(200)
    np.testing.assert_array_equal(small, big[:, :3, :])


# ------------------------------------------------------------- parameters

def test_params_load_from_yaml():
    cfg = load_devices("interceptor_devices")
    p = ImuParams.from_dict(cfg["imu"])
    assert p.rate_hz == 400.0
    assert p.gyro_range > 30.0 and p.accel_range > 150.0
    assert p.fifo_depth >= 8
    Imu(p, 2, np.random.default_rng(0))        # constructs cleanly


def test_params_validation_rejects_bad_values():
    with pytest.raises(ValueError):
        _params(rate_hz=0.0)
    with pytest.raises(ValueError):
        _params(gyro_noise_density=-1e-3)
    with pytest.raises(ValueError):
        _params(gyro_gm_tau_s=0.0)
    with pytest.raises(ValueError):
        _params(fifo_depth=0)
    with pytest.raises(ValueError):
        _params(gyro_range=0.0)
    with pytest.raises(ValueError):
        _params(gyro_noise_density=np.nan)
    with pytest.raises(ValueError):
        Imu(_params(), 0, np.random.default_rng(1))


# ------------------------------------------------------------ Allan suite

@pytest.mark.slow
def test_allan_recovers_configured_n_b_k_within_10_percent():
    """P2-1 gate pin: slope/region fits on 32768 s of generated data recover
    each configured coefficient within +-10% on every axis.

    Regions are placed where each component dominates; the two configured
    non-target components are subtracted analytically (stoch.avar_*) before
    the fit, the standard IEEE 952 identification procedure.
    """
    rate = 100.0
    gyro = dict(nd=1.5e-3, gm_sigma=6e-3, gm_tau=1.0, rw=5e-4)
    accel = dict(nd=1.5e-2, gm_sigma=6e-2, gm_tau=1.0, rw=5e-3)
    p = _params(
        rate_hz=rate,
        gyro_noise_density=gyro["nd"], gyro_gm_sigma=gyro["gm_sigma"],
        gyro_gm_tau_s=gyro["gm_tau"], gyro_rw_sigma=gyro["rw"],
        gyro_turn_on_sigma=0.02,       # constant offset: Allan must reject it
        accel_noise_density=accel["nd"], accel_gm_sigma=accel["gm_sigma"],
        accel_gm_tau_s=accel["gm_tau"], accel_rw_sigma=accel["rw"],
        accel_turn_on_sigma=0.2,
    )
    imu = Imu(p, 1, np.random.default_rng(2026))
    steps = 3_276_800                  # 32768 s at 100 Hz
    noise = imu.generate(steps)
    dt = 1.0 / rate

    m_white = np.array([1, 2, 4])
    m_gm = np.array([128, 189, 256])         # hump at 1.89 tau_c = 1.89 s
    m_rw = np.array([6400, 9216, 13056])

    for cols, c in ((slice(0, 3), gyro), (slice(3, 6), accel)):
        for ax in range(3):
            y = noise[:, 0, cols][:, ax]

            ad = oadev(y, dt, m_white)
            tau = m_white * dt
            resid = ad**2 - stoch.avar_gauss_markov(
                c["gm_sigma"], c["gm_tau"], tau) - stoch.avar_random_walk(
                c["rw"], tau)
            n_est = np.sqrt(np.mean(resid * tau))
            assert abs(n_est / c["nd"] - 1.0) < 0.10, \
                f"N axis {ax}: {n_est:.4e} vs {c['nd']:.4e}"

            ad = oadev(y, dt, m_gm)
            tau = m_gm * dt
            resid = ad**2 - stoch.avar_white(c["nd"], tau) - \
                stoch.avar_random_walk(c["rw"], tau)
            unit = stoch.avar_gauss_markov(1.0, c["gm_tau"], tau)
            b_est = np.sqrt(np.mean(resid / unit))
            assert abs(b_est / c["gm_sigma"] - 1.0) < 0.10, \
                f"B axis {ax}: {b_est:.4e} vs {c['gm_sigma']:.4e}"

            ad = oadev(y, dt, m_rw)
            tau = m_rw * dt
            resid = ad**2 - stoch.avar_white(c["nd"], tau) - \
                stoch.avar_gauss_markov(c["gm_sigma"], c["gm_tau"], tau)
            k_est = np.sqrt(np.mean(resid * 3.0 / tau))
            assert abs(k_est / c["rw"] - 1.0) < 0.10, \
                f"K axis {ax}: {k_est:.4e} vs {c['rw']:.4e}"
