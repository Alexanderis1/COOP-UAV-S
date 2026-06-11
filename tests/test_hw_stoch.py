"""P2-1: shared stochastic primitives for the hw device models.

Pins the contracts every hw device builds on: exact-ZOH Gauss-Markov with
stationary cold start, bias random walk, mid-tread quantization, and the
bit-exact equivalence between the per-tick step path and the vectorized
run path (the Allan suite generates millions of samples through run()
and that is only valid because run() IS the step loop, bit for bit).
"""

import numpy as np
import pytest

from coopuavs.hw import stoch


def _gm(sigma=0.5, tau=2.0, dt=0.1, shape=(4, 3), seed=9):
    rng = np.random.default_rng(seed)
    return stoch.GaussMarkov(sigma, tau, dt, rng.standard_normal(shape)), rng


# ---------------------------------------------------------------- quantize

def test_quantize_outputs_lie_on_lsb_grid_within_half_lsb():
    rng = np.random.default_rng(3)
    x = rng.uniform(-10.0, 10.0, size=(50,))
    q = stoch.quantize(x, 0.25)
    counts = q / 0.25
    np.testing.assert_allclose(counts, np.round(counts), atol=1e-12)
    assert np.max(np.abs(q - x)) <= 0.125 + 1e-12


def test_quantize_zero_lsb_is_passthrough():
    x = np.array([0.1234, -5.6789])
    np.testing.assert_array_equal(stoch.quantize(x, 0.0), x)


def test_quantize_rejects_negative_lsb():
    with pytest.raises(ValueError):
        stoch.quantize(np.zeros(3), -0.1)


# ------------------------------------------------------------- GaussMarkov

def test_gauss_markov_stationary_variance_from_first_step():
    # Cold start from the stationary distribution: the very first output of
    # a large ensemble already has variance sigma^2 (the Dryden precedent).
    sigma, tau, dt = 0.8, 5.0, 0.02
    rng = np.random.default_rng(7)
    gm = stoch.GaussMarkov(sigma, tau, dt, rng.standard_normal((8192, 1)))
    first = gm.step(rng.standard_normal((8192, 1)))
    assert abs(first.var() - sigma**2) / sigma**2 < 0.10


def test_gauss_markov_autocorrelation_decay():
    sigma, tau, dt = 1.0, 0.5, 0.05
    rng = np.random.default_rng(11)
    gm = stoch.GaussMarkov(sigma, tau, dt, rng.standard_normal((2048,)))
    steps = 400
    series = np.empty((steps, 2048))
    for k in range(steps):
        series[k] = gm.step(rng.standard_normal((2048,)))
    lag = 10                       # autocorr exp(-lag dt / tau) = exp(-1)
    num = np.mean(series[:-lag] * series[lag:])
    rho = num / series.var()
    assert abs(rho - np.exp(-1.0)) < 0.05


def test_gauss_markov_run_is_bitexact_with_step_loop():
    gm_a, rng_a = _gm()
    gm_b, rng_b = _gm()
    eps = rng_a.standard_normal((257, 4, 3))
    rng_b.standard_normal((257, 4, 3))   # keep generators aligned (unused)
    stepped = np.empty_like(eps)
    for k in range(eps.shape[0]):
        stepped[k] = gm_a.step(eps[k])
    ran = gm_b.run(eps)
    np.testing.assert_array_equal(stepped, ran)
    np.testing.assert_array_equal(gm_a.x, gm_b.x)


def test_gauss_markov_run_chunks_are_bitexact_with_one_call():
    gm_a, rng = _gm(seed=21)
    gm_b, _ = _gm(seed=21)
    eps = rng.standard_normal((300, 4, 3))
    whole = gm_a.run(eps)
    parts = np.concatenate([gm_b.run(eps[:97]), gm_b.run(eps[97:])], axis=0)
    np.testing.assert_array_equal(whole, parts)


def test_gauss_markov_validates_parameters():
    with pytest.raises(ValueError):
        stoch.GaussMarkov(-1.0, 1.0, 0.1, np.zeros(3))
    with pytest.raises(ValueError):
        stoch.GaussMarkov(1.0, 0.0, 0.1, np.zeros(3))
    with pytest.raises(ValueError):
        stoch.GaussMarkov(1.0, 1.0, 0.0, np.zeros(3))


# -------------------------------------------------------------- RandomWalk

def test_random_walk_variance_grows_linearly():
    sigma, dt = 0.3, 0.01
    rw = stoch.RandomWalk(sigma, dt, (4096,))
    rng = np.random.default_rng(5)
    steps = 2000
    for _ in range(steps):
        rw.step(rng.standard_normal((4096,)))
    expect = sigma**2 * steps * dt
    assert abs(rw.x.var() - expect) / expect < 0.15


def test_random_walk_run_is_bitexact_with_step_loop():
    rw_a = stoch.RandomWalk(0.3, 0.01, (4, 3))
    rw_b = stoch.RandomWalk(0.3, 0.01, (4, 3))
    eps = np.random.default_rng(13).standard_normal((211, 4, 3))
    stepped = np.empty_like(eps)
    for k in range(eps.shape[0]):
        stepped[k] = rw_a.step(eps[k])
    ran = np.concatenate([rw_b.run(eps[:50]), rw_b.run(eps[50:])], axis=0)
    np.testing.assert_array_equal(stepped, ran)
    np.testing.assert_array_equal(rw_a.x, rw_b.x)


# ---------------------------------------------------- analytic Allan curves

def test_analytic_avar_gauss_markov_limits():
    # tau << tau_c: GM looks like a rate random walk, K = sigma*sqrt(2/tau_c)
    # (driving-noise PSD); tau >> tau_c: white-like, N = sigma*sqrt(2 tau_c)
    # (Lorentzian S(0) = 2 sigma^2 tau_c).
    sigma, tau_c = 2.0, 10.0
    tau_lo, tau_hi = 1e-3, 1e5
    lo = stoch.avar_gauss_markov(sigma, tau_c, tau_lo)
    np.testing.assert_allclose(
        lo, stoch.avar_random_walk(sigma * np.sqrt(2.0 / tau_c), tau_lo), rtol=1e-3)
    hi = stoch.avar_gauss_markov(sigma, tau_c, tau_hi)
    np.testing.assert_allclose(
        hi, stoch.avar_white(sigma * np.sqrt(2.0 * tau_c), tau_hi), rtol=1e-3)


def test_analytic_avar_white_and_rw_shapes():
    tau = np.array([0.1, 1.0, 10.0])
    np.testing.assert_allclose(stoch.avar_white(2.0, tau), 4.0 / tau)
    np.testing.assert_allclose(stoch.avar_random_walk(2.0, tau), 4.0 * tau / 3.0)
