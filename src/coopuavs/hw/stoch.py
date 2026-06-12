"""Shared stochastic primitives for the hw device models.

Every Kalibr/PX4-style sensor error budget is built from four parts
[El-Sheimy, Hou & Niu 2008, "Analysis and modeling of inertial sensors
using Allan variance", IEEE TIM 57(1); IEEE Std 952-1997 Annex B/C]:

- white noise of density N (units/sqrt(Hz)): discrete sample sigma N/sqrt(dt);
- first-order Gauss-Markov bias (sigma_c, tau_c) — the bias-instability
  proxy: x_dot = -x/tau_c + w. Exact ZOH discretization
  (the battery.py precedent), phi = exp(-dt/tau_c):
      x[k] = phi x[k-1] + sigma_c sqrt(1 - phi^2) eps[k]
  cold-started from the exact stationary distribution x ~ N(0, sigma_c^2)
  (the Dryden precedent: the first sample already carries full statistics);
- bias random walk of coefficient K (units/sqrt(s)):
      b[k] = b[k-1] + K sqrt(dt) eps[k], b[0] = 0;
- a turn-on bias drawn once per device power-up (device classes own that).

Each process exposes a per-tick ``step`` and a vectorized ``run`` that are
bit-for-bit identical (pinned by tests): the @slow Allan suites generate
millions of samples through ``run`` and that is only a valid test of the
device because ``run`` IS the step loop.

Analytic Allan variances for the identification fits
[IEEE Std 952-1997 Annex C; GM curve re-derived from the autocorrelation
R(u) = sigma_c^2 e^(-|u|/tau_c) — IEEE writes it via the driving noise q
with sigma_c^2 = q^2 tau_c / 2]:

    AVAR_N(tau)  = N^2 / tau
    AVAR_K(tau)  = K^2 tau / 3
    AVAR_GM(tau) = sigma_c^2 (tau_c/tau) [2 - (tau_c/tau)
                   (3 - 4 e^(-tau/tau_c) + e^(-2 tau/tau_c))]
"""

from __future__ import annotations

import numpy as np
from scipy.signal import lfilter


def quantize(x: np.ndarray, lsb: float) -> np.ndarray:
    """Mid-tread quantization to the lsb grid; lsb == 0 is a passthrough
    (returns x unchanged, no copy). Ties round half-to-even (np.round)."""
    if lsb < 0.0:
        raise ValueError(f"lsb must be >= 0, got {lsb!r}")
    if lsb == 0.0:
        return x
    return np.round(x / lsb) * lsb


def _check_positive(name: str, v: float) -> float:
    if not (np.isfinite(v) and v > 0.0):
        raise ValueError(f"{name} must be finite and > 0, got {v!r}")
    return float(v)


def _check_nonneg(name: str, v: float) -> float:
    if not (np.isfinite(v) and v >= 0.0):
        raise ValueError(f"{name} must be finite and >= 0, got {v!r}")
    return float(v)


class GaussMarkov:
    """First-order Gauss-Markov process, exact ZOH, stationary cold start.

    eps0 is a standard-normal draw of the state shape (taken from the
    owner's child streams so the cold start is part of the device's
    deterministic draw layout).
    """

    def __init__(self, sigma: float, tau_s: float, dt: float, eps0: np.ndarray):
        sigma = _check_nonneg("sigma", sigma)
        tau_s = _check_positive("tau_s", tau_s)
        dt = _check_positive("dt", dt)
        self.phi = float(np.exp(-dt / tau_s))
        self.s = sigma * float(np.sqrt(1.0 - self.phi * self.phi))
        self.x = sigma * np.asarray(eps0, dtype=float)

    def step(self, eps: np.ndarray) -> np.ndarray:
        self.x = self.phi * self.x + self.s * eps
        return self.x

    def run(self, eps_seq: np.ndarray) -> np.ndarray:
        """Advance len(eps_seq) steps at once; bit-exact with the step loop
        (lfilter DF2T does the same multiply/add per sample), chunk-safe."""
        zi = (self.phi * self.x)[None, ...]
        y, _ = lfilter([self.s], [1.0, -self.phi], eps_seq, axis=0, zi=zi)
        self.x = y[-1].copy()
        return y


class RandomWalk:
    """Bias random walk b[k] = b[k-1] + sigma sqrt(dt) eps[k], b[0] = 0."""

    def __init__(self, sigma: float, dt: float, shape: tuple):
        sigma = _check_nonneg("sigma", sigma)
        dt = _check_positive("dt", dt)
        self.scale = sigma * float(np.sqrt(dt))
        self.x = np.zeros(shape)

    def step(self, eps: np.ndarray) -> np.ndarray:
        self.x = self.x + eps * self.scale
        return self.x

    def run(self, eps_seq: np.ndarray) -> np.ndarray:
        """Vectorized step loop: scale-then-accumulate with the carried
        state as cumsum row 0, so chunked calls associate additions exactly
        like step() does."""
        buf = np.concatenate([self.x[None, ...], eps_seq * self.scale], axis=0)
        out = np.cumsum(buf, axis=0)[1:]
        self.x = out[-1].copy()
        return out


def avar_white(noise_density: float, tau) -> np.ndarray:
    """Allan variance of white noise with density N (units/sqrt(Hz))."""
    tau = np.asarray(tau, dtype=float)
    return noise_density**2 / tau


def avar_random_walk(rw_sigma: float, tau) -> np.ndarray:
    """Allan variance of a (rate) random walk with coefficient K (units/sqrt(s))."""
    tau = np.asarray(tau, dtype=float)
    return rw_sigma**2 * tau / 3.0


def avar_gauss_markov(gm_sigma: float, gm_tau_s: float, tau) -> np.ndarray:
    """Allan variance of a first-order Gauss-Markov process, derived from
    R(u) = sigma^2 e^(-|u|/tau_c) (module docstring). Limits: rate-random-
    walk-like (K = sigma sqrt(2/tau_c)) for tau << tau_c, white-like
    (N = sigma sqrt(2 tau_c)) for tau >> tau_c."""
    tau = np.asarray(tau, dtype=float)
    r = tau / gm_tau_s
    bracket = 2.0 - (3.0 - 4.0 * np.exp(-r) + np.exp(-2.0 * r)) / r
    return gm_sigma**2 / r * bracket
