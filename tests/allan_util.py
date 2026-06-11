"""Overlapping Allan deviation estimator for the P2 hw sensor suites.

Test-side analysis helper (like golden_util): the analytic Allan variance
of each configured noise component lives with the model in
``coopuavs.hw.stoch`` (the dryden.analytic_psd precedent); this module only
provides the empirical estimator the @slow Allan tests compare against.

Estimator [IEEE Std 952-1997, Annex C; El-Sheimy et al. 2008, IEEE TIM
57(1)]: with theta the cumulative integral of the rate series y at step dt,
the fully-overlapping Allan variance at averaging time tau = m dt is

    AVAR(m) = 1 / (2 (m dt)^2 (K - 2m)) *
              sum_k (theta[k + 2m] - 2 theta[k + m] + theta[k])^2
"""

from __future__ import annotations

import numpy as np


def oadev(y: np.ndarray, dt: float, m_factors) -> np.ndarray:
    """Overlapping Allan deviation of 1-D rate series y at averaging factors m.

    Returns sigma(tau) for tau = m * dt, one value per entry of m_factors.
    """
    y = np.asarray(y, dtype=float)
    if y.ndim != 1:
        raise ValueError(f"oadev expects a 1-D series, got shape {y.shape}")
    theta = np.empty(y.size + 1)
    theta[0] = 0.0
    np.cumsum(y, out=theta[1:])
    theta *= dt
    out = np.empty(len(m_factors))
    n = theta.size
    for j, m in enumerate(m_factors):
        m = int(m)
        if not 1 <= m <= (n - 1) // 2:
            raise ValueError(f"averaging factor m={m} needs 2m+1 <= {n} phase points")
        d = theta[2 * m:] - 2.0 * theta[m:n - m] + theta[:n - 2 * m]
        out[j] = np.sqrt(np.mean(d * d) / (2.0 * (m * dt) ** 2))
    return out
