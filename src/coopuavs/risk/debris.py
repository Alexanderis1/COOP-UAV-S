"""Probabilistic crash/debris footprint model.

When an effector defeats a target at altitude, the wreck does not vanish —
it falls, and where it falls is the safety-critical question in a populated
area. The model used here is a sampled ballistic envelope:

* the wreck keeps a (mechanism-dependent) fraction of its horizontal
  velocity — a net-wrapped quadcopter mostly drops, a projectile-hit OWA
  airframe keeps tumbling forward;
* fall time comes from the intercept altitude under gravity with a terminal
  velocity cap;
* lateral dispersion grows with altitude and impact speed.

``footprint`` returns Monte-Carlo ground impact samples that the
:class:`~coopuavs.risk.zones.RiskMap` converts into expected collateral cost.
The same model is used twice: *before* a shot inside ROE authorisation, and
*after* a simulated kill to place the wreck in the world.
"""

from __future__ import annotations

import numpy as np

from ..core.messages import EffectorType
from ..sim.physics import GRAVITY

# Fraction of horizontal velocity the wreck retains after the kill.
_VELOCITY_RETENTION = {
    EffectorType.NET: 0.15,
    EffectorType.PROJECTILE: 0.65,
}

TERMINAL_FALL_SPEED = 45.0  # m/s, tumbling airframe


def velocity_retention(effector: EffectorType) -> float:
    """Fraction of horizontal velocity a wreck keeps for this mechanism."""
    return _VELOCITY_RETENTION[effector]


def retention_jitter(rng: np.random.Generator, size=None) -> np.ndarray:
    """Multiplicative jitter on velocity retention. Clamped: an unclamped
    normal tail yields wrecks flying backwards (negative retention) or
    carrying twice the airframe's horizontal speed."""
    return np.clip(rng.normal(1.0, 0.25, size=size), 0.0, 2.0)


def fall_time(alt: float, v_down0: float = 0.0) -> float:
    """Time for a wreck to fall ``alt`` metres: gravity-accelerated from an
    initial downward speed until the terminal velocity cap, then constant.
    Shared by the predictive footprint and the live debris objects
    (SIM-DEB-001) so both views of one fall agree."""
    if alt <= 0.0:
        return 0.0
    v0 = min(max(v_down0, 0.0), TERMINAL_FALL_SPEED)
    t_term = (TERMINAL_FALL_SPEED - v0) / GRAVITY
    d_term = v0 * t_term + 0.5 * GRAVITY * t_term**2
    if alt <= d_term:
        return float((-v0 + np.sqrt(v0**2 + 2.0 * GRAVITY * alt)) / GRAVITY)
    return float(t_term + (alt - d_term) / TERMINAL_FALL_SPEED)


class DebrisModel:
    def __init__(self, rng: np.random.Generator, n_samples: int = 256):
        self.rng = rng
        self.n_samples = n_samples

    def footprint(
        self,
        intercept_pos: np.ndarray,
        target_vel: np.ndarray,
        effector: EffectorType,
        n_samples: int | None = None,
    ) -> np.ndarray:
        """Sample ground impact points, returned as array of shape (N, 2)."""
        n = n_samples or self.n_samples
        alt = max(0.0, float(intercept_pos[2]))
        retention = _VELOCITY_RETENTION[effector]
        t_fall = fall_time(alt)

        v_xy = np.asarray(target_vel[:2], dtype=float) * retention
        # Per-sample randomness: retention jitter and growing lateral spread.
        carry = v_xy[None, :] * retention_jitter(self.rng, size=(n, 1)) * t_fall
        sigma = 0.15 * alt + 0.5 * np.linalg.norm(v_xy) * t_fall * 0.2 + 5.0
        spread = self.rng.normal(0.0, sigma, size=(n, 2))
        return intercept_pos[None, :2] + carry + spread

    def sample_impact(
        self,
        intercept_pos: np.ndarray,
        target_vel: np.ndarray,
        effector: EffectorType,
    ) -> np.ndarray:
        """Draw the single 'realised' wreck impact point after a kill."""
        return self.footprint(intercept_pos, target_vel, effector, n_samples=1)[0]
