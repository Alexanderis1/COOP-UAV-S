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

        # Fall time: constant-accel until terminal velocity, then constant.
        t_term = TERMINAL_FALL_SPEED / GRAVITY
        d_term = 0.5 * GRAVITY * t_term**2
        if alt <= d_term:
            t_fall = np.sqrt(2.0 * alt / GRAVITY)
        else:
            t_fall = t_term + (alt - d_term) / TERMINAL_FALL_SPEED

        v_xy = np.asarray(target_vel[:2], dtype=float) * retention
        # Per-sample randomness: retention jitter and growing lateral spread.
        retention_jitter = self.rng.normal(1.0, 0.25, size=(n, 1))
        carry = v_xy[None, :] * retention_jitter * t_fall
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
