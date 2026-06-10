"""Onboard effectors: net gun and kinetic projectile.

Probabilistic engagement model (no ballistic flyout in v0): each effector
has an engagement envelope and a kill-probability surface over range,
off-boresight angle and closing speed. The same object is used by the
shooter (to decide it is in parameters and report expected Pk) and by the
sim-side adjudicator (to roll the truth outcome).

Nets favour low closing speeds and short range but drop the target almost
vertically (debris-friendly); projectiles reach further and tolerate speed
but throw the wreck forward — the ROE sees that difference through
``risk.debris._VELOCITY_RETENTION``.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..core.messages import EffectorType


@dataclass
class Effector:
    type: EffectorType
    max_range: float
    optimal_range: float
    max_off_axis_deg: float
    max_closing_speed: float
    pk_max: float
    reload_time: float
    ammo: int

    def in_envelope(self, rel_pos: np.ndarray, own_vel: np.ndarray, target_vel: np.ndarray) -> bool:
        return self.p_kill(rel_pos, own_vel, target_vel) > 0.0

    def p_kill(self, rel_pos: np.ndarray, own_vel: np.ndarray, target_vel: np.ndarray) -> float:
        rng = float(np.linalg.norm(rel_pos))
        if rng > self.max_range or rng < 1.0:
            return 0.0

        own_speed = float(np.linalg.norm(own_vel))
        if own_speed > 1e-6:
            cos_off = float(own_vel @ rel_pos) / (own_speed * rng)
            off_axis = np.degrees(np.arccos(np.clip(cos_off, -1.0, 1.0)))
        else:
            off_axis = 180.0
        if off_axis > self.max_off_axis_deg:
            return 0.0

        closing = -float((target_vel - own_vel) @ rel_pos) / rng
        if closing > self.max_closing_speed:
            return 0.0

        # Pk surface: best at optimal range and boresight, degrading smoothly.
        range_f = 1.0 if rng <= self.optimal_range else (
            1.0 - 0.6 * (rng - self.optimal_range) / (self.max_range - self.optimal_range)
        )
        angle_f = 1.0 - 0.5 * (off_axis / self.max_off_axis_deg)
        speed_f = 1.0 - 0.4 * max(0.0, closing) / self.max_closing_speed
        return float(np.clip(self.pk_max * range_f * angle_f * speed_f, 0.0, 1.0))


def net_gun() -> Effector:
    return Effector(
        type=EffectorType.NET, max_range=40.0, optimal_range=18.0,
        max_off_axis_deg=25.0, max_closing_speed=45.0,
        pk_max=0.80, reload_time=6.0, ammo=2,
    )


def projectile_gun() -> Effector:
    return Effector(
        type=EffectorType.PROJECTILE, max_range=200.0, optimal_range=80.0,
        max_off_axis_deg=25.0, max_closing_speed=250.0,
        pk_max=0.65, reload_time=1.5, ammo=8,
    )


EFFECTOR_FACTORIES = {"net": net_gun, "projectile": projectile_gun}
