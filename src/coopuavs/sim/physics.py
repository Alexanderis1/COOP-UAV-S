"""Minimal flight dynamics for the time-stepped simulation.

A deliberate physics-lite layer: point-mass kinematics with speed and
acceleration limits. Good enough to study cooperation, guidance geometry and
risk-aware engagement; swapped for PX4 SITL dynamics on ROS 2 migration.
"""

from __future__ import annotations

import numpy as np

GRAVITY = 9.81


class PointMass:
    """Point-mass vehicle with saturated acceleration toward a commanded
    velocity vector."""

    def __init__(
        self,
        position: np.ndarray,
        velocity: np.ndarray | None = None,
        max_speed: float = 30.0,
        max_accel: float = 10.0,
    ):
        self.position = np.asarray(position, dtype=float).copy()
        self.velocity = (
            np.asarray(velocity, dtype=float).copy() if velocity is not None else np.zeros(3)
        )
        self.max_speed = max_speed
        self.max_accel = max_accel
        self.cmd_velocity = self.velocity.copy()

    def command_velocity(self, v_cmd: np.ndarray) -> None:
        speed = np.linalg.norm(v_cmd)
        if speed > self.max_speed:
            v_cmd = v_cmd * (self.max_speed / speed)
        self.cmd_velocity = np.asarray(v_cmd, dtype=float)

    def step(self, dt: float) -> None:
        dv = self.cmd_velocity - self.velocity
        dv_norm = np.linalg.norm(dv)
        max_dv = self.max_accel * dt
        if dv_norm > max_dv:
            dv = dv * (max_dv / dv_norm)
        self.velocity = self.velocity + dv
        self.position = self.position + self.velocity * dt


def time_to_go(p_rel: np.ndarray, v_rel: np.ndarray) -> float:
    """Closest-point-of-approach time for constant velocities (>= 0)."""
    vv = float(v_rel @ v_rel)
    if vv < 1e-9:
        return 0.0
    return max(0.0, -float(p_rel @ v_rel) / vv)
