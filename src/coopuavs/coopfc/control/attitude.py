"""Quaternion attitude P controller (400 Hz, plain-float).

rate_sp = 2 kp * vec(q_err) with q_err = q^-1 (x) q_sp taken on the
shortest path (sign-flip on negative scalar part) — the proportional
quaternion law of Brescianini, Hehn & D'Andrea, *Nonlinear quadrocopter
attitude control* (ETH Zurich tech report, 2013), as used by PX4. For
small errors 2 vec(q_err) is the rotation-vector error, so kp is 1/tau
of the closed attitude loop.

Yaw error is weighted down (yaw_weight < 1): tilt is what keeps the
thrust vector pointed, and the yaw axis has ~30x less actuation
authority on this airframe — full-priority yaw would steal roll/pitch
budget in the mixer for nothing. Output clamped to per-axis rate
limits (the rate loop's tracking envelope).
"""

from __future__ import annotations

from typing import NamedTuple

from coopuavs.coopfc.core import vec


class AttParams(NamedTuple):
    kp: float = 8.0                          # 1/s, tilt axes
    yaw_weight: float = 0.4                  # relative yaw priority
    rate_max: vec.Vec3 = (4.0, 4.0, 1.5)     # rad/s setpoint clamp


class AttCtl:
    """Stateless P law; class for symmetry with the stateful loops."""

    def __init__(self, params: AttParams = AttParams()):
        self.params = params

    def update(self, q_sp: vec.Quat, q: vec.Quat) -> vec.Vec3:
        p = self.params
        qe = vec.quat_multiply(vec.quat_conjugate(q), q_sp)
        if qe[0] < 0.0:                      # shortest path
            qe = (-qe[0], -qe[1], -qe[2], -qe[3])
        return (
            vec.clip(2.0 * p.kp * qe[1], -p.rate_max[0], p.rate_max[0]),
            vec.clip(2.0 * p.kp * qe[2], -p.rate_max[1], p.rate_max[1]),
            vec.clip(2.0 * p.kp * p.yaw_weight * qe[3],
                     -p.rate_max[2], p.rate_max[2]),
        )
