"""Position P loop -> velocity setpoint (50 Hz outer-outer loop).

``vel_sp = kp (pos_sp - pos)`` with the horizontal magnitude clamped
(direction preserved — the vehicle flies the straight line home, it
does not crab axis-wise) and the vertical component clamped
asymmetrically. The plain P-to-velocity cascade is the PX4
PositionControl structure [standard reference]; zero steady-state
position error needs no integrator here because the velocity loop
below already carries one.
"""

from __future__ import annotations

import math
from typing import NamedTuple

from coopuavs.coopfc.core import vec


class PosParams(NamedTuple):
    kp: float = 1.0            # 1/s
    vel_max_h: float = 15.0    # m/s horizontal magnitude
    vel_max_up: float = 5.0    # m/s
    vel_max_down: float = 3.0  # m/s


class PosCtl:
    """Stateless P law; class for symmetry with the stateful loops."""

    def __init__(self, params: PosParams = PosParams()):
        self.params = params

    def update(self, pos_sp: vec.Vec3, pos: vec.Vec3) -> vec.Vec3:
        p = self.params
        vx = p.kp * (pos_sp[0] - pos[0])
        vy = p.kp * (pos_sp[1] - pos[1])
        vz = p.kp * (pos_sp[2] - pos[2])
        vh = math.hypot(vx, vy)
        if vh > p.vel_max_h:
            s = p.vel_max_h / vh
            vx *= s
            vy *= s
        return (vx, vy, vec.clip(vz, -p.vel_max_down, p.vel_max_up))
