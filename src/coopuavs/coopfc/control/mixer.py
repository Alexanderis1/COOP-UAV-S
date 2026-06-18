"""Quad-X mixer with sequential desaturation (PX4 priority order).

Normalized domain: collective ``thrust`` in [0, 1], torque demands
``(roll, pitch, yaw)`` in [-1, 1]; outputs are 4 motor commands in
[0, 1]. Axis scaling (physical torque per unit demand) lives in the
rate-loop gains, not here.

Sign table derived from the airframe wrench (physics/multirotor.py,
body FLU, rotor order FR, BL, FL, BR as in interceptor_quad.yaml):
``tau_x = sum(y_i T_i)`` so +roll raises the left rotors (y > 0: BL,
FL); ``tau_y = -sum(x_i T_i)`` so +pitch raises the rear (x < 0: BL,
BR); ``tau_z = -sum(spin_i km w_i^2)`` so +yaw raises the CW pair
(spin -1: FL, BR).

Desaturation: motors outside [0, 1] are recovered in strict priority —
roll/pitch (vehicle attitude) > collective > yaw (PX4
ControlAllocationSequentialDesaturation order [project knowledge,
standard convention]):

1. shift collective by the smallest amount that re-centres the band
   (never past what the opposite bound allows),
2. scale YAW toward zero until everything fits,
3. scale ROLL+PITCH jointly (last resort — only reachable when the
   roll/pitch demand alone exceeds the full band),
4. final clip (numerical safety only).

Flags report which stage acted: the rate loops use ``sat_lo/sat_hi``
for conditional anti-windup; persistent saturation is a CBIT seam.
"""

from __future__ import annotations

from typing import NamedTuple

from coopuavs.coopfc.core import vec

# Per-motor (roll, pitch, yaw) signs, rotor order FR, BL, FL, BR.
_SIGNS = (
    (-1.0, -1.0, -1.0),   # FR: right (y<0), front (x>0), CCW
    (+1.0, +1.0, -1.0),   # BL: left, rear, CCW
    (+1.0, -1.0, +1.0),   # FL: left, front, CW
    (-1.0, +1.0, +1.0),   # BR: right, rear, CW
)


class MixFlags(NamedTuple):
    """What desaturation had to do this tick (all-False/zero = clean).

    ``axis_sat`` is the rate-loop anti-windup feedback, per axis
    (roll, pitch, yaw): +1 = more positive demand cannot be realized
    (a motor that must rise sits at 1.0 or one that must fall sits at
    0.0), -1 = mirror, 2 = blocked both ways, 0 = free.
    """

    sat_lo: bool          # some motor demanded < 0 before recovery
    sat_hi: bool          # some motor demanded > 1 before recovery
    yaw_scaled: bool      # yaw authority reduced to protect roll/pitch
    rp_scaled: bool       # roll/pitch jointly scaled (extreme demand)
    axis_sat: tuple[int, int, int] = (0, 0, 0)


class QuadXMixer:
    """Stateless quad-X mixer; one instance per vehicle for symmetry with
    the stateful controllers."""

    def mix(self, thrust: float, torque: vec.Vec3
            ) -> tuple[tuple[float, float, float, float], MixFlags]:
        t = vec.clip(thrust, 0.0, 1.0)
        r, p, y = torque

        def spread(rr: float, pp: float, yy: float, tt: float):
            u = tuple(tt + rr * s[0] + pp * s[1] + yy * s[2] for s in _SIGNS)
            return u, min(u), max(u)

        u, lo, hi = spread(r, p, y, t)
        sat_lo, sat_hi = lo < 0.0, hi > 1.0
        yaw_scaled = rp_scaled = False

        if sat_lo or sat_hi:
            # 1. collective shift: centre the band, bounded so the shift
            # itself cannot push the opposite side out.
            if hi - lo <= 1.0:
                shift = -lo if lo < 0.0 else 1.0 - hi
                t += shift
                u, lo, hi = spread(r, p, y, t)
            else:
                # band wider than the actuator range: centre it, then
                # shed yaw and (only if still needed) roll/pitch.
                t += 0.5 * (1.0 - (hi + lo))
                u, lo, hi = spread(r, p, y, t)
                if (lo < 0.0 or hi > 1.0) and y != 0.0:
                    # largest k in [0,1] with the yaw-scaled band inside
                    # [0,1]; the band is linear in k, so solve per motor.
                    k = 1.0
                    for s in _SIGNS:
                        base = t + r * s[0] + p * s[1]
                        contrib = y * s[2]
                        if contrib > 0.0:
                            k = min(k, (1.0 - base) / contrib)
                        elif contrib < 0.0:
                            k = min(k, -base / contrib)
                    k = vec.clip(k, 0.0, 1.0)
                    if k < 1.0:
                        yaw_scaled = True
                        y *= k
                    u, lo, hi = spread(r, p, y, t)
                if lo < 0.0 or hi > 1.0:
                    # roll/pitch span exceeds the full band even with
                    # yaw shed and the band centred: scale jointly so
                    # the span exactly fits, re-centre, flag it.
                    span = hi - lo
                    if span > 1e-12:
                        g = 1.0 / span
                        r *= g
                        p *= g
                        y *= g
                        rp_scaled = True
                        u, lo, hi = spread(r, p, y, t)
                        t += 0.5 * (1.0 - (hi + lo))
                        u, lo, hi = spread(r, p, y, t)

        u = tuple(vec.clip(ui, 0.0, 1.0) for ui in u)
        axis_sat = []
        for a in range(3):
            inc = any((ui >= 1.0 and s[a] > 0.0) or (ui <= 0.0 and s[a] < 0.0)
                      for ui, s in zip(u, _SIGNS))
            dec = any((ui >= 1.0 and s[a] < 0.0) or (ui <= 0.0 and s[a] > 0.0)
                      for ui, s in zip(u, _SIGNS))
            axis_sat.append(2 if inc and dec else 1 if inc else -1 if dec
                            else 0)
        return u, MixFlags(sat_lo, sat_hi, yaw_scaled, rp_scaled,
                           (axis_sat[0], axis_sat[1], axis_sat[2]))
