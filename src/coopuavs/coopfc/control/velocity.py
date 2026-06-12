"""World-frame velocity PI -> (attitude setpoint, normalized thrust).

50 Hz outer loop (plain-float). Velocity error (world ENU) drives an
acceleration setpoint (PI; conditional anti-windup as in rate.py);
adding gravity gives the specific-force demand f the thrust vector must
realize. Acceleration-to-attitude per the differential-flatness thrust
direction map (Mellinger & Kumar, ICRA 2011 / PX4 PositionControl):
desired body z = f/|f|, yaw free — solved in the yaw frame as
ZYX euler (R e_z = (sin(th) cos(ph), -sin(ph), cos(th) cos(ph)) in that
frame, so ph = -asin(f_y), th = atan2(f_x, f_z)).

Priorities under limits (PX4 convention): vertical before horizontal
(horizontal accel is scaled down by the tilt limit before the vertical
demand is ever touched); thrust magnitude maps through the quadratic
rotor curve u = u_hover sqrt(|f|/g) (T ~ omega^2, omega ~ linear in
command — the PX4 thrust-model linearization).

Defaults match `interceptor_quad` by value: u_hover = 0.463 from the
steady-state armature chain at the full-pack bus (omega_h 738 rad/s ->
V_m = Ke w + R i = 22.77 V; bus = OCV 50.4 V minus R0 sag at the
~34 A hover draw = 49.2 V); the velocity integrator absorbs the
residual (pack sag, SOC, R1 charge-up) — that residual is what the
zero-steady-state-error test pins.
"""

from __future__ import annotations

import math
from typing import NamedTuple

from coopuavs.coopfc import GRAVITY
from coopuavs.coopfc.core import vec


class VelParams(NamedTuple):
    kp: float = 4.0                    # 1/s
    ki: float = 2.0                    # 1/s^2
    i_lim: float = 5.0                 # m/s^2 per axis
    a_max_h: float = 12.0              # m/s^2 horizontal demand
    a_max_up: float = 14.0             # m/s^2
    a_max_down: float = 8.0            # m/s^2
    tilt_max: float = math.radians(45.0)
    u_hover: float = 0.463             # normalized hover command
    u_min: float = 0.05                # keep rotors spinning
    u_max: float = 0.95                # headroom for the rate loop
    # Attitude-setpoint slew, rad/s per tilt axis. During a hard vertical
    # brake fz is small, so the tilt cone lets a cone-saturating
    # horizontal demand command ±tilt_max — and a sign-flipping error
    # then steps the setpoint ±45° at the loop rate. The rate loop slams
    # torque chasing steps no airframe can follow and the mixer's
    # rp-priority desat drags average collective back to hover: the
    # vertical brake is lost (P4 gate-review finding, user decision
    # 2026-06-12). Slewing the setpoint keeps it followable; 6 rad/s
    # (~344°/s) is far above every P3 maneuver spec (30° step in
    # ~90 ms) and only engages on pathological steps.
    tilt_slew: float = 6.0


class VelCtl:
    """One vehicle's velocity loop; persistent integrator."""

    def __init__(self, params: VelParams = VelParams()):
        self.params = params
        self.i = [0.0, 0.0, 0.0]
        self._rp = [0.0, 0.0]          # last slewed (roll, pitch) setpoint

    def reset(self) -> None:
        self.i = [0.0, 0.0, 0.0]
        self._rp = [0.0, 0.0]

    def update(self, v_sp: vec.Vec3, v: vec.Vec3, yaw_sp: float, dt: float
               ) -> tuple[vec.Quat, float]:
        p = self.params
        e = (v_sp[0] - v[0], v_sp[1] - v[1], v_sp[2] - v[2])

        a = [p.kp * e[0] + self.i[0],
             p.kp * e[1] + self.i[1],
             p.kp * e[2] + self.i[2]]

        # Limits: vertical clamp; horizontal magnitude clamp, then the
        # tilt limit (vertical demand wins; horizontal sheds).
        az = vec.clip(a[2], -p.a_max_down, p.a_max_up)
        ah = math.hypot(a[0], a[1])
        scale = 1.0
        if ah > p.a_max_h:
            scale = p.a_max_h / ah
        fz = az + GRAVITY
        ah_tilt = max(fz, 0.1) * math.tan(p.tilt_max)
        if ah * scale > ah_tilt:
            scale = ah_tilt / ah
        ax, ay = a[0] * scale, a[1] * scale

        # Conditional integration per axis (clip direction freezes it).
        clipped_h = scale < 1.0
        for axis, (acc, err) in enumerate(((ax, e[0]), (ay, e[1]))):
            pushing = clipped_h and (err > 0.0) == (acc > 0.0) and err != 0.0
            if not pushing:
                self.i[axis] = vec.clip(self.i[axis] + p.ki * err * dt,
                                        -p.i_lim, p.i_lim)
        pushing_z = (az != a[2]) and (e[2] > 0.0) == (az > 0.0)
        if not pushing_z:
            self.i[2] = vec.clip(self.i[2] + p.ki * e[2] * dt,
                                 -p.i_lim, p.i_lim)

        # Specific force -> attitude setpoint (yaw frame euler solve).
        f = (ax, ay, fz)
        f_n = vec.v3_norm(f)
        fhat = vec.v3_normalize(f) if f_n > 1e-9 else (0.0, 0.0, 1.0)
        cy, sy = math.cos(-yaw_sp), math.sin(-yaw_sp)
        fl = (cy * fhat[0] - sy * fhat[1],
              sy * fhat[0] + cy * fhat[1], fhat[2])
        roll = -math.asin(vec.clip(fl[1], -1.0, 1.0))
        pitch = math.atan2(fl[0], fl[2])
        # Followable-setpoint slew (see tilt_slew in VelParams).
        step = p.tilt_slew * dt
        roll = vec.clip(roll, self._rp[0] - step, self._rp[0] + step)
        pitch = vec.clip(pitch, self._rp[1] - step, self._rp[1] + step)
        self._rp = [roll, pitch]
        q_sp = vec.quat_from_euler(roll, pitch, yaw_sp)

        u = p.u_hover * math.sqrt(max(f_n, 0.0) / GRAVITY)
        return q_sp, vec.clip(u, p.u_min, p.u_max)
