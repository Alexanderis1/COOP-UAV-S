"""Body-rate PID (innermost loop, 400 Hz, plain-float).

Per-axis ``out = kp e + ki int(e) - kd d(meas)/dt`` in normalized torque
[-1, 1] (PID-on-measurement derivative: no setpoint kick; first-order
LPF on the derivative). Anti-windup is conditional integration
(Astrom & Hagglund, *Advanced PID Control*, ISA 2006, ch. 3: stop
integrating while the loop is saturated in the direction the error
pushes) against BOTH the controller's own output clip and the mixer's
actuator-level saturation flags fed back by the FCU — integrator
clamping alone recovers late after long ramps into saturation (the
P3-5 ramp-recovery test pins the difference).

Defaults are tuned for `interceptor_quad` closed-loop through the P1
powertrain (motor lag ~20 ms inside the loop): normalized roll/pitch
authority ~300 rad/s^2 per unit demand at hover, yaw ~10 rad/s^2 —
yaw is physically ~30x weaker (drag-torque vs thrust-moment actuation),
hence the asymmetric gains and the gentler yaw acceptance test.
"""

from __future__ import annotations

import math
from typing import NamedTuple

from coopuavs.coopfc.core import vec


class RateParams(NamedTuple):
    kp: vec.Vec3 = (0.15, 0.15, 1.50)
    ki: vec.Vec3 = (1.20, 1.20, 4.00)     # 1/s on the error integral
    kd: vec.Vec3 = (0.0025, 0.0025, 0.0)  # s, on measured rate
    i_lim: vec.Vec3 = (0.30, 0.30, 0.50)  # integrator clamp, output units
    d_cutoff_hz: float = 40.0             # derivative LPF


class RateCtl:
    """One vehicle's rate loop; persistent integrator/derivative state."""

    def __init__(self, params: RateParams = RateParams()):
        self.params = params
        self.i = [0.0, 0.0, 0.0]
        self._d = [0.0, 0.0, 0.0]          # filtered derivative state
        self._last = [0.0, 0.0, 0.0]
        self._have_last = False

    def reset(self) -> None:
        self.i = [0.0, 0.0, 0.0]
        self._d = [0.0, 0.0, 0.0]
        self._have_last = False

    def update(self, sp: vec.Vec3, meas: vec.Vec3, dt: float,
               sat: tuple[int, int, int] = (0, 0, 0)) -> vec.Vec3:
        """One tick. `sat` is MixFlags.axis_sat from the previous tick:
        +1 = positive demand blocked, -1 = negative blocked, 2 = both."""
        p = self.params
        alpha = dt / (dt + 1.0 / (2.0 * math.pi * p.d_cutoff_hz))
        out = [0.0, 0.0, 0.0]
        for a in range(3):
            e = sp[a] - meas[a]
            if self._have_last:
                raw_d = (meas[a] - self._last[a]) / dt
                self._d[a] += alpha * (raw_d - self._d[a])
            self._last[a] = meas[a]
            u_unsat = p.kp[a] * e + self.i[a] - p.kd[a] * self._d[a]
            u = vec.clip(u_unsat, -1.0, 1.0)
            # Conditional integration: own clip or mixer saturation in
            # the direction the error keeps pushing freezes the term.
            pushing_hi = e > 0.0 and (u_unsat >= 1.0 or sat[a] >= 1)
            pushing_lo = e < 0.0 and (u_unsat <= -1.0 or sat[a] <= -1
                                      or sat[a] == 2)
            if not (pushing_hi or pushing_lo):
                self.i[a] = vec.clip(self.i[a] + p.ki[a] * e * dt,
                                     -p.i_lim[a], p.i_lim[a])
            out[a] = u
        self._have_last = True
        return (out[0], out[1], out[2])
