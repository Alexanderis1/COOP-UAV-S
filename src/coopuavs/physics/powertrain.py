"""Implicit motor-battery DC bus coupling: closed-form solve of the
algebraic loop, bus current limit and cell-voltage bounds.

Why implicit: the quasi-static armature (motor.py) makes pack current an
instantaneous function of bus voltage (d i_bus / d v_bus = sum_r(theta_r^2)
/ R_w) and the ECM battery (battery.py) feeds terminal voltage back through
R0 instantaneously, so composing the two models explicitly (one-step-lag
v_bus) is a fixed-point iteration with loop gain

    g = R0 sum_r(theta_r^2) / R_w        (= 3.6 theta^2 for interceptor_quad)

which diverges geometrically for g > 1 (above ~hover throttle for the
shipped parameter sets) at ANY dt — the instability is algebraic, not
stiffness, so no micro-step cures it. Pinned by
tests/test_motor_battery.py::test_explicit_lagged_composition_diverges_powertrain_does_not.

At each micro-step the bus equations, with rotor speeds omega and battery
state (SOC, V1) frozen at their pre-step values,

    i_bus = (sum_r(theta_r^2) v_bus - Ke sum_r(theta_r omega_r)) / R_w
    v_bus = OCV(SOC) - V1 - R0 i_bus

are solved simultaneously in closed form, batched over (N,) vehicles x R
rotors:

    v_bus = (OCV - V1 + (R0 / R_w) Ke sum_r(theta_r omega_r))
            / (1 + R0 sum_r(theta_r^2) / R_w)

step() then applies, in order:

1. Bus current limit: i_bus clamped to [-i_bus_max_a, +i_bus_max_a]
   (ESC/BMS limit; the airframe YAMLs size it ~1.5x the self-consistent
   steady full-throttle pack draw). Post-clamp bus voltage is the battery
   terminal voltage at the clamped current, v_bus = OCV - V1 - R0 i_clamped.
   ESC duty rollback is not modelled, so motor-side electrical power can
   transiently exceed the pack-side accounting while the limiter is active
   (spin-up inrush, hard throttle chops); the pack never sources or sinks
   more than the limit.
2. Cell-voltage bounds: v_bus clamped to [V_CELL_MIN, V_CELL_MAX] per cell
   (LiPo discharge cutoff floor / charge ceiling). BatteryEcm itself stays
   pure and unbounded; the envelope is enforced here only.
3. State advance: motor.step(dt, throttle, v_bus) integrates rotor speeds
   and battery.step(dt, i_bus) integrates SOC/V1. The pack current uses the
   pre-step omega, consistent with the algebraic solve; the O(dt) difference
   against the post-step armature current is ordinary explicit-integration
   error, stable because the feedthrough loop itself is solved implicitly.
"""

from __future__ import annotations

import numpy as np

from coopuavs.physics.battery import BatteryEcm
from coopuavs.physics.motor import MotorEsc

V_CELL_MIN = 3.0   # V/cell, LiPo discharge cutoff floor enforced on the bus
V_CELL_MAX = 4.2   # V/cell, LiPo charge ceiling enforced on the bus


class Powertrain:
    """Motor/ESC bank + battery pack behind one implicitly-solved DC bus."""

    def __init__(self, motor: MotorEsc, battery: BatteryEcm,
                 i_bus_max_a: float):
        if motor.n != battery.n:
            raise ValueError(
                f"motor bank n={motor.n} != battery pack n={battery.n}")
        if not i_bus_max_a > 0.0:
            raise ValueError(f"i_bus_max_a must be > 0, got {i_bus_max_a!r}")
        self.motor = motor
        self.battery = battery
        self.i_bus_max_a = float(i_bus_max_a)
        self.v_bus_min = V_CELL_MIN * battery.n_series
        self.v_bus_max = V_CELL_MAX * battery.n_series

    def solve_bus(self, throttle: np.ndarray
                  ) -> tuple[np.ndarray, np.ndarray]:
        """Unconstrained fixed point (v_bus (n,), i_bus (n,)) of the bus loop.

        Evaluated at the current (pre-step) rotor speeds and battery state.
        Pure (no state change); the current/voltage clamps live in step().
        """
        m, b = self.motor, self.battery
        theta = np.clip(throttle, 0.0, 1.0)
        s = np.sum(theta * theta, axis=1)
        bemf = m.ke * np.sum(theta * m.omega, axis=1)
        emf = b.ocv(b.soc) - b.v1
        v_bus = (emf + (b.r0 / m.r_w) * bemf) / (1.0 + b.r0 * s / m.r_w)
        i_bus = (s * v_bus - bemf) / m.r_w
        return v_bus, i_bus

    def step(self, dt: float, throttle: np.ndarray
             ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Advance one micro-step under (n, r) throttle in [0, 1].

        Returns (omega (n, r) rad/s, v_bus (n,) V, i_bus (n,) A drawn from
        the pack; negative = regenerative charge).
        """
        b = self.battery
        _, i_star = self.solve_bus(throttle)
        i_bus = np.clip(i_star, -self.i_bus_max_a, self.i_bus_max_a)
        v_bus = b.ocv(b.soc) - b.v1 - b.r0 * i_bus
        v_bus = np.clip(v_bus, self.v_bus_min, self.v_bus_max)
        omega, _ = self.motor.step(dt, throttle, v_bus)
        b.step(dt, i_bus)
        return omega, v_bus, i_bus
