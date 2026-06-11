"""Brushless motor + ESC rotor-speed dynamics, batched over (vehicle, rotor).

Average-value model (citations in docs/RESEARCH.md):

    V_m  = throttle * V_bus                  (ESC as ideal chopper)
    i    = (V_m - Ke w) / R_w                (armature electrics)
    J_r w_dot = Kt i - k_q w^2               (prop drag torque load k_q w^2)
    Ke = Kt = 60 / (2 pi KV_rpm)             (SI back-EMF = torque constant)

The electrical pole is fast against the mechanical one, so the armature is
quasi-static; the rotor speed responds with a linearized time constant
tau = J_r / (Kt Ke / R_w + 2 k_q w0) — tens of milliseconds for the
interceptor-class motor, pinned by tests to the 15-50 ms band. The
full-throttle speed ceiling solves Kt (V - Ke w)/R_w = k_q w^2 and therefore
tracks a sagging bus voltage (battery coupling).

Bus current drawn through the chopper: i_bus = sum_rotors(throttle * i).
Transient negative current (active braking) is passed through unclipped and
simply recharges the ECM battery; rotor speed itself is clamped at zero
(props cannot windmill backwards through the ESC).

Integration: midpoint RK2 per micro-step (dt << tau, latched before the
plant RK4 step per the micro-tick contract in docs/ORDERING.md).
"""

from __future__ import annotations

import numpy as np


class MotorEsc:
    """Batched motor/ESC bank: n vehicles x r rotors sharing one parameter set."""

    def __init__(self, n: int, rotors: int, kv_rpm_per_v: float, r_w: float,
                 j_r: float, k_q: float, omega0: float = 0.0):
        self.n = int(n)
        self.rotors = int(rotors)
        self.ke = 60.0 / (2.0 * np.pi * kv_rpm_per_v)   # V s/rad; equals Kt in N m/A
        self.r_w = float(r_w)
        self.j_r = float(j_r)
        self.k_q = float(k_q)
        self.omega = np.full((self.n, self.rotors), float(omega0))

    def _omega_dot(self, omega: np.ndarray, v_m: np.ndarray) -> np.ndarray:
        i = (v_m - self.ke * omega) / self.r_w
        return (self.ke * i - self.k_q * omega * omega) / self.j_r

    def step(self, dt: float, throttle: np.ndarray, v_bus: np.ndarray
             ) -> tuple[np.ndarray, np.ndarray]:
        """Advance rotor speeds one micro-step.

        throttle: (n, rotors) in [0, 1]; v_bus: (n,) volts.
        Returns (omega (n, rotors) rad/s, i_bus (n,) amps drawn from the pack).
        """
        v_m = np.clip(throttle, 0.0, 1.0) * v_bus[:, None]
        k1 = self._omega_dot(self.omega, v_m)
        k2 = self._omega_dot(self.omega + 0.5 * dt * k1, v_m)
        self.omega = np.maximum(self.omega + dt * k2, 0.0)
        i = (v_m - self.ke * self.omega) / self.r_w
        i_bus = np.sum(np.clip(throttle, 0.0, 1.0) * i, axis=1)
        return self.omega, i_bus
