"""Batched multirotor plant: rotor thrust/torque allocation, Cheeseman-Bennett
ground effect, Faessler rotor drag, parasitic drag, gravity.

Forces and moments (citations in docs/RESEARCH.md):

- Rotor thrust  T_i = kf w_i^2  along body +z (FLU up); yaw reaction torque
  tau_z = -sum(spin_i km w_i^2) with spin_i = +1 for CCW viewed from above
  [Mahony, Kumar & Corke 2012, IEEE RAM].
- Geometric moments tau = sum(r_i x T_i e_z): tau_x = sum(y_i T_i),
  tau_y = -sum(x_i T_i).
- Ground effect (in-ground-effect thrust gain at height z over rotor radius R)
  T_IGE / T_OGE = 1 / (1 - (R / 4z)^2), clamped to [1, max_gain]
  [Cheeseman & Bennett 1955, ARC R&M 3021]. Applied per vehicle using the
  CoM altitude as rotor-plane height; the torque coefficient km is left
  uncorrected (thrust-only correction).
- Rotor drag, linear in body-frame airspeed: f_b = -D v_air_body with
  D = diag(dx, dy, dz) lumped over the rotor set
  [Faessler, Franchi & Scaramuzza 2018, IEEE RAL].
- Parasitic drag, isotropic quadratic in world frame:
  f = -1/2 rho CdA |v_air| v_air (the 80 m/s dash terminal-speed knob).
- Gravity m g down is included here: the rigid-body integrator is wrench-pure.

Deviations (documented): rotor gyroscopic torque J_r w_i (omega x e_z) and
rotor-acceleration reaction torque are neglected (small against the
geometric moments for this airframe; revisit if the P1-7 oracle attitude
RMSE budget says otherwise).

Rotor speeds are inputs (latched by the motor model between plant RK4
steps); state-dependent terms (drag, ground effect) are re-evaluated every
RK4 stage via ``wrench_fn``.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from coopuavs.physics import GRAVITY
from coopuavs.physics import rigid_body as rb


@dataclass(frozen=True)
class MultirotorParams:
    """Immutable airframe parameter set (see params/interceptor_quad.yaml)."""

    name: str
    mass: float
    inertia: np.ndarray              # (3, 3) body FLU
    n_rotors: int
    rotor_radius: float
    kf: float
    km: float
    rotor_positions: np.ndarray      # (r, 3) body FLU
    rotor_spin: np.ndarray           # (r,) +1 CCW / -1 CW
    drag_linear_diag: np.ndarray     # (3,) N/(m/s)
    cda_iso: float                   # m^2
    ground_effect_max_gain: float
    motor: dict = field(default_factory=dict)    # MotorEsc kwargs (k_q = km)
    battery: dict = field(default_factory=dict)  # BatteryEcm kwargs

    @classmethod
    def from_dict(cls, cfg: dict) -> "MultirotorParams":
        rotors = cfg["rotors"]
        motor = dict(cfg["motor"])
        motor["k_q"] = float(rotors["km"])
        return cls(
            name=cfg["name"],
            mass=float(cfg["mass"]),
            inertia=np.diag(np.asarray(cfg["inertia_diag"], dtype=float)),
            n_rotors=int(rotors["count"]),
            rotor_radius=float(rotors["radius"]),
            kf=float(rotors["kf"]),
            km=float(rotors["km"]),
            rotor_positions=np.asarray(rotors["positions"], dtype=float),
            rotor_spin=np.asarray(rotors["spin"], dtype=float),
            drag_linear_diag=np.asarray(cfg["drag"]["linear_diag"], dtype=float),
            cda_iso=float(cfg["drag"]["cda_iso"]),
            ground_effect_max_gain=float(cfg["ground_effect"]["max_gain"]),
            motor=motor,
            battery=dict(cfg["battery"]),
        )


class MultirotorPlant:
    """N identical multirotors; wrench evaluation + RK4 step convenience."""

    def __init__(self, params: MultirotorParams, n: int):
        self.params = params
        self.n = int(n)
        self.mass = np.full(self.n, params.mass)
        self.inertia = np.repeat(params.inertia[None, :, :], self.n, axis=0)
        self.inertia_inv = np.linalg.inv(self.inertia)

    def _ground_effect(self, z: np.ndarray) -> np.ndarray:
        """Cheeseman-Bennett thrust gain, clamped to [1, max_gain]."""
        p = self.params
        x2 = (p.rotor_radius / (4.0 * np.maximum(z, 1e-6))) ** 2
        denom = 1.0 - x2
        gain = np.where(denom > 1e-9, 1.0 / np.maximum(denom, 1e-9),
                        p.ground_effect_max_gain)
        return np.clip(gain, 1.0, p.ground_effect_max_gain)

    def wrench(self, state: np.ndarray, rotor_omega: np.ndarray,
               wind_world: np.ndarray, rho) -> tuple[np.ndarray, np.ndarray]:
        """Total (force_world (n,3), torque_body (n,3)) incl. gravity.

        rotor_omega: (n, r) rad/s; wind_world: (n, 3) m/s; rho: air density
        (scalar or (n,)).
        """
        p = self.params
        quat = state[:, rb.QUAT]
        w2 = rotor_omega * rotor_omega
        thrust = p.kf * w2 * self._ground_effect(state[:, 2])[:, None]   # (n, r)

        tau = np.empty((self.n, 3))
        tau[:, 0] = thrust @ p.rotor_positions[:, 1]
        tau[:, 1] = -(thrust @ p.rotor_positions[:, 0])
        tau[:, 2] = -(p.km * w2) @ p.rotor_spin

        v_air_world = state[:, rb.VEL] - wind_world
        v_air_body = rb.quat_rotate(rb.quat_conjugate(quat), v_air_world)

        f_body = -p.drag_linear_diag * v_air_body                        # Faessler
        f_body[:, 2] += thrust.sum(axis=1)
        force = rb.quat_rotate(quat, f_body)
        speed = np.linalg.norm(v_air_world, axis=1, keepdims=True)
        force -= 0.5 * np.asarray(rho).reshape(-1, 1) * p.cda_iso * speed * v_air_world
        force[:, 2] -= self.mass * GRAVITY
        return force, tau

    def wrench_fn(self, rotor_omega: np.ndarray, wind_world: np.ndarray, rho):
        """Closure for rigid_body.rk4_step: inputs zero-order-held, state live."""
        return lambda s: self.wrench(s, rotor_omega, wind_world, rho)

    def step(self, state: np.ndarray, dt: float, rotor_omega: np.ndarray,
             wind_world: np.ndarray, rho) -> np.ndarray:
        return rb.rk4_step(state, dt, self.wrench_fn(rotor_omega, wind_world, rho),
                           self.mass, self.inertia, self.inertia_inv)
