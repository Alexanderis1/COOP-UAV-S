"""Batched fixed-wing aerodynamics per Beard & McLain, "Small Unmanned
Aircraft" (2012), chapter 4 — used by the 6DOF threat models (P6).

The B&M equations are written in NED/FRD. To keep them source-traceable
verbatim, this module computes air data, aero forces and moments in FRD and
maps to/from the package FLU body frame with the single proper rotation
M = diag(1, -1, -1) (180 deg about x; det = +1, so vectors and moment
pseudo-vectors transform identically): v_frd = M v_flu, f_flu = M f_frd.
The FRD inertia tensor maps as J_flu = M J_frd M (the Jxz product flips
sign). The rigid-body integrator then runs entirely in FLU/ENU.

Model content (B&M eq. numbers):
- Air data: Va, alpha = atan2(w, u), beta = asin(v/Va)            (2.8)
- Blended lift: linear + flat plate via sigmoid sigma(alpha)      (4.9-4.10)
- Induced drag: CD = CD_p + (CL0 + CL_alpha alpha)^2/(pi e AR)    (4.11)
- Stability->body force rotation CX/CZ                            (4.19)
- Lateral force/moments linear in beta, p, r, da, dr              (4.14)
- Propulsion: prop  T = 1/2 rho S_prop C_prop ((k dt)^2 - Va^2)   (4.15)
              jet   T = T_max dt (throttle-proportional, no washout)

Controls vector per vehicle: [delta_e, delta_a, delta_r, delta_t] in B&M
FRD sign conventions, delta_t clipped to [0, 1]. Gravity is applied here
(world -z), keeping the integrator wrench-pure.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from coopuavs.physics import GRAVITY
from coopuavs.physics import rigid_body as rb


@dataclass(frozen=True)
class FixedwingParams:
    """Immutable airframe set (see params/shahed_fw.yaml, jet_owa_fw.yaml)."""

    name: str
    mass: float
    inertia: np.ndarray          # (3, 3) body FLU (converted from FRD blocks)
    s_wing: float                # m^2
    b_wing: float                # m span
    c_wing: float                # m mean chord
    e_oswald: float
    aspect_ratio: float
    alpha0: float                # rad, stall blend center
    m_blend: float               # sigmoid sharpness
    prop: dict = field(default_factory=dict)
    aero: dict = field(default_factory=dict)

    @classmethod
    def from_dict(cls, cfg: dict) -> "FixedwingParams":
        j = cfg["inertia_frd"]
        j_frd = np.array([
            [j["jx"], 0.0, -j["jxz"]],
            [0.0, j["jy"], 0.0],
            [-j["jxz"], 0.0, j["jz"]],
        ])
        flip = np.diag([1.0, -1.0, -1.0])
        wing = cfg["wing"]
        return cls(
            name=cfg["name"],
            mass=float(cfg["mass"]),
            inertia=flip @ j_frd @ flip,
            s_wing=float(wing["s"]),
            b_wing=float(wing["b"]),
            c_wing=float(wing["c"]),
            e_oswald=float(wing["e"]),
            aspect_ratio=float(wing["b"]) ** 2 / float(wing["s"]),
            alpha0=float(cfg["stall"]["alpha0"]),
            m_blend=float(cfg["stall"]["m_blend"]),
            prop=dict(cfg["prop"]),
            aero={k: float(v) for k, v in cfg["aero"].items()},
        )


def stall_blend(params: FixedwingParams, alpha: np.ndarray) -> np.ndarray:
    """B&M eq. 4.10 sigmoid: 0 in the linear regime, 1 deep post-stall."""
    m, a0 = params.m_blend, params.alpha0
    e_neg = np.exp(-m * (alpha - a0))
    e_pos = np.exp(m * (alpha + a0))
    return (1.0 + e_neg + e_pos) / ((1.0 + e_neg) * (1.0 + e_pos))


def lift_coefficient(params: FixedwingParams, alpha: np.ndarray) -> np.ndarray:
    """B&M eq. 4.9: linear lift blended into the flat-plate model post stall."""
    a = params.aero
    sigma = stall_blend(params, alpha)
    linear = a["CL0"] + a["CL_alpha"] * alpha
    flat = 2.0 * np.sign(alpha) * np.sin(alpha) ** 2 * np.cos(alpha)
    return (1.0 - sigma) * linear + sigma * flat


def drag_coefficient(params: FixedwingParams, alpha: np.ndarray) -> np.ndarray:
    """B&M eq. 4.11: parasitic + induced drag on the linear lift."""
    a = params.aero
    cl_lin = a["CL0"] + a["CL_alpha"] * alpha
    return a["CD_p"] + cl_lin**2 / (np.pi * params.e_oswald * params.aspect_ratio)


class FixedwingPlant:
    """N identical fixed-wing airframes; wrench evaluation + RK4 convenience."""

    def __init__(self, params: FixedwingParams, n: int):
        self.params = params
        self.n = int(n)
        self.mass = np.full(self.n, params.mass)
        self.inertia = np.repeat(params.inertia[None, :, :], self.n, axis=0)
        self.inertia_inv = np.linalg.inv(self.inertia)

    def wrench(self, state: np.ndarray, controls: np.ndarray,
               wind_world: np.ndarray, rho) -> tuple[np.ndarray, np.ndarray]:
        """Total (force_world (n,3), torque_body_FLU (n,3)) incl. gravity.

        controls: (n, 4) [delta_e, delta_a, delta_r, delta_t] (B&M FRD signs).
        """
        p = self.params
        a = p.aero
        quat = state[:, rb.QUAT]

        v_air_world = state[:, rb.VEL] - wind_world
        v_flu = rb.quat_rotate(rb.quat_conjugate(quat), v_air_world)
        u, v, w = v_flu[:, 0], -v_flu[:, 1], -v_flu[:, 2]            # FRD
        om = state[:, rb.OMEGA]
        pr, qr, rr = om[:, 0], -om[:, 1], -om[:, 2]                  # FRD p, q, r

        va = np.sqrt(u * u + v * v + w * w)
        va_safe = np.maximum(va, 1e-6)
        alpha = np.arctan2(w, u)
        beta = np.arcsin(np.clip(v / va_safe, -1.0, 1.0))

        de, da, dr = controls[:, 0], controls[:, 1], controls[:, 2]
        dt = np.clip(controls[:, 3], 0.0, 1.0)

        rho = np.broadcast_to(np.asarray(rho, dtype=float), va.shape)
        qbar_s = 0.5 * rho * va * va * p.s_wing
        c2v = p.c_wing / (2.0 * va_safe)
        b2v = p.b_wing / (2.0 * va_safe)

        cl = lift_coefficient(p, alpha)
        cd = drag_coefficient(p, alpha)
        sa, ca = np.sin(alpha), np.cos(alpha)

        # stability -> body (B&M 4.19)
        cx = -cd * ca + cl * sa
        cx_q = -a["CD_q"] * ca + a["CL_q"] * sa
        cx_de = -a["CD_de"] * ca + a["CL_de"] * sa
        cz = -cd * sa - cl * ca
        cz_q = -a["CD_q"] * sa - a["CL_q"] * ca
        cz_de = -a["CD_de"] * sa - a["CL_de"] * ca

        if p.prop["model"] == "prop":
            thrust = 0.5 * rho * p.prop["s_prop"] * p.prop["c_prop"] * (
                (p.prop["k_motor"] * dt) ** 2 - va * va)
        elif p.prop["model"] == "jet":
            thrust = p.prop["t_max"] * dt
        else:
            raise ValueError(f"unknown propulsion model {p.prop['model']!r}")

        fx = qbar_s * (cx + cx_q * c2v * qr + cx_de * de) + thrust
        fz = qbar_s * (cz + cz_q * c2v * qr + cz_de * de)
        fy = qbar_s * (a["CY0"] + a["CY_beta"] * beta + a["CY_p"] * b2v * pr
                       + a["CY_r"] * b2v * rr + a["CY_da"] * da + a["CY_dr"] * dr)

        m_l = qbar_s * p.b_wing * (a["Cl0"] + a["Cl_beta"] * beta + a["Cl_p"] * b2v * pr
                                   + a["Cl_r"] * b2v * rr + a["Cl_da"] * da
                                   + a["Cl_dr"] * dr)
        m_m = qbar_s * p.c_wing * (a["Cm0"] + a["Cm_alpha"] * alpha
                                   + a["Cm_q"] * c2v * qr + a["Cm_de"] * de)
        m_n = qbar_s * p.b_wing * (a["Cn0"] + a["Cn_beta"] * beta + a["Cn_p"] * b2v * pr
                                   + a["Cn_r"] * b2v * rr + a["Cn_da"] * da
                                   + a["Cn_dr"] * dr)

        # FRD -> FLU (M = diag(1,-1,-1)) and out to the world frame
        f_flu = np.stack([fx, -fy, -fz], axis=1)
        torque = np.stack([m_l, -m_m, -m_n], axis=1)
        force = rb.quat_rotate(quat, f_flu)
        force[:, 2] -= self.mass * GRAVITY
        return force, torque

    def wrench_fn(self, controls: np.ndarray, wind_world: np.ndarray, rho):
        return lambda s: self.wrench(s, controls, wind_world, rho)

    def step(self, state: np.ndarray, dt: float, controls: np.ndarray,
             wind_world: np.ndarray, rho) -> np.ndarray:
        return rb.rk4_step(state, dt, self.wrench_fn(controls, wind_world, rho),
                           self.mass, self.inertia, self.inertia_inv)
