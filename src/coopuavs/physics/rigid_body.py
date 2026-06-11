"""Batched 6DOF rigid-body dynamics with quaternion attitude and RK4.

State batch is an ``(N, 13)`` float64 array; per-row layout::

    [0:3]   position, world ENU (m)
    [3:6]   velocity, world ENU (m/s)
    [6:10]  attitude quaternion, Hamilton scalar-first [w, x, y, z], body -> world
    [10:13] angular velocity, body FLU (rad/s)

Equations (citations in docs/RESEARCH.md, section "P1 physics core"):
- Quaternion kinematics  q_dot = 1/2 * q (x) (0, omega_body)
  [Sola 2017, "Quaternion kinematics for the error-state Kalman filter", eq. 199].
- Euler's rotational equation in body axes
  omega_dot = J^-1 (tau - omega x J omega)
  [Beard & McLain 2012, "Small Unmanned Aircraft", eq. 3.16-3.17].
- Newton translation in the world frame  v_dot = F_world / m.
- Classic fixed-step RK4; quaternion renormalized once per full step
  (intermediate stages may be slightly non-unit; this preserves order 4).

The integrator is force-model agnostic: callers supply ``wrench_fn(state) ->
(force_world (N,3), torque_body (N,3))`` with gravity included by the vehicle
model. Wind/actuator inputs are zero-order-held across one step by closing
over them in ``wrench_fn`` (micro-step contract: actuators latch before the
single batched RK4 step).
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

STATE_DIM = 13
POS = slice(0, 3)
VEL = slice(3, 6)
QUAT = slice(6, 10)
OMEGA = slice(10, 13)

WrenchFn = Callable[[np.ndarray], tuple[np.ndarray, np.ndarray]]


def _cross3(a: np.ndarray, b: np.ndarray, out: np.ndarray | None = None) -> np.ndarray:
    """Component-wise cross product (np.cross has prohibitive overhead at
    800 Hz x 4 RK4 stages; P1-6 perf gate)."""
    if out is None:
        out = np.empty_like(b) if a.shape == b.shape else \
            np.empty(np.broadcast(a, b).shape)
    a0, a1, a2 = a[..., 0], a[..., 1], a[..., 2]
    b0, b1, b2 = b[..., 0], b[..., 1], b[..., 2]
    out[..., 0] = a1 * b2 - a2 * b1
    out[..., 1] = a2 * b0 - a0 * b2
    out[..., 2] = a0 * b1 - a1 * b0
    return out


def quat_normalize(q: np.ndarray) -> np.ndarray:
    """Return unit quaternion(s); last axis is [w, x, y, z]."""
    return q / np.sqrt((q * q).sum(axis=-1, keepdims=True))


def quat_multiply(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Hamilton product p (x) q (scalar-first, composes body->world rotations)."""
    pw, px, py, pz = p[..., 0], p[..., 1], p[..., 2], p[..., 3]
    qw, qx, qy, qz = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return np.stack(
        [
            pw * qw - px * qx - py * qy - pz * qz,
            pw * qx + px * qw + py * qz - pz * qy,
            pw * qy - px * qz + py * qw + pz * qx,
            pw * qz + px * qy - py * qx + pz * qw,
        ],
        axis=-1,
    )


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    out = q.copy()
    out[..., 1:] = -out[..., 1:]
    return out


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate body-frame vector(s) v into the world frame: v' = q (x) v (x) q*.

    Uses the expanded sandwich form v' = v + w*t + qv x t with t = 2 qv x v
    (no quaternion products allocated).
    """
    qw = q[..., :1]
    qv = q[..., 1:]
    t = _cross3(qv, v)
    t *= 2.0
    out = _cross3(qv, t)
    out += qw * t
    out += v
    return out


def quat_rotate_inv(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate world-frame vector(s) into the body frame: v' = q* (x) v (x) q.

    Algebraically identical to quat_rotate(quat_conjugate(q), v) without
    materializing the conjugate: v' = v - w*t + qv x t, t = 2 qv x v.
    """
    qw = q[..., :1]
    qv = q[..., 1:]
    t = _cross3(qv, v)
    t *= 2.0
    out = _cross3(qv, t)
    out -= qw * t
    out += v
    return out


def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    """Rotation matrix R (body -> world), shape (..., 3, 3)."""
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    xx, yy, zz = x * x, y * y, z * z
    wx, wy, wz = w * x, w * y, w * z
    xy, xz, yz = x * y, x * z, y * z
    row0 = np.stack([1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)], axis=-1)
    row1 = np.stack([2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)], axis=-1)
    row2 = np.stack([2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)], axis=-1)
    return np.stack([row0, row1, row2], axis=-2)


def quat_from_axis_angle(axis: np.ndarray, angle: np.ndarray) -> np.ndarray:
    """Unit quaternion(s) for rotation of `angle` (rad) about unit `axis` (..., 3)."""
    axis = np.asarray(axis, dtype=float)
    angle = np.asarray(angle, dtype=float)
    half = 0.5 * angle
    w = np.cos(half)[..., None]
    xyz = axis * np.sin(half)[..., None]
    return np.concatenate([w, xyz], axis=-1)


def quat_derivative(q: np.ndarray, omega_body: np.ndarray,
                    out: np.ndarray | None = None) -> np.ndarray:
    """q_dot = 1/2 * q (x) (0, omega_body)  [Sola 2017 eq. 199]."""
    if out is None:
        out = np.empty_like(q)
    qw, qx, qy, qz = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    wx, wy, wz = omega_body[..., 0], omega_body[..., 1], omega_body[..., 2]
    out[..., 0] = -0.5 * (qx * wx + qy * wy + qz * wz)
    out[..., 1] = 0.5 * (qw * wx + qy * wz - qz * wy)
    out[..., 2] = 0.5 * (qw * wy - qx * wz + qz * wx)
    out[..., 3] = 0.5 * (qw * wz + qx * wy - qy * wx)
    return out


def derivatives(
    state: np.ndarray,
    force_world: np.ndarray,
    torque_body: np.ndarray,
    mass: np.ndarray,
    inertia: np.ndarray,
    inertia_inv: np.ndarray,
) -> np.ndarray:
    """Time derivative of the (N, 13) state under the given wrench.

    mass: (N,); inertia / inertia_inv: (N, 3, 3) body-frame tensors.
    """
    deriv = np.empty_like(state)
    deriv[:, POS] = state[:, VEL]
    deriv[:, VEL] = force_world / mass[:, None]
    omega = state[:, OMEGA]
    quat_derivative(state[:, QUAT], omega, out=deriv[:, QUAT])
    j_omega = (inertia @ omega[..., None])[..., 0]
    gyro = _cross3(omega, j_omega)
    np.subtract(torque_body, gyro, out=gyro)
    deriv[:, OMEGA] = (inertia_inv @ gyro[..., None])[..., 0]
    return deriv


def rk4_step(
    state: np.ndarray,
    dt: float,
    wrench_fn: WrenchFn,
    mass: np.ndarray,
    inertia: np.ndarray,
    inertia_inv: np.ndarray,
) -> np.ndarray:
    """One classic RK4 step for the whole batch; returns a new (N, 13) array.

    The wrench is re-evaluated at every stage (state-dependent aerodynamics);
    inputs held by closure are zero-order-held across the step.
    """

    def f(s: np.ndarray) -> np.ndarray:
        force, torque = wrench_fn(s)
        return derivatives(s, force, torque, mass, inertia, inertia_inv)

    k1 = f(state)
    k2 = f(state + (0.5 * dt) * k1)
    k3 = f(state + (0.5 * dt) * k2)
    k4 = f(state + dt * k3)
    acc = k2
    acc += k3
    acc *= 2.0
    acc += k1
    acc += k4
    acc *= dt / 6.0
    acc += state
    acc[:, QUAT] = quat_normalize(acc[:, QUAT])
    return acc
