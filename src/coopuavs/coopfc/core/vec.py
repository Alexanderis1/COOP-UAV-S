"""Plain-float 3-vector and quaternion math for the flight software.

Runs in the 400 Hz rate loop and 100 Hz attitude loop, so it is tuples +
``math`` only — no numpy (fenced by tests/test_coopfc_fence.py; perf
budget "no numpy/allocation in >=100 Hz paths" — tuple construction is
the one allocation we accept).

Conventions (same as the P1 physics core, by value): Hamilton unit
quaternion, scalar-first ``(w, x, y, z)``, body -> world; world ENU
z-up; body FLU. Euler angles are intrinsic Z-Y'-X'' (yaw about world z,
then pitch about the new y, then roll), ``q = qz(yaw) (x) qy(pitch) (x)
qx(roll)`` — validated against scipy Rotation 'ZYX' and the physics
helpers in tests/test_coopfc_vec.py.

Equations (citations in docs/RESEARCH.md, section "P3 CoopFC"):
- Hamilton product and sandwich rotation [Sola 2017, eq. 12, 24].
- Exact exponential-map attitude integration q(t+dt) = q (x)
  exp(omega*dt/2) for body-frame rate [Sola 2017, eq. 225-228].
"""

from __future__ import annotations

import math

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]

# Below this rotation increment the exponential map switches to its
# small-angle series (sin(a/2)/a -> 1/2): keeps quat_integrate finite at
# omega = 0 with error O(angle^2) below double precision.
_SMALL_ANGLE = 1e-9

# Gimbal-lock band for quat_to_euler: sin(pitch) within ~1e-9 of +-1
# (pitch within ~45 urad of the pole) uses the degenerate-axis branch.
_POLE_S = 1.0 - 1e-9


# ------------------------------------------------------------------- vectors


def v3_add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def v3_sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def v3_scale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def v3_dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def v3_cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def v3_norm(a: Vec3) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def v3_normalize(a: Vec3) -> Vec3:
    n = v3_norm(a)
    if n == 0.0:
        raise ValueError("cannot normalize a zero vector")
    return (a[0] / n, a[1] / n, a[2] / n)


# ------------------------------------------------------------------- scalars


def clip(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def wrap_pi(angle: float) -> float:
    """Wrap to [-pi, pi] (IEEE remainder: half-even at the boundary)."""
    return math.remainder(angle, math.tau)


# --------------------------------------------------------------- quaternions


def quat_multiply(p: Quat, q: Quat) -> Quat:
    """Hamilton product p (x) q (composes body->world rotations)."""
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return (
        pw * qw - px * qx - py * qy - pz * qz,
        pw * qx + px * qw + py * qz - pz * qy,
        pw * qy - px * qz + py * qw + pz * qx,
        pw * qz + px * qy - py * qx + pz * qw,
    )


def quat_conjugate(q: Quat) -> Quat:
    return (q[0], -q[1], -q[2], -q[3])


def quat_normalize(q: Quat) -> Quat:
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n == 0.0:
        raise ValueError("cannot normalize a zero quaternion")
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def quat_rotate(q: Quat, v: Vec3) -> Vec3:
    """Rotate body-frame v into the world frame: v' = q (x) v (x) q*.

    Expanded sandwich v' = v + w*t + qv x t with t = 2 qv x v (same form
    as the physics core).
    """
    w = q[0]
    qx, qy, qz = q[1], q[2], q[3]
    tx = 2.0 * (qy * v[2] - qz * v[1])
    ty = 2.0 * (qz * v[0] - qx * v[2])
    tz = 2.0 * (qx * v[1] - qy * v[0])
    return (
        v[0] + w * tx + qy * tz - qz * ty,
        v[1] + w * ty + qz * tx - qx * tz,
        v[2] + w * tz + qx * ty - qy * tx,
    )


def quat_rotate_inv(q: Quat, v: Vec3) -> Vec3:
    """Rotate world-frame v into the body frame: v' = q* (x) v (x) q."""
    w = q[0]
    qx, qy, qz = q[1], q[2], q[3]
    tx = 2.0 * (qy * v[2] - qz * v[1])
    ty = 2.0 * (qz * v[0] - qx * v[2])
    tz = 2.0 * (qx * v[1] - qy * v[0])
    return (
        v[0] - w * tx + qy * tz - qz * ty,
        v[1] - w * ty + qz * tx - qx * tz,
        v[2] - w * tz + qx * ty - qy * tx,
    )


def quat_from_axis_angle(axis: Vec3, angle: float) -> Quat:
    """Unit quaternion for a rotation of `angle` (rad) about unit `axis`."""
    half = 0.5 * angle
    s = math.sin(half)
    return (math.cos(half), axis[0] * s, axis[1] * s, axis[2] * s)


def quat_from_euler(roll: float, pitch: float, yaw: float) -> Quat:
    """Intrinsic Z-Y'-X'': q = qz(yaw) (x) qy(pitch) (x) qx(roll)."""
    cr, sr = math.cos(0.5 * roll), math.sin(0.5 * roll)
    cp, sp = math.cos(0.5 * pitch), math.sin(0.5 * pitch)
    cy, sy = math.cos(0.5 * yaw), math.sin(0.5 * yaw)
    return (
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        cy * sp * cr + sy * cp * sr,
        sy * cp * cr - cy * sp * sr,
    )


def quat_to_euler(q: Quat) -> tuple[float, float, float]:
    """(roll, pitch, yaw), inverse of quat_from_euler.

    pitch = asin(2(wy - xz)) clamped into its domain so a unit-norm
    rounding excursion at the +-90 deg pole cannot NaN. At the pole only
    yaw -+ roll is observable and the regular atan2 pair collapses, so a
    dedicated branch returns roll = 0 and folds the whole in-plane angle
    into yaw (scipy's gimbal-lock convention; recomposition preserves the
    rotation — pinned in tests).
    """
    w, x, y, z = q
    s = 2.0 * (w * y - x * z)
    if s > _POLE_S:
        return (0.0, 0.5 * math.pi, 2.0 * math.atan2(-x, w))
    if s < -_POLE_S:
        return (0.0, -0.5 * math.pi, 2.0 * math.atan2(x, w))
    pitch = math.asin(clip(s, -1.0, 1.0))
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (roll, pitch, yaw)


def quat_integrate(q: Quat, omega_body: Vec3, dt: float) -> Quat:
    """Advance attitude one step at constant body rate (exact map).

    q(t+dt) = q (x) exp(omega*dt/2) [Sola 2017 eq. 225-228]; exact for
    constant omega over the step, hence composes without integration
    drift. Renormalized every call to bound float round-off.
    """
    ax = omega_body[0] * dt
    ay = omega_body[1] * dt
    az = omega_body[2] * dt
    angle = math.sqrt(ax * ax + ay * ay + az * az)
    if angle < _SMALL_ANGLE:
        dq = (1.0, 0.5 * ax, 0.5 * ay, 0.5 * az)
    else:
        s = math.sin(0.5 * angle) / angle
        dq = (math.cos(0.5 * angle), ax * s, ay * s, az * s)
    return quat_normalize(quat_multiply(q, dq))
