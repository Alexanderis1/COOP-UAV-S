"""P3-1: coopfc/core/vec.py — plain-float vector/quaternion math.

Flight-software math validated against two independent references:
scipy.spatial.transform.Rotation (the plan's named oracle) and the P1
physics helpers (frozen conventions: Hamilton scalar-first [w,x,y,z],
body -> world, world ENU, body FLU). vec.py itself must stay numpy-free
(enforced by test_coopfc_fence) — numpy here is test-side only.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from coopuavs.coopfc.core import vec
from coopuavs.physics import rigid_body as rb

RNG = np.random.default_rng(20260611)
TOL = 1e-12


def random_quat() -> tuple[float, float, float, float]:
    q = RNG.normal(size=4)
    q /= np.linalg.norm(q)
    return tuple(float(c) for c in q)


def random_vec() -> tuple[float, float, float]:
    return tuple(float(c) for c in RNG.normal(size=3) * 10.0)


def as_scipy(q):  # wxyz -> Rotation (scipy stores xyzw)
    w, x, y, z = q
    return Rotation.from_quat([x, y, z, w])


def quat_close(p, q, tol=1e-9):
    """Same rotation: q and -q are identical attitudes."""
    dot = abs(sum(a * b for a, b in zip(p, q)))
    return abs(dot - 1.0) < tol


# ---------------------------------------------------------------- quaternions


def test_hamilton_product_literals():
    i = (0.0, 1.0, 0.0, 0.0)
    j = (0.0, 0.0, 1.0, 0.0)
    k = (0.0, 0.0, 0.0, 1.0)
    assert vec.quat_multiply(i, j) == k
    assert vec.quat_multiply(j, i) == (0.0, 0.0, 0.0, -1.0)
    assert vec.quat_multiply(i, i) == (-1.0, 0.0, 0.0, 0.0)
    ident = (1.0, 0.0, 0.0, 0.0)
    q = random_quat()
    assert vec.quat_multiply(ident, q) == pytest.approx(q, abs=0)


def test_quat_multiply_matches_physics():
    for _ in range(50):
        p, q = random_quat(), random_quat()
        ours = vec.quat_multiply(p, q)
        ref = rb.quat_multiply(np.array(p), np.array(q))
        assert ours == pytest.approx(tuple(ref), abs=TOL)


def test_quat_rotate_matches_scipy_and_physics():
    for _ in range(50):
        q, v = random_quat(), random_vec()
        ours = vec.quat_rotate(q, v)
        assert ours == pytest.approx(tuple(as_scipy(q).apply(v)), abs=1e-9)
        assert ours == pytest.approx(tuple(rb.quat_rotate(np.array(q), np.array(v))), abs=TOL)


def test_quat_rotate_inv_is_inverse():
    for _ in range(20):
        q, v = random_quat(), random_vec()
        assert vec.quat_rotate_inv(q, vec.quat_rotate(q, v)) == pytest.approx(v, abs=1e-9)
        ref = rb.quat_rotate_inv(np.array(q), np.array(v))
        assert vec.quat_rotate_inv(q, v) == pytest.approx(tuple(ref), abs=TOL)


def test_quat_conjugate_and_normalize():
    q = random_quat()
    qc = vec.quat_conjugate(q)
    assert vec.quat_multiply(q, qc) == pytest.approx((1.0, 0.0, 0.0, 0.0), abs=1e-12)
    scaled = tuple(3.0 * c for c in q)
    assert vec.quat_normalize(scaled) == pytest.approx(q, abs=1e-12)
    with pytest.raises(ValueError):
        vec.quat_normalize((0.0, 0.0, 0.0, 0.0))


def test_quat_from_axis_angle_matches_scipy():
    for axis in [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
                 tuple(float(c) for c in RNG.normal(size=3))]:
        n = math.sqrt(sum(c * c for c in axis))
        unit = tuple(c / n for c in axis)
        for angle in (-2.0, -0.3, 0.0, 0.7, 3.0):
            ours = vec.quat_from_axis_angle(unit, angle)
            sp = Rotation.from_rotvec(np.array(unit) * angle)
            x, y, z, w = sp.as_quat()
            assert quat_close(ours, (w, x, y, z), tol=1e-12)


# --------------------------------------------------------------------- euler


def test_euler_matches_scipy_zyx_intrinsic():
    # Convention: intrinsic Z-Y'-X'' (yaw about world z, then pitch, then
    # roll) — q = qz(yaw) (x) qy(pitch) (x) qx(roll).
    for _ in range(50):
        roll, yaw = RNG.uniform(-math.pi, math.pi, size=2)
        pitch = RNG.uniform(-1.5, 1.5)
        ours = vec.quat_from_euler(float(roll), float(pitch), float(yaw))
        sp = Rotation.from_euler("ZYX", [yaw, pitch, roll])
        x, y, z, w = sp.as_quat()
        assert quat_close(ours, (w, x, y, z), tol=1e-12)

        back = vec.quat_to_euler(ours)
        ref = sp.as_euler("ZYX")  # [yaw, pitch, roll]
        assert back == pytest.approx((ref[2], ref[1], ref[0]), abs=1e-9)


def test_euler_round_trip():
    for _ in range(50):
        roll, yaw = (float(a) for a in RNG.uniform(-math.pi, math.pi, size=2))
        pitch = float(RNG.uniform(-1.5, 1.5))
        r, p, y = vec.quat_to_euler(vec.quat_from_euler(roll, pitch, yaw))
        assert (r, p, y) == pytest.approx((roll, pitch, yaw), abs=1e-9)


def test_euler_gimbal_lock_no_nan():
    for pitch in (math.pi / 2, -math.pi / 2):
        q = vec.quat_from_euler(0.3, pitch, 0.5)
        r, p, y = vec.quat_to_euler(q)
        assert all(math.isfinite(a) for a in (r, p, y))
        # Same attitude when rebuilt (roll/yaw individually degenerate at
        # the pole; the rotation must survive).
        q2 = vec.quat_from_euler(r, p, y)
        assert quat_close(q, q2, tol=1e-6)


# --------------------------------------------------------------- integration


def test_quat_integrate_matches_axis_angle():
    # Constant body rate: exact solution is q0 (x) exp(omega*t/2).
    q0 = random_quat()
    omega = (0.4, -1.1, 2.2)
    dt = 0.25
    n = math.sqrt(sum(c * c for c in omega))
    expect = vec.quat_multiply(
        q0, vec.quat_from_axis_angle(tuple(c / n for c in omega), n * dt))
    assert vec.quat_integrate(q0, omega, dt) == pytest.approx(expect, abs=1e-12)


def test_quat_integrate_composes_exactly():
    # 400 small exact-map steps about a fixed body axis == one big step.
    q0 = (1.0, 0.0, 0.0, 0.0)
    omega = (0.0, 0.0, 1.5)
    q = q0
    for _ in range(400):
        q = vec.quat_integrate(q, omega, 1.0 / 400.0)
    expect = vec.quat_from_axis_angle((0.0, 0.0, 1.0), 1.5)
    assert q == pytest.approx(expect, abs=1e-9)


def test_quat_integrate_zero_rate_is_identity():
    q = random_quat()
    out = vec.quat_integrate(q, (0.0, 0.0, 0.0), 0.0025)
    assert quat_close(out, q, tol=1e-15)
    assert all(math.isfinite(c) for c in out)


def test_quat_integrate_stays_unit():
    q = random_quat()
    for _ in range(1000):
        q = vec.quat_integrate(q, (3.0, -2.0, 1.0), 0.0025)
    assert math.sqrt(sum(c * c for c in q)) == pytest.approx(1.0, abs=1e-9)


# ------------------------------------------------------------------- vectors


def test_vec3_ops_literals():
    assert vec.v3_cross((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)) == (0.0, 0.0, 1.0)
    assert vec.v3_add((1.0, 2.0, 3.0), (4.0, 5.0, 6.0)) == (5.0, 7.0, 9.0)
    assert vec.v3_sub((4.0, 5.0, 6.0), (1.0, 2.0, 3.0)) == (3.0, 3.0, 3.0)
    assert vec.v3_scale((1.0, -2.0, 3.0), 2.0) == (2.0, -4.0, 6.0)
    assert vec.v3_dot((1.0, 2.0, 3.0), (4.0, 5.0, 6.0)) == 32.0
    assert vec.v3_norm((3.0, 4.0, 0.0)) == 5.0
    assert vec.v3_normalize((0.0, 3.0, 4.0)) == (0.0, 0.6, 0.8)
    with pytest.raises(ValueError):
        vec.v3_normalize((0.0, 0.0, 0.0))


def test_v3_cross_matches_numpy():
    for _ in range(20):
        a, b = random_vec(), random_vec()
        assert vec.v3_cross(a, b) == pytest.approx(tuple(np.cross(a, b)), abs=TOL)


# ------------------------------------------------------------------- scalars


def test_wrap_pi():
    assert vec.wrap_pi(0.5) == 0.5
    assert vec.wrap_pi(math.pi + 0.3) == pytest.approx(-math.pi + 0.3)
    assert vec.wrap_pi(-math.pi - 0.3) == pytest.approx(math.pi - 0.3)
    assert vec.wrap_pi(4.0 * math.pi + 0.1) == pytest.approx(0.1)
    assert abs(vec.wrap_pi(math.pi)) == pytest.approx(math.pi)


def test_clip():
    assert vec.clip(5.0, -1.0, 1.0) == 1.0
    assert vec.clip(-5.0, -1.0, 1.0) == -1.0
    assert vec.clip(0.3, -1.0, 1.0) == 0.3
