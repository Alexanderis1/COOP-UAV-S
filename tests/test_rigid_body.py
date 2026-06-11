"""P1-1: batched quaternion 6DOF rigid body + RK4 integrator (physics/rigid_body.py).

State row (13,): [pos ENU (m), vel ENU (m/s), quat wxyz body->world, omega body (rad/s)].
World = ENU z-up, body = FLU, Hamilton quaternion scalar-first.

Anchors: free-fall parabola (RK4 integrates polynomials of degree <= 4 exactly),
constant-rate principal-axis rotation vs analytic quaternion, torque-free
energy/angular-momentum conservation (<1e-9 relative over 60 s vacuum),
classic order-4 convergence slope, batch==scalar equivalence, and helper
cross-checks against scipy.spatial.transform.Rotation.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from coopuavs.physics import rigid_body as rb

GRAVITY_ENU = np.array([0.0, 0.0, -9.81])


def make_state(pos, vel, quat, omega):
    s = np.zeros(rb.STATE_DIM)
    s[rb.POS], s[rb.VEL], s[rb.QUAT], s[rb.OMEGA] = pos, vel, quat, omega
    return s[None, :]


def spherical_inertia(n, j=0.02):
    inertia = np.repeat(np.eye(3)[None, :, :] * j, n, axis=0)
    return inertia, np.linalg.inv(inertia)


def zero_wrench(state):
    n = state.shape[0]
    return np.zeros((n, 3)), np.zeros((n, 3))


def to_scipy(q_wxyz):
    """Our scalar-first wxyz -> scipy scalar-last xyzw."""
    return np.roll(np.asarray(q_wxyz), -1, axis=-1)


# ---------------------------------------------------------------- state layout

def test_state_layout():
    assert rb.STATE_DIM == 13
    assert (rb.POS, rb.VEL, rb.QUAT, rb.OMEGA) == (
        slice(0, 3), slice(3, 6), slice(6, 10), slice(10, 13))


# ------------------------------------------------------------- quat helpers vs scipy

def test_quat_rotate_matches_scipy():
    rng = np.random.default_rng(11)
    q = rb.quat_normalize(rng.normal(size=(50, 4)))
    v = rng.normal(size=(50, 3))
    expected = Rotation.from_quat(to_scipy(q)).apply(v)
    np.testing.assert_allclose(rb.quat_rotate(q, v), expected, atol=1e-12)


def test_quat_to_rotmat_matches_scipy():
    rng = np.random.default_rng(12)
    q = rb.quat_normalize(rng.normal(size=(20, 4)))
    expected = Rotation.from_quat(to_scipy(q)).as_matrix()
    np.testing.assert_allclose(rb.quat_to_rotmat(q), expected, atol=1e-12)


def test_quat_rotate_inv_matches_scipy_and_roundtrips():
    """Gate-review pin: quat_rotate_inv was previously verified by nothing —
    the mutant quat_rotate_inv = quat_rotate survived the whole suite."""
    rng = np.random.default_rng(14)
    q = rb.quat_normalize(rng.normal(size=(50, 4)))
    v = rng.normal(size=(50, 3))
    expected = Rotation.from_quat(to_scipy(q)).apply(v, inverse=True)
    np.testing.assert_allclose(rb.quat_rotate_inv(q, v), expected, atol=1e-12)
    np.testing.assert_allclose(rb.quat_rotate_inv(q, rb.quat_rotate(q, v)), v,
                               atol=1e-12)
    np.testing.assert_allclose(rb.quat_rotate_inv(q, v),
                               rb.quat_rotate(rb.quat_conjugate(q), v), atol=1e-12)


def test_quat_multiply_composes_rotations():
    rng = np.random.default_rng(13)
    p = rb.quat_normalize(rng.normal(size=(20, 4)))
    q = rb.quat_normalize(rng.normal(size=(20, 4)))
    # Hamilton body->world: R(p (x) q) = R(p) @ R(q)
    left = rb.quat_to_rotmat(rb.quat_multiply(p, q))
    right = rb.quat_to_rotmat(p) @ rb.quat_to_rotmat(q)
    np.testing.assert_allclose(left, right, atol=1e-12)


def test_quat_from_axis_angle_90deg_z():
    # ENU: +90 deg about z takes body x (East) to world y (North).
    q = rb.quat_from_axis_angle(np.array([[0.0, 0.0, 1.0]]), np.array([np.pi / 2]))
    v = rb.quat_rotate(q, np.array([[1.0, 0.0, 0.0]]))
    np.testing.assert_allclose(v, [[0.0, 1.0, 0.0]], atol=1e-12)


# ------------------------------------------------------------------ analytic motion

def test_free_fall_matches_parabola():
    """RK4 reproduces p = p0 + v0 t + g t^2/2 to round-off (polynomial exactness)."""
    n, dt, t_end = 3, 1.0 / 800.0, 2.0
    rng = np.random.default_rng(21)
    mass = np.array([1.0, 4.0, 12.0])
    inertia, inertia_inv = spherical_inertia(n, 0.1)
    p0 = rng.normal(scale=50.0, size=(n, 3))
    v0 = rng.normal(scale=10.0, size=(n, 3))
    state = np.zeros((n, rb.STATE_DIM))
    state[:, rb.POS], state[:, rb.VEL] = p0, v0
    state[:, rb.QUAT] = [1.0, 0.0, 0.0, 0.0]

    def gravity_wrench(s):
        return mass[:, None] * GRAVITY_ENU, np.zeros((n, 3))

    steps = round(t_end / dt)
    for _ in range(steps):
        state = rb.rk4_step(state, dt, gravity_wrench, mass, inertia, inertia_inv)
    t = steps * dt
    np.testing.assert_allclose(state[:, rb.POS], p0 + v0 * t + 0.5 * GRAVITY_ENU * t * t,
                               rtol=0, atol=1e-9)
    np.testing.assert_allclose(state[:, rb.VEL], v0 + GRAVITY_ENU * t, rtol=0, atol=1e-9)


def test_constant_rate_rotation_about_z():
    """Spherical inertia, no torque: q(t) = q0 (x) exp(omega t / 2) exactly."""
    dt, t_end, wz = 1.0 / 800.0, 2.0, 1.3
    mass = np.array([2.0])
    inertia, inertia_inv = spherical_inertia(1)
    state = make_state([0, 0, 0], [0, 0, 0], [1, 0, 0, 0], [0, 0, wz])
    steps = round(t_end / dt)
    for _ in range(steps):
        state = rb.rk4_step(state, dt, zero_wrench, mass, inertia, inertia_inv)
    theta = wz * steps * dt
    q_expected = np.array([np.cos(theta / 2), 0.0, 0.0, np.sin(theta / 2)])
    err = min(np.abs(state[0, rb.QUAT] - q_expected).max(),
              np.abs(state[0, rb.QUAT] + q_expected).max())
    assert err < 1e-9
    np.testing.assert_allclose(state[0, rb.OMEGA], [0, 0, wz], atol=1e-12)
    np.testing.assert_allclose(state[0, rb.POS], 0.0, atol=1e-12)


def test_torque_free_tumble_conserves_energy_and_momentum():
    """Asymmetric torque-free tumble, 60 s vacuum at 800 Hz:
    rotational energy and world-frame angular momentum drift < 1e-9 relative."""
    dt, t_end = 1.0 / 800.0, 60.0
    mass = np.array([1.5])
    inertia = np.array([np.diag([0.02, 0.03, 0.04])])
    inertia_inv = np.linalg.inv(inertia)
    state = make_state([0, 0, 0], [0, 0, 0], [1, 0, 0, 0], [0.3, -0.4, 0.5])

    def energy_momentum(s):
        w = s[:, rb.OMEGA]
        jw = (inertia @ w[..., None])[..., 0]
        e_rot = 0.5 * float(np.sum(w * jw))
        l_world = rb.quat_rotate(s[:, rb.QUAT], jw)
        return e_rot, l_world[0]

    e0, l0 = energy_momentum(state)
    for _ in range(round(t_end / dt)):
        state = rb.rk4_step(state, dt, zero_wrench, mass, inertia, inertia_inv)
    e1, l1 = energy_momentum(state)
    assert abs(e1 - e0) / abs(e0) < 1e-9
    assert np.linalg.norm(l1 - l0) / np.linalg.norm(l0) < 1e-9
    assert abs(np.linalg.norm(state[0, rb.QUAT]) - 1.0) < 1e-12


def test_rk4_order_slope():
    """Global error on a vigorous torque-free tumble halves ~16x per dt halving."""
    mass = np.array([1.0])
    inertia = np.array([np.diag([0.02, 0.03, 0.04])])
    inertia_inv = np.linalg.inv(inertia)
    state0 = make_state([0, 0, 0], [0, 0, 0], [1, 0, 0, 0], [2.0, -1.5, 1.0])

    def integrate(dt, t_end=1.0):
        s = state0.copy()
        for _ in range(round(t_end / dt)):
            s = rb.rk4_step(s, dt, zero_wrench, mass, inertia, inertia_inv)
        return s

    ref = integrate(1.0 / 3200.0)
    errs = [np.abs(integrate(dt) - ref).max() for dt in (1 / 50, 1 / 100, 1 / 200)]
    slopes = [np.log2(errs[i] / errs[i + 1]) for i in range(len(errs) - 1)]
    for slope in slopes:
        assert 3.5 < slope < 4.5, f"RK4 slope {slope}, errors {errs}"


# ------------------------------------------------------------------ batch semantics

def test_batch_equals_scalar():
    """Stepping N bodies together == stepping each alone (state-dependent wrench)."""
    rng = np.random.default_rng(31)
    n = 6
    state = rng.normal(size=(n, rb.STATE_DIM))
    state[:, rb.QUAT] = rb.quat_normalize(state[:, rb.QUAT])
    mass = rng.uniform(0.5, 15.0, size=n)
    diag = rng.uniform(0.01, 0.2, size=(n, 3))
    inertia = np.zeros((n, 3, 3))
    inertia[:, [0, 1, 2], [0, 1, 2]] = diag
    inertia_inv = np.linalg.inv(inertia)

    def wrench(s):
        return -0.3 * s[:, rb.VEL] + 0.05, -0.02 * s[:, rb.OMEGA]

    batched = rb.rk4_step(state, 0.01, wrench, mass, inertia, inertia_inv)
    for i in range(n):
        single = rb.rk4_step(state[i:i + 1], 0.01, wrench, mass[i:i + 1],
                             inertia[i:i + 1], inertia_inv[i:i + 1])
        np.testing.assert_allclose(batched[i], single[0], rtol=0, atol=1e-13)


def test_quat_norm_preserved_over_many_steps():
    rng = np.random.default_rng(41)
    n = 4
    state = rng.normal(size=(n, rb.STATE_DIM))
    state[:, rb.QUAT] = rb.quat_normalize(state[:, rb.QUAT])
    mass = np.ones(n)
    inertia, inertia_inv = spherical_inertia(n)
    for _ in range(2000):
        state = rb.rk4_step(state, 1.0 / 400.0, zero_wrench, mass, inertia, inertia_inv)
    np.testing.assert_allclose(np.linalg.norm(state[:, rb.QUAT], axis=1), 1.0,
                               rtol=0, atol=1e-12)


def test_gyroscopic_torque_direction():
    """Spinning top: J asymmetric, omega off-axis -> omega_dot = -J^-1 (w x Jw) != 0."""
    inertia = np.array([np.diag([0.02, 0.02, 0.06])])
    inertia_inv = np.linalg.inv(inertia)
    state = make_state([0, 0, 0], [0, 0, 0], [1, 0, 0, 0], [1.0, 0.0, 10.0])
    deriv = rb.derivatives(state, np.zeros((1, 3)), np.zeros((1, 3)),
                           np.array([1.0]), inertia, inertia_inv)
    w = state[0, rb.OMEGA]
    jw = inertia[0] @ w
    expected = inertia_inv[0] @ (-np.cross(w, jw))
    np.testing.assert_allclose(deriv[0, rb.OMEGA], expected, atol=1e-14)
    assert np.linalg.norm(expected) > 0.1


def test_derivatives_shape_contract():
    n = 7
    rng = np.random.default_rng(51)
    state = rng.normal(size=(n, rb.STATE_DIM))
    state[:, rb.QUAT] = rb.quat_normalize(state[:, rb.QUAT])
    inertia, inertia_inv = spherical_inertia(n)
    d = rb.derivatives(state, np.zeros((n, 3)), np.zeros((n, 3)),
                       np.ones(n), inertia, inertia_inv)
    assert d.shape == (n, rb.STATE_DIM)
    np.testing.assert_allclose(d[:, rb.POS], state[:, rb.VEL], atol=0)
