"""P3-4a: coopfc/estimation/alignment.py — initial leveling + gyro bias +
mag yaw, with a variance gate that refuses to align while moving.

Accuracy gates are 3-5x the propagated statistical sigma at the device
noise magnitudes of hw/params/interceptor_devices.yaml (e.g. tilt sigma
~= sigma_a / (sqrt(N) g) ~= 0.006 deg at N=800 -> gate 0.05 deg), so
they catch sign/axis errors without flaking. The honesty contract: the
returned P0 must *cover* the systematic errors alignment cannot observe
(accel turn-on bias -> tilt, hard-iron -> yaw).
"""

from __future__ import annotations

import math

import numpy as np

from coopuavs.coopfc.core import vec
from coopuavs.coopfc.estimation.alignment import Aligner

G = 9.81
RNG = np.random.default_rng(42)

# Device-class magnitudes (mirror interceptor_devices.yaml by value).
SIGMA_GYRO = 1.74e-3   # rad/s per sample (8.7e-5 * sqrt(400))
SIGMA_ACCEL = 0.028    # m/s^2 per sample (2.0e-3 * sqrt(200) approx)
SIGMA_MAG = 0.3        # uT per axis
MAG_UT, DECL_DEG, INCL_DEG = 50.0, 4.0, 63.0


def field_enu():
    d, i = math.radians(DECL_DEG), math.radians(INCL_DEG)
    return (MAG_UT * math.cos(i) * math.sin(d),
            MAG_UT * math.cos(i) * math.cos(d),
            -MAG_UT * math.sin(i))


def feed_static(aligner, q_true, gyro_bias=(0.0, 0.0, 0.0),
                accel_bias=(0.0, 0.0, 0.0), n_imu=800, n_mag=100,
                noisy=True):
    """Static-vehicle IMU/mag stream at the true attitude."""
    f_body = vec.quat_rotate_inv(q_true, (0.0, 0.0, G))  # specific force
    m_body = vec.quat_rotate_inv(q_true, field_enu())
    mag_every = max(1, n_imu // n_mag)
    for k in range(n_imu):
        ng = RNG.normal(0.0, SIGMA_GYRO, 3) if noisy else np.zeros(3)
        na = RNG.normal(0.0, SIGMA_ACCEL, 3) if noisy else np.zeros(3)
        aligner.add_imu(
            tuple(gyro_bias[i] + float(ng[i]) for i in range(3)),
            tuple(f_body[i] + accel_bias[i] + float(na[i]) for i in range(3)))
        if k % mag_every == 0:
            nm = RNG.normal(0.0, SIGMA_MAG, 3) if noisy else np.zeros(3)
            aligner.add_mag(tuple(m_body[i] + float(nm[i]) for i in range(3)))


def make_aligner(**kw):
    kw.setdefault("mag_declination_deg", DECL_DEG)
    return Aligner(**kw)


def test_needs_enough_samples():
    a = make_aligner(n_imu=800)
    assert a.result() is None
    feed_static(a, vec.quat_from_euler(0.0, 0.0, 0.0), n_imu=799)
    assert a.result() is None


def test_leveling_accuracy_and_yaw():
    roll, pitch, yaw = math.radians(3.0), math.radians(-2.0), math.radians(50.0)
    q_true = vec.quat_from_euler(roll, pitch, yaw)
    a = make_aligner(n_imu=800)
    feed_static(a, q_true)
    res = a.result()
    assert res is not None and res.ok
    r, p, y = vec.quat_to_euler(res.q0)
    assert abs(r - roll) < math.radians(0.05)
    assert abs(p - pitch) < math.radians(0.05)
    assert abs(vec.wrap_pi(y - yaw)) < math.radians(0.5)


def test_gyro_bias_recovered():
    bias = (3.5e-3, -2.0e-3, 1.0e-3)
    a = make_aligner(n_imu=800)
    feed_static(a, vec.quat_from_euler(0.0, 0.0, 1.0), gyro_bias=bias)
    res = a.result()
    for i in range(3):
        assert abs(res.gyro_bias[i] - bias[i]) < 5e-4


def test_yaw_zero_attitude_identity():
    # Noise-free sanity: identity attitude recovered exactly-ish.
    a = make_aligner(n_imu=800)
    feed_static(a, (1.0, 0.0, 0.0, 0.0), noisy=False)
    res = a.result()
    r, p, y = vec.quat_to_euler(res.q0)
    assert abs(r) < 1e-12 and abs(p) < 1e-12 and abs(vec.wrap_pi(y)) < 1e-9


def test_p0_covers_accel_bias_tilt_error():
    # 0.2 m/s^2 turn-on accel bias is unobservable at one attitude: it
    # tilts the leveling by ~b/g. P0's tilt variance must cover it.
    bias = (0.2, -0.1, 0.05)
    a = make_aligner(n_imu=800, accel_turn_on_sigma=0.2)
    feed_static(a, vec.quat_from_euler(0.0, 0.0, 0.5), accel_bias=bias)
    res = a.result()
    r, p, _ = vec.quat_to_euler(res.q0)
    tilt_err = math.sqrt(r * r + p * p)
    sigma_tilt = math.sqrt(res.p0_diag[6])  # δθx
    assert tilt_err < 3.0 * math.sqrt(2.0) * sigma_tilt
    assert sigma_tilt >= 0.2 / G * 0.9  # at least the bias prior, m/s^2 over g


def test_variance_gate_rejects_motion():
    a = make_aligner(n_imu=800)
    q = vec.quat_from_euler(0.0, 0.0, 0.0)
    f_body = vec.quat_rotate_inv(q, (0.0, 0.0, G))
    for k in range(800):
        wobble = 0.5 * math.sin(2.0 * math.pi * k / 100.0)  # m/s^2 sway
        a.add_imu((0.0, 0.0, 0.0),
                  (f_body[0] + wobble, f_body[1], f_body[2]))
        if k % 8 == 0:
            a.add_mag(field_enu())
    res = a.result()
    assert res is not None
    assert res.ok is False  # gated, FCU PBIT must retry


def test_p0_layout_and_positivity():
    a = make_aligner(n_imu=800)
    feed_static(a, vec.quat_from_euler(0.1, 0.05, 2.0))
    res = a.result()
    assert len(res.p0_diag) == 15  # [δp, δv, δθ, δbg, δba]
    assert all(v > 0.0 for v in res.p0_diag)
    # Position/velocity priors are wide-open until GNSS arrives.
    assert res.p0_diag[0] >= 100.0
    assert res.p0_diag[3] >= 1.0
