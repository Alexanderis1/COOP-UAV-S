"""P3-4b/c: coopfc/estimation/ekf.py — deterministic unit contracts.

Fast checks of the filter mechanics: noise-free convergence, covariance
symmetry/PD under load, the delayed-horizon OOSM correctness (a 120 ms
late fix must fuse against the state *of its measurement time*), the
output predictor (published state is at `now`, not the horizon), the
chi-square spoof gate, GPS-denied behavior, divergence guards, and
run-twice determinism. Statistical NEES/NIS consistency lives in the
@slow Monte-Carlo suite (test_coopfc_ekf_mc.py).
"""

from __future__ import annotations

import math

import numpy as np

from coopuavs.coopfc.core import vec
from coopuavs.coopfc.estimation.alignment import AlignResult
from coopuavs.coopfc.estimation.ekf import Ekf, EkfParams

G = 9.81
IMU_HZ = 400
EKF_HZ = 50
GPS_HZ = 10
GPS_LAT = 0.12

FIELD = EkfParams()
M_WORLD = (FIELD.mag_field_ut * math.cos(math.radians(FIELD.mag_inclination_deg))
           * math.sin(math.radians(FIELD.mag_declination_deg)),
           FIELD.mag_field_ut * math.cos(math.radians(FIELD.mag_inclination_deg))
           * math.cos(math.radians(FIELD.mag_declination_deg)),
           -FIELD.mag_field_ut * math.sin(math.radians(FIELD.mag_inclination_deg)))


def perfect_align(q0=(1.0, 0.0, 0.0, 0.0)) -> AlignResult:
    return AlignResult(
        ok=True, q0=q0, gyro_bias=(0.0, 0.0, 0.0),
        p0_diag=(1e4,) * 3 + (25.0,) * 3 + (4e-4, 4e-4, 1e-2)
        + (1e-5,) * 3 + (4e-2,) * 3,
        gyro_std=(0.0,) * 3, accel_std=(0.0,) * 3)


class TruthSim:
    """Noise-free kinematic truth + perfect sensors, wired to an Ekf.

    accel_fn(t) -> world acceleration; attitude held at q0 (specific
    force = R^T (a_w + g e_z)). Runs the full timing lattice: IMU 400 Hz,
    EKF 50 Hz, GPS 10 Hz with 120 ms delivery latency, baro+mag 50 Hz.
    """

    def __init__(self, ekf: Ekf, accel_fn=None, q0=(1.0, 0.0, 0.0, 0.0),
                 gps_offset=(0.0, 0.0, 0.0), gps_enabled=True):
        self.ekf = ekf
        self.accel_fn = accel_fn or (lambda t: (0.0, 0.0, 0.0))
        self.q0 = q0
        self.gps_offset = gps_offset
        self.gps_enabled = gps_enabled
        self.t_pos = (0.0, 0.0, 0.0)
        self.t_vel = (0.0, 0.0, 0.0)
        self.pending_fix = []  # (deliver_t, fix_stamp, pos, vel)
        self.states = []
        self.truths = []  # (pos, vel) at each update instant

    def run(self, seconds: float, t0: float = 0.0):
        n = round(seconds * IMU_HZ)
        for k in range(n):
            t = t0 + k / IMU_HZ
            a_w = self.accel_fn(t)
            # devices sample truth at tick start
            f_body = vec.quat_rotate_inv(
                self.q0, (a_w[0], a_w[1], a_w[2] + G))
            self.ekf.on_imu(t, (0.0, 0.0, 0.0), f_body)
            if self.gps_enabled and k % (IMU_HZ // GPS_HZ) == 0:
                self.pending_fix.append(
                    (t + GPS_LAT, t,
                     vec.v3_add(self.t_pos, self.gps_offset), self.t_vel))
            while self.pending_fix and self.pending_fix[0][0] <= t + 1e-9:
                _, st, pos, gvel = self.pending_fix.pop(0)
                self.ekf.on_gps(st, pos, gvel, 3)
            if k % (IMU_HZ // EKF_HZ) == 0:
                self.ekf.on_baro(t, self.t_pos[2])
                self.ekf.on_mag(t, vec.quat_rotate_inv(self.q0, M_WORLD))
                self.states.append(self.ekf.update(t))
                self.truths.append((self.t_pos, self.t_vel))
            # advance truth (exact for piecewise-constant accel)
            dt = 1.0 / IMU_HZ
            self.t_pos = tuple(self.t_pos[i] + self.t_vel[i] * dt
                               + 0.5 * a_w[i] * dt * dt for i in range(3))
            self.t_vel = tuple(self.t_vel[i] + a_w[i] * dt for i in range(3))
        return self.states[-1]


def test_static_noise_free_convergence():
    ekf = Ekf(perfect_align())
    sim = TruthSim(ekf)
    s = sim.run(5.0)
    assert not s.diverged
    assert vec.v3_norm(s.pos) < 1e-2
    assert vec.v3_norm(s.vel) < 1e-2
    r, p, y = vec.quat_to_euler(s.q)
    assert max(abs(r), abs(p), abs(vec.wrap_pi(y))) < math.radians(0.1)
    assert s.sigma_pos_h < 2.0  # converged from the 100 m prior


def test_covariance_symmetric_pd_under_load():
    ekf = Ekf(perfect_align())
    TruthSim(ekf, accel_fn=lambda t: (math.sin(t), math.cos(t), 0.5 * math.sin(2 * t))).run(5.0)
    P = ekf.P
    assert np.max(np.abs(P - P.T)) < 1e-12
    assert np.all(np.linalg.eigvalsh(P) > 0.0)


def test_output_predictor_reaches_now_not_horizon():
    # Constant world accel: at t the truth velocity is a*t. A filter
    # publishing the horizon state would lag by lag_s * a = 0.14 m/s.
    ekf = Ekf(perfect_align())
    sim = TruthSim(ekf, accel_fn=lambda t: (1.0, 0.0, 0.0))
    s = sim.run(5.0)
    # Last update ran at the final 50 Hz lattice point; truth velocity
    # there is a*t. A filter publishing the horizon state would sit
    # 0.14 m/s behind — the gate splits the two cleanly.
    t_up = s.stamp
    assert abs(s.vel[0] - t_up) < 0.05
    assert s.vel[0] > t_up - 0.07


def test_oosm_late_fix_fuses_at_measurement_time():
    # Constant velocity 20 m/s: fusing a 120 ms-late fix against the
    # delivery-time state would leave a persistent 2.4 m innovation
    # (instantly visible vs the converged sub-metre accuracy).
    ekf = Ekf(perfect_align())
    sim = TruthSim(ekf, accel_fn=lambda t: (5.0, 0.0, 0.0) if t < 4.0 else (0.0, 0.0, 0.0))
    s = sim.run(10.0)
    err = vec.v3_sub(s.pos, sim.truths[-1][0])  # same-instant truth
    # delivery-time fusion = 2.4 m; horizon-pass (one 50 Hz period)
    # fusion = 0.4 m; exact-stamp fusion = noise-free residual only
    assert vec.v3_norm(err) < 0.05


def test_spoof_step_rejected():
    ekf = Ekf(perfect_align())
    sim = TruthSim(ekf)
    sim.run(5.0)
    pos_before = sim.states[-1].pos
    assert ekf.rejected["gps_pos"] == 0
    # 50 m east spoof step on all subsequent fixes
    sim.gps_offset = (50.0, 0.0, 0.0)
    s = sim.run(3.0, t0=5.0)
    assert ekf.rejected["gps_pos"] >= 25  # ~all spoofed fixes gated
    assert abs(s.pos[0] - pos_before[0]) < 0.5  # estimate held


def test_gps_denied_grows_sigma_keeps_attitude():
    ekf = Ekf(perfect_align())
    sim = TruthSim(ekf)
    sim.run(5.0)
    sigma_aided = math.sqrt(ekf.P[0, 0])  # raw filter covariance
    sim.gps_enabled = False
    s = sim.run(10.0, t0=5.0)
    assert not s.diverged
    assert math.sqrt(ekf.P[0, 0]) > 2.0 * sigma_aided  # honest growth
    r, p, _ = vec.quat_to_euler(s.q)
    assert max(abs(r), abs(p)) < math.radians(0.2)  # mag+gravity hold tilt
    assert abs(s.pos[2] - sim.t_pos[2]) < 1.0       # baro holds altitude


def test_nonfinite_measurement_rejected_not_fatal():
    ekf = Ekf(perfect_align())
    sim = TruthSim(ekf)
    sim.run(1.0)
    ekf.on_baro(1.0, math.nan)
    ekf.on_gps(0.9, (math.inf, 0.0, 0.0), (0.0, 0.0, 0.0), 3)
    s = sim.run(1.0, t0=1.0)
    assert ekf.rejected["nonfinite"] == 2
    assert not s.diverged


def test_non_3d_fix_ignored():
    ekf = Ekf(perfect_align())
    ekf.on_gps(0.0, (5.0, 5.0, 5.0), (0.0, 0.0, 0.0), 0)  # FIX_NONE
    ekf.on_gps(0.0, (5.0, 5.0, 5.0), (0.0, 0.0, 0.0), 2)  # FIX_2D
    s = ekf.update(0.2)
    assert vec.v3_norm(s.pos) < 1e-9  # nothing fused


def test_run_twice_bit_identical():
    def run():
        ekf = Ekf(perfect_align())
        sim = TruthSim(ekf, accel_fn=lambda t: (math.sin(t), 0.2, 0.0))
        s = sim.run(3.0)
        return (s.pos, s.vel, s.q, ekf.P.tobytes())

    assert run() == run()


def _dense_joseph(P, idx, innov, r_cov, gain_rows=None):
    """Textbook dense-H Joseph reference for a selection measurement."""
    m = len(idx)
    H = np.zeros((m, 15))
    H[np.arange(m), list(idx)] = 1.0
    S = H @ P @ H.T + r_cov
    S_inv = np.linalg.inv(S)
    K = P @ H.T @ S_inv
    if gain_rows is not None:
        keep = np.zeros(15, dtype=bool)
        keep[list(gain_rows)] = True
        K = np.where(keep[:, None], K, 0.0)
    dx = K @ innov
    ikh = np.eye(15) - K @ H
    Pn = ikh @ P @ ikh.T + K @ r_cov @ K.T
    return dx, 0.5 * (Pn + Pn.T)


def test_fuse_sel_matches_dense_joseph_reference():
    # P3 review (cut finding): the selection-indexed fusion's claimed
    # value-identity to the dense Joseph form was a one-time out-of-band
    # sha256 check with the dense path deleted — this DEFAULT-suite pin
    # re-derives every sensor block (incl. the baro partial update)
    # against a test-side dense reference so a selection-index regression
    # cannot hide behind the @slow-only statistical suites.
    ekf = Ekf(perfect_align(q0=vec.quat_from_euler(0.05, -0.03, 0.2)))
    sim = TruthSim(ekf, accel_fn=lambda t: (math.sin(t), 0.3, 0.1),
                   q0=vec.quat_from_euler(0.05, -0.03, 0.2))
    sim.run(2.0)
    assert not ekf.diverged
    p = ekf.params

    # GPS: position then velocity blocks, sequential like _fuse_gps
    P0 = ekf.P.copy()
    pos_b, vel_b = np.array(ekf.p), np.array(ekf.v)
    meas_pos = tuple(pos_b + (0.8, -0.5, 0.3))
    meas_vel = tuple(vel_b + (0.05, -0.02, 0.01))
    dx_p, P1 = _dense_joseph(P0, (0, 1, 2), np.array(meas_pos) - pos_b,
                             np.diag([p.gps_sigma_pos_h ** 2
                                      + p.gps_gm_sigma_h ** 2] * 2
                                     + [p.gps_sigma_pos_v ** 2
                                        + p.gps_gm_sigma_v ** 2]))
    vel_mid = vel_b + dx_p[3:6]      # pos block also moves velocity
    dx_v, P2 = _dense_joseph(P1, (3, 4, 5), np.array(meas_vel) - vel_mid,
                             np.diag([p.gps_sigma_vel ** 2] * 3))
    ekf._fuse_gps(meas_pos, meas_vel)
    assert np.allclose(ekf.P, P2, rtol=1e-8, atol=1e-14)
    assert np.allclose(np.array(ekf.p) - pos_b, dx_p[0:3] + dx_v[0:3],
                       rtol=1e-8, atol=1e-12)
    assert np.allclose(np.array(ekf.v) - vel_b, dx_p[3:6] + dx_v[3:6],
                       rtol=1e-8, atol=1e-12)

    # Baro: scalar partial update, gain masked to rows (2, 5, 14)
    P0 = ekf.P.copy()
    z_b = ekf.p[2]
    n_baro = ekf.nis["baro"][1]
    dx_b, P1 = _dense_joseph(
        P0, (2,), np.array([0.4]),
        np.array([[p.baro_sigma_m ** 2 + p.baro_drift_m ** 2]]),
        gain_rows=(2, 5, 14))
    ekf._fuse_baro(z_b + 0.4)
    assert ekf.nis["baro"][1] == n_baro + 1      # accepted, not gated
    assert np.allclose(ekf.P, P1, rtol=1e-8, atol=1e-14)
    assert abs((ekf.p[2] - z_b) - dx_b[2]) < 1e-12
    keep = np.zeros(15, dtype=bool)
    keep[[2, 5, 14]] = True
    assert np.all(dx_b[~keep] == 0.0)            # mask honored

    # Mag: scalar yaw fusion (lift P_yaw above the information floor —
    # converged filters legitimately sit on it and would floor-skip)
    ekf.P[8, 8] = 2.0 * ekf._yaw_floor_var
    P0 = ekf.P.copy()
    roll, pitch, yaw_est = vec.quat_to_euler(ekf.q)
    yaw_meas = yaw_est + 0.02
    decl = math.radians(p.mag_declination_deg)
    # field whose leveled atan2 yields yaw_meas (invert the model)
    ang = (0.5 * math.pi - decl) - yaw_meas
    m_l = (math.cos(ang), math.sin(ang), 0.0)
    field = vec.quat_rotate_inv(vec.quat_from_euler(roll, pitch, 0.0), m_l)
    n_mag = ekf.nis["mag"][1]
    dx_m, P1 = _dense_joseph(P0, (8,), np.array([0.02]),
                             np.array([[ekf._r_mag_yaw]]))
    ekf._fuse_mag(field)
    assert ekf.nis["mag"][1] == n_mag + 1
    assert np.allclose(ekf.P, P1, rtol=1e-7, atol=1e-13)

    # spoof-sized innovation: both reference NIS and the filter gate it
    n_rej = ekf.rejected["gps_pos"]
    ekf._fuse_gps(tuple(np.array(ekf.p) + 500.0), tuple(ekf.v))
    assert ekf.rejected["gps_pos"] == n_rej + 1


def test_diverged_latches_and_freezes_mainline():
    ekf = Ekf(perfect_align())
    sim = TruthSim(ekf)
    sim.run(1.0)
    ekf.P[0, 0] = math.nan
    ekf._guard()
    assert ekf.diverged
    s = sim.run(1.0, t0=1.0)
    assert s.diverged  # flag visible to the FCU failsafe seam


def test_diverged_stops_buffering_measurements():
    # P3 review F8: once diverged the mainline (the only drain) never
    # runs again, so intake must stop appending — otherwise the gps/
    # baro/mag deques grow unboundedly for the rest of the run.
    ekf = Ekf(perfect_align())
    sim = TruthSim(ekf)
    sim.run(1.0)
    ekf.P[0, 0] = math.nan
    ekf._guard()
    assert ekf.diverged
    lens = (len(ekf._gps), len(ekf._baro), len(ekf._mag))
    for i in range(100):
        t = 1.0 + i * 0.02
        ekf.on_gps(t, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 3)
        ekf.on_baro(t, 0.0)
        ekf.on_mag(t, (0.0, 50.0, 0.0))
    assert (len(ekf._gps), len(ekf._baro), len(ekf._mag)) == lens


def test_late_measurement_is_counted_not_silent():
    # P3 review F1: a stamp at/behind the fusion horizon can no longer
    # be fused at its own time — it is still used (better than dropped)
    # but the contract violation is counted (CBIT seam). The FCU driver
    # scheduling must keep this at zero against the real device timing
    # (pinned in test_coopfc_bench.py).
    ekf = Ekf(perfect_align())
    sim = TruthSim(ekf)
    sim.run(1.0)
    late = ekf.horizon - 0.01
    ekf.on_gps(late, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 3)
    ekf.on_baro(late, 0.0)
    ekf.on_mag(late, (0.0, 50.0, 0.0))
    assert ekf.late_meas == {"gps": 1, "baro": 1, "mag": 1}
    # on-time stamps stay uncounted
    ekf.on_baro(ekf.horizon + 0.02, 0.0)
    assert ekf.late_meas["baro"] == 1
