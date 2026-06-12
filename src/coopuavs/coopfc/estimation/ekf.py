"""Sola error-state 15-state EKF with PX4-EKF2-style delayed horizon.

Architecture (PX4-EKF2 fusion pattern): the filter mainline lives at a
*fusion horizon* ``lag_s`` behind the present, so every measurement —
including the 120 ms-late GNSS fix — arrives "from the future" relative
to the horizon and is simply buffered until the horizon passes its
timestamp (stamp order, structural OOSM; the 0.5 s IMU ring buffer
bounds the history). The control-facing state at *now* is the horizon
state replayed through the buffered IMU samples (output predictor).
The "from the future" premise is host scheduling's to keep: ``lag_s``
must cover device latency PLUS any driver poll quantization — a stamp
already behind the horizon at intake is counted in ``late_meas`` (CBIT
seam) and fused late.

Error state (see estimation/__init__): [δp δv δθ δb_g δb_a], local/right
attitude error. Equations: Sola 2017 "Quaternion kinematics for the
error-state Kalman filter" — nominal kinematics eq. 255-259, error-state
transition eq. 270, injection eq. 282 (full citations in
docs/RESEARCH.md, "P3 CoopFC flight stack").

Fusion is sequential per sensor block with a chi-square innovation gate
(reject if NIS > gate^2 * dof — the PX4 convention); rejected counts are
the CBIT EKF_INNOV/GPS spoof seam. Measurement noise is inflated with
the *unmodeled* correlated error processes of the device suite (GNSS GM
wander, baro drift, mag hard-iron) — honesty over optimality, validated
by the NEES/NIS Monte-Carlo (@slow).

Robustness contract: the filter never raises in flight — non-finite
measurements are rejected and counted, a non-finite or non-PD covariance
latches ``diverged`` (the FCU failsafe seam) while the output keeps
dead-reckoning on the last good state.
"""

from __future__ import annotations

import math
from collections import deque
from typing import NamedTuple

import numpy as np

from coopuavs.coopfc import GRAVITY
from coopuavs.coopfc.core import vec
from coopuavs.coopfc.estimation.alignment import AlignResult, mag_yaw

# Error-state slices.
DP = slice(0, 3)
DV = slice(3, 6)
DTH = slice(6, 9)
DBG = slice(9, 12)
DBA = slice(12, 15)

_STAMP_EPS = 1e-9  # derived-float stamp comparisons get epsilon slack


def _strapdown_step(q, v, p, b_g, b_a, gyro, accel, dt):
    """One IMU sample of Sola eq. 255-259 nominal kinematics (ZOH from
    the sample stamp, trapezoid position). The SINGLE strapdown source
    for both the mainline integration and the output replay — the two
    must never diverge (a scheme tweak applied to one copy would be a
    test-evading offset between horizon and published state). Plain
    floats: this is the hot path."""
    w = (gyro[0] - b_g[0], gyro[1] - b_g[1], gyro[2] - b_g[2])
    f = (accel[0] - b_a[0], accel[1] - b_a[1], accel[2] - b_a[2])
    a_w = vec.quat_rotate(q, f)
    a_w = (a_w[0], a_w[1], a_w[2] - GRAVITY)
    p = (p[0] + v[0] * dt + 0.5 * a_w[0] * dt * dt,
         p[1] + v[1] * dt + 0.5 * a_w[1] * dt * dt,
         p[2] + v[2] * dt + 0.5 * a_w[2] * dt * dt)
    v = (v[0] + a_w[0] * dt, v[1] + a_w[1] * dt, v[2] + a_w[2] * dt)
    q = vec.quat_integrate(q, w, dt)
    return q, v, p


class EkfParams(NamedTuple):
    """Noise/architecture parameters; defaults match the interceptor
    device suite (hw/params/interceptor_devices.yaml) by value."""

    imu_rate_hz: float = 400.0
    update_rate_hz: float = 50.0
    lag_s: float = 0.14              # fusion horizon lag >= max sensor latency
    buffer_s: float = 0.5            # IMU/measurement ring depth
    # IMU continuous-time noise (Kalibr convention).
    gyro_noise_density: float = 8.7e-5    # rad/s/sqrt(Hz)
    gyro_rw_sigma: float = 1.0e-5         # rad/s/sqrt(s) bias RW
    accel_noise_density: float = 2.0e-3   # m/s^2/sqrt(Hz)
    accel_rw_sigma: float = 6.0e-5        # m/s^2/sqrt(s) bias RW
    # GNSS white + unmodeled GM inflation.
    gps_sigma_pos_h: float = 0.4
    gps_sigma_pos_v: float = 0.8
    gps_gm_sigma_h: float = 1.2
    gps_gm_sigma_v: float = 2.4
    gps_sigma_vel: float = 0.1
    gps_gate: float = 5.0
    # Baro (driver delivers metres) white + drift inflation.
    baro_sigma_m: float = 0.25
    baro_drift_m: float = 1.25
    baro_gate: float = 5.0
    # Mag field model + white + GM + hard-iron inflation.
    mag_field_ut: float = 50.0
    mag_declination_deg: float = 4.0
    mag_inclination_deg: float = 63.0
    mag_sigma_ut: float = 0.3
    mag_gm_sigma_ut: float = 0.5
    mag_hard_iron_sigma_ut: float = 2.0
    mag_gate: float = 5.0
    # GNSS-denied detection: no accepted position fusion for this long.
    denied_after_s: float = 1.0


class NavState(NamedTuple):
    """Control-facing output at `stamp` (= now, output-predicted)."""

    stamp: float
    q: vec.Quat          # body -> world
    vel: vec.Vec3        # m/s world ENU
    pos: vec.Vec3        # m world ENU
    omega: vec.Vec3      # rad/s body, bias-corrected latest gyro
    sigma_pos_h: float   # m, 1-sigma horizontal (max of x/y)
    sigma_pos_v: float   # m, 1-sigma vertical
    sigma_vel: float     # m/s, 1-sigma (max axis)
    diverged: bool


def _mag_field_enu(magnitude_ut: float, decl_deg: float, incl_deg: float):
    d, i = math.radians(decl_deg), math.radians(incl_deg)
    return np.array([magnitude_ut * math.cos(i) * math.sin(d),
                     magnitude_ut * math.cos(i) * math.cos(d),
                     -magnitude_ut * math.sin(i)])


def _skew(v) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def _rotmat(q: vec.Quat) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


class Ekf:
    """One vehicle's navigation filter; wire topics in at the FCU level."""

    def __init__(self, align: AlignResult, params: EkfParams = EkfParams(),
                 pos0: vec.Vec3 = (0.0, 0.0, 0.0),
                 vel0: vec.Vec3 = (0.0, 0.0, 0.0)):
        p = params
        self.params = p
        self._imu_dt = 1.0 / p.imu_rate_hz
        # Nominal state at the fusion horizon (plain floats: the output
        # replay path is hot).
        self.q = align.q0
        self.v = vel0
        self.p = pos0
        self.b_g = align.gyro_bias
        self.b_a = (0.0, 0.0, 0.0)
        self.P = np.diag(align.p0_diag).astype(float)
        self.horizon = 0.0
        # Time the nominal (horizon) state refers to: stamp of the last
        # integrated IMU sample + one IMU period (each sample is ZOH
        # forward from its stamp). NEES tooling compares truth here.
        self.state_time = 0.0
        self.diverged = False
        # Ring buffers (stamp-ordered appends by construction).
        n_imu = round(p.buffer_s * p.imu_rate_hz)
        self._imu: deque = deque(maxlen=n_imu)      # (stamp, gyro, accel)
        self._gps: deque = deque()                  # GpsMsg-shaped tuples
        self._baro: deque = deque()                 # (stamp, alt_m)
        self._mag: deque = deque()                  # (stamp, field_ut)
        self._last_gyro = (0.0, 0.0, 0.0)
        self.rejected = {"gps_pos": 0, "gps_vel": 0, "baro": 0, "mag": 0,
                         "nonfinite": 0}
        # Measurements that arrived with a stamp already at/behind the
        # fusion horizon: they can only be fused late, against a state
        # later than their own time (CBIT seam — the host's driver
        # scheduling must keep these at zero; still buffered, since a
        # slightly-stale fusion beats dropping the information).
        self.late_meas = {"gps": 0, "baro": 0, "mag": 0}
        # CBIT mag exclusion (P5-1d, user decision 2026-06-12): a
        # latched MAG_FAULT clears this and mag frames are dropped at
        # intake — a known-corrupted yaw source must not be fused OR
        # spam the reject tallies. Yaw then rides the gyro plus the
        # GPS-maneuver observability pathway (see the yaw-floor note in
        # _fuse_mag); naive course-as-yaw would be wrong for this fleet
        # (velocity-commanded strafing: course != heading).
        self.mag_trusted = True
        self.mag_excluded = 0
        # Accepted-fusion NIS tallies (per sensor: [sum, count]) — the
        # NIS half of the MC consistency suite reads these.
        self.nis = {"gps_pos": [0.0, 0], "gps_vel": [0.0, 0],
                    "baro": [0.0, 0], "mag": [0.0, 0]}
        self._m_world = _mag_field_enu(p.mag_field_ut, p.mag_declination_deg,
                                       p.mag_inclination_deg)
        # Measurement covariances (diagonals), inflated for unmodeled
        # correlated errors (GM wander / drift / hard-iron).
        self._r_gps_pos = np.array([
            p.gps_sigma_pos_h ** 2 + p.gps_gm_sigma_h ** 2,
            p.gps_sigma_pos_h ** 2 + p.gps_gm_sigma_h ** 2,
            p.gps_sigma_pos_v ** 2 + p.gps_gm_sigma_v ** 2])
        self._r_gps_vel = np.full(3, p.gps_sigma_vel ** 2)
        self._r_baro = p.baro_sigma_m ** 2 + p.baro_drift_m ** 2
        # Heading fusion measurement variance: white + GM + hard-iron
        # over the horizontal field magnitude.
        b_h = p.mag_field_ut * math.cos(math.radians(p.mag_inclination_deg))
        self._r_mag_yaw = ((p.mag_sigma_ut ** 2 + p.mag_gm_sigma_ut ** 2
                            + p.mag_hard_iron_sigma_ut ** 2) / (b_h * b_h))
        self._yaw_floor_var = (p.mag_hard_iron_sigma_ut / b_h) ** 2
        self.mag_tilt_skips = 0
        self.mag_floor_skips = 0
        # Unmodeled-error budget (variances, 9-dof pos/vel/att): colored
        # measurement errors a 15-state filter cannot estimate — GNSS GM
        # wander, baro drift, mag hard-iron yaw, accel-GM tilt with its
        # gravity leak into velocity. P alone under-reports truth error
        # by exactly these floors (the filter averages colored errors as
        # if white), so every *reported* sigma adds them, and the MC
        # NEES suite scores against P + diag(budget). RESEARCH.md "P3".
        # Tilt/yaw/vel floors are the hard-iron leak chain, with coupling
        # factors calibrated once against the device-suite MC
        # (test_coopfc_ekf_mc.py; rationale in RESEARCH.md "P3"): mag
        # fusion at non-zero roll leaks yaw bias into tilt through the
        # euler H row (~0.15x), tilt leaks g*sin into velocity between
        # GPS-vel corrections (~0.25 s effective), and occasional
        # floor-duty-cycle mag fusions leave ~0.3x of the hard-iron
        # level as correlated yaw residual.
        yaw_hi = p.mag_hard_iron_sigma_ut / b_h
        tilt_var = (0.15 * yaw_hi) ** 2
        vel_var = (GRAVITY * 0.15 * yaw_hi * 0.25) ** 2
        self.budget9 = np.array([
            p.gps_gm_sigma_h ** 2, p.gps_gm_sigma_h ** 2,
            max(p.gps_gm_sigma_v, p.baro_drift_m) ** 2,
            vel_var, vel_var, vel_var,
            tilt_var, tilt_var,
            (0.3 * yaw_hi) ** 2,
        ])
        self.last_gps_fuse: float | None = None
        self._denied_injected = False
        # Every measurement model is a pure state SELECTION (H rows are
        # unit vectors), so fusion uses indexed forms instead of 0/1 H
        # matmuls — value-identical (the dropped terms are exact +0.0
        # accumulations), ~2x cheaper per block (fleet perf, P3-8).
        # Constant per-sensor R matrices, precomputed once:
        self._R_gps_pos = np.diag(self._r_gps_pos)
        self._R_gps_vel = np.diag(self._r_gps_vel)
        self._R_baro = np.array([[self._r_baro]])
        self._R_mag_yaw = np.array([[self._r_mag_yaw]])
        # Baro partial-update gain mask {dp_z, dv_z, db_a_z}, built once
        # (constant per filter; rebuilt-per-fusion was pure churn).
        self._baro_gain_mask = np.zeros(15, dtype=bool)
        self._baro_gain_mask[[2, 5, 14]] = True
        # Prediction scratch (allocation-free per segment):
        self._eye15 = np.eye(15)
        self._F = np.eye(15)
        self._Q = np.zeros((15, 15))
        self._qidx = np.arange(3, 15)
        self._t1 = np.empty((15, 15))
        self._t2 = np.empty((15, 15))

    # ------------------------------------------------------------- intake

    def on_imu(self, stamp: float, gyro: vec.Vec3, accel: vec.Vec3) -> None:
        """Call at the IMU driver rate (400 Hz); plain-float cheap."""
        self._imu.append((stamp, gyro, accel))
        self._last_gyro = gyro

    def on_gps(self, fix_stamp: float, pos: vec.Vec3, vel: vec.Vec3,
               fix_type: int) -> None:
        if self.diverged:      # mainline (the only drain) never runs again
            return
        if fix_type >= 3:
            # late = strictly behind the horizon (a stamp AT the horizon
            # still fuses against the state of its own instant)
            if fix_stamp < self.horizon - _STAMP_EPS:
                self.late_meas["gps"] += 1
            self._gps.append((fix_stamp, pos, vel))

    def on_baro(self, stamp: float, alt_m: float) -> None:
        if self.diverged:
            return
        if stamp < self.horizon - _STAMP_EPS:
            self.late_meas["baro"] += 1
        self._baro.append((stamp, alt_m))

    def on_mag(self, stamp: float, field_ut: vec.Vec3) -> None:
        if not self.mag_trusted:
            self.mag_excluded += 1
            return
        if self.diverged:
            return
        if stamp < self.horizon - _STAMP_EPS:
            self.late_meas["mag"] += 1
        self._mag.append((stamp, field_ut))

    # ------------------------------------------------------------ mainline

    def update(self, now: float) -> NavState:
        """50 Hz task: advance the horizon, fuse, output-predict to now."""
        horizon_new = max(self.horizon, now - self.params.lag_s)
        if not self.diverged:
            self._mainline(horizon_new)
        self.horizon = horizon_new
        # GNSS-denial transition: while aided, the unmodeled-error
        # budget stays *out of P* (static floors keep fusion weights
        # sharp); once position aiding stops, the attitude/velocity
        # floors are real locked-in errors at denial onset that the
        # dynamics will double-integrate — inject them into P once per
        # denial event so the first seconds of predicted drift don't
        # start from the sharp aided covariance (the gyro-RW process
        # noise takes over the growth within ~10 s; MC denied suite).
        if (not self.diverged and self.last_gps_fuse is not None
                and not self._denied_injected
                and self.state_time - self.last_gps_fuse
                > self.params.denied_after_s):
            idx = np.arange(3, 9)
            self.P[idx, idx] += self.budget9[3:9]
            self._denied_injected = True
        # IMU history at or before the horizon is consumed (the output
        # replay starts there) — drop it so buffer scans stay short.
        while self._imu and self._imu[0][0] <= self.horizon + _STAMP_EPS:
            self._imu.popleft()
        return self._output(now)

    def _mainline(self, horizon_new: float) -> None:
        """Walk IMU samples in (horizon, horizon_new], fusing each
        buffered measurement at exactly its stamp.

        Device stamps live on the IMU lattice (all rates divide the
        device clock), so pausing the walk *before* integrating the
        sample at a measurement's stamp fuses against the nominal state
        of the measurement instant — no sub-period skew (a 20 m/s
        vehicle would otherwise see a 0.4 m systematic pull from one
        50 Hz period of mismatch; pinned by the OOSM test). Covariance
        is predicted per inter-fusion segment with segment-mean rates.
        """
        dt_im = self._imu_dt
        self._seg = [0.0] * 6
        self._seg_n = 0
        self._seg_q = self.q
        for stamp, gyro, accel in self._imu:
            if stamp <= self.horizon + _STAMP_EPS:
                continue
            if stamp > horizon_new + _STAMP_EPS:
                break
            if self._meas_due(stamp):
                self._flush_segment(dt_im)
                self._fuse_until(stamp)
                if self.diverged:
                    return
                self._seg_q = self.q
            self._integrate_nominal(gyro, accel, dt_im)
            self.state_time = stamp + dt_im
            s = self._seg
            s[0] += gyro[0]
            s[1] += gyro[1]
            s[2] += gyro[2]
            s[3] += accel[0]
            s[4] += accel[1]
            s[5] += accel[2]
            self._seg_n += 1
        self._flush_segment(dt_im)
        self._fuse_until(horizon_new)

    def _flush_segment(self, dt_im: float) -> None:
        n = self._seg_n
        if not n:
            return
        s = self._seg
        w_mean = np.array(s[0:3]) / n - np.array(self.b_g)
        f_mean = np.array(s[3:6]) / n - np.array(self.b_a)
        self._predict_cov(self._seg_q, w_mean, f_mean, n * dt_im)
        self._seg = [0.0] * 6
        self._seg_n = 0
        self._seg_q = self.q

    def _meas_due(self, t: float) -> bool:
        lim = t + _STAMP_EPS
        return ((bool(self._gps) and self._gps[0][0] <= lim)
                or (bool(self._baro) and self._baro[0][0] <= lim)
                or (bool(self._mag) and self._mag[0][0] <= lim))

    def _integrate_nominal(self, gyro, accel, dt) -> None:
        """One IMU sample into the horizon state (shared strapdown)."""
        self.q, self.v, self.p = _strapdown_step(
            self.q, self.v, self.p, self.b_g, self.b_a, gyro, accel, dt)

    def _predict_cov(self, q, w_mean, f_mean, dt) -> None:
        """Error-state transition, Sola eq. 270 (first order, mean rates).

        Scratch buffers, not fresh allocations (the 50 Hz x N-vehicle
        fleet path); identical arithmetic to the textbook form."""
        p = self.params
        rot = _rotmat(q)
        eye3 = np.eye(3)
        F = self._F
        np.copyto(F, self._eye15)
        F[DP, DV] = eye3 * dt
        F[DV, DTH] = -rot @ _skew(f_mean) * dt
        F[DV, DBA] = -rot * dt
        F[DTH, DTH] = eye3 - _skew(w_mean) * dt
        F[DTH, DBG] = -eye3 * dt

        Q = self._Q
        qi = self._qidx
        Q[qi, qi] = np.repeat(
            [p.accel_noise_density ** 2 * dt, p.gyro_noise_density ** 2 * dt,
             p.gyro_rw_sigma ** 2 * dt, p.accel_rw_sigma ** 2 * dt], 3)

        np.matmul(F, self.P, out=self._t1)
        np.matmul(self._t1, F.T, out=self._t2)
        self._t2 += Q
        self.P = 0.5 * (self._t2 + self._t2.T)
        self._guard()

    def _guard(self) -> None:
        d = np.diagonal(self.P)
        if not np.all(np.isfinite(self.P)) or np.any(d <= 0.0):
            self.diverged = True

    # -------------------------------------------------------------- fusion

    def _fuse_until(self, t: float) -> None:
        """Fuse every buffered measurement with stamp <= t, in stamp
        order across sensors (deterministic tie-break: gps, baro, mag)."""

        def due(buf):
            return buf and buf[0][0] <= t + _STAMP_EPS

        while True:
            candidates = []
            if due(self._gps):
                candidates.append((self._gps[0][0], 0, self._gps))
            if due(self._baro):
                candidates.append((self._baro[0][0], 1, self._baro))
            if due(self._mag):
                candidates.append((self._mag[0][0], 2, self._mag))
            if not candidates:
                return
            _, kind, buf = min(candidates)
            item = buf.popleft()
            if kind == 0:
                _, pos, gvel = item
                self._fuse_gps(pos, gvel)
            elif kind == 1:
                self._fuse_baro(item[1])
            else:
                self._fuse_mag(item[1])
            if self.diverged:
                return

    def _fuse_sel(self, idx: tuple[int, ...], innov: np.ndarray,
                  r_cov: np.ndarray, gate: float, name: str,
                  gain_mask: np.ndarray | None = None) -> bool:
        """Fuse a measurement whose H rows select state indices `idx`.

        Indexed equivalents of the dense forms (the H matmuls only ever
        accumulated exact +0.0 terms): S = P[idx,idx] + R;
        K = P[:,idx] S^-1. The Joseph update is expanded for a selection
        H into rank-m form — (I-KH)P = P - K P[idx,:] and
        X (I-KH)^T = X - X[:,idx] K^T — two rank-m products instead of
        two dense 15x15 matmuls (~5x cheaper on the fleet path; exact
        for ANY gain, including the masked partial update). Algebraic
        identity to the textbook dense Joseph form is pinned by
        test_coopfc_ekf.test_fuse_sel_matches_dense_joseph_reference.
        """
        if not np.all(np.isfinite(innov)):
            self.rejected["nonfinite"] += 1
            return False
        ix = np.ix_(idx, idx)
        S = self.P[ix] + r_cov
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.diverged = True
            return False
        nis = float(innov @ S_inv @ innov)
        dof = innov.shape[0]
        if nis > gate * gate * dof:
            self.rejected[name] += 1
            return False
        tally = self.nis[name]
        tally[0] += nis
        tally[1] += 1
        K = self.P[:, idx] @ S_inv
        if gain_mask is not None:
            # Partial update (Brink 2017 partial-update Schmidt-KF
            # form): zero the gain outside the mask rows. The Joseph
            # form below is exact for ANY gain, so P stays consistent
            # with the suboptimal-by-design K.
            K = np.where(gain_mask[:, None], K, 0.0)
        dx = K @ innov
        self._inject(dx)
        A = self.P - K @ self.P[idx, :]
        P = A - A[:, idx] @ K.T + K @ r_cov @ K.T
        self.P = 0.5 * (P + P.T)
        self._guard()
        return True

    def _inject(self, dx: np.ndarray) -> None:
        """Sola eq. 282: inject error into nominal, reset error to zero."""
        self.p = (self.p[0] + dx[0], self.p[1] + dx[1], self.p[2] + dx[2])
        self.v = (self.v[0] + dx[3], self.v[1] + dx[4], self.v[2] + dx[5])
        dth = (dx[6], dx[7], dx[8])
        ang = math.sqrt(dth[0] ** 2 + dth[1] ** 2 + dth[2] ** 2)
        if ang > 0.0:
            self.q = vec.quat_integrate(self.q, dth, 1.0)
        self.b_g = (self.b_g[0] + dx[9], self.b_g[1] + dx[10], self.b_g[2] + dx[11])
        self.b_a = (self.b_a[0] + dx[12], self.b_a[1] + dx[13], self.b_a[2] + dx[14])

    def _fuse_gps(self, pos, gvel) -> None:
        innov = np.array(pos) - np.array(self.p)
        if self._fuse_sel((0, 1, 2), innov, self._R_gps_pos,
                          self.params.gps_gate, "gps_pos"):
            self.last_gps_fuse = self.state_time
            self._denied_injected = False
        if self.diverged:
            return
        innov = np.array(gvel) - np.array(self.v)
        self._fuse_sel((3, 4, 5), innov, self._R_gps_vel,
                       self.params.gps_gate, "gps_vel")

    def _fuse_baro(self, alt_m: float) -> None:
        """Baro altitude, partial update: vertical channel only.

        The baro drift (GM, tau 600 s — effectively one offset per
        flight) is fused 50 times a second; treated as white it buys
        sqrt(N) fake information. Confined to {dp_z, dv_z, db_a_z} the
        damage is self-contained (R is inflated by the drift variance);
        let through the maneuver-built cross-covariances it quietly
        conditions TILT and YAW — measured on the GNSS-denied MC suite:
        claimed sigma_vel 3.1 m/s vs 67.5 m/s honest, a 20x covariance
        suppression while the true drift went km-class (the 4-sigma
        honesty gate caught it). Same colored-error mechanism as the
        mag yaw information floor above.
        """
        innov = np.array([alt_m - self.p[2]])
        self._fuse_sel((2,), innov, self._R_baro, self.params.baro_gate,
                       "baro", gain_mask=self._baro_gain_mask)

    def _fuse_mag(self, field_ut) -> None:
        """Heading-only mag fusion (the PX4-EKF2 default).

        3D vector fusion would let the per-power-up hard-iron offset
        (~2 uT of 50 uT = 2.3 deg of apparent field direction) pull
        *tilt*, which then leaks g*sin(err) into velocity — measured
        3.6 deg tilt / 0.19 m/s vel error in the MC suite. Heading
        fusion confines the hard-iron damage to yaw, where the
        unmodeled-error budget covers it. Skipped beyond ~72 deg tilt
        (leveling degenerates; counted, not rejected).
        """
        # Information floor: the hard-iron error is one fixed draw per
        # power-up, not white — re-fusing it at 50 Hz must not average
        # P_yaw below its variance. Once P_yaw reaches the floor, stop
        # fusing: P stays honest at sigma_yaw ~ hard-iron level, and
        # (crucially) GPS-velocity evidence during maneuvers then flows
        # into yaw instead of being mis-attributed to tilt/accel-bias
        # (measured 4-7 deg tilt walk before this guard; MC suite).
        if self.P[8, 8] <= self._yaw_floor_var:
            self.mag_floor_skips += 1
            return
        roll, pitch, yaw_est = vec.quat_to_euler(self.q)
        if abs(math.cos(pitch)) < 0.3:
            self.mag_tilt_skips += 1
            return
        yaw_mag = mag_yaw(roll, pitch,
                          (float(field_ut[0]), float(field_ut[1]),
                           float(field_ut[2])),
                          math.radians(self.params.mag_declination_deg))
        innov = np.array([vec.wrap_pi(yaw_mag - yaw_est)])
        # Yaw axis ONLY — deliberately not the full euler row
        # [0, sinφ/cosθ, cosφ/cosθ]: its tilt-coupling terms would let
        # thousands of *correlated* (hard-iron-biased) heading fusions
        # buy fake tilt information at non-zero roll (measured: P_tilt
        # quietly collapsed during GNSS-denial and the predicted drift
        # under-claimed 4x). The dropped terms are O(sin(tilt)) and the
        # colored mag cannot legitimately fund tilt knowledge.
        self._fuse_sel((8,), innov, self._R_mag_yaw, self.params.mag_gate,
                       "mag")

    # -------------------------------------------------------------- output

    def _output(self, now: float) -> NavState:
        """Replay buffered IMU from the horizon to `now` (output
        predictor; nominal only, no covariance).

        Full replay every update is the EXACT output prediction — a
        PX4-style incremental predictor (integrate each sample once,
        apply the post-fusion horizon delta as an additive correction)
        would be ~7x cheaper but approximate. Fidelity-first: the cost
        is bounded by the lag_s window (~56 samples) and covered by the
        @perf gates."""
        q, v, p = self.q, self.v, self.p
        b_g, b_a = self.b_g, self.b_a
        dt = self._imu_dt
        for stamp, gyro, accel in self._imu:
            if stamp <= self.horizon + _STAMP_EPS:
                continue
            if stamp > now + _STAMP_EPS:
                break
            q, v, p = _strapdown_step(q, v, p, b_g, b_a, gyro, accel, dt)
        g = self._last_gyro
        omega = (g[0] - b_g[0], g[1] - b_g[1], g[2] - b_g[2])
        d = np.diagonal(self.P)[:9] + self.budget9  # honest reporting
        return NavState(
            stamp=now, q=q, vel=v, pos=p, omega=omega,
            sigma_pos_h=float(np.sqrt(max(d[0], d[1]))),
            sigma_pos_v=float(np.sqrt(d[2])),
            sigma_vel=float(np.sqrt(max(d[3], d[4], d[5]))),
            diverged=self.diverged,
        )
