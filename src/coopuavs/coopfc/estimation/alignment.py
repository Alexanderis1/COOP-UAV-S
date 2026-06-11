"""Static initial alignment: leveling, gyro bias, mag yaw, honest P0.

Pre-arm procedure (called by PBIT, P3-6): accumulate IMU at the driver
rate and mag at its rate while the vehicle sits still; once ``n_imu``
samples are in, compute

- roll/pitch from the mean specific force (at rest f_b = R^T g e_z, so
  ĝ_b = (-sinθ, sinφ cosθ, cosφ cosθ): pitch = asin(-ĝ_x),
  roll = atan2(ĝ_y, ĝ_z));
- gyro bias = mean gyro (truth rate is zero at rest);
- yaw from the leveled mag mean against the known theater declination:
  ψ = (π/2 - decl) - atan2(m_ly, m_lx)  (world ENU: x east, y north);
- a *variance gate*: per-axis sample stds above the thresholds mean the
  vehicle was moving or vibrating — result.ok = False and the FCU must
  retry (never silently align on motion).

P0 honesty: the returned 15-diag covariance covers what alignment cannot
observe — accel turn-on bias leaks b/g into tilt, hard-iron into yaw —
plus the statistical variance of each estimate. Position/velocity priors
stay wide-open until GNSS fusion. Plain-float on purpose: the add path
runs at 400 Hz inside PBIT.
"""

from __future__ import annotations

import math
from typing import NamedTuple

from coopuavs.coopfc import GRAVITY
from coopuavs.coopfc.core import vec


class AlignResult(NamedTuple):
    ok: bool                 # variance gate verdict; if False, retry
    q0: vec.Quat             # initial attitude, body -> world
    gyro_bias: vec.Vec3      # rad/s
    p0_diag: tuple           # 15 floats: [δp, δv, δθ, δb_g, δb_a] variances
    gyro_std: vec.Vec3       # diagnostics (per-axis sample stds)
    accel_std: vec.Vec3


class _Acc:
    """Per-axis running sum / sum-of-squares."""

    __slots__ = ("n", "s", "s2")

    def __init__(self):
        self.n = 0
        self.s = [0.0, 0.0, 0.0]
        self.s2 = [0.0, 0.0, 0.0]

    def add(self, x) -> None:
        self.n += 1
        for i in range(3):
            self.s[i] += x[i]
            self.s2[i] += x[i] * x[i]

    def mean(self):
        return tuple(si / self.n for si in self.s)

    def var(self):
        m = self.mean()
        return tuple(max(0.0, self.s2[i] / self.n - m[i] * m[i]) for i in range(3))


class Aligner:
    """Feed add_imu/add_mag while static; result() when n_imu reached."""

    def __init__(self,
                 n_imu: int = 800,
                 n_mag_min: int = 10,
                 mag_declination_deg: float = 0.0,
                 mag_inclination_deg: float = 63.0,
                 gyro_std_max: float = 0.02,      # rad/s gate
                 accel_std_max: float = 0.15,     # m/s^2 gate
                 accel_turn_on_sigma: float = 0.2,  # m/s^2 (device class)
                 gyro_gm_sigma: float = 4.0e-5,     # rad/s residual wander
                 mag_hard_iron_sigma: float = 2.0,  # uT per power-up
                 mag_field_ut: float = 50.0,
                 pos_prior_var: float = 1.0e4,    # m^2 — open until GNSS
                 vel_prior_var: float = 25.0):    # (m/s)^2
        if n_imu < 2:
            raise ValueError(f"n_imu must be >= 2, got {n_imu!r}")
        self.n_imu = n_imu
        self.n_mag_min = n_mag_min
        self.decl = math.radians(mag_declination_deg)
        self.incl = math.radians(mag_inclination_deg)
        self.gyro_std_max = gyro_std_max
        self.accel_std_max = accel_std_max
        self.accel_turn_on_sigma = accel_turn_on_sigma
        self.gyro_gm_sigma = gyro_gm_sigma
        self.mag_hard_iron_sigma = mag_hard_iron_sigma
        self.mag_field_ut = mag_field_ut
        self.pos_prior_var = pos_prior_var
        self.vel_prior_var = vel_prior_var
        self._gyro = _Acc()
        self._accel = _Acc()
        self._mag = _Acc()

    def add_imu(self, gyro: vec.Vec3, accel: vec.Vec3) -> None:
        self._gyro.add(gyro)
        self._accel.add(accel)

    def add_mag(self, field_ut: vec.Vec3) -> None:
        self._mag.add(field_ut)

    def result(self) -> AlignResult | None:
        if self._gyro.n < self.n_imu or self._mag.n < self.n_mag_min:
            return None

        gyro_var = self._gyro.var()
        accel_var = self._accel.var()
        gyro_std = tuple(math.sqrt(v) for v in gyro_var)
        accel_std = tuple(math.sqrt(v) for v in accel_var)
        ok = (max(gyro_std) <= self.gyro_std_max
              and max(accel_std) <= self.accel_std_max)

        # Leveling from the mean specific force.
        f = self._accel.mean()
        g_hat = vec.v3_normalize(f)
        pitch = math.asin(vec.clip(-g_hat[0], -1.0, 1.0))
        roll = math.atan2(g_hat[1], g_hat[2])

        # Yaw: level the mag mean with roll/pitch only, compare horizontal
        # angles against the theater declination (ENU: x east, y north).
        q_rp = vec.quat_from_euler(roll, pitch, 0.0)
        m_l = vec.quat_rotate(q_rp, self._mag.mean())
        yaw = vec.wrap_pi((0.5 * math.pi - self.decl)
                          - math.atan2(m_l[1], m_l[0]))
        q0 = vec.quat_from_euler(roll, pitch, yaw)

        # Honest P0 (variances). Statistical part scales 1/n; systematic
        # part is the unobservable turn-on bias / hard-iron leakage.
        n = self._gyro.n
        g2 = GRAVITY * GRAVITY
        tilt_var = (max(accel_var) / (n * g2)
                    + (self.accel_turn_on_sigma / GRAVITY) ** 2)
        b_h = self.mag_field_ut * math.cos(self.incl)  # horizontal field
        yaw_var = (max(self._mag.var()) / (max(1, self._mag.n) * b_h * b_h)
                   + (self.mag_hard_iron_sigma / b_h) ** 2)
        gyro_bias_var = (max(gyro_var) / n + self.gyro_gm_sigma ** 2)
        accel_bias_var = self.accel_turn_on_sigma ** 2

        p0 = ((self.pos_prior_var,) * 3
              + (self.vel_prior_var,) * 3
              + (tilt_var, tilt_var, yaw_var)
              + (gyro_bias_var,) * 3
              + (accel_bias_var,) * 3)
        return AlignResult(ok=ok, q0=q0, gyro_bias=self._gyro.mean(),
                           p0_diag=p0, gyro_std=gyro_std, accel_std=accel_std)
