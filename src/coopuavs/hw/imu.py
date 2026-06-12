"""Stochastic IMU model, batched over vehicles (PHY-UAV-011, SIM-SEN-001).

Measurement model per axis, body FLU (citations in docs/RESEARCH.md,
section "P2 hardware device models"):

    gyro  = omega_body + b0 + b_rw + b_gm + n_white  -> clip(full scale) -> quantize
    accel = f_body     + b0 + b_rw + b_gm + n_white  -> clip(full scale) -> quantize

(full scale = floor(range/lsb)*lsb, the top quantizer code inside the
configured range, so a saturated reading never rounds outside it.)

with the specific force f_body = q^-1 (a_world - g_world) — what a strapped
accelerometer triad reads: +g up at rest, zero in free fall. Error-budget
terms follow the Kalibr/PX4 convention [El-Sheimy et al. 2008, IEEE TIM
57(1); IEEE Std 952-1997]: white noise of density N (units/sqrt(Hz)),
first-order Gauss-Markov bias (the bias-instability proxy), bias random
walk K (units/sqrt(s)) and a per-power-up turn-on bias; range saturation
then ADC quantization close the chain (hw/stoch.py primitives).

Draw layout (frozen; the device consumes the same draws per tick whatever
sigmas are enabled, so re-tuning never shifts any stream):

    construction, per vehicle child:  standard_normal(12)
        [0:3] gyro turn-on  [3:6] accel turn-on
        [6:9] gyro GM cold start  [9:12] accel GM cold start
    per tick, per vehicle child:      standard_normal(18)
        [0:3] gyro white  [3:6] accel white  [6:9] gyro RW  [9:12] accel RW
        [12:15] gyro GM  [15:18] accel GM

``generate()`` produces the additive measurement noise (no clip/quantize,
which are truth-dependent) through the vectorized stoch ``run`` paths;
it is bit-for-bit the sample() loop (pinned) — the @slow Allan suite
depends on that equivalence.

The FIFO models the sensor's internal buffer between device rate and the
driver's drain rate (P3): ring of ``fifo_depth`` frames, overflow drops the
oldest and latches a flag until the next read (CBIT IMU_STALE hook, P5).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from coopuavs.hw import stoch
from coopuavs.physics import GRAVITY
from coopuavs.physics import rigid_body as rb

_INIT_DRAWS = 12
_TICK_DRAWS = 18
_CHUNK_BUDGET_ELEMS = 4_718_592   # generate() transient cap (~36 MB float64)


def _full_scale(range_: float, lsb: float) -> float:
    """Largest representable reading: the top quantizer code inside the
    configured range (floor(range/lsb) counts), so a saturated reading can
    never round to a grid point outside full scale — a real ADC's max code
    lies within full scale. lsb == 0 keeps the raw range."""
    if lsb == 0.0 or not np.isfinite(range_):
        return range_
    return np.floor(range_ / lsb) * lsb


def _require(cond: bool, msg: str) -> None:
    if not cond:
        raise ValueError(msg)


@dataclass(frozen=True)
class ImuParams:
    """Immutable IMU parameter set (see hw/params/interceptor_devices.yaml).

    Units: gyro rad/s, accel m/s^2; noise_density per sqrt(Hz); gm_sigma is
    the stationary GM std; rw_sigma per sqrt(s); lsb 0 = no quantization;
    range may be inf (no saturation)."""

    rate_hz: float
    gyro_noise_density: float
    gyro_gm_sigma: float
    gyro_gm_tau_s: float
    gyro_rw_sigma: float
    gyro_turn_on_sigma: float
    gyro_lsb: float
    gyro_range: float
    accel_noise_density: float
    accel_gm_sigma: float
    accel_gm_tau_s: float
    accel_rw_sigma: float
    accel_turn_on_sigma: float
    accel_lsb: float
    accel_range: float
    fifo_depth: int

    def __post_init__(self):
        _require(np.isfinite(self.rate_hz) and self.rate_hz > 0.0,
                 f"rate_hz must be finite > 0, got {self.rate_hz!r}")
        for side in ("gyro", "accel"):
            for field in ("noise_density", "gm_sigma", "rw_sigma",
                          "turn_on_sigma", "lsb"):
                v = getattr(self, f"{side}_{field}")
                _require(np.isfinite(v) and v >= 0.0,
                         f"{side}_{field} must be finite >= 0, got {v!r}")
            tau = getattr(self, f"{side}_gm_tau_s")
            _require(np.isfinite(tau) and tau > 0.0,
                     f"{side}_gm_tau_s must be finite > 0, got {tau!r}")
            rng_ = getattr(self, f"{side}_range")
            _require(not np.isnan(rng_) and rng_ > 0.0,
                     f"{side}_range must be > 0 (inf allowed), got {rng_!r}")
        _require(isinstance(self.fifo_depth, int)
                 and not isinstance(self.fifo_depth, bool)
                 and self.fifo_depth >= 1,
                 f"fifo_depth must be an int >= 1, got {self.fifo_depth!r}")

    @classmethod
    def from_dict(cls, cfg: dict) -> "ImuParams":
        g, a = cfg["gyro"], cfg["accel"]
        return cls(
            rate_hz=float(cfg["rate_hz"]),
            gyro_noise_density=float(g["noise_density"]),
            gyro_gm_sigma=float(g["gm_sigma"]),
            gyro_gm_tau_s=float(g["gm_tau_s"]),
            gyro_rw_sigma=float(g["rw_sigma"]),
            gyro_turn_on_sigma=float(g["turn_on_sigma"]),
            gyro_lsb=float(g["lsb"]),
            gyro_range=float(g["range"]),
            accel_noise_density=float(a["noise_density"]),
            accel_gm_sigma=float(a["gm_sigma"]),
            accel_gm_tau_s=float(a["gm_tau_s"]),
            accel_rw_sigma=float(a["rw_sigma"]),
            accel_turn_on_sigma=float(a["turn_on_sigma"]),
            accel_lsb=float(a["lsb"]),
            accel_range=float(a["range"]),
            fifo_depth=int(cfg["fifo_depth"]),
        )


class Imu:
    """n identical IMUs; one spawned child stream per vehicle (P0 contract)."""

    def __init__(self, params: ImuParams, n: int, rng: np.random.Generator):
        _require(isinstance(n, int) and not isinstance(n, bool) and n >= 1,
                 f"n must be an int >= 1, got {n!r}")
        self.params = params
        self.n = n
        self.dt = 1.0 / params.rate_hz
        self._children = rng.spawn(n)
        init = np.stack([g.standard_normal(_INIT_DRAWS) for g in self._children])
        self._b0_gyro = params.gyro_turn_on_sigma * init[:, 0:3]
        self._b0_accel = params.accel_turn_on_sigma * init[:, 3:6]
        self._gm_gyro = stoch.GaussMarkov(
            params.gyro_gm_sigma, params.gyro_gm_tau_s, self.dt, init[:, 6:9])
        self._gm_accel = stoch.GaussMarkov(
            params.accel_gm_sigma, params.accel_gm_tau_s, self.dt, init[:, 9:12])
        self._rw_gyro = stoch.RandomWalk(params.gyro_rw_sigma, self.dt, (n, 3))
        self._rw_accel = stoch.RandomWalk(params.accel_rw_sigma, self.dt, (n, 3))
        self._wn_gyro = params.gyro_noise_density / np.sqrt(self.dt)
        self._wn_accel = params.accel_noise_density / np.sqrt(self.dt)
        self._fs_gyro = _full_scale(params.gyro_range, params.gyro_lsb)
        self._fs_accel = _full_scale(params.accel_range, params.accel_lsb)
        self._g_world = np.array([0.0, 0.0, -GRAVITY])
        self._eps = np.empty((n, _TICK_DRAWS))
        self._fifo = np.empty((params.fifo_depth, n, 6))
        self._fifo_write = 0
        self._fifo_count = 0
        self._fifo_overflow = False
        # P5-2a fault seam (SIM-SIL-003), lazy: scales the WHITE noise
        # only (vibration/EMI class) on the same draws — no layout
        # change, no-fault path bit-identical.
        self._wn_fault: np.ndarray | None = None

    def set_noise_scale(self, i: int, scale: float) -> None:
        """Scale vehicle ``i``'s white gyro/accel noise (1.0 = healthy)."""
        if not scale > 0.0:
            raise ValueError(f"noise scale must be > 0, got {scale!r}")
        if self._wn_fault is None:
            self._wn_fault = np.ones((self.n, 1))
        self._wn_fault[i, 0] = float(scale)

    def sample(self, quat: np.ndarray, omega_body: np.ndarray,
               accel_world: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """One device tick at rate_hz: returns (gyro (n,3) rad/s,
        accel (n,3) m/s^2 specific force), both body FLU, and pushes the
        frame onto the FIFO. accel_world is the truth CoM acceleration
        (the plant's force_world / m, gravity included)."""
        eps = self._eps
        for i, g in enumerate(self._children):
            g.standard_normal(out=eps[i])
        p = self.params

        gyro = omega_body + self._b0_gyro
        gyro += self._rw_gyro.step(eps[:, 6:9])
        gyro += self._gm_gyro.step(eps[:, 12:15])
        if self._wn_fault is None:
            gyro += eps[:, 0:3] * self._wn_gyro
        else:
            gyro += eps[:, 0:3] * (self._wn_gyro * self._wn_fault)
        np.clip(gyro, -self._fs_gyro, self._fs_gyro, out=gyro)
        gyro = stoch.quantize(gyro, p.gyro_lsb)

        f_body = rb.quat_rotate_inv(quat, accel_world - self._g_world)
        accel = f_body + self._b0_accel
        accel += self._rw_accel.step(eps[:, 9:12])
        accel += self._gm_accel.step(eps[:, 15:18])
        if self._wn_fault is None:
            accel += eps[:, 3:6] * self._wn_accel
        else:
            accel += eps[:, 3:6] * (self._wn_accel * self._wn_fault)
        np.clip(accel, -self._fs_accel, self._fs_accel, out=accel)
        accel = stoch.quantize(accel, p.accel_lsb)

        self._fifo_push(gyro, accel)
        return gyro, accel

    def generate(self, steps: int) -> np.ndarray:
        """Vectorized additive measurement noise, (steps, n, 6) with gyro in
        [:, :, 0:3] and accel in [:, :, 3:6]; no clip/quantize (those are
        truth-dependent). Bit-exact with the sample() loop and advances the
        same state/streams — analysis use (Allan suite), not a fast path."""
        out = np.empty((steps, self.n, 6))
        chunk = max(1, _CHUNK_BUDGET_ELEMS // (self.n * _TICK_DRAWS))
        done = 0
        while done < steps:
            c = min(chunk, steps - done)
            eps = np.empty((c, self.n, _TICK_DRAWS))
            for i, g in enumerate(self._children):
                eps[:, i, :] = g.standard_normal((c, _TICK_DRAWS))
            seg = out[done:done + c]
            seg[:, :, 0:3] = self._b0_gyro + self._rw_gyro.run(eps[:, :, 6:9])
            seg[:, :, 0:3] += self._gm_gyro.run(eps[:, :, 12:15])
            seg[:, :, 0:3] += eps[:, :, 0:3] * self._wn_gyro
            seg[:, :, 3:6] = self._b0_accel + self._rw_accel.run(eps[:, :, 9:12])
            seg[:, :, 3:6] += self._gm_accel.run(eps[:, :, 15:18])
            seg[:, :, 3:6] += eps[:, :, 3:6] * self._wn_accel
            done += c
        return out

    def _fifo_push(self, gyro: np.ndarray, accel: np.ndarray) -> None:
        if self._fifo_count == self.params.fifo_depth:
            self._fifo_overflow = True       # full: overwrite the oldest
        else:
            self._fifo_count += 1
        w = self._fifo_write
        self._fifo[w, :, 0:3] = gyro
        self._fifo[w, :, 3:6] = accel
        self._fifo_write = (w + 1) % self.params.fifo_depth

    def fifo_read(self) -> tuple[np.ndarray, bool]:
        """Drain the FIFO: ((k, n, 6) oldest-first, overflowed-since-last-read).
        Reading clears both the buffer and the overflow flag."""
        k = self._fifo_count
        idx = (self._fifo_write - k + np.arange(k)) % self.params.fifo_depth
        frames = self._fifo[idx].copy()
        overflowed = self._fifo_overflow
        self._fifo_count = 0
        self._fifo_overflow = False
        return frames, overflowed
