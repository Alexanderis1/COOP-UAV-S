"""GNSS receiver model, batched over vehicles (PHY-UAV-011, SIM-SEN-001).

Error model (citations in docs/RESEARCH.md, "P2 hardware device models"):

    pos = truth_pos + b_gm + n_white      (per-axis, world ENU; horizontal /
    vel = truth_vel + n_white              vertical sigmas split)

The position error is white receiver noise over a slowly wandering
first-order Gauss-Markov term — the standard GNSS error decomposition
[Groves 2013, "Principles of GNSS, Inertial, and Multisensor Integrated
Navigation Systems", 2nd ed., ch. 9: correlated ionosphere/troposphere/
multipath residuals over tracking-loop noise]. Velocity is white (Doppler-
derived, far less correlated error).

Timing is integer-tick exact (the sil/clock.py rule, never float-compared):
``tick()`` is called once per device-clock tick at ``clock_hz``; the truth
is sampled every ``clock_hz / rate_hz`` ticks and the resulting fix is
returned exactly ``latency_s * clock_hz`` ticks later (120 ms = 96 ticks at
800 Hz). Both ratios must come out integral or construction fails — a rate
pairing that doesn't is a scenario error, never rounded.

``fix_type`` ships per vehicle (u-blox convention: 0 none / 2 2D / 3 3D);
the model emits FIX_3D — degradation and denial arrive with P5 fault
injection on this field.

Draw layout (frozen): construction, per vehicle child: standard_normal(3)
(GM cold start); per SAMPLE, per vehicle child: standard_normal(9) =
[0:3] GM, [3:6] position white, [6:9] velocity white.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

from coopuavs.hw import stoch

FIX_NONE = 0
FIX_2D = 2
FIX_3D = 3

_TOL = 1e-9


def _require(cond: bool, msg: str) -> None:
    if not cond:
        raise ValueError(msg)


@dataclass(frozen=True)
class GpsParams:
    """Immutable GNSS parameter set (see hw/params/interceptor_devices.yaml).

    Units: metres / seconds; sigmas are per-axis stds (h applies to ENU
    x and y, v to z); gm_* is the stationary correlated-wander process."""

    rate_hz: float
    latency_s: float
    sigma_pos_h: float
    sigma_pos_v: float
    gm_sigma_h: float
    gm_sigma_v: float
    gm_tau_s: float
    sigma_vel: float

    def __post_init__(self):
        _require(np.isfinite(self.rate_hz) and self.rate_hz > 0.0,
                 f"rate_hz must be finite > 0, got {self.rate_hz!r}")
        _require(np.isfinite(self.latency_s) and self.latency_s >= 0.0,
                 f"latency_s must be finite >= 0, got {self.latency_s!r}")
        for field in ("sigma_pos_h", "sigma_pos_v", "gm_sigma_h",
                      "gm_sigma_v", "sigma_vel"):
            v = getattr(self, field)
            _require(np.isfinite(v) and v >= 0.0,
                     f"{field} must be finite >= 0, got {v!r}")
        _require(np.isfinite(self.gm_tau_s) and self.gm_tau_s > 0.0,
                 f"gm_tau_s must be finite > 0, got {self.gm_tau_s!r}")

    @classmethod
    def from_dict(cls, cfg: dict) -> "GpsParams":
        return cls(
            rate_hz=float(cfg["rate_hz"]),
            latency_s=float(cfg["latency_s"]),
            sigma_pos_h=float(cfg["sigma_pos_h"]),
            sigma_pos_v=float(cfg["sigma_pos_v"]),
            gm_sigma_h=float(cfg["gm_sigma_h"]),
            gm_sigma_v=float(cfg["gm_sigma_v"]),
            gm_tau_s=float(cfg["gm_tau_s"]),
            sigma_vel=float(cfg["sigma_vel"]),
        )


@dataclass(frozen=True)
class GpsFix:
    """One delivered fix batch: measured at ``stamp_ticks`` (device clock),
    handed to the caller ``latency`` ticks later."""

    pos: np.ndarray        # (n, 3) m, world ENU
    vel: np.ndarray        # (n, 3) m/s, world ENU
    fix_type: np.ndarray   # (n,) uint8, FIX_* constants
    stamp_ticks: int
    stamp_s: float


def _exact_divisor(value: float, name: str) -> int:
    _require(value >= 1.0 - _TOL and abs(value - round(value)) <= _TOL * max(1.0, value),
             name)
    return round(value)


class Gps:
    """n identical receivers; one spawned child stream per vehicle."""

    def __init__(self, params: GpsParams, n: int, rng: np.random.Generator,
                 clock_hz: int):
        _require(isinstance(n, int) and not isinstance(n, bool) and n >= 1,
                 f"n must be an int >= 1, got {n!r}")
        _require(isinstance(clock_hz, int) and not isinstance(clock_hz, bool)
                 and clock_hz > 0,
                 f"clock_hz must be a positive integer, got {clock_hz!r}")
        self.params = params
        self.n = n
        self.clock_hz = clock_hz
        self._period_ticks = _exact_divisor(
            clock_hz / params.rate_hz,
            f"{params.rate_hz} Hz does not divide the {clock_hz} Hz device "
            "clock exactly")
        lat = params.latency_s * clock_hz
        _require(abs(lat - round(lat)) <= _TOL * max(1.0, lat),
                 f"latency {params.latency_s} s is not an integer number of "
                 f"ticks at {clock_hz} Hz")
        self._latency_ticks = round(lat)
        self._children = rng.spawn(n)
        init = np.stack([g.standard_normal(3) for g in self._children])
        self._gm = stoch.GaussMarkov(1.0, params.gm_tau_s,
                                     self._period_ticks / clock_hz, init)
        self._gm_scale = np.array([params.gm_sigma_h, params.gm_sigma_h,
                                   params.gm_sigma_v])
        self._wn_pos = np.array([params.sigma_pos_h, params.sigma_pos_h,
                                 params.sigma_pos_v])
        self._eps = np.empty((n, 9))
        self._k = 0
        self._queue: deque[tuple[int, GpsFix]] = deque()

    def tick(self, pos_world: np.ndarray, vel_world: np.ndarray) -> GpsFix | None:
        """One device-clock tick: samples truth on the rate lattice, returns
        the fix whose exact latency expires this tick (else None)."""
        k = self._k
        self._k = k + 1
        if k % self._period_ticks == 0:
            self._queue.append(
                (k + self._latency_ticks, self._measure(k, pos_world, vel_world)))
        if self._queue and self._queue[0][0] == k:
            return self._queue.popleft()[1]
        return None

    def _measure(self, k: int, pos_world: np.ndarray,
                 vel_world: np.ndarray) -> GpsFix:
        eps = self._eps
        for i, g in enumerate(self._children):
            g.standard_normal(out=eps[i])
        pos = pos_world + self._gm_scale * self._gm.step(eps[:, 0:3])
        pos += eps[:, 3:6] * self._wn_pos
        vel = vel_world + eps[:, 6:9] * self.params.sigma_vel
        fix_type = np.full(self.n, FIX_3D, dtype=np.uint8)
        return GpsFix(pos=pos, vel=vel, fix_type=fix_type,
                      stamp_ticks=k, stamp_s=k / self.clock_hz)
