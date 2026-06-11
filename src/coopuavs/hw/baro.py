"""Barometric pressure sensor model, batched over vehicles (PHY-UAV-011).

Measurement (citations in docs/RESEARCH.md, "P2 hardware device models"):

    p = p_ISA(alt) + b_gm + n_white  ->  quantize(lsb_pa)

True static pressure comes from the shared ISA model (physics/atmosphere.py
— the same atmosphere the plants fly in, one datum); the error budget is
white ADC/conversion noise over a slow Gauss-Markov drift (temperature and
weather-trend effects lumped, the PX4 baro-bias convention: EKF2 estimates
exactly such a slowly-varying baro offset). ``altitude_from_pressure`` is
the exact ISA inverse — the P3 driver's conversion — making the round trip
pinnable: sigma_h = sigma_p / (rho g0) by the hydrostatic relation.

Draw layout (frozen): construction, per vehicle child: standard_normal(1)
(GM cold start); per sample: standard_normal(2) = [gm, white].
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from coopuavs.hw import stoch
from coopuavs.physics import atmosphere


def _require(cond: bool, msg: str) -> None:
    if not cond:
        raise ValueError(msg)


def altitude_from_pressure(p_pa) -> np.ndarray:
    """Exact inverse of atmosphere.pressure (ISA troposphere):
    h = (T0/L) (1 - (p/p0)^(R L / g0))."""
    p = np.asarray(p_pa, dtype=float)
    if not np.all(np.isfinite(p)) or np.any(p <= 0.0):
        raise ValueError("altitude_from_pressure requires finite p > 0")
    exponent = atmosphere.R_AIR * atmosphere.ISA_LAPSE / atmosphere.G0
    return (atmosphere.ISA_T0 / atmosphere.ISA_LAPSE) * (
        1.0 - (p / atmosphere.ISA_P0) ** exponent)


@dataclass(frozen=True)
class BaroParams:
    """Immutable barometer parameter set (hw/params/interceptor_devices.yaml).
    Units: Pa; gm_* is the stationary drift process."""

    rate_hz: float
    sigma_pa: float
    gm_sigma_pa: float
    gm_tau_s: float
    lsb_pa: float

    def __post_init__(self):
        _require(np.isfinite(self.rate_hz) and self.rate_hz > 0.0,
                 f"rate_hz must be finite > 0, got {self.rate_hz!r}")
        for field in ("sigma_pa", "gm_sigma_pa", "lsb_pa"):
            v = getattr(self, field)
            _require(np.isfinite(v) and v >= 0.0,
                     f"{field} must be finite >= 0, got {v!r}")
        _require(np.isfinite(self.gm_tau_s) and self.gm_tau_s > 0.0,
                 f"gm_tau_s must be finite > 0, got {self.gm_tau_s!r}")

    @classmethod
    def from_dict(cls, cfg: dict) -> "BaroParams":
        return cls(rate_hz=float(cfg["rate_hz"]),
                   sigma_pa=float(cfg["sigma_pa"]),
                   gm_sigma_pa=float(cfg["gm_sigma_pa"]),
                   gm_tau_s=float(cfg["gm_tau_s"]),
                   lsb_pa=float(cfg["lsb_pa"]))


class Baro:
    """n identical barometers; one spawned child stream per vehicle."""

    def __init__(self, params: BaroParams, n: int, rng: np.random.Generator):
        _require(isinstance(n, int) and not isinstance(n, bool) and n >= 1,
                 f"n must be an int >= 1, got {n!r}")
        self.params = params
        self.n = n
        self._children = rng.spawn(n)
        init = np.concatenate([g.standard_normal(1) for g in self._children])
        self._gm = stoch.GaussMarkov(params.gm_sigma_pa, params.gm_tau_s,
                                     1.0 / params.rate_hz, init)
        self._eps = np.empty((n, 2))

    def sample(self, alt_m: np.ndarray) -> np.ndarray:
        """One device tick at rate_hz: static pressure (n,) Pa at the given
        true altitudes (m AMSL; the ISA validity checks apply)."""
        eps = self._eps
        for i, g in enumerate(self._children):
            g.standard_normal(out=eps[i])
        p = atmosphere.pressure(alt_m) + self._gm.step(eps[:, 0])
        p += eps[:, 1] * self.params.sigma_pa
        return stoch.quantize(p, self.params.lsb_pa)
