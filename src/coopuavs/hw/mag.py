"""Magnetometer model, batched over vehicles (PHY-UAV-011).

Measurement, body FLU, microtesla (citations in docs/RESEARCH.md):

    m = q^-1 B_world + b_hard_iron + b_gm + n_white  ->  quantize(lsb_ut)

B_world is the constant theater geomagnetic field (the sim arena is a few
km — field variation over it is far below sensor noise), built from
magnitude / declination / inclination in ENU:

    B = |B| [cos I sin D, cos I cos D, -sin I]    (D east of true north,
                                                   I dip below horizontal)

The error budget is the PX4/EKF2 magnetometer convention: a per-power-up
hard-iron offset (calibration residual; soft iron neglected — documented
deviation), a slow Gauss-Markov bias (vehicle current draw and temperature
lumped), and white noise.

Draw layout (frozen): construction, per vehicle child: standard_normal(6) =
[hard iron, GM cold start]; per sample: standard_normal(6) = [gm, white].
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from coopuavs.hw import stoch
from coopuavs.physics import rigid_body as rb


def _require(cond: bool, msg: str) -> None:
    if not cond:
        raise ValueError(msg)


def theater_field_enu(magnitude_ut: float, declination_deg: float,
                      inclination_deg: float) -> np.ndarray:
    """Theater geomagnetic field vector (3,) in world ENU, microtesla."""
    _require(np.isfinite(magnitude_ut) and magnitude_ut > 0.0,
             f"magnitude_ut must be finite > 0, got {magnitude_ut!r}")
    d = np.radians(declination_deg)
    i = np.radians(inclination_deg)
    return magnitude_ut * np.array(
        [np.cos(i) * np.sin(d), np.cos(i) * np.cos(d), -np.sin(i)])


@dataclass(frozen=True)
class MagParams:
    """Immutable magnetometer parameter set (hw/params/interceptor_devices.yaml).
    Units: microtesla (sensor-native); angles in degrees."""

    rate_hz: float
    magnitude_ut: float
    declination_deg: float
    inclination_deg: float
    sigma_ut: float
    gm_sigma_ut: float
    gm_tau_s: float
    hard_iron_sigma_ut: float
    lsb_ut: float

    def __post_init__(self):
        _require(np.isfinite(self.rate_hz) and self.rate_hz > 0.0,
                 f"rate_hz must be finite > 0, got {self.rate_hz!r}")
        _require(np.isfinite(self.magnitude_ut) and self.magnitude_ut > 0.0,
                 f"magnitude_ut must be finite > 0, got {self.magnitude_ut!r}")
        _require(np.isfinite(self.declination_deg)
                 and abs(self.declination_deg) <= 180.0,
                 f"declination_deg must be in [-180, 180], got "
                 f"{self.declination_deg!r}")
        _require(np.isfinite(self.inclination_deg)
                 and abs(self.inclination_deg) <= 90.0,
                 f"inclination_deg must be in [-90, 90], got "
                 f"{self.inclination_deg!r}")
        for field in ("sigma_ut", "gm_sigma_ut", "hard_iron_sigma_ut", "lsb_ut"):
            v = getattr(self, field)
            _require(np.isfinite(v) and v >= 0.0,
                     f"{field} must be finite >= 0, got {v!r}")
        _require(np.isfinite(self.gm_tau_s) and self.gm_tau_s > 0.0,
                 f"gm_tau_s must be finite > 0, got {self.gm_tau_s!r}")

    @classmethod
    def from_dict(cls, cfg: dict) -> "MagParams":
        return cls(rate_hz=float(cfg["rate_hz"]),
                   magnitude_ut=float(cfg["magnitude_ut"]),
                   declination_deg=float(cfg["declination_deg"]),
                   inclination_deg=float(cfg["inclination_deg"]),
                   sigma_ut=float(cfg["sigma_ut"]),
                   gm_sigma_ut=float(cfg["gm_sigma_ut"]),
                   gm_tau_s=float(cfg["gm_tau_s"]),
                   hard_iron_sigma_ut=float(cfg["hard_iron_sigma_ut"]),
                   lsb_ut=float(cfg["lsb_ut"]))


class Mag:
    """n identical magnetometers; one spawned child stream per vehicle."""

    def __init__(self, params: MagParams, n: int, rng: np.random.Generator):
        _require(isinstance(n, int) and not isinstance(n, bool) and n >= 1,
                 f"n must be an int >= 1, got {n!r}")
        self.params = params
        self.n = n
        self._b_world = np.broadcast_to(
            theater_field_enu(params.magnitude_ut, params.declination_deg,
                              params.inclination_deg), (n, 3))
        self._children = rng.spawn(n)
        init = np.stack([g.standard_normal(6) for g in self._children])
        self._hard_iron = params.hard_iron_sigma_ut * init[:, 0:3]
        self._gm = stoch.GaussMarkov(params.gm_sigma_ut, params.gm_tau_s,
                                     1.0 / params.rate_hz, init[:, 3:6])
        self._eps = np.empty((n, 6))

    def sample(self, quat: np.ndarray) -> np.ndarray:
        """One device tick at rate_hz: field (n, 3) body FLU, microtesla."""
        eps = self._eps
        for i, g in enumerate(self._children):
            g.standard_normal(out=eps[i])
        m = rb.quat_rotate_inv(quat, self._b_world)
        m += self._hard_iron
        m += self._gm.step(eps[:, 0:3])
        m += eps[:, 3:6] * self.params.sigma_ut
        return stoch.quantize(m, self.params.lsb_ut)
