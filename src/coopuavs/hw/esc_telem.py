"""ESC telemetry model, batched over vehicles (PHY-UAV-013).

BLHeli32/KISS-style telemetry: per-rotor mechanical shaft rpm plus pack
bus voltage and current, quantized to the protocol granularities, at the
configured frame rate. (The wire protocols transmit eRPM = shaft rpm x
pole pairs; this model carries the already-converted shaft rpm — the
pole-pair division is a driver constant, so `rpm_lsb` models the
granularity AFTER that conversion.) (PHY-UAV-013 wants health data northbound at
>= 1 Hz; the FCU consumes these frames in P3 and the MC republishes the
UavHealth digest in P5). Inputs are the Powertrain step outputs
(omega, v_bus, i_bus) — the telemetry chain adds only measurement noise
and quantization, never its own electrical model.

Deviations (documented in TRACEABILITY): voltage/current are pack-level
(the BatteryEcm has no per-cell states; per-cell imbalance telemetry
arrives with the P5 CELL_IMBALANCE fault work) and there is no temperature
channel (no thermal model — RESEARCH.md known limitation).

Draw layout (frozen): per sample, per vehicle child:
standard_normal(rotors + 2) = [rpm white x rotors, voltage white,
current white].
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from coopuavs.hw import stoch

RPM_PER_RAD_S = 60.0 / (2.0 * np.pi)


def _require(cond: bool, msg: str) -> None:
    if not cond:
        raise ValueError(msg)


@dataclass(frozen=True)
class EscTelemParams:
    """Immutable ESC telemetry parameter set
    (hw/params/interceptor_devices.yaml). Units: rpm / V / A."""

    rate_hz: float
    sigma_rpm: float
    sigma_v: float
    sigma_i: float
    rpm_lsb: float
    v_lsb: float
    i_lsb: float

    def __post_init__(self):
        _require(np.isfinite(self.rate_hz) and self.rate_hz > 0.0,
                 f"rate_hz must be finite > 0, got {self.rate_hz!r}")
        for field in ("sigma_rpm", "sigma_v", "sigma_i",
                      "rpm_lsb", "v_lsb", "i_lsb"):
            v = getattr(self, field)
            _require(np.isfinite(v) and v >= 0.0,
                     f"{field} must be finite >= 0, got {v!r}")

    @classmethod
    def from_dict(cls, cfg: dict) -> "EscTelemParams":
        return cls(rate_hz=float(cfg["rate_hz"]),
                   sigma_rpm=float(cfg["sigma_rpm"]),
                   sigma_v=float(cfg["sigma_v"]),
                   sigma_i=float(cfg["sigma_i"]),
                   rpm_lsb=float(cfg["rpm_lsb"]),
                   v_lsb=float(cfg["v_lsb"]),
                   i_lsb=float(cfg["i_lsb"]))


@dataclass(frozen=True)
class EscTelemFrame:
    """One telemetry frame batch."""

    rpm: np.ndarray       # (n, rotors) mechanical shaft rpm
    voltage: np.ndarray   # (n,) pack bus V
    current: np.ndarray   # (n,) pack bus A


class EscTelem:
    """n identical telemetry chains; one spawned child stream per vehicle."""

    def __init__(self, params: EscTelemParams, n: int, rotors: int,
                 rng: np.random.Generator):
        _require(isinstance(n, int) and not isinstance(n, bool) and n >= 1,
                 f"n must be an int >= 1, got {n!r}")
        _require(isinstance(rotors, int) and not isinstance(rotors, bool)
                 and rotors >= 1,
                 f"rotors must be an int >= 1, got {rotors!r}")
        self.params = params
        self.n = n
        self.rotors = rotors
        self._children = rng.spawn(n)
        self._eps = np.empty((n, rotors + 2))

    def sample(self, rotor_omega: np.ndarray, v_bus: np.ndarray,
               i_bus: np.ndarray) -> EscTelemFrame:
        """One frame at rate_hz from the Powertrain step outputs:
        rotor_omega (n, rotors) rad/s, v_bus (n,) V, i_bus (n,) A."""
        eps = self._eps
        for i, g in enumerate(self._children):
            g.standard_normal(out=eps[i])
        p = self.params
        r = self.rotors
        rpm = rotor_omega * RPM_PER_RAD_S + eps[:, :r] * p.sigma_rpm
        volt = v_bus + eps[:, r] * p.sigma_v
        curr = i_bus + eps[:, r + 1] * p.sigma_i
        return EscTelemFrame(rpm=stoch.quantize(rpm, p.rpm_lsb),
                             voltage=stoch.quantize(volt, p.v_lsb),
                             current=stoch.quantize(curr, p.i_lsb))
