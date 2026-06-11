"""Seeker gimbal model: travel-limited, rate-limited two-axis servo with a
boresight FOV cone (PHY-UAV-012), batched over vehicles.

Closes the documented PHY-UAV-012 deviation "no gimbal FOV constraint":
the seeker only sees inside a half-angle cone about a boresight that has to
be slewed there first. Pointing is azimuth/elevation in the carrying body's
FLU axes (az positive toward +y/left about +z, el positive up):

    boresight = [cos el cos az, cos el sin az, sin el]

Servo: per axis and per step of dt,

    delta = clip(err * min(dt/tau, 1), -slew_max dt, +slew_max dt)

— a first-order lag of time constant tau under a hard slew-rate limit
[standard rate-limited servo form, e.g. Beard & McLain 2012 ch. 6 actuator
models]; min(dt/tau, 1) makes the discretization deadbeat (never
overshooting) when stepped slower than tau. Commands are clipped to the
mechanical travel limits at acceptance.

Deviations (documented): no gimbal dynamics coupling to airframe angular
rate (the stabilization loop is assumed ideal inside the slew budget) and a
single combined EO/IR cone rather than separate channels — the same
combined-channel simplification the legacy seeker carries (TRACEABILITY
PHY-UAV-012 row).

Deterministic device (no RNG): pointing noise is already carried by the
seeker's detection sigma.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def _require(cond: bool, msg: str) -> None:
    if not cond:
        raise ValueError(msg)


@dataclass(frozen=True)
class SeekerGimbalParams:
    """Immutable gimbal parameter set (hw/params/interceptor_devices.yaml)."""

    fov_half_deg: float
    slew_max_dps: float
    tau_s: float
    az_max_deg: float       # symmetric azimuth travel +-az_max
    el_min_deg: float
    el_max_deg: float

    def __post_init__(self):
        _require(np.isfinite(self.fov_half_deg)
                 and 0.0 < self.fov_half_deg <= 90.0,
                 f"fov_half_deg must be in (0, 90], got {self.fov_half_deg!r}")
        _require(np.isfinite(self.slew_max_dps) and self.slew_max_dps > 0.0,
                 f"slew_max_dps must be finite > 0, got {self.slew_max_dps!r}")
        _require(np.isfinite(self.tau_s) and self.tau_s > 0.0,
                 f"tau_s must be finite > 0, got {self.tau_s!r}")
        _require(np.isfinite(self.az_max_deg)
                 and 0.0 < self.az_max_deg <= 180.0,
                 f"az_max_deg must be in (0, 180], got {self.az_max_deg!r}")
        _require(np.isfinite(self.el_min_deg) and np.isfinite(self.el_max_deg)
                 and -90.0 <= self.el_min_deg < self.el_max_deg <= 90.0,
                 "el limits must satisfy -90 <= el_min < el_max <= 90, got "
                 f"[{self.el_min_deg!r}, {self.el_max_deg!r}]")

    @classmethod
    def from_dict(cls, cfg: dict) -> "SeekerGimbalParams":
        return cls(fov_half_deg=float(cfg["fov_half_deg"]),
                   slew_max_dps=float(cfg["slew_max_dps"]),
                   tau_s=float(cfg["tau_s"]),
                   az_max_deg=float(cfg["az_max_deg"]),
                   el_min_deg=float(cfg["el_min_deg"]),
                   el_max_deg=float(cfg["el_max_deg"]))


class SeekerGimbal:
    """n identical gimbals; boresight starts at the zero pose clipped into
    the travel band (az = el = 0 when the band contains it)."""

    def __init__(self, params: SeekerGimbalParams, n: int):
        _require(isinstance(n, int) and not isinstance(n, bool) and n >= 1,
                 f"n must be an int >= 1, got {n!r}")
        self.params = params
        self.n = n
        self._az_lim = np.radians(params.az_max_deg)
        self._el_lim = (np.radians(params.el_min_deg),
                        np.radians(params.el_max_deg))
        el0 = float(np.clip(0.0, *self._el_lim))
        self.az = np.zeros(n)
        self.el = np.full(n, el0)
        self._az_cmd = np.zeros(n)
        self._el_cmd = np.full(n, el0)
        self._slew = np.radians(params.slew_max_dps)
        self._cos_fov = np.cos(np.radians(params.fov_half_deg))

    def command(self, az_el: np.ndarray) -> None:
        """Set pointing commands (n, 2) rad; clipped to the travel limits."""
        az_el = np.asarray(az_el, dtype=float)
        _require(az_el.shape == (self.n, 2),
                 f"command expects shape ({self.n}, 2), got {az_el.shape}")
        _require(bool(np.all(np.isfinite(az_el))),
                 "command requires finite az/el")
        self._az_cmd = np.clip(az_el[:, 0], -self._az_lim, self._az_lim)
        self._el_cmd = np.clip(az_el[:, 1], *self._el_lim)

    def point_at(self, dir_body: np.ndarray) -> None:
        """Command the boresight toward (n, 3) body-FLU directions
        (any nonzero magnitude)."""
        d = np.asarray(dir_body, dtype=float)
        _require(d.shape == (self.n, 3),
                 f"point_at expects shape ({self.n}, 3), got {d.shape}")
        _require(bool(np.all(np.linalg.norm(d, axis=1) > 0.0)),
                 "point_at requires nonzero direction vectors")
        az = np.arctan2(d[:, 1], d[:, 0])
        el = np.arctan2(d[:, 2], np.hypot(d[:, 0], d[:, 1]))
        self.command(np.stack([az, el], axis=1))

    def step(self, dt: float) -> None:
        """Advance the servo one step of dt seconds."""
        _require(np.isfinite(dt) and dt > 0.0,
                 f"dt must be finite > 0, got {dt!r}")
        gain = min(dt / self.params.tau_s, 1.0)
        lim = self._slew * dt
        self.az = self.az + np.clip((self._az_cmd - self.az) * gain, -lim, lim)
        self.el = self.el + np.clip((self._el_cmd - self.el) * gain, -lim, lim)

    def boresight_body(self) -> np.ndarray:
        """Current boresight unit vectors (n, 3), body FLU."""
        ce = np.cos(self.el)
        return np.stack([ce * np.cos(self.az), ce * np.sin(self.az),
                         np.sin(self.el)], axis=1)

    def in_fov(self, dir_body: np.ndarray) -> np.ndarray:
        """(n,) bool: directions inside the closed FOV cone (edge inclusive,
        with an ulp guard); zero-length directions are outside."""
        d = np.asarray(dir_body, dtype=float)
        _require(d.shape == (self.n, 3),
                 f"in_fov expects shape ({self.n}, 3), got {d.shape}")
        norm = np.linalg.norm(d, axis=1)
        dot = np.einsum("ij,ij->i", d, self.boresight_body())
        with np.errstate(invalid="ignore", divide="ignore"):
            cos_angle = np.where(norm > 0.0, dot / norm, -2.0)
        return cos_angle >= self._cos_fov - 1e-12
