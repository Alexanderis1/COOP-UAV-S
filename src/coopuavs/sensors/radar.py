"""Ground surveillance radar at the base station.

Models the dominant physical effects without waveform-level detail:

* detection probability driven by RCS and the radar equation's R^4 falloff;
* a radar horizon — targets below the minimum elevation angle (terrain and
  clutter masking) are invisible, which is exactly why low-flying FPVs leak
  through and other sensor modalities exist;
* range-dependent Cartesian measurement noise plus Doppler radial velocity.
"""

from __future__ import annotations

import numpy as np

from ..core.messages import Detection
from ..threats.enemy_drone import EnemyDrone
from .base import Sensor


class Radar(Sensor):
    channel = "radar"

    def __init__(
        self,
        name,
        world,
        position,
        max_range: float = 12000.0,
        rate_hz: float = 5.0,
        reference_rcs: float = 0.5,     # RCS giving snr=1 (Pd = pd_max/2) at half effective range
        pd_max: float = 0.95,
        min_elevation_deg: float = 1.5,
        sigma_at_max_range: float = 60.0,
    ):
        super().__init__(name, world, position, max_range, rate_hz)
        self.reference_rcs = reference_rcs
        self.pd_max = pd_max
        self.min_elevation = np.deg2rad(min_elevation_deg)
        self.sigma_at_max_range = sigma_at_max_range

    def weather_factor(self) -> float:
        return self.world.weather.radar_range_factor()

    def observe(self, enemy: EnemyDrone, t: float,
                trans: float = 1.0) -> Detection | None:
        rel = enemy.position - self.position
        rng_m = float(np.linalg.norm(rel))
        elevation = np.arcsin(np.clip(rel[2] / (rng_m + 1e-9), -1, 1))
        if elevation < self.min_elevation:
            return None

        # Radar-equation-shaped Pd = pd_max * snr / (1 + snr): SNR ~ rcs / R^4,
        # normalised so a reference_rcs target at half (weather-effective) max
        # range has snr = 1 and thus sees pd_max / 2; pd_max is the asymptotic
        # ceiling approached at close range. Precipitation mildly shrinks the
        # effective range. Building transmittance applies two-way (SIM-SEN-005).
        snr = (enemy.rcs / self.reference_rcs) * (0.5 * self.effective_range() / (rng_m + 1e-9)) ** 4
        snr *= trans ** 2
        pd = self.pd_max * snr / (1.0 + snr)
        if self.rng.random() > pd:
            return None

        sigma = self.sigma_at_max_range * (rng_m / self.max_range) + 2.0
        noisy = enemy.position + self.rng.normal(0.0, sigma, 3)
        los = rel / (rng_m + 1e-9)
        vr = float(enemy.velocity @ los) + self.rng.normal(0.0, 0.5)

        return Detection(
            header=self._header(t),
            sensor_id=self.name,
            position=noisy,
            cov=np.eye(3) * sigma**2,
            radial_velocity=vr,
            snr=float(snr),
        )
