"""Passive RF direction finder.

Reports a bearing to any emitting target plus the demodulated signature
hash. Crucially it CANNOT separate a Gerbera-style decoy from the OWA it
imitates — they emit the same signature by design — so RF alone inflates
the threat picture. That is the decoy problem the classifier must solve
with other modalities.

Bearing-only geometry is encoded as an anisotropic covariance: tight across
the line of sight, enormous along it.
"""

from __future__ import annotations

import numpy as np

from ..core.messages import Detection
from ..threats.enemy_drone import EnemyDrone
from .base import Sensor


class RfSensor(Sensor):
    def __init__(
        self,
        name,
        world,
        position,
        max_range: float = 15000.0,
        rate_hz: float = 1.0,
        bearing_sigma_deg: float = 2.0,
        pd: float = 0.9,
    ):
        super().__init__(name, world, position, max_range, rate_hz)
        self.bearing_sigma = np.deg2rad(bearing_sigma_deg)
        self.pd = pd

    def observe(self, enemy: EnemyDrone, t: float) -> Detection | None:
        if self.rng.random() > self.pd:
            return None
        rel = enemy.position - self.position
        rng_m = float(np.linalg.norm(rel))
        los = rel / (rng_m + 1e-9)

        # Perturb the line of sight by the bearing error.
        noise = self.rng.normal(0.0, self.bearing_sigma, 3)
        los_noisy = los + np.cross(noise, los)
        los_noisy /= np.linalg.norm(los_noisy) + 1e-9

        # Place the pseudo-position at an assumed range with the uncertainty
        # stretched along the line of sight.
        r_assumed = min(rng_m * self.rng.uniform(0.6, 1.6), self.max_range)
        pseudo = self.position + los_noisy * r_assumed
        cross_sigma = r_assumed * self.bearing_sigma + 10.0
        along_sigma = 0.6 * self.max_range
        cov = (
            np.eye(3) * cross_sigma**2
            + np.outer(los_noisy, los_noisy) * (along_sigma**2 - cross_sigma**2)
        )

        return Detection(
            header=self._header(t),
            sensor_id=self.name,
            position=pseudo,
            cov=cov,
            rf_signature=enemy.rf_signature,
        )
