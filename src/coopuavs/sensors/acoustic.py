"""Acoustic sensor array.

Short-ranged but immune to RF silence and radar horizon: it hears low-flying
FPVs the radar cannot see, and an internal-combustion OWA engine sounds
nothing like a decoy's small motor — so acoustics contribute both gap-filling
detections and a useful decoy cue. Bearing-only with coarse range from
intensity.
"""

from __future__ import annotations

import numpy as np

from ..core.messages import Detection, ThreatClass
from ..threats.enemy_drone import EnemyDrone
from .base import Sensor

# Acoustic class evidence: heavy piston engine (OWA) vs electric props.
_ACOUSTIC_LIKELIHOODS: dict[ThreatClass, dict[ThreatClass, float]] = {
    ThreatClass.OWA_STRATEGIC: {ThreatClass.OWA_STRATEGIC: 0.7, ThreatClass.DECOY: 0.1},
    ThreatClass.OWA_JET: {ThreatClass.OWA_JET: 0.7},
    ThreatClass.DECOY: {ThreatClass.DECOY: 0.6, ThreatClass.OWA_STRATEGIC: 0.2},
    ThreatClass.FPV: {ThreatClass.FPV: 0.6, ThreatClass.LOITERING: 0.2},
    ThreatClass.LOITERING: {ThreatClass.LOITERING: 0.6, ThreatClass.FPV: 0.2},
}


class AcousticSensor(Sensor):
    channel = "acoustic"

    def __init__(
        self,
        name,
        world,
        position,
        max_range: float = 900.0,
        rate_hz: float = 1.0,
        pd: float = 0.8,
    ):
        super().__init__(name, world, position, max_range, rate_hz)
        self.pd = pd

    def weather_factor(self) -> float:
        # Wind masking noise and rain hiss raise the detection floor.
        return self.world.weather.acoustic_range_factor()

    def observe(self, enemy: EnemyDrone, t: float,
                trans: float = 1.0) -> Detection | None:
        rel = enemy.position - self.position
        rng_m = float(np.linalg.norm(rel))
        # Audibility falls off with (weather-effective) range; buildings in
        # the path muffle but rarely silence (diffraction, SIM-SEN-005).
        if self.rng.random() > self.pd * trans * (1.0 - rng_m / self.effective_range()):
            return None
        los = rel / (rng_m + 1e-9)

        cross_sigma = 0.06 * rng_m + 5.0
        along_sigma = 0.4 * rng_m + 20.0
        noisy = enemy.position + self.rng.normal(0.0, cross_sigma, 3) \
            + los * self.rng.normal(0.0, along_sigma)
        cov = (
            np.eye(3) * cross_sigma**2
            + np.outer(los, los) * (along_sigma**2 - cross_sigma**2)
        )

        base = {c: 0.05 for c in ThreatClass if c != ThreatClass.UNKNOWN}
        base.update(_ACOUSTIC_LIKELIHOODS.get(enemy.threat_class, {}))

        return Detection(
            header=self._header(t),
            sensor_id=self.name,
            position=noisy,
            cov=cov,
            class_likelihoods=base,
        )
