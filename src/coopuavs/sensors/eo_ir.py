"""EO/IR camera tower (and, later, interceptor-mounted seeker).

The discriminator of the sensor suite: at close range a thermal/visual look
separates a plywood-and-foam decoy from a 200 kg OWA airframe (engine heat
plume, structure, payload bulges). Classification quality degrades smoothly
with range — encoded as a likelihood sharpening factor — so EO/IR only
resolves the decoy question once the target is close, by which time the
interceptors may already be committed. Positional accuracy is angular-good,
range-poor (monocular).
"""

from __future__ import annotations

import numpy as np

from ..core.messages import Detection, ThreatClass
from ..threats.enemy_drone import EnemyDrone
from .base import Sensor

# How easily each true class is confused with the others at long range.
_CONFUSABLE: dict[ThreatClass, ThreatClass] = {
    ThreatClass.DECOY: ThreatClass.OWA_STRATEGIC,
    ThreatClass.OWA_STRATEGIC: ThreatClass.DECOY,
    ThreatClass.OWA_JET: ThreatClass.OWA_STRATEGIC,
    ThreatClass.LOITERING: ThreatClass.FPV,
    ThreatClass.FPV: ThreatClass.LOITERING,
}


class EoIrSensor(Sensor):
    def __init__(
        self,
        name,
        world,
        position,
        max_range: float = 4000.0,
        rate_hz: float = 2.0,
        full_id_range: float = 1200.0,   # below this, near-certain ID
        pd: float = 0.85,
    ):
        super().__init__(name, world, position, max_range, rate_hz)
        self.full_id_range = full_id_range
        self.pd = pd

    def weather_factor(self) -> float:
        # Fog/precipitation attenuation and the EO-vs-IR lighting crossover;
        # the model is documented in :mod:`coopuavs.sim.weather`.
        return self.world.weather.eo_ir_range_factor()

    def observe(self, enemy: EnemyDrone, t: float) -> Detection | None:
        if self.rng.random() > self.pd:
            return None
        rel = enemy.position - self.position
        rng_m = float(np.linalg.norm(rel))
        los = rel / (rng_m + 1e-9)

        cross_sigma = 0.003 * rng_m + 1.0      # ~3 mrad angular error
        along_sigma = 0.25 * rng_m + 20.0      # poor monocular range
        noisy = enemy.position + self.rng.normal(0.0, cross_sigma, 3) \
            + los * self.rng.normal(0.0, along_sigma)
        cov = (
            np.eye(3) * cross_sigma**2
            + np.outer(los, los) * (along_sigma**2 - cross_sigma**2)
        )

        # Class likelihoods: at zero quality the look is *uninformative*
        # (true and confusable class equally likely — never inverted), and
        # ramps to near-certain identification inside full_id_range. Weather
        # shrinks both ranges, so identification quality degrades too.
        wx = self.weather_factor()
        eff_range, eff_id = self.max_range * wx, self.full_id_range * wx
        quality = float(np.clip(
            (eff_range - rng_m) / (eff_range - eff_id + 1e-9), 0.0, 1.0
        ))
        chance = 0.25
        p_true = chance + (0.98 - chance) * quality
        p_confusable = chance + (0.02 - chance) * quality
        confusable = _CONFUSABLE.get(enemy.threat_class)
        likelihoods = {c: 0.02 + 0.10 * (1.0 - quality)
                       for c in ThreatClass if c != ThreatClass.UNKNOWN}
        likelihoods[enemy.threat_class] = p_true
        if confusable is not None:
            likelihoods[confusable] = p_confusable

        return Detection(
            header=self._header(t),
            sensor_id=self.name,
            position=noisy,
            cov=cov,
            class_likelihoods=likelihoods,
        )
