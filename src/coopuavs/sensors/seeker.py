"""Onboard terminal seeker carried by each interceptor.

Every credible C-UAS interceptor (Anvil, DroneHunter, Sting) closes the last
few hundred metres on its own optical/radar seeker, not on the ground track
— a ground radar's tens-of-metres error and a CV filter's lag against a
weaving target are bigger than the effector envelope itself.

Modelled as a short-range, high-rate, low-noise sensor whose position rides
on the UAV airframe and whose detections feed the *same* fusion pipeline as
every other sensor: near an intercept the system track snaps to seeker
quality automatically, and any close-range decoy identification it makes
propagates to the C2 (which may then call off the engagement — ammunition
saved is the point of decoy discrimination).
"""

from __future__ import annotations

import numpy as np

from ..core.messages import Detection, ThreatClass
from ..interceptors.uav import InterceptorUav
from ..threats.enemy_drone import EnemyDrone
from .base import Sensor
from .eo_ir import _CONFUSABLE


class OnboardSeeker(Sensor):
    def __init__(
        self,
        name,
        world,
        uav: InterceptorUav,
        max_range: float = 600.0,
        rate_hz: float = 10.0,
        sigma: float = 3.0,
        id_quality: float = 0.85,
    ):
        super().__init__(name, world, uav.body.position, max_range, rate_hz)
        self.uav = uav
        self.sigma = sigma
        self.id_quality = id_quality

    def update(self, t: float, dt: float) -> None:
        self.position = self.uav.body.position   # seeker rides the airframe
        super().update(t, dt)

    def observe(self, enemy: EnemyDrone, t: float,
                trans: float = 1.0) -> Detection | None:
        # Optical seeker: any solid building blocks the look (the base
        # class skips trans == 0; the default eo_ir channel applies).
        rng_m = float(np.linalg.norm(enemy.position - self.position))
        noisy = enemy.position + self.rng.normal(0.0, self.sigma, 3)

        # Close-range visual identification: strong evidence for the true
        # class, residual confusion shrinking with proximity.
        q = self.id_quality * (1.0 - 0.5 * rng_m / self.max_range)
        likelihoods = {c: 0.02 for c in ThreatClass if c != ThreatClass.UNKNOWN}
        likelihoods[enemy.threat_class] = 0.25 + 0.73 * q
        confusable = _CONFUSABLE.get(enemy.threat_class)
        if confusable is not None:
            likelihoods[confusable] = 0.25 + (0.02 - 0.25) * q

        return Detection(
            header=self._header(t),
            sensor_id=self.name,
            position=noisy,
            cov=np.eye(3) * self.sigma**2,
            class_likelihoods=likelihoods,
        )
