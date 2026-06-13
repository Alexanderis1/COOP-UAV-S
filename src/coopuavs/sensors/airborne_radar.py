"""Airborne look-down early-warning radar (PHY-SNT-004).

The sensor that gives the high-altitude CAP sentinels their reason to be
forward: a pulse-Doppler search radar carried on the patrol platform,
looking *out and down* from above the ground clutter instead of *up* from
inside it.

Why it changes the engagement against a high-altitude diver
-----------------------------------------------------------
The base-station :class:`~coopuavs.sensors.radar.Radar` is twice penalised
against a jet OWA that cruises high and then dives:

* the radar **horizon** — the ``min_elevation`` clutter/terrain gate — and
  the ``R^4`` falloff together mean a confident track only forms once the
  target has closed a long way, and a 100 m/s diver spends that margin
  flying into the city faster than any 80 m/s interceptor can re-cut it;
* the look geometry is *up* into free space only for targets already high
  and near — a far, high diver sits at long slant range where ``Pd`` is low.

A sentinel orbiting at patrol altitude *forward* of the defended area turns
the geometry around: the diver is at or below the sentinel's own altitude
and only a few km of slant range away the instant it appears, so the
look-down radar paints it with high ``Pd`` seconds-to-tens-of-seconds before
the ground radar earns a confirmed track. Those seconds are exactly the
corridor the cooperative cutoff (``c2/assignment``) needs to post blockers
ahead of a target it cannot tail-chase. The value is *time margin*, not
extra kills per se — see ``scenarios/high_diver_raid.yaml`` and
``tests/test_airborne_radar.py``.

Model
-----
Same radar-equation-shaped ``Pd = pd_max * snr / (1 + snr)`` with ``snr ~
rcs / R^4`` as the ground radar (so the two are calibrated on one physical
story), with three deliberate differences:

* **look-down, no horizon** — the default ``min_elevation_deg`` is fully
  negative: an airborne platform has no terrain horizon and must see targets
  *below* it (that is the whole point). Depression angles pass.
* **longer reach** — a search radar carried for early warning, not the
  base's point-defence set; the default ``max_range`` spans the map so the
  binding limit is ``Pd`` (slant range), not a hard cut.
* **look-down ground clutter** — a target low over the ground competes with
  main-lobe clutter; ``Pd`` for targets below ``clutter_alt`` is scaled by
  ``clutter_factor``. This keeps the airborne set an *air-picture* sensor
  (it does not trivially solve the low-level FPV problem the acoustic
  pickets exist for, SIM-SEN rationale) while still owning the high divers.

Mount it on a sentinel with :func:`coopuavs.sensors.base.mounted` exactly
like the EO/IR and RF payloads; the position then tracks the platform every
scan and occlusion/weather coupling apply unchanged.
"""

from __future__ import annotations

import numpy as np

from ..core.messages import Detection
from ..threats.enemy_drone import EnemyDrone
from .radar import Radar


class AirborneRadar(Radar):
    #: Occlusion channel of MATERIAL_TRANSMISSION — radar, like the ground set.
    channel = "radar"

    def __init__(
        self,
        name,
        world,
        position,
        max_range: float = 22000.0,
        rate_hz: float = 5.0,
        reference_rcs: float = 0.4,      # a touch more sensitive than the base set
        pd_max: float = 0.97,
        min_elevation_deg: float = -90.0,  # look-down: no terrain horizon
        sigma_at_max_range: float = 90.0,
        clutter_alt: float = 250.0,      # below this AGL, main-lobe ground clutter
        clutter_factor: float = 0.30,    # Pd multiplier in the clutter band
    ):
        super().__init__(
            name, world, position,
            max_range=max_range, rate_hz=rate_hz, reference_rcs=reference_rcs,
            pd_max=pd_max, min_elevation_deg=min_elevation_deg,
            sigma_at_max_range=sigma_at_max_range,
        )
        self.clutter_alt = float(clutter_alt)
        self.clutter_factor = float(clutter_factor)

    def observe(self, enemy: EnemyDrone, t: float,
                trans: float = 1.0) -> Detection | None:
        # Written fresh (not super().observe) so the look-down clutter factor
        # folds into Pd *before* the single detection draw — the ground
        # Radar.observe stays byte-identical for the verified baseline.
        rel = enemy.position - self.position
        rng_m = float(np.linalg.norm(rel))
        elevation = np.arcsin(np.clip(rel[2] / (rng_m + 1e-9), -1, 1))
        if elevation < self.min_elevation:
            return None    # never trips at the default -90 deg (look-down)

        # SNR ~ rcs / R^4, normalised so a reference_rcs target at half the
        # weather-effective max range has snr = 1 (Radar rationale); building
        # transmittance applies two-way (SIM-SEN-005).
        snr = (enemy.rcs / self.reference_rcs) \
            * (0.5 * self.effective_range() / (rng_m + 1e-9)) ** 4
        snr *= trans ** 2
        pd = self.pd_max * snr / (1.0 + snr)
        # Look-down main-lobe clutter degrades detection of low targets.
        if enemy.position[2] < self.clutter_alt:
            pd *= self.clutter_factor
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
