"""Hostile drone entities.

Threat profiles are calibrated to the operational taxonomy in the README
(Shahed-type OWA, jet OWA, FPV kamikaze, Lancet-type loitering munition,
Gerbera-type decoy). The crucial adversarial property is that a DECOY flies
the same profile and emits the same RF signature as the OWA it imitates —
discrimination must come from behaviour and close-range sensing, not from
the spawn data, which perception never sees.

Enemy drones are *not* cooperative: each runs an independent
waypoint-cruise-then-terminal-dive policy with optional weaving, which is
exactly the asymmetry the friendly side exploits.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..core.messages import ThreatClass
from ..sim.physics import PointMass


@dataclass
class ThreatProfile:
    speed: float            # cruise speed m/s
    cruise_alt: float       # m AGL
    dive_range: float       # horizontal distance to start terminal dive, m
    mass: float             # kg, drives debris severity (future use)
    rcs: float              # m^2-equivalent radar cross-section
    warhead: bool           # decoys carry none
    weave_ampl: float = 0.0  # lateral weave amplitude, m/s
    weave_period: float = 11.0


THREAT_PROFILES: dict[ThreatClass, ThreatProfile] = {
    ThreatClass.OWA_STRATEGIC: ThreatProfile(
        speed=55.0, cruise_alt=1500.0, dive_range=2500.0, mass=200.0, rcs=0.5,
        warhead=True, weave_ampl=4.0,
    ),
    ThreatClass.OWA_JET: ThreatProfile(
        speed=100.0, cruise_alt=3000.0, dive_range=4000.0, mass=200.0, rcs=0.6,
        warhead=True,
    ),
    ThreatClass.FPV: ThreatProfile(
        speed=33.0, cruise_alt=80.0, dive_range=300.0, mass=3.0, rcs=0.02,
        warhead=True, weave_ampl=6.0, weave_period=5.0,
    ),
    ThreatClass.LOITERING: ThreatProfile(
        speed=80.0, cruise_alt=400.0, dive_range=1500.0, mass=12.0, rcs=0.08,
        warhead=True,
    ),
    # Decoy mimics the strategic OWA: same speed, altitude, RCS and RF
    # signature; lighter airframe and no warhead.
    ThreatClass.DECOY: ThreatProfile(
        speed=55.0, cruise_alt=1500.0, dive_range=2500.0, mass=18.0, rcs=0.5,
        warhead=False, weave_ampl=4.0,
    ),
}

# RF signature emitted per class; the decoy intentionally collides with the
# strategic OWA signature (Gerbera tactic).
RF_SIGNATURES: dict[ThreatClass, str] = {
    ThreatClass.OWA_STRATEGIC: "sig-owa-a",
    ThreatClass.OWA_JET: "sig-owa-jet",
    ThreatClass.FPV: "sig-fpv",
    ThreatClass.LOITERING: "sig-loiter",
    ThreatClass.DECOY: "sig-owa-a",
}


class EnemyDrone:
    """One hostile platform, flying autonomously toward its target asset."""

    def __init__(
        self,
        drone_id: str,
        threat_class: ThreatClass,
        position: np.ndarray,
        target: np.ndarray,
        rng: np.random.Generator,
    ):
        self.id = drone_id
        self.threat_class = threat_class
        self.profile = THREAT_PROFILES[threat_class]
        self.rf_signature = RF_SIGNATURES[threat_class]
        self.target = np.asarray(target, dtype=float)
        self.rng = rng
        self.alive = True
        self.killed = False          # defeated by an interceptor
        self.reached_target = False  # leaker — defence failure
        self._phase = rng.uniform(0.0, 2 * np.pi)

        p = self.profile
        direction = self.target[:2] - position[:2]
        direction = direction / (np.linalg.norm(direction) + 1e-9)
        v0 = np.array([direction[0] * p.speed, direction[1] * p.speed, 0.0])
        self.body = PointMass(position, v0, max_speed=p.speed, max_accel=15.0)

    # -- accessors used by sensors/world -------------------------------------

    @property
    def position(self) -> np.ndarray:
        return self.body.position

    @property
    def velocity(self) -> np.ndarray:
        return self.body.velocity

    @property
    def rcs(self) -> float:
        return self.profile.rcs

    # -- behaviour -------------------------------------------------------------

    def step(self, dt: float, t: float) -> None:
        if not self.alive:
            return
        p = self.profile
        to_target = self.target - self.position
        dist_xy = float(np.linalg.norm(to_target[:2]))

        if dist_xy < p.dive_range:
            # Terminal phase: straight at the asset.
            v_cmd = to_target / (np.linalg.norm(to_target) + 1e-9) * p.speed
        else:
            # Cruise toward the target at cruise altitude, with weave.
            heading = to_target[:2] / (dist_xy + 1e-9)
            lateral = np.array([-heading[1], heading[0]])
            weave = p.weave_ampl * np.sin(2 * np.pi * t / p.weave_period + self._phase)
            v_xy = heading * p.speed + lateral * weave
            v_z = np.clip((p.cruise_alt - self.position[2]) * 0.2, -10.0, 10.0)
            v_cmd = np.array([v_xy[0], v_xy[1], v_z])

        self.body.command_velocity(v_cmd)
        self.body.step(dt)

        if np.linalg.norm(self.position - self.target) < 30.0 or self.position[2] <= 0.0:
            self.alive = False
            self.reached_target = True

    def kill(self) -> None:
        self.alive = False
        self.killed = True
