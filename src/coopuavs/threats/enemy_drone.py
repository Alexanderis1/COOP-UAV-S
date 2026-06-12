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

Reactive evasion (SIM-THR-003): the agile classes (FPV, LOITERING) dodge
the nearest interceptor when it closes inside ``EVASION_RANGE`` — a lateral
break away from the pursuer's line of sight blended with the objective
heading, plus an altitude drop toward terrain masking. This is sim-side
logic and reads friendly truth (``world.friendlies``), which is legitimate:
a real FPV pilot sees the interceptor out of the window.
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
    terminal_speed: float = 0.0  # m/s in the terminal dive (0 -> = cruise)
    weave_ampl: float = 0.0  # lateral weave amplitude, m/s
    weave_period: float = 11.0

    def dive_speed(self) -> float:
        return self.terminal_speed if self.terminal_speed > 0.0 else self.speed

    def top_speed(self) -> float:
        return max(self.speed, self.terminal_speed)


# Profiles calibrated to verified 2025 open-source data on the Ukrainian
# theatre (see docs/RESEARCH.md, "Threat realism" appendix):
#   * Shahed-136/Geran-2 cruise ~180-210 km/h (50-58 m/s), but the 2025
#     profile flies HIGH (2.0-2.8 km AGL) and dives steeply (<=60 deg) at a
#     much greater speed onto the aimpoint;
#   * Geran-3 (jet Shahed-238): recorded cruise ~300-350 km/h (~85-97 m/s),
#     sprinting toward ~550-600 km/h (~150-167 m/s) in the terminal phase;
#   * Lancet-3 loitering munition: slow cruise ~80-110 km/h (22-30 m/s) with
#     a fast ~300 km/h (~83 m/s) terminal dive — the previous 80 m/s cruise
#     was unrealistically fast;
#   * FPV kamikaze: ~110-140 km/h typical, low and agile;
#   * Gerbera decoy: deliberately mirrors the strategic OWA profile.
THREAT_PROFILES: dict[ThreatClass, ThreatProfile] = {
    ThreatClass.OWA_STRATEGIC: ThreatProfile(
        speed=53.0, cruise_alt=2200.0, dive_range=2800.0, mass=200.0, rcs=0.5,
        warhead=True, terminal_speed=105.0, weave_ampl=4.0,
    ),
    ThreatClass.OWA_JET: ThreatProfile(
        speed=95.0, cruise_alt=2800.0, dive_range=4500.0, mass=200.0, rcs=0.6,
        warhead=True, terminal_speed=155.0,
    ),
    ThreatClass.FPV: ThreatProfile(
        speed=38.0, cruise_alt=80.0, dive_range=300.0, mass=3.0, rcs=0.02,
        warhead=True, terminal_speed=60.0, weave_ampl=6.0, weave_period=5.0,
    ),
    ThreatClass.LOITERING: ThreatProfile(
        speed=28.0, cruise_alt=400.0, dive_range=1500.0, mass=12.0, rcs=0.08,
        warhead=True, terminal_speed=83.0,
    ),
    # Decoy mimics the strategic OWA: same speed, altitude, RCS and RF
    # signature; lighter airframe and no warhead.
    ThreatClass.DECOY: ThreatProfile(
        speed=53.0, cruise_alt=2200.0, dive_range=2800.0, mass=18.0, rcs=0.5,
        warhead=False, terminal_speed=105.0, weave_ampl=4.0,
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

# Reactive evasion (SIM-THR-003): only the agile, man/seeker-in-the-loop
# classes manoeuvre against interceptors.
EVASIVE_CLASSES = {ThreatClass.FPV, ThreatClass.LOITERING}
EVASION_RANGE = 275.0      # m — start dodging inside this
EVASION_MIN_ALT = 30.0     # m — never dive into the ground while dodging
EVASION_SINK_RATE = 8.0    # m/s altitude-drop component at full evasion


class EnemyDrone:
    """One hostile platform, flying autonomously toward its target asset."""

    def __init__(
        self,
        drone_id: str,
        threat_class: ThreatClass,
        position: np.ndarray,
        target: np.ndarray,
        rng: np.random.Generator,
        world: "object | None" = None,
        target_name: str = "",
    ):
        self.id = drone_id
        self.threat_class = threat_class
        self.profile = THREAT_PROFILES[threat_class]
        self.rf_signature = RF_SIGNATURES[threat_class]
        self.target = np.asarray(target, dtype=float)
        self.target_name = target_name
        self.rng = rng
        self.world = world           # sim-side back-reference for evasion
        self.alive = True
        self.killed = False          # defeated by an interceptor
        self.reached_target = False  # leaker — defence failure
        # Evaluation bookkeeping (SIM-GT-002/003), written by sim-side nodes.
        self.spawn_t: float = 0.0
        self.acquired: bool = False
        self.acquired_t: float | None = None
        self.track_id: int | None = None
        self._phase = rng.uniform(0.0, 2 * np.pi)

        p = self.profile
        direction = self.target[:2] - position[:2]
        direction = direction / (np.linalg.norm(direction) + 1e-9)
        v0 = np.array([direction[0] * p.speed, direction[1] * p.speed, 0.0])
        # max_speed admits the terminal-dive sprint; cruise is commanded below.
        self.body = PointMass(position, v0, max_speed=p.top_speed(), max_accel=15.0)

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
            # Terminal phase: steep dive straight at the asset, accelerating
            # from cruise toward the (faster) terminal speed (SIM-THR-004).
            v_cmd = to_target / (np.linalg.norm(to_target) + 1e-9) * p.dive_speed()
        else:
            # Cruise toward the target at cruise altitude, with weave.
            heading = to_target[:2] / (dist_xy + 1e-9)
            lateral = np.array([-heading[1], heading[0]])
            weave = p.weave_ampl * np.sin(2 * np.pi * t / p.weave_period + self._phase)
            v_xy = heading * p.speed + lateral * weave
            v_z = np.clip((p.cruise_alt - self.position[2]) * 0.2, -10.0, 10.0)
            v_cmd = np.array([v_xy[0], v_xy[1], v_z])

        if self.threat_class in EVASIVE_CLASSES and self.world is not None:
            v_cmd = self._evade(v_cmd)

        self.body.command_velocity(v_cmd)
        self.body.step(dt)

        if np.linalg.norm(self.position - self.target) < 30.0:
            self.alive = False
            self.reached_target = True
        elif self.position[2] <= 0.0:
            # Ground impact away from the target: a crash, not a leaker.
            self.alive = False

    def _evade(self, v_cmd: np.ndarray) -> np.ndarray:
        """Blend the objective heading with a dodge against the nearest
        interceptor: lateral break across its line of sight + altitude drop,
        weighted up as the pursuer closes (SIM-THR-003)."""
        nearest, dist = None, EVASION_RANGE
        for uav in self.world.friendlies.values():
            d = float(np.linalg.norm(uav.position - self.position))
            if d < dist:
                nearest, dist = uav, d
        if nearest is None:
            return v_cmd

        w = 1.0 - dist / EVASION_RANGE          # 0 at the edge, 1 at contact
        speed = self.profile.speed
        los_xy = (self.position - nearest.position)[:2]
        n = np.linalg.norm(los_xy)
        if n < 1e-6:
            return v_cmd
        los_xy /= n
        # Two lateral break directions; pick the one that best preserves the
        # objective heading so the dodge is a weave, not a retreat.
        perp = np.array([-los_xy[1], los_xy[0]])
        if float(perp @ v_cmd[:2]) < 0.0:
            perp = -perp
        dodge_xy = 0.6 * perp + 0.4 * los_xy     # break across and away

        blend = (1.0 - 0.8 * w) * (v_cmd[:2] / (np.linalg.norm(v_cmd[:2]) + 1e-9)) \
            + 0.8 * w * dodge_xy
        bn = np.linalg.norm(blend)
        v_xy = blend / (bn + 1e-9) * speed
        v_z = v_cmd[2] - w * EVASION_SINK_RATE
        if self.position[2] < EVASION_MIN_ALT:
            v_z = max(v_z, 0.0)
        return np.array([v_xy[0], v_xy[1], v_z])

    def kill(self) -> None:
        self.alive = False
        self.killed = True
