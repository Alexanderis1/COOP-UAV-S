"""Engagement adjudicator — the sim-side referee.

Listens to ``engagement/fire`` events and resolves them against ground
truth: true relative geometry decides the real kill probability, one RNG
roll decides the outcome, and on a kill the debris model places the actual
wreck on the risk map. Plays the role a Gazebo contact/effects plugin will
play after migration; no tactical logic lives here.

Two shooter families are adjudicated:

* **UAV effectors** — Pk from the effector envelope against true relative
  geometry (SIM-EFF-001/002), unchanged from v0.1;
* **ground turrets** (SIM-EFF-003/004) — per-burst hit probability from
  barrel dispersion, range, time of flight and target speed; on a miss the
  rounds that crossed the target's range continue on the ballistic line and
  their terminal ground impacts are scored against the risk map and stored
  in ``world.stray_impacts`` — stray rounds are collateral too, not only
  wrecks.
"""

from __future__ import annotations

import numpy as np

from ..core.messages import EngagementResult, FireRequest, Header
from ..core.node import Node
from ..interceptors.uav import InterceptorUav
from ..risk.debris import velocity_retention
from ..sim.physics import GRAVITY
from .debris_objects import FallingDebris
from .world import World

# Effective lethal radius of one HE/frag round against a small airframe, m.
TURRET_LETHAL_RADIUS = 2.0
# Fraction of target speed treated as unpredictable lateral motion over TOF.
# 0.15 models lead-corrected fire control against largely ballistic cruise
# profiles (v0.3 hit-rate review: at 0.3 the evasion term dominated sigma
# beyond ~400 m and turrets sprayed hopeless 2-5% bursts — pure stray-round
# pollution; the residual 15% covers weave and estimate error).
TURRET_EVASION_FACTOR = 0.15


class EngagementAdjudicator(Node):
    def __init__(
        self,
        world: World,
        uavs: dict[str, InterceptorUav],
        turrets: dict[str, object] | None = None,
    ):
        super().__init__("adjudicator", world.bus, rate_hz=50.0)
        self.world = world
        self.uavs = uavs
        self.turrets = turrets or {}
        self._result_pub = self.create_publisher("engagement/result")
        self.create_subscription("engagement/fire", self._on_fire)

    def _on_fire(self, msg: FireRequest) -> None:
        if msg.target_kind == "debris":
            self._on_debris_fire(msg)
            return
        if msg.uav_id in self.turrets:
            self._on_turret_fire(msg, self.turrets[msg.uav_id])
            return

        uav = self.uavs[msg.uav_id]
        target = self._nearest_enemy(msg.predicted_intercept)
        result = EngagementResult(
            header=Header(stamp=self.world.t),
            task_id=msg.task_id,
            track_id=msg.track_id,
            uav_id=msg.uav_id,
        )

        if target is not None:
            if not self.world.occlusion.clear(uav.position, target.position):
                # A building stands in the sight line (SIM-EFF-006): the
                # munition cannot reach the target; no Pk roll.
                self.world.log_event(
                    "fire_blocked_los", uav_id=msg.uav_id, enemy_id=target.id,
                    effector=uav.effector.type.value,
                )
                self._result_pub.publish(result)
                return
            rel = target.position - uav.position
            pk_true = uav.effector.p_kill(rel, uav.velocity, target.velocity)
            result.pk = pk_true
            result.effector = uav.effector.type
            if self.world.rng.random() < pk_true:
                self._register_kill(result, target, uav.effector.type, msg.uav_id, pk_true)
            else:
                self.world.log_event(
                    "miss", uav_id=msg.uav_id, enemy_id=target.id,
                    effector=uav.effector.type.value, pk=round(pk_true, 3),
                    target_kind="track",
                )
        else:
            self.world.log_event("fire_no_target", uav_id=msg.uav_id,
                                 effector=uav.effector.type.value,
                                 target_kind="track")

        self._result_pub.publish(result)

    # -- turret bursts (SIM-EFF-003/004) ---------------------------------------

    def _on_turret_fire(self, msg: FireRequest, turret) -> None:
        result = EngagementResult(
            header=Header(stamp=self.world.t),
            task_id=msg.task_id,
            track_id=msg.track_id,
            uav_id=msg.uav_id,
        )
        target = self._nearest_enemy(msg.predicted_intercept, gate=150.0)
        n_rounds = msg.rounds if msg.rounds > 0 else turret.rounds_per_burst

        if target is not None and not self.world.occlusion.clear(
                turret.position, target.position):
            # Masked by a building (SIM-EFF-006): the burst cannot connect,
            # but the rounds were fired and still land somewhere.
            self.world.log_event(
                "fire_blocked_los", uav_id=msg.uav_id, enemy_id=target.id,
                effector=msg.effector.value,
            )
            self._stray_rounds(turret, msg.predicted_intercept, n_rounds)
            self._result_pub.publish(result)
            return

        if target is not None:
            dist = float(np.linalg.norm(target.position - turret.position))
            tof = dist / turret.muzzle_velocity
            # Hit probability per round: gun dispersion grows linearly with
            # range, target unpredictability with speed * TOF; a round kills
            # inside the lethal radius of the resulting 2D error ellipse.
            sigma2 = (turret.dispersion_mrad * 1e-3 * dist) ** 2 \
                + (TURRET_EVASION_FACTOR * float(np.linalg.norm(target.velocity)) * tof) ** 2
            p_round = 1.0 - float(np.exp(-(TURRET_LETHAL_RADIUS**2) / (2.0 * sigma2 + 1e-9)))
            p_burst = 1.0 - (1.0 - p_round) ** n_rounds
            result.pk = p_burst
            result.effector = msg.effector
            if self.world.rng.random() < p_burst:
                self._register_kill(result, target, msg.effector, msg.uav_id, p_burst)
                stray_rounds = max(0, n_rounds - 1)   # the killing round stops
            else:
                self.world.log_event(
                    "miss", uav_id=msg.uav_id, enemy_id=target.id,
                    effector=msg.effector.value, pk=round(p_burst, 3),
                    target_kind="track",
                )
                stray_rounds = n_rounds
        else:
            self.world.log_event("fire_no_target", uav_id=msg.uav_id,
                                 effector=msg.effector.value,
                                 target_kind="track")
            stray_rounds = n_rounds

        self._stray_rounds(turret, msg.predicted_intercept, stray_rounds)
        self._result_pub.publish(result)

    def _stray_rounds(self, turret, aim: np.ndarray, n: int) -> None:
        """Terminal ground impacts of the rounds that missed: continue past
        the target on the dispersed ballistic line until z = 0, then score
        the impact cell (SIM-EFF-003)."""
        if n <= 0:
            return
        p0 = turret.position
        rel = aim - p0
        dist = float(np.linalg.norm(rel))
        if dist < 1.0:
            return
        u = rel / dist
        # Orthonormal basis across the line of fire for dispersion errors.
        ref = np.array([0.0, 0.0, 1.0]) if abs(u[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        e1 = np.cross(u, ref)
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(u, e1)
        tof = dist / turret.muzzle_velocity

        for _ in range(n):
            err = self.world.rng.normal(0.0, turret.dispersion_mrad * 1e-3, 2)
            d = u + e1 * err[0] + e2 * err[1]
            v0 = d / np.linalg.norm(d) * turret.muzzle_velocity
            # Ballistic time to ground: z0 + v0z t - g t^2 / 2 = 0.
            disc = v0[2] ** 2 + 2.0 * GRAVITY * max(p0[2], 0.0)
            t_ground = (v0[2] + np.sqrt(disc)) / GRAVITY
            if t_ground <= tof:
                continue   # fell short of the target range — already scored as miss
            impact = p0[:2] + v0[:2] * t_ground
            zone = self.world.env.risk_map.zone_at(impact[0], impact[1])
            self.world.stray_impacts.append({
                "t": round(self.world.t, 3),
                "pos": [float(impact[0]), float(impact[1]), 0.0],
                "zone": zone,
                "shooter": turret.turret_id,
            })

    # -- debris interception (SIM-DEB-003) ----------------------------------------

    def _on_debris_fire(self, msg: FireRequest) -> None:
        """A kinetic shot at a falling wreck: hit removes the object (its
        fragments are negligible, SIM-DEB-004), miss lets it keep falling.
        Both shooter families resolve here."""
        result = EngagementResult(
            header=Header(stamp=self.world.t),
            task_id=msg.task_id, track_id=msg.track_id, uav_id=msg.uav_id,
            effector=msg.effector, target_kind="debris",
        )
        turret = self.turrets.get(msg.uav_id)
        deb = self.world.debris.get(msg.debris_id)
        if deb is None:
            # Landed or already neutralized while the round was in flight.
            self.world.log_event("fire_no_target", uav_id=msg.uav_id,
                                 target_kind="debris", debris_id=msg.debris_id)
            self._result_pub.publish(result)
            return

        shooter_pos = turret.position if turret is not None \
            else self.uavs[msg.uav_id].position
        if not self.world.occlusion.clear(shooter_pos, deb.position):
            self.world.log_event(
                "fire_blocked_los", uav_id=msg.uav_id, debris_id=deb.debris_id,
                effector=msg.effector.value, target_kind="debris",
            )
            if turret is not None:
                n = msg.rounds if msg.rounds > 0 else turret.rounds_per_burst
                self._stray_rounds(turret, msg.predicted_intercept, n)
            self._result_pub.publish(result)
            return

        if turret is not None:
            dist = float(np.linalg.norm(deb.position - turret.position))
            tof = dist / turret.muzzle_velocity
            n_rounds = msg.rounds if msg.rounds > 0 else turret.rounds_per_burst
            sigma2 = (turret.dispersion_mrad * 1e-3 * dist) ** 2 \
                + (TURRET_EVASION_FACTOR * float(np.linalg.norm(deb.velocity)) * tof) ** 2
            p_round = 1.0 - float(np.exp(-(TURRET_LETHAL_RADIUS**2) / (2.0 * sigma2 + 1e-9)))
            pk = 1.0 - (1.0 - p_round) ** n_rounds
        else:
            uav = self.uavs[msg.uav_id]
            rel = deb.position - uav.position
            pk = uav.effector.p_kill(rel, uav.velocity, deb.velocity)

        result.pk = pk
        if self.world.rng.random() < pk:
            impact = deb.predicted_impact()
            saved_zone = self.world.env.risk_map.zone_at(impact[0], impact[1])
            del self.world.debris[deb.debris_id]
            self.world.debris_intercepted.append({
                "t": round(self.world.t, 3), "debris_id": deb.debris_id,
                "shooter": msg.uav_id, "effector": msg.effector.value,
                "saved_zone": saved_zone,
            })
            result.hit = True
            self.world.log_event(
                "debris_neutralized", uav_id=msg.uav_id,
                debris_id=deb.debris_id, effector=msg.effector.value,
                saved_zone=saved_zone.name, pk=round(pk, 3),
                target_kind="debris",
            )
            stray = max(0, (msg.rounds or getattr(turret, "rounds_per_burst", 1)) - 1) \
                if turret is not None else 0
        else:
            self.world.log_event(
                "miss", uav_id=msg.uav_id, debris_id=deb.debris_id,
                effector=msg.effector.value, pk=round(pk, 3),
                target_kind="debris",
            )
            stray = (msg.rounds if msg.rounds > 0
                     else turret.rounds_per_burst) if turret is not None else 0
        if turret is not None and stray > 0:
            self._stray_rounds(turret, msg.predicted_intercept, stray)
        self._result_pub.publish(result)

    # -- shared kill bookkeeping --------------------------------------------------

    def _register_kill(self, result: EngagementResult, target, effector_type,
                       shooter_id: str, pk: float) -> None:
        target.kill()
        # The wreck becomes a live falling object (SIM-DEB-001): mechanism-
        # dependent horizontal velocity retention with the same jitter the
        # predictive footprint samples, integrated by the world until it
        # lands or is intercepted.
        retention = velocity_retention(effector_type) \
            * float(self.world.rng.normal(1.0, 0.25))
        vel = np.array([target.velocity[0] * retention,
                        target.velocity[1] * retention, 0.0])
        deb = FallingDebris(
            debris_id=f"deb-{target.id}",
            source_id=target.id,
            position=target.position,
            velocity=vel,
            mechanism=effector_type,
            spawn_t=self.world.t,
            track_ref=self.world.next_debris_ref(),
        )
        self.world.debris[deb.debris_id] = deb
        impact = deb.predicted_impact()
        zone = self.world.env.risk_map.zone_at(impact[0], impact[1])
        result.hit = True
        result.debris_impact = np.array([impact[0], impact[1], 0.0])
        result.debris_zone = zone
        result.effector = effector_type
        result.pk = pk
        self.world.log_event(
            "kill", uav_id=shooter_id, enemy_id=target.id,
            threat_class=target.threat_class.value,
            effector=effector_type.value,
            debris_zone=zone.name, pk=round(pk, 3),
            target_kind="track",
        )
        self.world.log_event(
            "debris_spawn", debris_id=deb.debris_id, enemy_id=target.id,
            zone=zone.name, t_impact=round(deb.time_to_impact(), 2),
        )

    def _nearest_enemy(self, aim_point: np.ndarray, gate: float = 300.0):
        """Nearest live enemy to the munition's *aim point*. The shot was
        cleared and released at a predicted intercept; what it can plausibly
        hit is whatever flies near that point (track-vs-truth error is tens
        of metres). Resolving by distance to the shooter instead can hand
        the kill to a bystander behind the launch rail and attribute it to
        the engaged track. True shooter-target geometry still decides the
        kill probability, so an aim point with nothing in effector reach
        resolves as a miss."""
        best, best_d = None, gate
        for enemy in self.world.enemies.values():
            if not enemy.alive:
                continue
            d = float(np.linalg.norm(enemy.position - aim_point))
            if d < best_d:
                best, best_d = enemy, d
        return best
