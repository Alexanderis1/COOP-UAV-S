"""Engagement adjudicator — the sim-side referee.

Listens to ``engagement/fire`` events and resolves them against ground
truth: true relative geometry decides the real kill probability, one RNG
roll decides the outcome, and on a kill the debris model places the actual
wreck on the risk map. Plays the role a Gazebo contact/effects plugin will
play after migration; no tactical logic lives here.
"""

from __future__ import annotations

import numpy as np

from ..core.messages import EngagementResult, FireRequest, Header
from ..core.node import Node
from ..interceptors.uav import InterceptorUav
from .world import World


class EngagementAdjudicator(Node):
    def __init__(self, world: World, uavs: dict[str, InterceptorUav]):
        super().__init__("adjudicator", world.bus, rate_hz=50.0)
        self.world = world
        self.uavs = uavs
        self._result_pub = self.create_publisher("engagement/result")
        self.create_subscription("engagement/fire", self._on_fire)

    def _on_fire(self, msg: FireRequest) -> None:
        uav = self.uavs[msg.uav_id]
        target = self._nearest_enemy(uav.position, msg.predicted_intercept)
        result = EngagementResult(
            header=Header(stamp=self.world.t),
            task_id=msg.task_id,
            track_id=msg.track_id,
            uav_id=msg.uav_id,
        )

        if target is not None:
            rel = target.position - uav.position
            pk_true = uav.effector.p_kill(rel, uav.velocity, target.velocity)
            if self.world.rng.random() < pk_true:
                target.kill()
                impact = self.world.debris_model.sample_impact(
                    target.position, target.velocity, uav.effector.type
                )
                zone = self.world.env.risk_map.zone_at(impact[0], impact[1])
                self.world.wrecks.append(
                    {"t": self.world.t, "enemy_id": target.id, "pos": impact.tolist(),
                     "zone": zone, "effector": uav.effector.type.value}
                )
                result.hit = True
                result.debris_impact = np.array([impact[0], impact[1], 0.0])
                result.debris_zone = zone
                self.world.log_event(
                    "kill", uav_id=msg.uav_id, enemy_id=target.id,
                    threat_class=target.threat_class.value,
                    effector=uav.effector.type.value,
                    debris_zone=zone.name, pk=round(pk_true, 3),
                )
            else:
                self.world.log_event(
                    "miss", uav_id=msg.uav_id, enemy_id=target.id, pk=round(pk_true, 3)
                )
        else:
            self.world.log_event("fire_no_target", uav_id=msg.uav_id)

        self._result_pub.publish(result)

    def _nearest_enemy(self, shooter_pos: np.ndarray, aim_point: np.ndarray):
        """Nearest live enemy to the shooter; the munition only threatens
        what is physically nearby, whatever the track said."""
        best, best_d = None, 300.0
        for enemy in self.world.enemies.values():
            if not enemy.alive:
                continue
            d = float(np.linalg.norm(enemy.position - shooter_pos))
            if d < best_d:
                best, best_d = enemy, d
        return best
