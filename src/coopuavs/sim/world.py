"""Time-stepped simulation world.

The world owns ground truth: the environment, every hostile platform, the
clock and the RNG. Software nodes (sensors, fusion, C2, interceptor agents)
only see the world through messages on the bus — except the explicitly
sim-side nodes (sensors, the engagement adjudicator) which act as the
"physics plugins" and are the only ones allowed to touch ground truth.
That boundary is what makes the later ROS 2 / Gazebo migration mechanical.
"""

from __future__ import annotations

from typing import Callable

import numpy as np

from ..core.bus import MessageBus
from ..core.node import Node
from ..risk.debris import DebrisModel
from ..threats.enemy_drone import EnemyDrone
from .environment import Environment


class World:
    def __init__(self, env: Environment, dt: float = 0.05, seed: int = 0):
        self.env = env
        self.dt = dt
        self.t = 0.0
        self.bus = MessageBus()
        self.rng = np.random.default_rng(seed)
        self.debris_model = DebrisModel(self.rng)

        self.enemies: dict[str, EnemyDrone] = {}
        self.nodes: list[Node] = []
        self._spawn_queue: list[tuple[float, Callable[[], EnemyDrone]]] = []
        self.events: list[dict] = []
        self.wrecks: list[dict] = []

    # -- construction ----------------------------------------------------------

    def add_node(self, node: Node) -> None:
        self.nodes.append(node)

    def schedule_enemy(self, spawn_time: float, factory: Callable[[], EnemyDrone]) -> None:
        self._spawn_queue.append((spawn_time, factory))
        self._spawn_queue.sort(key=lambda e: e[0])

    def log_event(self, kind: str, **data) -> None:
        self.events.append({"t": round(self.t, 3), "kind": kind, **data})

    # -- stepping ----------------------------------------------------------------

    def step(self) -> None:
        while self._spawn_queue and self._spawn_queue[0][0] <= self.t:
            _, factory = self._spawn_queue.pop(0)
            enemy = factory()
            self.enemies[enemy.id] = enemy
            self.log_event("enemy_spawn", enemy_id=enemy.id,
                           threat_class=enemy.threat_class.value)

        for enemy in self.enemies.values():
            was_alive = enemy.alive
            enemy.step(self.dt, self.t)
            if was_alive and enemy.reached_target:
                self.log_event("leaker", enemy_id=enemy.id,
                               threat_class=enemy.threat_class.value,
                               warhead=enemy.profile.warhead)

        for node in self.nodes:
            node.maybe_update(self.t, self.dt)

        self.t += self.dt

    def run(
        self,
        duration: float,
        on_step: Callable[["World"], None] | None = None,
        stop_when_clear: bool = True,
    ) -> dict:
        """Run until ``duration`` or until the raid is fully resolved."""
        end = self.t + duration
        while self.t < end:
            self.step()
            if on_step is not None:
                on_step(self)
            if (
                stop_when_clear
                and not self._spawn_queue
                and self.enemies
                and not any(e.alive for e in self.enemies.values())
            ):
                break
        return self.summary()

    # -- scoring -------------------------------------------------------------------

    def summary(self) -> dict:
        enemies = list(self.enemies.values())
        kills = [e for e in enemies if e.killed]
        leakers = [e for e in enemies if e.reached_target]
        armed_leakers = [e for e in leakers if e.profile.warhead]
        return {
            "t_end": round(self.t, 2),
            "enemies_total": len(enemies),
            "kills": len(kills),
            "kills_decoy": sum(not e.profile.warhead for e in kills),
            "leakers": len(leakers),
            "armed_leakers": len(armed_leakers),
            "wrecks_by_zone": self._wrecks_by_zone(),
            "events": len(self.events),
        }

    def _wrecks_by_zone(self) -> dict[str, int]:
        out: dict[str, int] = {}
        for w in self.wrecks:
            out[w["zone"].name] = out.get(w["zone"].name, 0) + 1
        return out
