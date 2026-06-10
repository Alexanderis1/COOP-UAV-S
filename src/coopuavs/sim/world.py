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
from ..core.messages import reset_message_seq
from ..core.node import Node
from ..risk.debris import DebrisModel
from ..threats.enemy_drone import EnemyDrone
from .environment import Environment
from .occlusion import OcclusionGrid
from .weather import WeatherState


class World:
    def __init__(
        self,
        env: Environment,
        dt: float = 0.05,
        seed: int = 0,
        weather: WeatherState | None = None,
    ):
        self.env = env
        self.dt = dt
        self.t = 0.0
        self.bus = MessageBus()
        # Restart message numbering with the world clock: run N of a batch
        # must produce the same recording as the same seed run standalone.
        reset_message_seq()
        self.rng = np.random.default_rng(seed)
        self.debris_model = DebrisModel(self.rng)
        self.weather = weather or WeatherState(self.rng)
        # Building LOS occlusion (SIM-SEN-005/SIM-EFF-006); scenarios may
        # disable it (`occlusion: {enabled: false}` restores v0.1 sensing).
        self.occlusion = OcclusionGrid(env.buildings, env.bounds)

        # Simulated network layer (SIM-COM-001); attached by the CommsModel
        # itself when the scenario builds one. None = synchronous bus.
        self.comms = None

        self.enemies: dict[str, EnemyDrone] = {}
        # Friendly truth registry (sim-side only): interceptor airframes by
        # id, used by wind displacement and reactive threat evasion.
        self.friendlies: dict[str, object] = {}
        self.turrets: dict[str, object] = {}
        self.nodes: list[Node] = []
        self._spawn_queue: list[tuple[float, Callable[[], EnemyDrone]]] = []
        self.events: list[dict] = []
        self.wrecks: list[dict] = []
        self.stray_impacts: list[dict] = []   # {"t", "pos", "zone", "shooter"}

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
        if self.comms is not None:
            # Deliver radio traffic whose latency has elapsed (SIM-COM-001)
            # and refresh link-quality telemetry, before any node runs.
            self.comms.step(self.t)
        self.weather.step(self.dt)

        while self._spawn_queue and self._spawn_queue[0][0] <= self.t:
            _, factory = self._spawn_queue.pop(0)
            enemy = factory()
            enemy.spawn_t = self.t
            self.enemies[enemy.id] = enemy
            self.log_event("enemy_spawn", enemy_id=enemy.id,
                           threat_class=enemy.threat_class.value)

        windy = self.weather.wind_speed > 0.0 or self.weather.gust_std > 0.0
        for enemy in self.enemies.values():
            was_alive = enemy.alive
            enemy.step(self.dt, self.t)
            if windy and enemy.alive:
                enemy.body.position += self.weather.wind_at(enemy.position[2]) * self.dt
            if was_alive and enemy.reached_target:
                self.log_event("leaker", enemy_id=enemy.id,
                               threat_class=enemy.threat_class.value,
                               warhead=enemy.profile.warhead)

        for node in self.nodes:
            node.maybe_update(self.t, self.dt)

        if windy:
            # Truth-side wind displacement of friendly airframes (SIM-PHX-003);
            # the agents fight the drift through their velocity loops.
            for uav in self.friendlies.values():
                if uav.position[2] > 1.0:
                    uav.body.position += self.weather.wind_at(uav.position[2]) * self.dt

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
            "strays_by_zone": self._strays_by_zone(),
            "events": len(self.events),
        }

    def _wrecks_by_zone(self) -> dict[str, int]:
        out: dict[str, int] = {}
        for w in self.wrecks:
            out[w["zone"].name] = out.get(w["zone"].name, 0) + 1
        return out

    def _strays_by_zone(self) -> dict[str, int]:
        out: dict[str, int] = {}
        for s in self.stray_impacts:
            out[s["zone"].name] = out.get(s["zone"].name, 0) + 1
        return out
