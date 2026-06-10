"""Sensor base class.

Sensors are *sim-side* nodes: they are the only software allowed to read
ground truth (``world.enemies``), in the same way a Gazebo sensor plugin
reads the physics engine. Their output — :class:`Detection` messages on the
``detections`` topic — is the only thing perception ever sees.

Each concrete sensor implements ``observe(enemy, t)`` returning a Detection
or ``None`` (missed / out of envelope). The base class handles scan rate and
fan-out over all live targets.
"""

from __future__ import annotations

import numpy as np

from ..core.messages import Detection, Header
from ..core.node import Node
from ..sim.world import World
from ..threats.enemy_drone import EnemyDrone

DETECTIONS_TOPIC = "detections"


class Sensor(Node):
    def __init__(
        self,
        name: str,
        world: World,
        position: np.ndarray,
        max_range: float,
        rate_hz: float = 2.0,
    ):
        super().__init__(name, world.bus, rate_hz=rate_hz)
        self.world = world
        self.position = np.asarray(position, dtype=float)
        self.max_range = max_range
        self.rng = world.rng
        self._pub = self.create_publisher(DETECTIONS_TOPIC)

    def update(self, t: float, dt: float) -> None:
        max_range = self.effective_range()
        for enemy in self.world.enemies.values():
            if not enemy.alive:
                continue
            if np.linalg.norm(enemy.position - self.position) > max_range:
                continue
            det = self.observe(enemy, t)
            if det is not None:
                self._pub.publish(det)

    def weather_factor(self) -> float:
        """Environment-coupled range multiplier (SIM-SEN-003); 1.0 unless a
        concrete sensor declares a sensitivity to the weather model."""
        return 1.0

    def effective_range(self) -> float:
        return self.max_range * self.weather_factor()

    def observe(self, enemy: EnemyDrone, t: float) -> Detection | None:  # pragma: no cover
        raise NotImplementedError

    def _header(self, t: float) -> Header:
        return Header(stamp=t, frame_id="map")
