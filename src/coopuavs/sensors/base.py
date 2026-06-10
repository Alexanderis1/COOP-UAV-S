"""Sensor base class.

Sensors are *sim-side* nodes: they are the only software allowed to read
ground truth (``world.enemies``), in the same way a Gazebo sensor plugin
reads the physics engine. Their output — :class:`Detection` messages on the
``detections`` topic — is the only thing perception ever sees.

Each concrete sensor implements ``observe(enemy, t, trans)`` returning a
Detection or ``None`` (missed / out of envelope). The base class handles
scan rate, fan-out over all live targets, and the building line-of-sight
query (SIM-SEN-005): ``trans`` is the material/channel transmittance of
the sight line — 1.0 when unobstructed, 0.0 when fully masked (the base
class never calls ``observe`` at 0).
"""

from __future__ import annotations

import numpy as np

from ..core.messages import Detection, Header
from ..core.node import Node
from ..sim.world import World
from ..threats.enemy_drone import EnemyDrone

DETECTIONS_TOPIC = "detections"


class Sensor(Node):
    #: Occlusion channel of MATERIAL_TRANSMISSION (sim/occlusion.py).
    channel = "eo_ir"

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
        occlusion = self.world.occlusion
        for enemy in self.world.enemies.values():
            if not enemy.alive:
                continue
            if np.linalg.norm(enemy.position - self.position) > max_range:
                continue
            trans = occlusion.transmittance(self.position, enemy.position, self.channel)
            if trans <= 0.0:
                continue   # sight line fully masked by buildings
            det = self.observe(enemy, t, trans)
            if det is not None:
                self._pub.publish(det)

    def weather_factor(self) -> float:
        """Environment-coupled range multiplier (SIM-SEN-003); 1.0 unless a
        concrete sensor declares a sensitivity to the weather model."""
        return 1.0

    def effective_range(self) -> float:
        return self.max_range * self.weather_factor()

    def observe(self, enemy: EnemyDrone, t: float,
                trans: float = 1.0) -> Detection | None:  # pragma: no cover
        raise NotImplementedError

    def _header(self, t: float) -> Header:
        return Header(stamp=t, frame_id="map")


def mounted(sensor_cls: type[Sensor]) -> type[Sensor]:
    """A sensor class variant that rides a friendly airframe: the sensor
    position tracks the platform every scan (the
    :class:`~coopuavs.sensors.seeker.OnboardSeeker` pattern, reused for
    sentinel payloads, PHY-SNT-001). Occlusion and weather coupling apply
    unchanged — an airborne look simply starts from the platform."""

    class MountedSensor(sensor_cls):
        def __init__(self, name, world, uav, **kwargs):
            super().__init__(name, world, uav.body.position, **kwargs)
            self._platform = uav

        def update(self, t: float, dt: float) -> None:
            self.position = self._platform.body.position
            super().update(t, dt)

    MountedSensor.__name__ = f"Mounted{sensor_cls.__name__}"
    MountedSensor.__qualname__ = MountedSensor.__name__
    return MountedSensor
