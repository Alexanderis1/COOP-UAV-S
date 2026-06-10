"""Live falling debris (SIM-DEB-001/002).

A kill no longer drops an instantaneous wreck: the adjudicator spawns a
:class:`FallingDebris` that the world integrates every tick — gravity-
accelerated descent capped at the tumbling-airframe terminal velocity,
horizontal velocity preserved (the mechanism-dependent retention was
applied at spawn). The analytic ``predicted_impact`` uses the same
fall-time law as the predictive ROE footprint (``risk/debris.py``), so the
C2's intercept deadline and the truth trajectory agree.

:class:`DebrisReporter` is the sim-side stand-in for a debris-tracking
radar: it publishes every live object on ``debris/state`` for the C2's
debris-intercept tasking (PHY-GCS-006) and the display.
"""

from __future__ import annotations

import numpy as np

from ..core.messages import DebrisArray, DebrisState, EffectorType, Header
from ..core.node import Node
from ..risk.debris import TERMINAL_FALL_SPEED, fall_time
from .physics import GRAVITY

DEBRIS_TOPIC = "debris/state"


class FallingDebris:
    """One falling wreck: point ballistics consistent with the footprint
    model. Fragments of an intercepted wreck are negligible (SIM-DEB-004)
    and are never spawned as new objects."""

    def __init__(
        self,
        debris_id: str,
        source_id: str,
        position: np.ndarray,
        velocity: np.ndarray,
        mechanism: EffectorType,
        spawn_t: float = 0.0,
        track_ref: int = 0,
    ):
        self.debris_id = debris_id
        self.source_id = source_id
        self.track_ref = track_ref
        self.position = np.asarray(position, dtype=float).copy()
        self.velocity = np.asarray(velocity, dtype=float).copy()
        self.mechanism = mechanism
        self.spawn_t = spawn_t

    def step(self, dt: float) -> None:
        self.velocity[2] = max(self.velocity[2] - GRAVITY * dt, -TERMINAL_FALL_SPEED)
        self.position += self.velocity * dt

    @property
    def landed(self) -> bool:
        return self.position[2] <= 0.0

    def time_to_impact(self) -> float:
        return fall_time(float(self.position[2]), v_down0=-float(self.velocity[2]))

    def predicted_impact(self) -> np.ndarray:
        """Analytic ground impact point [x, y, 0] from the current state."""
        t_fall = self.time_to_impact()
        xy = self.position[:2] + self.velocity[:2] * t_fall
        return np.array([xy[0], xy[1], 0.0])


class DebrisReporter(Node):
    """Publishes the live debris picture (SIM-DEB-002) at record rate."""

    def __init__(self, world, rate_hz: float = 5.0):
        super().__init__("debris_reporter", world.bus, rate_hz=rate_hz)
        self.world = world
        self._pub = self.create_publisher(DEBRIS_TOPIC)

    def update(self, t: float, dt: float) -> None:
        states = []
        for deb in self.world.debris.values():
            impact = deb.predicted_impact()
            states.append(DebrisState(
                header=Header(stamp=t),
                debris_id=deb.debris_id,
                track_ref=deb.track_ref,
                position=deb.position.copy(),
                velocity=deb.velocity.copy(),
                predicted_impact=impact,
                impact_zone=self.world.env.risk_map.zone_at(impact[0], impact[1]),
                t_impact=deb.time_to_impact(),
            ))
        self._pub.publish(DebrisArray(header=Header(stamp=t), debris=states))
