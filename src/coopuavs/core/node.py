"""Node base class — the unit of composition, shaped like ``rclpy.node.Node``.

A node owns publishers/subscriptions and an ``update(t, dt)`` tick driven by
the simulation scheduler at the node's declared ``rate_hz``. Under ROS 2 the
tick becomes a wall-clock timer and the bus calls become rclpy calls; the
node body is unchanged.
"""

from __future__ import annotations

from typing import Any, Callable

from .bus import MessageBus, Publisher


class Node:
    def __init__(self, name: str, bus: MessageBus, rate_hz: float = 10.0):
        if rate_hz <= 0.0:
            raise ValueError(f"node '{name}': rate_hz must be positive, got {rate_hz}")
        self.name = name
        self.bus = bus
        self.rate_hz = rate_hz
        self._next_tick = 0.0
        # Comms endpoint this node's traffic rides on (SIM-COM-001): a UAV id
        # for airborne nodes, None for the wired ground segment. Set *before*
        # creating publishers/subscriptions to take effect.
        self.comms_endpoint: str | None = None

    # -- middleware facade --------------------------------------------------

    def create_publisher(self, topic: str) -> Publisher:
        return self.bus.create_publisher(topic, endpoint=self.comms_endpoint)

    def create_subscription(self, topic: str, callback: Callable[[Any], None]) -> None:
        self.bus.subscribe(topic, callback, endpoint=self.comms_endpoint)

    # -- scheduling ----------------------------------------------------------

    def maybe_update(self, t: float, dt: float) -> None:
        """Called by the world every sim step; fires :meth:`update` at rate_hz.

        The next deadline advances from the *previous deadline*, not from
        the (quantised-up) fire time: rebasing on the fire time accumulates
        the quantisation error and a period that is not a multiple of the
        world dt would run systematically below its declared rate."""
        if t + 1e-9 >= self._next_tick:
            period = 1.0 / self.rate_hz
            self._next_tick += period
            if self._next_tick <= t + 1e-9:   # missed deadlines don't backlog
                self._next_tick = t + period
            self.update(t, dt)

    def update(self, t: float, dt: float) -> None:  # pragma: no cover - interface
        """Override with the node's periodic work."""
