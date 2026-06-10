"""In-process publish/subscribe bus mimicking the ROS 2 topic graph.

The bus is the single seam between COOP-UAV-S logic and the middleware: every
module talks only to :class:`MessageBus` through :class:`Publisher` and
subscription callbacks. Migrating to ROS 2 means re-implementing these two
small classes on top of ``rclpy`` — node code does not change.

Delivery is synchronous and deterministic (callbacks run in subscription
order during ``publish``), which keeps simulation runs reproducible.

Comms routing (SIM-COM-001): publishers and subscriptions may carry a
*comms endpoint* tag — the radio they sit behind (a UAV id, or ``None`` for
the wired ground segment). When a router (the
:class:`~coopuavs.core.comms.CommsModel`) is attached, deliveries whose
topic and endpoints it claims are handed to it for latency/loss simulation
instead of being invoked synchronously. Without a router the bus behaves
exactly as in v0.1.
"""

from __future__ import annotations

import fnmatch
from collections import defaultdict
from typing import Any, Callable

Callback = Callable[[Any], None]


class Publisher:
    """Handle returned by :meth:`MessageBus.create_publisher`."""

    def __init__(self, bus: "MessageBus", topic: str, endpoint: str | None = None):
        self._bus = bus
        self.topic = topic
        self.endpoint = endpoint

    def publish(self, msg: Any) -> None:
        self._bus.publish(self.topic, msg, sender=self.endpoint)


class MessageBus:
    def __init__(self) -> None:
        self._subs: dict[str, list[tuple[Callback, str | None]]] = defaultdict(list)
        self._pattern_subs: list[tuple[str, Callable[[str, Any], None]]] = []
        # Optional comms router (duck-typed: .routes(topic, sender, receiver)
        # and .send(topic, msg, callback, sender, receiver)).
        self.router: Any | None = None

    def create_publisher(self, topic: str, endpoint: str | None = None) -> Publisher:
        return Publisher(self, topic, endpoint)

    def subscribe(self, topic: str, callback: Callback, endpoint: str | None = None) -> None:
        self._subs[topic].append((callback, endpoint))

    def subscribe_pattern(self, pattern: str, callback: Callable[[str, Any], None]) -> None:
        """Subscribe to all topics matching a glob (e.g. ``uav/*/state``).

        Pattern subscribers receive ``(topic, msg)`` and are always
        delivered synchronously (evaluation-side taps, not radio links).
        A hook for ad-hoc instrumentation that needs the whole graph; the
        shipped Recorder uses plain per-topic subscriptions.
        """
        self._pattern_subs.append((pattern, callback))

    def publish(self, topic: str, msg: Any, sender: str | None = None) -> None:
        router = self.router
        for cb, endpoint in self._subs.get(topic, ()):
            if router is not None and router.routes(topic, sender, endpoint):
                router.send(topic, msg, cb, sender, endpoint)
            else:
                cb(msg)
        for pattern, cb in self._pattern_subs:
            if fnmatch.fnmatch(topic, pattern):
                cb(topic, msg)
