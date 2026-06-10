"""In-process publish/subscribe bus mimicking the ROS 2 topic graph.

The bus is the single seam between COOP-UAV-S logic and the middleware: every
module talks only to :class:`MessageBus` through :class:`Publisher` and
subscription callbacks. Migrating to ROS 2 means re-implementing these two
small classes on top of ``rclpy`` — node code does not change.

Delivery is synchronous and deterministic (callbacks run in subscription
order during ``publish``), which keeps simulation runs reproducible.
"""

from __future__ import annotations

import fnmatch
from collections import defaultdict
from typing import Any, Callable

Callback = Callable[[Any], None]


class Publisher:
    """Handle returned by :meth:`MessageBus.create_publisher`."""

    def __init__(self, bus: "MessageBus", topic: str):
        self._bus = bus
        self.topic = topic

    def publish(self, msg: Any) -> None:
        self._bus.publish(self.topic, msg)


class MessageBus:
    def __init__(self) -> None:
        self._subs: dict[str, list[Callback]] = defaultdict(list)
        self._pattern_subs: list[tuple[str, Callable[[str, Any], None]]] = []

    def create_publisher(self, topic: str) -> Publisher:
        return Publisher(self, topic)

    def subscribe(self, topic: str, callback: Callback) -> None:
        self._subs[topic].append(callback)

    def subscribe_pattern(self, pattern: str, callback: Callable[[str, Any], None]) -> None:
        """Subscribe to all topics matching a glob (e.g. ``uav/*/state``).

        Pattern subscribers receive ``(topic, msg)`` — used by recorders and
        the dashboard bridge, which need the whole graph.
        """
        self._pattern_subs.append((pattern, callback))

    def publish(self, topic: str, msg: Any) -> None:
        for cb in self._subs.get(topic, ()):
            cb(msg)
        for pattern, cb in self._pattern_subs:
            if fnmatch.fnmatch(topic, pattern):
                cb(topic, msg)
