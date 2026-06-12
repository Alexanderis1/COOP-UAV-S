"""uORB-style latest-value topic store — the intra-FCU dataflow seam.

One TopicStore per VirtualMCU. Semantics (the determinism contract):

- A topic holds the *latest* message only; publish overwrites. A slow
  subscriber sees the newest sample, never a backlog (matches PX4 uORB).
- Subscriptions are polled at the subscriber's own task rate. Nothing
  runs in the publisher's stack — no callbacks, so scheduler order alone
  decides who computes when (contrast core/bus.py on the sim side, which
  is deliberately synchronous-in-stack).
- Each subscription tracks its own `updated` flag and missed count.
- A message type declared at advertise is enforced on every publish;
  conflicting re-declarations are build errors.
"""

from __future__ import annotations


class _Topic:
    __slots__ = ("name", "msg_type", "value", "generation")

    def __init__(self, name: str):
        self.name = name
        self.msg_type: type | None = None
        self.value = None
        self.generation = 0


class Publication:
    __slots__ = ("_topic",)

    def __init__(self, topic: _Topic):
        self._topic = topic

    def publish(self, msg) -> None:
        t = self._topic
        if t.msg_type is not None and not isinstance(msg, t.msg_type):
            raise TypeError(
                f"topic {t.name!r} carries {t.msg_type.__name__}, "
                f"got {type(msg).__name__}"
            )
        t.value = msg
        t.generation += 1


class Subscription:
    __slots__ = ("_topic", "_read_generation")

    def __init__(self, topic: _Topic):
        self._topic = topic
        self._read_generation = 0

    @property
    def updated(self) -> bool:
        return self._topic.generation > self._read_generation

    def missed(self) -> int:
        """Publishes since the last read, beyond the one `read` returns."""
        return max(0, self._topic.generation - self._read_generation - 1)

    def read(self):
        """Latest message (None if never published); clears `updated`."""
        self._read_generation = self._topic.generation
        return self._topic.value


class TopicStore:
    """Name-keyed registry; advertise/subscribe in any order."""

    def __init__(self):
        self._topics: dict[str, _Topic] = {}

    def _topic(self, name: str) -> _Topic:
        topic = self._topics.get(name)
        if topic is None:
            topic = self._topics[name] = _Topic(name)
        return topic

    def advertise(self, name: str, msg_type: type | None = None) -> Publication:
        topic = self._topic(name)
        if msg_type is not None:
            if topic.msg_type is not None and topic.msg_type is not msg_type:
                raise ValueError(
                    f"topic {name!r} already advertised as "
                    f"{topic.msg_type.__name__}, not {msg_type.__name__}"
                )
            topic.msg_type = msg_type
        return Publication(topic)

    def subscribe(self, name: str) -> Subscription:
        return Subscription(self._topic(name))
