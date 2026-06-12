"""Port/Mailbox isolation primitives (PLAN_PROBLEM1 architecture, P4-3).

A hosted processor (``sil/host.py VirtualMCU``) touches the world ONLY
through its ports: world-side code posts messages into inboxes — a bus
callback may append and nothing more — and drains outboxes at its own
node cadence; the hosted software drains inboxes and posts to outboxes
at ITS tick. Nothing else crosses the boundary, which makes the seam
the future subprocess transport attach point (hybrid deferred by
design, not designed away).

Mailboxes are bounded: overflow refuses the NEWEST message (the
coop-link Channel backpressure convention — accepted traffic keeps its
timing independent of later load) and tallies the drop as a CBIT seam.
"""

from __future__ import annotations

from collections import deque


class Mailbox:
    """Append-only FIFO queue, drained whole at the owner's tick."""

    __slots__ = ("name", "maxlen", "dropped", "_q")

    def __init__(self, name: str, maxlen: int = 256):
        if maxlen < 1:
            raise ValueError(f"mailbox {name!r}: maxlen must be >= 1")
        self.name = name
        self.maxlen = int(maxlen)
        self.dropped = 0
        self._q: deque = deque()

    def post(self, msg) -> bool:
        """World/app side: queue one message; False = refused (full)."""
        if len(self._q) >= self.maxlen:
            self.dropped += 1
            return False
        self._q.append(msg)
        return True

    def drain(self) -> list:
        """Owner side: everything queued, FIFO, and clear."""
        out = list(self._q)
        self._q.clear()
        return out

    def __len__(self) -> int:
        return len(self._q)


class Ports:
    """Name-keyed mailbox registry; one per VirtualMCU."""

    def __init__(self):
        self._boxes: dict[str, Mailbox] = {}

    def box(self, name: str, maxlen: int = 256) -> Mailbox:
        box = self._boxes.get(name)
        if box is None:
            box = self._boxes[name] = Mailbox(name, maxlen)
        return box
