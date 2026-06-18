"""Shared driver skeleton: seq-tracked HAL reads + staleness counting."""

from __future__ import annotations


class Driver:
    """Poll a HAL port once per tick; subclasses convert + publish."""

    __slots__ = ("_port", "_pub", "_last_seq", "stale_ticks", "stale_after",
                 "bad_frames")

    def __init__(self, port, pub, stale_after: int):
        if stale_after < 1:
            raise ValueError(f"stale_after must be >= 1, got {stale_after!r}")
        self._port = port
        self._pub = pub
        self._last_seq = 0
        self.stale_ticks = 0
        self.stale_after = stale_after
        self.bad_frames = 0

    @property
    def stale(self) -> bool:
        return self.stale_ticks >= self.stale_after

    def tick(self, now: float) -> None:
        seq, frame = self._port.read()
        if seq == self._last_seq:
            self.stale_ticks += 1
            return
        self._last_seq = seq
        if self._convert(now, frame):
            self.stale_ticks = 0
        else:
            # A frame arrived but was unusable: counts against freshness
            # (a wedged sensor streaming garbage is stale for control
            # purposes) and is tallied for CBIT.
            self.bad_frames += 1
            self.stale_ticks += 1

    def _convert(self, now: float, frame) -> bool:
        """Convert + publish; return False to reject the frame."""
        raise NotImplementedError
