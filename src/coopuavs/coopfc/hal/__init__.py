"""HalIO — the MCU-portable hardware abstraction seam.

A HAL port is a named, seq-stamped, latest-frame mailbox. The host side
(VirtualMCU in P4, the bench in P3-8) *writes* raw device frames; the
driver side *reads* and tracks the sequence number to detect staleness.
On a real MCU the same driver code would sit on SPI/I2C reads — nothing
in coopfc knows whether a frame came from hw/ models or silicon, which
is the whole point of the fence.

Frame payloads are plain Python scalars/tuples by convention (the
>=100 Hz paths are numpy-free); the normative per-port shapes are
documented on each driver in coopfc/drivers/.
"""

from __future__ import annotations


class HalPort:
    """Latest-frame mailbox; seq increments per write, 0 = never written."""

    __slots__ = ("name", "_seq", "_frame")

    def __init__(self, name: str):
        self.name = name
        self._seq = 0
        self._frame = None

    def write(self, frame) -> None:
        """Host side: deliver one raw device frame (overwrites)."""
        self._frame = frame
        self._seq += 1

    def read(self):
        """Driver side: (seq, latest frame); (0, None) before first write."""
        return (self._seq, self._frame)


class HalIO:
    """Name-keyed port registry; one per VirtualMCU."""

    def __init__(self):
        self._ports: dict[str, HalPort] = {}

    def port(self, name: str) -> HalPort:
        port = self._ports.get(name)
        if port is None:
            port = self._ports[name] = HalPort(name)
        return port
