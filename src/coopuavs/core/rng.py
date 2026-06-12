"""Name-keyed RNG streams (fixes DESIGN_REVIEW 5.1).

One shared generator consumed in call order means every draw depends on
exact callback sequencing — determinism that cannot survive a ROS 2
executor. Here each consumer gets a stream derived purely from
``(run_seed, name)``: adding, removing or reordering consumers never
shifts anyone else's draws.
"""

from __future__ import annotations

import hashlib

import numpy as np


def _name_key(name: str) -> int:
    # Platform-stable name hash (Python's hash() is salted per process).
    return int.from_bytes(hashlib.sha256(name.encode("utf-8")).digest(), "little")


class RngRegistry:
    """Per-consumer ``np.random.Generator`` streams keyed by stable name."""

    def __init__(self, seed: int):
        if seed < 0:
            raise ValueError(f"seed must be non-negative, got {seed!r}")
        self._seed = int(seed)
        self._streams: dict[str, np.random.Generator] = {}

    def stream(self, name: str) -> np.random.Generator:
        gen = self._streams.get(name)
        if gen is None:
            seq = np.random.SeedSequence([self._seed, _name_key(name)])
            gen = self._streams[name] = np.random.default_rng(seq)
        return gen
