"""Two-level simulation clock for the SITL micro-loop (SIM-SIL-002).

The world keeps its macro step (`World.dt`, the bus epoch). Inside each
macro step the SITL engine runs `K = base_hz * dt` micro-ticks. Time here
is an integer tick count; seconds are always *derived* (`tick / tick_hz`),
never accumulated, so a million ticks cannot drift. Rates must divide the
tick rate exactly — a rate that doesn't is a scenario error, not something
to round silently.
"""

from __future__ import annotations

from typing import Callable

_DIVISOR_TOL = 1e-9


class VirtualClock:
    """Integer-tick clock; `now` is derived, drift-free."""

    __slots__ = ("tick_hz", "tick")

    def __init__(self, tick_hz: int, start_tick: int = 0):
        if isinstance(tick_hz, bool) or not isinstance(tick_hz, int) or tick_hz <= 0:
            raise ValueError(f"tick_hz must be a positive integer, got {tick_hz!r}")
        self.tick_hz = tick_hz
        self.tick = start_tick

    @property
    def now(self) -> float:
        return self.tick / self.tick_hz

    def advance(self, n: int = 1) -> None:
        if n < 1:
            raise ValueError(f"advance step must be >= 1, got {n!r}")
        self.tick += n


class RateGroupScheduler:
    """Named tasks at exact divisors of the clock rate, run in registration
    order — that order is the determinism contract for per-board software."""

    def __init__(self, clock: VirtualClock):
        self.clock = clock
        self._tasks: list[tuple[str, int, Callable[[float], None]]] = []

    def add(self, name: str, rate_hz: float, fn: Callable[[float], None]) -> None:
        if rate_hz <= 0:
            raise ValueError(f"task {name!r}: rate must be positive, got {rate_hz!r}")
        divisor = self.clock.tick_hz / rate_hz
        if divisor < 1.0 or abs(divisor - round(divisor)) > _DIVISOR_TOL:
            raise ValueError(
                f"task {name!r}: {rate_hz} Hz does not divide the "
                f"{self.clock.tick_hz} Hz tick rate exactly"
            )
        self._tasks.append((name, round(divisor), fn))

    def run_due(self) -> None:
        tick = self.clock.tick
        now = self.clock.now
        for _name, divisor, fn in self._tasks:
            if tick % divisor == 0:
                fn(now)


class MicroScheduler:
    """Runs K micro-ticks per world macro step (installed as `World.micro`).

    `base_hz * world_dt` must be an integral K >= 1 — validated at build so
    a bad rate pairing fails the scenario, never rounds.
    """

    def __init__(self, world_dt: float, base_hz: int):
        k = base_hz * world_dt
        if k < 1.0 - _DIVISOR_TOL or abs(k - round(k)) > _DIVISOR_TOL:
            raise ValueError(
                f"base_hz={base_hz} gives {k!r} micro-ticks per {world_dt} s "
                "macro step; it must be an integer >= 1"
            )
        self.k = round(k)
        self.clock = VirtualClock(base_hz)
        self.tasks = RateGroupScheduler(self.clock)

    def add(self, name: str, rate_hz: float, fn: Callable[[float], None]) -> None:
        self.tasks.add(name, rate_hz, fn)

    def run_macro_step(self, t: float, dt: float) -> None:
        for _ in range(self.k):
            self.tasks.run_due()
            self.clock.advance()
