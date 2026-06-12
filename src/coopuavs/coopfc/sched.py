"""FCU-internal rate-group scheduler (the flight software's main loop).

Driven one integer tick at a time by the VirtualMCU host (P4); time is
derived (`tick / tick_hz`), never accumulated, so it cannot drift — the
same integer-tick contract as the sim-side sil/clock.py, re-implemented
here because coopfc is import-fenced. Boot time is tick 0 and every task
fires there (alignment), then in registration order whenever its divisor
divides the tick: registration order IS the within-tick pipeline
(drivers -> estimator -> controllers -> mixer -> ...), the determinism
contract for per-board software.

Overrun accounting is *modeled, not measured*: wall-clock execution time
would differ across hosts and runs, so a task instead declares
``cost_ticks`` — its modeled execution time. A due fire that lands inside
the previous fire's busy window is skipped and counted as an overrun;
``overrun_fault_after`` consecutive overruns latch the task's fault flag
(read by the P5 CBIT SCHED_OVERRUN monitor; fault injection raises a
task's cost to model CPU overload). Exceptions propagate to the host —
the per-MCU "processor crash" fence lives there, not here.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import NamedTuple


class TaskStats(NamedTuple):
    fires: int
    overruns: int
    faulted: bool


class _Task:
    __slots__ = ("name", "divisor", "fn", "cost_ticks", "fault_after",
                 "fires", "overruns", "consecutive", "busy_until", "faulted")

    def __init__(self, name: str, divisor: int, fn: Callable[[float], None],
                 cost_ticks: int, fault_after: int | None):
        self.name = name
        self.divisor = divisor
        self.fn = fn
        self.cost_ticks = cost_ticks
        self.fault_after = fault_after
        self.fires = 0
        self.overruns = 0
        self.consecutive = 0
        self.busy_until = 0
        self.faulted = False


class Scheduler:
    """Named tasks at exact divisors of the tick rate, run in registration
    order; one `run_tick()` call per host micro-tick."""

    def __init__(self, tick_hz: int):
        if isinstance(tick_hz, bool) or not isinstance(tick_hz, int) or tick_hz <= 0:
            raise ValueError(f"tick_hz must be a positive integer, got {tick_hz!r}")
        self.tick_hz = tick_hz
        self.tick = 0
        self._tasks: list[_Task] = []
        self._by_name: dict[str, _Task] = {}

    @property
    def now(self) -> float:
        return self.tick / self.tick_hz

    def add(self, name: str, rate_hz: float, fn: Callable[[float], None],
            cost_ticks: int = 0, overrun_fault_after: int | None = 1) -> None:
        if name in self._by_name:
            # Same lesson as RNG stream names: a silent collision couples
            # two consumers — duplicate task names are build errors.
            raise ValueError(f"task {name!r} already registered")
        if rate_hz <= 0:
            raise ValueError(f"task {name!r}: rate must be positive, got {rate_hz!r}")
        divisor = self.tick_hz / rate_hz
        if divisor < 1.0 or abs(divisor - round(divisor)) > 1e-9:
            raise ValueError(
                f"task {name!r}: {rate_hz} Hz does not divide the "
                f"{self.tick_hz} Hz tick rate exactly"
            )
        if cost_ticks < 0:
            raise ValueError(f"task {name!r}: cost_ticks must be >= 0")
        task = _Task(name, round(divisor), fn, cost_ticks, overrun_fault_after)
        self._tasks.append(task)
        self._by_name[name] = task

    def run_tick(self) -> None:
        """Fire every task due at the current tick, then advance."""
        tick = self.tick
        now = tick / self.tick_hz
        for task in self._tasks:
            if tick % task.divisor == 0:
                if tick < task.busy_until:
                    task.overruns += 1
                    task.consecutive += 1
                    if (task.fault_after is not None
                            and task.consecutive >= task.fault_after):
                        task.faulted = True
                else:
                    task.busy_until = tick + task.cost_ticks
                    task.fires += 1
                    task.consecutive = 0
                    task.fn(now)
        self.tick = tick + 1

    def stats(self, name: str) -> TaskStats:
        task = self._by_name[name]
        return TaskStats(task.fires, task.overruns, task.faulted)

    def faults(self) -> list[str]:
        """Names of fault-latched tasks, in registration order."""
        return [t.name for t in self._tasks if t.faulted]
