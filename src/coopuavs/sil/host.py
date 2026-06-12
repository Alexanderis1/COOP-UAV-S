"""VirtualMCU — one in-process hosted processor (SIM-SIL-001/003).

Hosts a software app behind ``core/ports.py`` mailboxes with its own
integer-tick :class:`~coopuavs.sil.clock.VirtualClock`. The app factory
receives exactly ``(clock, rng, ports)`` — no World, bus or environment
can physically enter (that ctor shape plus the mc/coopfc import fence
IS the isolation contract).

Exception fence: any exception escaping ``app.tick`` is a *processor
crash*, not a simulation error — the MCU latches ``crashed`` with the
reason, stops ticking (its clock freezes with it), and the world-side
glue turns the latch into a CBIT/world event (the free SIM-SIL-003
fault mode). The simulation keeps running; the airframe now flies
whatever its FCU does about a silent mission computer.

Scheduling: the fleet engine asks ``due(base_tick)`` at every micro
tick (ORDERING §6 step 3, "MC tick if due") and calls ``run_tick`` on
the divisor — the MCU rate must divide the base rate exactly, the
sil/clock.py contract.
"""

from __future__ import annotations

from .clock import VirtualClock


class VirtualMCU:
    def __init__(self, name: str, *, tick_hz: int, base_hz: int,
                 app_factory, rng, ports=None):
        from ..core.ports import Ports
        if base_hz % tick_hz:
            raise ValueError(
                f"MCU {name!r}: {tick_hz} Hz does not divide the "
                f"{base_hz} Hz base rate")
        self.name = name
        self.every = base_hz // tick_hz
        self.clock = VirtualClock(tick_hz)
        self.ports = ports if ports is not None else Ports()
        self.app = app_factory(self.clock, rng, self.ports)
        self.crashed = False
        self.crash_reason = ""

    def due(self, base_tick: int) -> bool:
        return base_tick % self.every == 0

    def run_tick(self) -> None:
        """One app tick behind the crash fence (never raises)."""
        if self.crashed:
            return
        try:
            self.app.tick(self.clock.now)
        except Exception as exc:                    # noqa: BLE001 — the fence
            self.crashed = True
            self.crash_reason = f"{type(exc).__name__}: {exc}"
            return
        self.clock.advance()
