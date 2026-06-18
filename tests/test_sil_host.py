"""P4-3a: Port/Mailbox isolation primitives + the VirtualMCU host.

The mailbox seam is the ONLY way world-side code talks to hosted MC
software: bus callbacks append to inboxes (nothing more), the app
drains them at its own tick and posts to outboxes the world-side shell
drains at node cadence. The VirtualMCU exception fence converts an app
exception into a latched 'processor crash' (SIM-SIL-003 fault mode) —
the processor stops, the simulation does not.
"""

from __future__ import annotations

import pytest

from coopuavs.core.ports import Mailbox, Ports
from coopuavs.sil.host import VirtualMCU


# ---------------------------------------------------------------- mailboxes

def test_mailbox_fifo_drain_clears():
    box = Mailbox("tracks")
    box.post("a")
    box.post("b")
    assert len(box) == 2
    assert box.drain() == ["a", "b"]
    assert box.drain() == []
    assert len(box) == 0


def test_mailbox_bounded_drops_newest_deterministically():
    """Overflow refuses the NEWEST message (the Channel backpressure
    convention: accepted traffic keeps timing independent of later
    load) and tallies the drop for CBIT."""
    box = Mailbox("tracks", maxlen=2)
    assert box.post(1) and box.post(2)
    assert box.post(3) is False
    assert box.dropped == 1
    assert box.drain() == [1, 2]
    assert box.post(3) is True          # space again after drain


def test_ports_registry_caches_instances():
    ports = Ports()
    a = ports.box("uav_state")
    assert ports.box("uav_state") is a
    assert ports.box("other") is not a


# ----------------------------------------------------------------- VirtualMCU

class _App:
    def __init__(self, clock, rng, ports):
        self.clock = clock
        self.ports = ports
        self.ticks: list[float] = []
        self.blow_up_at: int | None = None

    def tick(self, now: float) -> None:
        if self.blow_up_at is not None and len(self.ticks) == self.blow_up_at:
            raise RuntimeError("segfault, basically")
        self.ticks.append(now)
        self.ports.box("out").post(now)


def _mcu(tick_hz=10, base_hz=800):
    return VirtualMCU("mc/u1", tick_hz=tick_hz, base_hz=base_hz,
                      app_factory=_App, rng=None)


def test_mcu_ticks_at_divisor_with_derived_time():
    mcu = _mcu()
    assert mcu.every == 80
    for k in range(240):                 # 0.3 s of base ticks
        if mcu.due(k):
            mcu.run_tick()
    assert mcu.app.ticks == [0.0, 0.1, 0.2]
    assert mcu.ports.box("out").drain() == [0.0, 0.1, 0.2]


def test_mcu_rate_must_divide_base():
    with pytest.raises(ValueError, match="divide"):
        _mcu(tick_hz=7)


def test_crash_fence_latches_and_stops_the_processor():
    mcu = _mcu()
    mcu.app.blow_up_at = 2
    for k in range(800):
        if mcu.due(k):
            mcu.run_tick()               # must never raise
    assert mcu.crashed is True
    assert "RuntimeError" in mcu.crash_reason
    assert mcu.app.ticks == [0.0, 0.1]   # frozen at the crash
    # the clock froze with it: a dead processor accrues no time
    assert mcu.clock.now == pytest.approx(0.2)


def test_app_factory_receives_clock_rng_ports():
    sentinel_rng = object()
    seen = {}

    def factory(clock, rng, ports):
        seen["clock"], seen["rng"], seen["ports"] = clock, rng, ports
        return _App(clock, rng, ports)

    mcu = VirtualMCU("mc/x", tick_hz=10, base_hz=800,
                     app_factory=factory, rng=sentinel_rng)
    assert seen["clock"] is mcu.clock
    assert seen["rng"] is sentinel_rng
    assert seen["ports"] is mcu.ports
