"""P0-4: VirtualClock + micro-step world scheduler (SIM-SIL-002 plumbing).

Integer-tick clock, so time is derived (tick / hz) and cannot drift over 1e6
ticks; rate groups fire exact counts in registration order with no silent
rounding; and the world micro-step seam is provably inert: with a no-op
micro scheduler attached (K=1 and K=4) the P0-3 characterization pins
reproduce bit-for-bit.
"""

from __future__ import annotations

import copy

import pytest

import golden_util as gu
from coopuavs.sil.clock import MicroScheduler, RateGroupScheduler, VirtualClock
from coopuavs.sim import scenario as scenario_mod


# -- VirtualClock ------------------------------------------------------------

def test_no_drift_over_1e6_ticks():
    clk = VirtualClock(800)
    clk.advance(1_000_000)
    assert clk.tick == 1_000_000
    assert clk.now == 1250.0  # exactly 1e6 / 800 — derived, not accumulated


def test_time_is_derived_not_accumulated():
    clk = VirtualClock(400)
    for _ in range(12_345):
        clk.advance()
    assert clk.now == 12_345 / 400


@pytest.mark.parametrize("bad", [0, -5, 1.5, 800.0])
def test_clock_rejects_bad_tick_hz(bad):
    with pytest.raises((ValueError, TypeError)):
        VirtualClock(bad)


def test_advance_rejects_nonpositive():
    clk = VirtualClock(800)
    with pytest.raises(ValueError):
        clk.advance(0)


# -- RateGroupScheduler --------------------------------------------------------

def test_rate_groups_fire_exact_counts_over_10s():
    clk = VirtualClock(800)
    sched = RateGroupScheduler(clk)
    counts = {hz: 0 for hz in (800, 400, 100, 50, 10, 1)}
    for hz in counts:
        def bump(now, hz=hz):
            counts[hz] += 1
        sched.add(f"task{hz}", hz, bump)
    for _ in range(8000):  # 10 s at 800 Hz
        sched.run_due()
        clk.advance()
    assert counts == {800: 8000, 400: 4000, 100: 1000, 50: 500, 10: 100, 1: 10}


def test_rate_groups_run_in_registration_order():
    def trace_run():
        clk = VirtualClock(800)
        sched = RateGroupScheduler(clk)
        seq: list[tuple[int, str]] = []
        for name, hz in (("c", 100), ("a", 800), ("b", 400)):
            def rec(now, name=name):
                seq.append((clk.tick, name))
            sched.add(name, hz, rec)
        for _ in range(16):
            sched.run_due()
            clk.advance()
        return seq

    first = trace_run()
    assert first[:3] == [(0, "c"), (0, "a"), (0, "b")]  # registration order
    assert first == trace_run()  # deterministic


@pytest.mark.parametrize("bad_hz", [300, 1600, 0, -10])
def test_non_divisor_rate_rejected(bad_hz):
    sched = RateGroupScheduler(VirtualClock(800))
    with pytest.raises(ValueError):
        sched.add("bad", bad_hz, lambda now: None)


def test_sub_hz_divisor_rate_accepted():
    clk = VirtualClock(800)
    sched = RateGroupScheduler(clk)
    fired = []
    sched.add("halfhz", 0.5, fired.append)
    for _ in range(1600):  # 2 s
        sched.run_due()
        clk.advance()
    assert fired == [0.0]  # tick 0 only; next due at t=2.0


# -- MicroScheduler ------------------------------------------------------------

def test_micro_scheduler_k_from_rates():
    assert MicroScheduler(world_dt=0.05, base_hz=800).k == 40
    assert MicroScheduler(world_dt=0.05, base_hz=20).k == 1


@pytest.mark.parametrize("base_hz", [30, 7, 19])
def test_micro_scheduler_rejects_non_integral_k(base_hz):
    with pytest.raises(ValueError):
        MicroScheduler(world_dt=0.05, base_hz=base_hz)


# -- World seam: inert at K=1 and K=4, pins bit-for-bit --------------------------

def _run_small_with_micro(base_hz: int):
    from test_end_to_end import SMALL_SCENARIO

    sc = scenario_mod.build(copy.deepcopy(SMALL_SCENARIO))
    world = sc.world
    micro = MicroScheduler(world_dt=world.dt, base_hz=base_hz)
    ticks_seen = []
    micro.add("noop-probe", base_hz, ticks_seen.append)
    world.micro = micro
    summary = sc.run()
    return world, micro, ticks_seen, {"events": world.events, "summary": summary}


@pytest.mark.parametrize("base_hz,k", [(20, 1), (80, 4)])
def test_micro_seam_reproduces_pins_bit_for_bit(base_hz, k):
    world, micro, ticks_seen, payload = _run_small_with_micro(base_hz)
    golden = gu.golden_path("small_scenario").read_text(encoding="utf-8")
    assert gu.to_json(payload) == golden
    macro_steps = round(world.t / world.dt)
    assert micro.clock.tick == k * macro_steps
    assert len(ticks_seen) == k * macro_steps
