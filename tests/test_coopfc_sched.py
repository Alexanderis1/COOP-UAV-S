"""P3-2: coopfc/sched.py — FCU-internal rate-group scheduler.

The flight software's own scheduler (the sim-side sil/clock.py twin lives
across the fence; coopfc cannot import it). Driven one integer tick at a
time by the VirtualMCU host. Contracts under test:

- exact fire counts: a task at rate r fires exactly r*T times over T
  seconds (tick 0 fires everything — boot alignment, no drift);
- deterministic order: registration order is the within-tick pipeline;
- overruns are *modeled*, not measured: a task declares cost_ticks and a
  fire that lands inside the previous fire's busy window is skipped and
  counted (wall-clock would be nondeterministic across runs/hosts);
  consecutive overruns past the task's threshold latch a fault (the P5
  CBIT SCHED_OVERRUN seam).
"""

from __future__ import annotations

import pytest

from coopuavs.coopfc.sched import Scheduler


def run_seconds(sched: Scheduler, seconds: float) -> None:
    for _ in range(round(seconds * sched.tick_hz)):
        sched.run_tick()


def test_exact_fire_counts_over_10_s():
    sched = Scheduler(800)
    rates = [800, 400, 100, 50, 10, 1]
    counts = {r: 0 for r in rates}
    for r in rates:
        def fn(now, r=r):
            counts[r] += 1
        sched.add(f"task_{r}", r, fn)
    run_seconds(sched, 10.0)
    for r in rates:
        assert counts[r] == r * 10, f"{r} Hz fired {counts[r]} times"


def test_now_is_derived_not_accumulated():
    sched = Scheduler(800)
    seen = []
    sched.add("t10", 10, seen.append)
    run_seconds(sched, 1.0)
    # Bit-exact against the derived form tick/tick_hz; an accumulated
    # `t += 0.1` would already differ (0.1 is not a binary float).
    assert seen == [(i * 80) / 800 for i in range(10)]


def test_registration_order_is_the_pipeline():
    sched = Scheduler(400)
    log = []
    sched.add("imu", 400, lambda now: log.append("imu"))
    sched.add("ekf", 100, lambda now: log.append("ekf"))
    sched.add("rate_ctl", 400, lambda now: log.append("rate_ctl"))
    sched.run_tick()  # tick 0: all due
    assert log == ["imu", "ekf", "rate_ctl"]
    log.clear()
    sched.run_tick()  # tick 1: only 400 Hz due
    assert log == ["imu", "rate_ctl"]


def test_rates_must_divide_tick_rate():
    sched = Scheduler(800)
    with pytest.raises(ValueError):
        sched.add("bad", 3, lambda now: None)
    with pytest.raises(ValueError):
        sched.add("toofast", 1600, lambda now: None)
    with pytest.raises(ValueError):
        sched.add("nonpos", 0, lambda now: None)


def test_duplicate_names_rejected():
    # Same lesson as RNG streams: a silent name collision couples consumers.
    sched = Scheduler(800)
    sched.add("imu", 400, lambda now: None)
    with pytest.raises(ValueError, match="imu"):
        sched.add("imu", 100, lambda now: None)


def test_tick_hz_validated():
    with pytest.raises(ValueError):
        Scheduler(0)
    with pytest.raises(ValueError):
        Scheduler(800.0)  # must be an integer, not a float


def test_overrun_skips_and_counts():
    # period 2 ticks, cost 3 ticks: fires at 0, 4, 8...; every other due
    # tick lands in the busy window and is skipped.
    sched = Scheduler(800)
    fired = []
    sched.add("heavy", 400, lambda now: fired.append(sched.tick), cost_ticks=3)
    for _ in range(16):
        sched.run_tick()
    assert fired == [0, 4, 8, 12]
    stats = sched.stats("heavy")
    assert stats.fires == 4
    assert stats.overruns == 4


def test_zero_cost_never_overruns():
    sched = Scheduler(800)
    sched.add("light", 800, lambda now: None)
    run_seconds(sched, 1.0)
    assert sched.stats("light").overruns == 0
    assert sched.faults() == []


def test_cost_equal_to_period_is_back_to_back_not_overrun():
    sched = Scheduler(800)
    sched.add("full", 400, lambda now: None, cost_ticks=2)
    for _ in range(8):
        sched.run_tick()
    assert sched.stats("full").fires == 4
    assert sched.stats("full").overruns == 0


def test_consecutive_overruns_latch_fault():
    sched = Scheduler(800)
    sched.add("heavy", 400, lambda now: None, cost_ticks=3, overrun_fault_after=1)
    for _ in range(4):
        sched.run_tick()
    assert sched.stats("heavy").faulted is True
    assert sched.faults() == ["heavy"]


def test_fault_threshold_counts_consecutive_not_total():
    # skip/fire alternation: consecutive overruns never exceed 1, so a
    # threshold of 2 must never latch even as totals grow.
    sched = Scheduler(800)
    sched.add("heavy", 400, lambda now: None, cost_ticks=3, overrun_fault_after=2)
    for _ in range(40):
        sched.run_tick()
    assert sched.stats("heavy").overruns == 10
    assert sched.stats("heavy").faulted is False
    assert sched.faults() == []


def test_overrun_fault_disabled_with_none():
    sched = Scheduler(800)
    sched.add("heavy", 400, lambda now: None, cost_ticks=5, overrun_fault_after=None)
    for _ in range(40):
        sched.run_tick()
    assert sched.stats("heavy").overruns > 0
    assert sched.stats("heavy").faulted is False


def test_run_twice_identical():
    def build():
        sched = Scheduler(800)
        log = []
        sched.add("a", 400, lambda now: log.append(("a", sched.tick)))
        sched.add("b", 100, lambda now: log.append(("b", sched.tick)), cost_ticks=10)
        sched.add("c", 50, lambda now: log.append(("c", sched.tick)))
        run_seconds(sched, 2.0)
        return log, sched.stats("b").overruns

    assert build() == build()


def test_exception_propagates_to_host():
    # The per-MCU exception fence ("processor crash") is the VirtualMCU
    # host's job (P4); the scheduler must not swallow.
    sched = Scheduler(800)

    def boom(now):
        raise RuntimeError("driver died")

    sched.add("bad", 800, boom)
    with pytest.raises(RuntimeError, match="driver died"):
        sched.run_tick()
