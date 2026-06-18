"""P3-9 @oracle: bench waypoint square vs ArduCopter SITL envelope.

Both autopilots fly the same geometry (200 m square, 50 m AGL, 10 m/s
commanded): ours = the full P3-8 bench (CoopFC + P2 devices + P1
physics), the oracle = the official prebuilt ArduCopter SITL (EKF3,
default '+' quad, recorded offline by
scripts/oracle/export_ardupilot_square.py into
tests/fixtures/oracle/ardupilot_square.json — procedure in
tests/fixtures/oracle/README.md).

This is an ENVELOPE cross-check of two complete, independent stacks on
different airframes (12 kg interceptor vs ~2 kg SITL quad) — bands are
deliberately wide and assert flight-class agreement, not model match:

- both complete the lap; lap-time ratio in [0.5, 2.0]
- leg cross-track: ours < 2 m (the P3-8 gate) and within 2x + 1 m of
  the oracle's class
- cruise ground speed on the legs within 30% of commanded for both
- altitude hold band: both within +-4 m of the 50 m AGL command

Run with `pytest -m oracle` (separate process, repo convention).
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest

from coopuavs.sil.bench import Bench

pytestmark = pytest.mark.oracle

FIXTURE = Path(__file__).parent / "fixtures" / "oracle" / "ardupilot_square.json"
CRUISE = 10.0
WP_RADIUS = 5.0
TURN_EXCLUDE = 25.0          # m around corners excluded from leg metrics


def leg_metrics(samples, corners):
    """(worst cross-track, mean leg ground speed, lap time, alt band)."""
    xt, speeds = [], []
    t_start = t_end = None
    alt_min, alt_max = math.inf, -math.inf
    prev = (0.0, 0.0)
    legs = []
    for c in corners:
        legs.append((prev, tuple(c)))
        prev = tuple(c)
    for s in samples:
        p = (s["x"], s["y"])
        v = math.hypot(s["vx"], s["vy"])
        for a, b in legs:
            ab = (b[0] - a[0], b[1] - a[1])
            n = math.hypot(*ab)
            ap = (p[0] - a[0], p[1] - a[1])
            along = (ap[0] * ab[0] + ap[1] * ab[1]) / n
            if TURN_EXCLUDE < along < n - TURN_EXCLUDE:
                d_a = math.hypot(p[0] - a[0], p[1] - a[1])
                d_b = math.hypot(p[0] - b[0], p[1] - b[1])
                if d_a > TURN_EXCLUDE and d_b > TURN_EXCLUDE \
                        and abs(ap[0] * ab[1] - ap[1] * ab[0]) / n < 40.0:
                    xt.append(abs(ap[0] * ab[1] - ap[1] * ab[0]) / n)
                    speeds.append(v)
                    if t_start is None:
                        t_start = s["t"]
                    t_end = s["t"]
                break
        alt_min, alt_max = min(alt_min, s["alt"]), max(alt_max, s["alt"])
    return max(xt), float(np.mean(speeds)), t_end - t_start, (alt_min, alt_max)


def fly_bench_square():
    b = Bench(seed=2)
    b.boot_and_arm()
    b.run(3.0)
    nav_sub = b.fcu.topics.subscribe("nav_state")
    start = b.fcu.nav.pos
    z_hold = start[2]
    corners = [(200.0, 0.0), (200.0, 200.0), (0.0, 200.0), (0.0, 0.0)]
    b.fcu.cmd_velocity((0.0, 0.0, 0.0))
    ok, why = b.fcu.cmd_set_mode("OFFBOARD")
    assert ok, why
    samples = []
    origin = (start[0], start[1])
    for wp in corners:
        target = (origin[0] + wp[0], origin[1] + wp[1])

        def guide(bb, target=target):
            if bb.k % 80 == 0:
                nav = nav_sub.read()
                dx, dy = target[0] - nav.pos[0], target[1] - nav.pos[1]
                d = math.hypot(dx, dy)
                sp = min(CRUISE, max(1.0, d))
                bb.fcu.cmd_velocity((sp * dx / d, sp * dy / d,
                                     1.0 * (z_hold - nav.pos[2])))
                s = bb.state[0]
                samples.append({"t": bb.now, "x": s[0] - origin[0],
                                "y": s[1] - origin[1],
                                "alt": s[2] - (z_hold - 50.0),
                                "vx": s[3], "vy": s[4]})
                return d < WP_RADIUS
            return False
        assert b.run(60.0, until=guide), f"corner {wp} not reached"
    assert not b.fcu.failsafe, b.fcu.failsafe
    return samples, corners


def test_square_envelope_vs_ardupilot():
    assert FIXTURE.exists(), (
        "missing oracle fixture — run "
        "scripts/oracle/export_ardupilot_square.py (procedure in "
        "tests/fixtures/oracle/README.md)")
    ref = json.loads(FIXTURE.read_text())
    ref_samples = [{"t": r["t_boot_s"], "x": r["x"], "y": r["y"],
                    "alt": r["alt_agl"], "vx": r["vx"], "vy": r["vy"]}
                   for r in ref["samples"]]
    ref_xt, ref_speed, ref_lap, ref_alt = leg_metrics(
        ref_samples, ref["corners_en"])

    ours, corners = fly_bench_square()
    our_xt, our_speed, our_lap, our_alt = leg_metrics(ours, corners)

    print(f"\noracle: xt {ref_xt:.2f} m, speed {ref_speed:.1f} m/s, "
          f"lap {ref_lap:.0f} s, alt [{ref_alt[0]:.1f}, {ref_alt[1]:.1f}]")
    print(f"bench:  xt {our_xt:.2f} m, speed {our_speed:.1f} m/s, "
          f"lap {our_lap:.0f} s, alt [{our_alt[0]:.1f}, {our_alt[1]:.1f}]")

    assert our_xt < 2.0                                   # P3-8 gate
    assert our_xt < 2.0 * ref_xt + 1.0                    # same class
    assert 0.5 <= our_lap / ref_lap <= 2.0
    for speed in (our_speed, ref_speed):
        assert abs(speed - CRUISE) / CRUISE < 0.30
    for lo, hi in (our_alt, ref_alt):
        assert 46.0 < lo and hi < 54.0
