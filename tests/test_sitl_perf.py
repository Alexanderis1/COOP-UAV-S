"""P4-8 perf gate: the reference raid in sitl fidelity, headless.

Gate (plan): residential_raid with `fidelity.fleet=sitl` (8 interceptor
FCU+MC pairs, full sensor/C2 pipeline) at RTF >= 0.5x headless. Measured
2026-06-12: 0.81x (1.24 s CPU/sim-s) over a boot+raid slice; committed
profile in docs/PERF_P4_SITL.md. A miss pulls the plan's fallback levers
(rate profiles, mixed fidelity) before proceeding — never silently.

Windows timing discipline (P2 lessons): time.process_time over a slice
long enough (20 sim-s) to resolve far above the 15.625 ms quantum, and
@perf runs as its own pytest process on a settled machine — concurrent
load or a hot run right after the heavy suites reads low.
"""

from __future__ import annotations

import time
from pathlib import Path

import pytest
import yaml

from coopuavs.sim import scenario as scenario_mod

RTF_GATE = 0.5
SIM_SLICE_S = 20.0


@pytest.mark.perf
def test_residential_raid_sitl_rtf():
    cfg = yaml.safe_load(
        Path("scenarios/residential_raid.yaml").read_text())
    cfg["fidelity"] = {"fleet": "sitl"}
    cfg["sitl"] = {"fcu": {"fcu.vel_max_h": 80.0, "fcu.vel_max_up": 20.0,
                           "fcu.vel_max_down": 20.0}}
    sc = scenario_mod.build(cfg)
    sc.world.run(2.0, stop_when_clear=False)         # spin-up out of the rep
    t0 = time.process_time()
    sc.world.run(SIM_SLICE_S, stop_when_clear=False)
    cpu = time.process_time() - t0
    rtf = SIM_SLICE_S / cpu
    print(f"\nresidential_raid sitl: RTF {rtf:.2f}x "
          f"({cpu / SIM_SLICE_S:.3f} s CPU/sim-s; gate >= {RTF_GATE}x, "
          f"baseline 0.81x)")
    assert rtf >= RTF_GATE, (
        f"RTF {rtf:.2f}x under the {RTF_GATE}x gate — pull the fallback "
        "levers (PLAN_PROBLEM1 perf budget) before proceeding")
