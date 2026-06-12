"""P4-6 sitl end-to-end: own re-baselined floors, never pointmass pins.

SITL_SMALL_SCENARIO (one low FPV vs two axis-posted shooters) through
the FULL stack: device noise → EKF → MC apps on VirtualMCUs → coop-link
→ FCU → batched plant, with the C2/ROE/adjudication pipeline unchanged.

Baseline 2026-06-12 (seeds 0..9): 10/10 kills, 0 leakers, 0 CRITICAL
wrecks, t_end 32-66 s. Floors below carry headroom against that
measurement; a tripped floor is stop-and-replan, never a tolerance bump
(the P0 doctrine). Truth quarantine is asserted in-flight: the ops
picture (UavState) is genuinely the estimate, never truth.
"""

from __future__ import annotations

import copy

import numpy as np
import pytest

from coopuavs.sim import scenario as scenario_mod
from test_sitl_stage1 import SITL_SMALL_SCENARIO

CI_SEEDS = (0, 1, 2)


def _run_seed(seed: int, probe=None):
    sc = scenario_mod.build(copy.deepcopy(SITL_SMALL_SCENARIO), seed=seed)
    summary = sc.run(on_step=probe)
    return sc, summary


def test_three_seed_kill_floor_and_critical_invariant():
    quarantine: list[float] = []

    def probe(world):
        fv = world.friendlies["u1"]
        quarantine.append(float(np.linalg.norm(
            fv.position - fv.tactical.body.position)))

    for seed in CI_SEEDS:
        sc, summary = _run_seed(seed, probe=probe if seed == CI_SEEDS[0] else None)
        assert summary["kills"] >= 1, (seed, summary)
        assert summary["wrecks_by_zone"].get("CRITICAL", 0) == 0, (seed, summary)
        kinds = {e["kind"] for e in sc.world.events}
        assert {"enemy_spawn", "acquired", "kill"} <= kinds, (seed, kinds)

    # truth quarantine, observed in flight: estimate differs from truth
    # (GM-wander class) yet stays bounded
    diffs = np.asarray(quarantine[100:])
    assert diffs.max() > 1e-3, "ops picture suspiciously equals truth"
    assert diffs.max() < 10.0, f"nav error {diffs.max():.1f} m unbounded"


def test_run_twice_deterministic():
    sc1, s1 = _run_seed(2)
    sc2, s2 = _run_seed(2)
    assert s1 == s2
    assert sc1.world.events == sc2.world.events


@pytest.mark.slow
def test_ten_seed_floor():
    kills, crit = 0, 0
    for seed in range(10):
        _, summary = _run_seed(seed)
        kills += summary["kills"]
        crit += summary["wrecks_by_zone"].get("CRITICAL", 0)
    assert kills >= 9, f"{kills}/10 kills (baseline 10/10)"
    assert crit == 0, "CRITICAL wreck invariant broken"
