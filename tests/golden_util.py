"""Shared helpers for the P0-3 characterization pins (golden event streams).

The two reference runs and their canonical JSON encoding live here so the
test (tests/test_characterization.py) and the recorder
(scripts/record_golden.py) can never drift apart.

Golden files are re-recorded ONLY at a sanctioned re-baseline (P0-7 of
docs/PLAN_PROBLEM1.md); any other mismatch is a behavior regression.
"""

from __future__ import annotations

import copy
import json
from pathlib import Path

import numpy as np

from coopuavs.sim import scenario as scenario_mod

REPO_ROOT = Path(__file__).resolve().parents[1]
GOLDEN_DIR = REPO_ROOT / "tests" / "fixtures" / "golden"
URBAN_RAID_YAML = REPO_ROOT / "scenarios" / "urban_raid.yaml"
URBAN_RAID_SECONDS = 60.0  # mirrors test_end_to_end.test_deterministic_urban_raid


def canonical(obj):
    """Recursively convert numpy scalars/arrays so json.dumps is exact."""
    if isinstance(obj, dict):
        return {k: canonical(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [canonical(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return [canonical(v) for v in obj.tolist()]
    if isinstance(obj, np.generic):
        return obj.item()
    return obj


def to_json(payload: dict) -> str:
    return json.dumps(canonical(payload), sort_keys=True, indent=1)


def run_small_scenario() -> dict:
    from test_end_to_end import SMALL_SCENARIO

    sc = scenario_mod.build(copy.deepcopy(SMALL_SCENARIO))
    summary = sc.run()
    return {"events": sc.world.events, "summary": summary}


def run_urban_raid() -> dict:
    sc = scenario_mod.load(str(URBAN_RAID_YAML), seed=7)
    summary = sc.world.run(URBAN_RAID_SECONDS, stop_when_clear=False)
    return {"events": sc.world.events, "summary": summary}


RUNS = {
    "small_scenario": run_small_scenario,
    "urban_raid_60s_seed7": run_urban_raid,
}


def golden_path(name: str) -> Path:
    return GOLDEN_DIR / f"{name}.json"


def record_all() -> None:
    GOLDEN_DIR.mkdir(parents=True, exist_ok=True)
    for name, run in RUNS.items():
        text = to_json(run())
        path = golden_path(name)
        path.write_text(text, encoding="utf-8", newline="\n")
        print(f"recorded {path.relative_to(REPO_ROOT)} ({len(text)} bytes)")
