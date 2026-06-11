"""10-seed Monte-Carlo outcome report (PLAN_PROBLEM1 P0-6/P0-7).

Captures per-seed battle outcomes of scenarios/residential_raid.yaml before
and after the RngRegistry migration so the stochastic re-baseline is an
evidence-backed comparison, not a shrug. The hit-rate floors
(tests/test_hit_rate.py: 5-seed kills >= 10, shots/kill <= 9.0) are
recomputed here over seeds 0-4 alongside the full 10-seed aggregate.

Usage: python scripts/mc_report.py <label>     # e.g. before / after
Writes docs/reports/rng_rebaseline_<label>.json
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from coopuavs.sim import scenario as scenario_mod  # noqa: E402

SCENARIO = REPO_ROOT / "scenarios" / "residential_raid.yaml"
SEEDS = range(10)
FLOOR_SEEDS = range(5)  # the tests/test_hit_rate.py floor population


def run_seed(seed: int) -> dict:
    sc = scenario_mod.load(str(SCENARIO), seed=seed)
    summary = sc.run()
    econ = sc.eval_tracker.metrics()["economics"]
    return {
        "seed": seed,
        "kills": summary["kills"],
        "leakers": summary["leakers"],
        "armed_leakers": summary["armed_leakers"],
        "enemies_total": summary["enemies_total"],
        "wrecks_by_zone": summary["wrecks_by_zone"],
        "strays_by_zone": summary["strays_by_zone"],
        "debris_intercepted": summary["debris_intercepted"],
        "shots": econ["shots"],
        "t_end": summary["t_end"],
    }


def main(label: str) -> None:
    rows = [run_seed(seed) for seed in SEEDS]
    floor_rows = [r for r in rows if r["seed"] in FLOOR_SEEDS]
    floor_kills = sum(r["kills"] for r in floor_rows)
    floor_shots = sum(r["shots"] for r in floor_rows)
    report = {
        "label": label,
        "scenario": SCENARIO.name,
        "seeds": list(SEEDS),
        "rows": rows,
        "aggregate_10seed": {
            "kills": sum(r["kills"] for r in rows),
            "shots": sum(r["shots"] for r in rows),
            "leakers": sum(r["leakers"] for r in rows),
            "critical_wrecks": sum(
                r["wrecks_by_zone"].get("CRITICAL", 0) for r in rows),
        },
        "floors_5seed": {
            "kills": floor_kills,
            "shots": floor_shots,
            "shots_per_kill": round(floor_shots / floor_kills, 3) if floor_kills else None,
            "floor_kills_min": 10,
            "floor_shots_per_kill_max": 9.0,
            "pass": floor_kills >= 10 and floor_shots <= 9.0 * floor_kills,
        },
    }
    out = REPO_ROOT / "docs" / "reports" / f"rng_rebaseline_{label}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=1), encoding="utf-8", newline="\n")
    print(json.dumps(report["floors_5seed"], indent=1))
    print(json.dumps(report["aggregate_10seed"], indent=1))
    print(f"wrote {out.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("usage: python scripts/mc_report.py <label>")
    main(sys.argv[1])
