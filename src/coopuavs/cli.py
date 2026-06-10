"""Command-line entry point.

Examples
--------
Run headless, print the engagement summary::

    coopuavs run scenarios/residential_raid.yaml --headless

Run, record, and open the 3D replay dashboard::

    coopuavs run scenarios/residential_raid.yaml

Stream the battle live to the dashboard at 4x real time::

    coopuavs run scenarios/residential_raid.yaml --live --speed 4

Batch Monte-Carlo over seeds (defence effectiveness statistics)::

    coopuavs batch scenarios/residential_raid.yaml -n 20
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .sim import scenario as scenario_mod


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="coopuavs", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    run_p = sub.add_parser("run", help="run one scenario")
    run_p.add_argument("scenario", type=Path)
    run_p.add_argument("--headless", action="store_true", help="no dashboard")
    run_p.add_argument("--live", action="store_true", help="stream live instead of replay")
    run_p.add_argument("--seed", type=int, default=None)
    run_p.add_argument("--speed", type=float, default=4.0, help="live time scale")
    run_p.add_argument("--port", type=int, default=8000)
    run_p.add_argument("--record", type=Path, default=None, help="recording output path")

    batch_p = sub.add_parser("batch", help="Monte-Carlo over seeds")
    batch_p.add_argument("scenario", type=Path)
    batch_p.add_argument("-n", "--runs", type=int, default=10)

    args = parser.parse_args(argv)
    if args.command == "run":
        _cmd_run(args)
    elif args.command == "batch":
        _cmd_batch(args)


def _cmd_run(args) -> None:
    sc = scenario_mod.load(args.scenario, seed=args.seed)

    if args.live:
        from .viz.server import serve_live
        summary = serve_live(sc.world, sc.recorder, sc.duration,
                             port=args.port, speed=args.speed)
        print(json.dumps(summary, indent=2))
        return

    summary = sc.run()
    print(json.dumps(summary, indent=2))

    record = args.record or Path("runs") / f"{sc.name}.json"
    path = sc.recorder.save(record)
    print(f"recording: {path}")
    if not args.headless:
        from .viz.server import serve_replay
        serve_replay(path, port=args.port)


def _cmd_batch(args) -> None:
    rows = []
    for seed in range(args.runs):
        sc = scenario_mod.load(args.scenario, seed=seed)
        summary = sc.run()
        summary["seed"] = seed
        rows.append(summary)
        print(json.dumps(summary))

    n = len(rows)
    agg = {
        "runs": n,
        "mean_kills": sum(r["kills"] for r in rows) / n,
        "mean_armed_leakers": sum(r["armed_leakers"] for r in rows) / n,
        "decoy_shots": sum(r["kills_decoy"] for r in rows) / n,
        "critical_zone_wrecks": sum(
            r["wrecks_by_zone"].get("CRITICAL", 0) for r in rows
        ),
    }
    print("---")
    print(json.dumps(agg, indent=2))


if __name__ == "__main__":
    main()
