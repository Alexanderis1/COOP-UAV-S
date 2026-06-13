"""Command-line entry point.

Examples
--------
Start the ICD-RUNTIME backend (web console + /ops + /eval websockets);
runs are launched from the console's scenario form::

    coopuavs serve --preset scenarios/residential_raid.yaml

Run headless, print the engagement summary::

    coopuavs run scenarios/residential_raid.yaml --headless

Run, record, and open the 3D replay dashboard::

    coopuavs run scenarios/residential_raid.yaml

Serve the console with the YAML scenario auto-started at 4x real time
(the serve layer keeps accepting new runs afterwards)::

    coopuavs run scenarios/residential_raid.yaml --live --speed 4

Batch Monte-Carlo over seeds (defence effectiveness statistics)::

    coopuavs batch scenarios/residential_raid.yaml -n 20
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .sim import scenario as scenario_mod

HOST_HELP = ("bind address for the HTTP and websocket servers; WARNING: a "
             "non-loopback value exposes an unauthenticated control channel "
             "to the network (default: 127.0.0.1)")


def _positive_int(value: str) -> int:
    n = int(value)
    if n < 1:
        raise argparse.ArgumentTypeError("must be >= 1")
    return n


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="coopuavs", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    run_p = sub.add_parser("run", help="run one scenario")
    run_p.add_argument("scenario", type=Path)
    run_p.add_argument("--headless", action="store_true", help="no dashboard")
    run_p.add_argument("--live", action="store_true",
                       help="serve the console with this scenario auto-started "
                            "(the ICD-RUNTIME backend; replaces the v0.1 live stream)")
    run_p.add_argument("--seed", type=int, default=None)
    run_p.add_argument("--speed", type=float, default=4.0, help="live time scale")
    run_p.add_argument("--port", type=int, default=8000)
    run_p.add_argument("--ws-port", type=int, default=8001)
    run_p.add_argument("--host", default="127.0.0.1", help=HOST_HELP)
    run_p.add_argument("--record", type=Path, default=None, help="recording output path")

    serve_p = sub.add_parser(
        "serve",
        help="ICD-RUNTIME backend: web console + /ops + /eval websockets; "
             "idle until the console launches a parametric run",
    )
    serve_p.add_argument("--port", type=int, default=8000, help="HTTP port (frontend)")
    serve_p.add_argument("--ws-port", type=int, default=8001, help="websocket port (/ops, /eval)")
    serve_p.add_argument("--host", default="127.0.0.1", help=HOST_HELP)
    serve_p.add_argument("--preset", type=Path,
                         default=Path("scenarios/residential_raid.yaml"),
                         help="preset supplying map/zones/sensors/fleet/turrets/ROE")

    batch_p = sub.add_parser("batch", help="Monte-Carlo over seeds")
    batch_p.add_argument("scenario", type=Path)
    batch_p.add_argument("-n", "--runs", type=_positive_int, default=10)

    city_p = sub.add_parser(
        "citygen",
        help="generate the deterministic urban scenario (SIM-ENV-006)",
    )
    city_p.add_argument("--seed", type=int, default=7)
    city_p.add_argument("-o", "--output", type=Path,
                        default=Path("scenarios/urban_raid.yaml"))

    train_p = sub.add_parser(
        "train",
        help="train the learned cooperation (WTA) policy with MAPPO "
             "(requires the [train] extra: pip install -e '.[train]')")
    train_p.add_argument("scenario", type=Path, nargs="?",
                         default=Path("scenarios/high_diver_raid.yaml"))
    train_p.add_argument("--steps", type=int, default=2_000_000,
                         help="total environment steps (ignored once --minutes elapses)")
    train_p.add_argument("--minutes", type=float, default=None,
                         help="wall-clock training budget; stops and checkpoints "
                              "when reached (use for fixed-time runs, e.g. Colab)")
    train_p.add_argument("--n-envs", type=int, default=8,
                         help="parallel env workers (≈ cores−2 on the box)")
    train_p.add_argument("--rollout", type=int, default=64, help="steps/rollout")
    train_p.add_argument("--horizon", type=int, default=220, help="episode length")
    train_p.add_argument("--lr", type=float, default=3e-4)
    train_p.add_argument("--seed", type=int, default=0)
    train_p.add_argument("--out", type=Path, default=Path("runs/marl"))
    train_p.add_argument("--sync", action="store_true",
                         help="single-process envs (debug; default multiprocess)")
    train_p.add_argument("--no-randomize", action="store_true",
                         help="train on the fixed scenario raid, not randomised")

    eval_p = sub.add_parser(
        "eval",
        help="A/B a trained policy vs the classical allocator over a seed sweep")
    eval_p.add_argument("scenario", type=Path, nargs="?",
                        default=Path("scenarios/high_diver_raid.yaml"))
    eval_p.add_argument("--policy", type=Path, required=True,
                        help="trained policy checkpoint (policy.pt)")
    eval_p.add_argument("-n", "--runs", type=_positive_int, default=10)

    args = parser.parse_args(argv)
    if args.command == "run":
        _cmd_run(args)
    elif args.command == "serve":
        from .viz.server import serve
        serve(args.preset, port=args.port, ws_port=args.ws_port, host=args.host)
    elif args.command == "batch":
        _cmd_batch(args)
    elif args.command == "citygen":
        from .sim import citygen
        path = citygen.write_yaml(citygen.generate(args.seed), args.output)
        print(f"scenario written: {path}")
    elif args.command == "train":
        _cmd_train(args)
    elif args.command == "eval":
        _cmd_eval(args)


def _cmd_run(args) -> None:
    if args.live:
        if args.record:
            print("warning: --record is ignored with --live; the serve "
                  "backend streams frames instead of writing a replay file",
                  file=sys.stderr)
        from .viz.server import serve
        serve(args.scenario, port=args.port, ws_port=args.ws_port,
              auto_start=True, seed=args.seed, speed=args.speed, host=args.host)
        return

    sc = scenario_mod.load(args.scenario, seed=args.seed)
    summary = sc.run()
    print(json.dumps(summary, indent=2))

    record = args.record or Path("runs") / f"{sc.name}.json"
    path = sc.recorder.save(record)
    print(f"recording: {path}")
    if not args.headless:
        from .viz.server import serve_replay
        serve_replay(path, port=args.port, host=args.host)


def _cmd_train(args) -> None:
    from .rl.mappo import MappoConfig, train
    cfg = MappoConfig(
        scenario=str(args.scenario), total_steps=args.steps, n_envs=args.n_envs,
        rollout_steps=args.rollout, horizon=args.horizon, lr=args.lr,
        seed=args.seed, out_dir=str(args.out), subproc=not args.sync,
        randomize=not args.no_randomize,
        time_budget_s=(args.minutes * 60.0 if args.minutes else None))
    budget = f"{args.minutes:g} min" if args.minutes else f"{args.steps} steps"
    print(f"training MAPPO on {args.scenario} -> {args.out} "
          f"({args.n_envs} envs, budget: {budget})")
    train(cfg)
    print(f"policy written: {args.out / 'policy.pt'}")


def _cmd_eval(args) -> None:
    from .rl.evaluate import ab_compare
    res = ab_compare(args.scenario, range(args.runs), policy=str(args.policy))
    print(json.dumps({"seeds": res["seeds"], "greedy": res["greedy"],
                      "learned": res["learned"]}, indent=2))
    g, le = res["greedy"], res["learned"]
    print("\n--- learned vs greedy (lower armed_leakers / debris better) ---")
    for k in ("armed_leakers", "kills", "jet_leaks", "debris_cost", "strays",
              "ammo_per_kill", "jet_mean_acq"):
        print(f"  {k:16s} greedy={g.get(k)!s:>8}  learned={le.get(k)!s:>8}")


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
        "mean_decoy_kills": sum(r["kills_decoy"] for r in rows) / n,
        "total_critical_zone_wrecks": sum(
            r["wrecks_by_zone"].get("CRITICAL", 0) for r in rows
        ),
    }
    print("---")
    print(json.dumps(agg, indent=2))


if __name__ == "__main__":
    main()
