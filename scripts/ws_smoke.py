"""End-to-end websocket smoke test for the ICD-RUNTIME contract.

Drives a live ``coopuavs serve`` backend exactly the way the web interface
does: connects to /ops and /eval, launches a parametric run, exercises
pause/resume/speed, answers every authorisation request, and verifies the
message inventory at the end. Exit code 0 = the contract round-trips.

Usage (backend already running on the default ports)::

    python scripts/ws_smoke.py [--host localhost] [--ops-port 8001]
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys

REQUIRED_OPS = {"scene", "run_started", "frame", "summary"}
REQUIRED_EVAL = {"truth"}

START_RUN = {
    "type": "start_run",
    "data": {
        "threats": {
            "owa_strategic": {"count": 2, "target": "auto"},
            "fpv": {"count": 1, "target": "auto"},
            "decoy": {"count": 1, "target": "auto"},
        },
        "weather": {"wind_speed": 6.0, "wind_dir_deg": 270.0,
                    "fog": 0.2, "precip": 0.0, "daylight": 0.1},
        "duration": 240.0,
        "speed": 10.0,
        "seed": 7,
        "posture": "human_confirm",
    },
}


async def main(host: str, ops_port: int) -> int:
    import websockets

    seen_ops: set[str] = set()
    seen_eval: set[str] = set()
    auth_answered = 0
    ghost_seen = acquired_seen = False
    summary = None

    async with websockets.connect(f"ws://{host}:{ops_port}/ops") as ops, \
            websockets.connect(f"ws://{host}:{ops_port}/eval") as ev:

        await ops.send(json.dumps(START_RUN))

        async def eval_reader():
            nonlocal ghost_seen, acquired_seen
            async for raw in ev:
                msg = json.loads(raw)
                seen_eval.add(msg["type"])
                if msg["type"] == "truth":
                    for e in msg["data"]["enemies"]:
                        if e["acquired"]:
                            acquired_seen = True
                        else:
                            ghost_seen = True

        eval_task = asyncio.create_task(eval_reader())

        exercised_pause = False
        async with asyncio.timeout(180):
            async for raw in ops:
                msg = json.loads(raw)
                seen_ops.add(msg["type"])
                if msg["type"] == "error":
                    print("backend error:", msg["data"], file=sys.stderr)
                    return 1
                if msg["type"] == "auth_request":
                    await ops.send(json.dumps({
                        "type": "authorize",
                        "data": {"id": msg["data"]["id"], "approve": True},
                    }))
                    auth_answered += 1
                if msg["type"] == "frame" and not exercised_pause \
                        and msg["data"]["t"] > 20:
                    exercised_pause = True
                    for ctl in ({"type": "pause", "data": {}},
                                {"type": "resume", "data": {}},
                                {"type": "set_speed", "data": {"speed": 10.0}}):
                        await ops.send(json.dumps(ctl))
                if msg["type"] == "summary":
                    summary = msg["data"]
                    break

        eval_task.cancel()

    failures = []
    if missing := REQUIRED_OPS - seen_ops:
        failures.append(f"missing /ops message types: {sorted(missing)}")
    if missing := REQUIRED_EVAL - seen_eval:
        failures.append(f"missing /eval message types: {sorted(missing)}")
    if not ghost_seen:
        failures.append("never saw an unacquired (ghost) enemy on /eval")
    if not acquired_seen:
        failures.append("no enemy was ever acquired on /eval")
    if summary is None:
        failures.append("no summary received")

    print(json.dumps({
        "ops_types": sorted(seen_ops),
        "eval_types": sorted(seen_eval),
        "auth_requests_answered": auth_answered,
        "ghost_seen": ghost_seen,
        "acquired_seen": acquired_seen,
        "summary": summary,
    }, indent=2))
    for f in failures:
        print("FAIL:", f, file=sys.stderr)
    return 1 if failures else 0


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="localhost")
    p.add_argument("--ops-port", type=int, default=8001)
    a = p.parse_args()
    sys.exit(asyncio.run(main(a.host, a.ops_port)))
