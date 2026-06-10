"""Serve layer integration: /ops + /eval round trip against a live
CommandServer on free ports — the ICD-RUNTIME §1-§4 contract in miniature."""

import asyncio
import json

import pytest

websockets = pytest.importorskip("websockets")

from coopuavs.viz.server import CommandServer  # noqa: E402

PRESET = {
    "name": "serve-preset",
    "seed": 1,
    "dt": 0.05,
    "duration": 240.0,
    "record_hz": 5.0,
    "environment": {
        "bounds": [-2500.0, -2500.0, 2500.0, 2500.0],
        "cell_size": 100.0,
        "default_zone": "SAFE",
        "zones": [{"rect": [-800, -800, 800, 800], "class": "DANGEROUS"}],
        "assets": [{"name": "substation", "position": [0.0, 0.0, 0.0], "value": 1.0}],
    },
    "base_station": {"rate_hz": 1.0},
    "sensors": [
        {"type": "radar", "name": "radar-1", "position": [0.0, -1000.0, 10.0],
         "max_range": 9000.0},
    ],
    "interceptors": [
        {"id": "u1", "home": [-200.0, -1000.0, 0.0], "effector": "projectile",
         "max_speed": 80.0},
        {"id": "u2", "home": [200.0, -1000.0, 0.0], "effector": "projectile",
         "max_speed": 80.0},
    ],
    "threats": [],
}

START_RUN = {
    "type": "start_run",
    "data": {
        "threats": {"owa_strategic": {"count": 2, "target": "auto",
                                      "first_time": 3.0, "spacing": 6.0}},
        "duration": 150.0,
        "speed": 10.0,
        "seed": 7,
        "posture": "human_confirm",
    },
}


async def _drive() -> dict:
    server = CommandServer(PRESET, host="127.0.0.1", ws_port=0)
    await server.start()
    out = {"ops_types": set(), "eval_types": set(), "approved": 0,
           "summary": None, "run_started": None, "errors": []}
    try:
        uri = f"ws://127.0.0.1:{server.ws_port}"
        async with websockets.connect(f"{uri}/ops") as ops, \
                websockets.connect(f"{uri}/eval") as ev:

            # Structured rejection for an unknown asset (HMI-SCN-003).
            bad = json.loads(json.dumps(START_RUN))
            bad["data"]["threats"]["owa_strategic"]["target"] = "power-plant"
            await ops.send(json.dumps(bad))
            msg = json.loads(await ops.recv())
            assert msg["type"] == "error"
            assert "power-plant" in msg["data"]["message"]

            await ops.send(json.dumps(START_RUN))

            async def eval_reader():
                async for raw in ev:
                    out["eval_types"].add(json.loads(raw)["type"])

            eval_task = asyncio.create_task(eval_reader())
            second_start_sent = False

            async with asyncio.timeout(60):
                async for raw in ops:
                    msg = json.loads(raw)
                    out["ops_types"].add(msg["type"])
                    if msg["type"] == "run_started":
                        out["run_started"] = msg["data"]
                        # start_run while active must be refused.
                        await ops.send(json.dumps(START_RUN))
                        second_start_sent = True
                    elif msg["type"] == "error":
                        assert second_start_sent, msg
                        out["errors"].append(msg["data"]["message"])
                    elif msg["type"] == "auth_request":
                        await ops.send(json.dumps({
                            "type": "authorize",
                            "data": {"id": msg["data"]["id"], "approve": True},
                        }))
                        out["approved"] += 1
                    elif msg["type"] == "summary":
                        out["summary"] = msg["data"]
                        break
            eval_task.cancel()
    finally:
        await server.stop()
    return out


def test_ops_eval_round_trip_with_human_confirm():
    out = asyncio.run(_drive())

    assert {"run_started", "scene", "frame", "summary"} <= out["ops_types"]
    assert out["run_started"]["seed"] == 7              # echoed (HMI-SCN-002)
    assert "truth" in out["eval_types"]
    assert out["errors"] and "already active" in out["errors"][0]

    assert out["approved"] >= 1                          # human-on-the-loop closed
    metrics = out["summary"]["metrics"]
    assert metrics["auth"]["requests"] >= 1
    assert metrics["auth"]["approved"] >= 1
    assert metrics["auth"]["approved"] <= metrics["auth"]["requests"]
    assert out["summary"]["kills"] >= 1                  # approval released a shot
