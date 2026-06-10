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

            async def ops_reader():
                nonlocal second_start_sent
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

            await asyncio.wait_for(ops_reader(), 60)   # 3.10-compatible timeout
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


async def _recv_type(ws, msg_type: str, timeout: float = 10.0) -> dict:
    """Skip interleaved frames etc. until a message of msg_type arrives."""
    async def _wait():
        while True:
            msg = json.loads(await ws.recv())
            if msg["type"] == msg_type:
                return msg
    return await asyncio.wait_for(_wait(), timeout)


async def _drive_bad_messages() -> None:
    server = CommandServer(PRESET, host="127.0.0.1", ws_port=0)
    await server.start()
    try:
        uri = f"ws://127.0.0.1:{server.ws_port}"
        async with websockets.connect(f"{uri}/ops") as ops:
            # Valid JSON that is not an object (or with non-object data)
            # must produce an error reply, not crash the connection.
            for raw in ("5", '"pause"', '[1, 2]',
                        json.dumps({"type": "pause", "data": [1, 2]})):
                await ops.send(raw)
                msg = await _recv_type(ops, "error")
                assert "JSON object" in msg["data"]["message"]

            # Oversized parametric requests: structured rejection (HMI-SCN-003).
            big = {"type": "start_run",
                   "data": {"threats": {"fpv": {"count": 100000}}, "seed": 1}}
            await ops.send(json.dumps(big))
            msg = await _recv_type(ops, "error")
            assert "maximum" in msg["data"]["message"]
            long_run = {"type": "start_run",
                        "data": {"threats": {"fpv": {"count": 1}},
                                 "duration": 1e9, "seed": 1}}
            await ops.send(json.dumps(long_run))
            msg = await _recv_type(ops, "error")
            assert "duration" in msg["data"]["message"]

            # Non-finite speed on a live run is refused, not clamped to nan.
            start = json.loads(json.dumps(START_RUN))
            start["data"]["speed"] = 0.1                 # keep the run alive
            await ops.send(json.dumps(start))
            await _recv_type(ops, "run_started")
            for bad_speed in ("NaN", "Infinity", '"nan"'):
                await ops.send('{"type": "set_speed", "data": {"speed": %s}}'
                               % bad_speed)
                msg = await _recv_type(ops, "error")
                assert "finite" in msg["data"]["message"]
            await ops.send(json.dumps({"type": "stop_run", "data": {}}))
            await _recv_type(ops, "summary", timeout=15.0)
    finally:
        await server.stop()


def test_malformed_and_oversized_requests_get_error_replies():
    asyncio.run(_drive_bad_messages())


async def _recv_frame_where(ws, pred, timeout: float = 10.0) -> dict:
    """Skip messages until a frame whose run block satisfies pred arrives."""
    async def _wait():
        while True:
            msg = json.loads(await ws.recv())
            if msg["type"] == "frame" and pred(msg["data"]["run"]):
                return msg["data"]
    return await asyncio.wait_for(_wait(), timeout)


async def _drive_pause_state_push() -> None:
    server = CommandServer(PRESET, host="127.0.0.1", ws_port=0)
    await server.start()
    try:
        uri = f"ws://127.0.0.1:{server.ws_port}"
        async with websockets.connect(f"{uri}/ops") as ops:
            start = json.loads(json.dumps(START_RUN))
            start["data"]["speed"] = 0.1                 # keep the run alive
            await ops.send(json.dumps(start))
            await _recv_type(ops, "frame")

            # The pause must become visible to already-connected clients: a
            # paused controller emits no frames on its own, so without the
            # command-driven push the PAUSE button could never show RESUME.
            await ops.send(json.dumps({"type": "pause", "data": {}}))
            frame = await _recv_frame_where(ops, lambda r: r["status"] == "paused")
            # The push re-sends the last frame; its events/decisions were
            # already delivered once and must not be duplicated.
            assert frame["events"] == [] and frame["decisions"] == []

            # Speed and posture changes made while paused propagate the same
            # way (no tick loop is running to carry them).
            await ops.send(json.dumps({"type": "set_speed", "data": {"speed": 2.0}}))
            frame = await _recv_frame_where(ops, lambda r: r["speed"] == 2.0)
            assert frame["run"]["status"] == "paused"
            await ops.send(json.dumps({"type": "set_posture",
                                       "data": {"posture": "weapons_hold"}}))
            await _recv_frame_where(ops, lambda r: r["posture"] == "weapons_hold")

            # Resume flips the status back immediately, ahead of the next
            # recorder-cadence frame (0.2 sim s is 2 wall s at speed 0.1).
            await ops.send(json.dumps({"type": "resume", "data": {}}))
            await _recv_frame_where(ops, lambda r: r["status"] == "running")

            await ops.send(json.dumps({"type": "stop_run", "data": {}}))
            await _recv_type(ops, "summary", timeout=15.0)
    finally:
        await server.stop()


def test_pause_state_reaches_connected_clients():
    asyncio.run(_drive_pause_state_push())


async def _drive_origins() -> None:
    server = CommandServer(PRESET, host="127.0.0.1", ws_port=0)
    await server.start()
    try:
        uri = f"ws://127.0.0.1:{server.ws_port}/ops"
        # No Origin (non-browser clients) and the server's own host are
        # accepted: the command is processed ("no active run" error reply).
        async with websockets.connect(uri) as ws:
            await ws.send(json.dumps({"type": "pause", "data": {}}))
            msg = await _recv_type(ws, "error")
            assert "no active run" in msg["data"]["message"]
        async with websockets.connect(uri, origin="http://127.0.0.1:8000") as ws:
            await ws.send(json.dumps({"type": "pause", "data": {}}))
            msg = await _recv_type(ws, "error")
            assert "no active run" in msg["data"]["message"]
        # A cross-site Origin is rejected before any message is processed.
        async with websockets.connect(uri, origin="http://evil.example") as ws:
            with pytest.raises(websockets.exceptions.ConnectionClosed):
                await asyncio.wait_for(ws.recv(), 5)
            assert ws.close_code == 4403
    finally:
        await server.stop()


def test_origin_check_rejects_cross_site_browsers():
    asyncio.run(_drive_origins())
