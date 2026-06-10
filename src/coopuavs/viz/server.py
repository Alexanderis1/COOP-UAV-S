"""ICD-RUNTIME serve layer — the long-running backend process (ICD §1).

One process, three surfaces:

* **HTTP** — static frontend from ``viz/web`` (plus ``/recording.json``
  in replay mode);
* **WS ``/ops``** — operational channel: scene/frames/auth flow/run
  lifecycle southbound, the §3 control commands northbound;
* **WS ``/eval``** — evaluation-only channel: ground truth + live metrics
  (SRS ICD-002). Same port as /ops, separate path.

Lifecycle: idle until a ``start_run`` arrives on /ops; the request is
turned into a scenario via :func:`~coopuavs.sim.scenario.build_parametric`
over the preset, a :class:`~coopuavs.sim.runctl.RunController` ticks it
from the wall clock inside the asyncio loop (~20 Hz), every recorded frame
is broadcast on /ops and the matching truth payload on /eval, the
orchestrator's northbound ``auth_request``/``auth_resolved`` messages are
forwarded per ICD §2.3, and on completion the ``summary`` goes out and the
server returns to idle, ready for the next ``start_run``.

``serve_replay`` (the ``coopuavs run`` post-run dashboard) is unchanged
from v0.1.
"""

from __future__ import annotations

import asyncio
import copy
import functools
import http.server
import json
import random
import sys
import threading
import time
from pathlib import Path
from urllib.parse import urlsplit

import yaml

from ..sim import scenario as scenario_mod
from ..sim.runctl import RunController
from ..sim.scenario import Scenario

WEB_DIR = Path(__file__).parent / "web"

TICK_PERIOD_S = 0.05          # ~20 Hz controller ticking
MAX_TICK_WALL_S = 0.25        # clamp catch-up bursts after event-loop stalls


# ---------------------------------------------------------------------------
# Static HTTP (frontend + replay file)
# ---------------------------------------------------------------------------


class _Handler(http.server.SimpleHTTPRequestHandler):
    # Per-server state: _start_http builds a fresh subclass per server so two
    # serve()/serve_replay() instances in one process do not clobber each other.
    recording_path: Path | None = None
    ws_port: int | None = None

    def do_GET(self):  # noqa: N802 (http.server API)
        path = self.path.split("?")[0]
        if path == "/runtime-config.json":
            self._send_json(json.dumps({"ws_port": self.ws_port}).encode())
            return
        if path == "/recording.json" and self.recording_path:
            self._send_json(self.recording_path.read_bytes())
            return
        super().do_GET()

    def _send_json(self, data: bytes) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *args):  # quiet
        pass


def _start_http(port: int, recording: Path | None, host: str = "127.0.0.1",
                ws_port: int | None = None) -> threading.Thread:
    bound = type("_BoundHandler", (_Handler,),
                 {"recording_path": recording, "ws_port": ws_port})
    handler = functools.partial(bound, directory=str(WEB_DIR))
    httpd = http.server.ThreadingHTTPServer((host, port), handler)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    return thread


def serve_replay(recording: Path, port: int = 8000, host: str = "127.0.0.1") -> None:
    _start_http(port, recording, host=host)
    print(f"Dashboard (replay): http://localhost:{port}/?replay=1")
    try:
        # Not Event().wait(): an untimed wait in the main thread cannot be
        # interrupted by Ctrl+C on Windows (bpo-35935); sleep can.
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass


# ---------------------------------------------------------------------------
# /ops + /eval websocket backend
# ---------------------------------------------------------------------------


class CommandServer:
    """The /ops + /eval websocket endpoint pair on one port (ICD §1)."""

    def __init__(self, preset_cfg: dict, host: str = "127.0.0.1", ws_port: int = 8001):
        self.preset_cfg = preset_cfg
        self.host = host
        self.ws_port = ws_port
        self.ops_clients: set = set()
        self.eval_clients: set = set()
        self.ctl: RunController | None = None
        self.scenario: Scenario | None = None
        self._truth_idx = 0
        self._northbound: list[tuple[str, dict]] = []
        self._run_task: asyncio.Task | None = None
        self._server = None

    # -- lifecycle -------------------------------------------------------------

    async def start(self) -> None:
        import websockets

        self._server = await websockets.serve(self._handler, self.host, self.ws_port)
        self.ws_port = self._server.sockets[0].getsockname()[1]

    async def stop(self) -> None:
        if self._run_task is not None:
            self._run_task.cancel()
            try:
                await self._run_task
            except (asyncio.CancelledError, Exception):
                pass
            self._run_task = None
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()

    @property
    def active(self) -> bool:
        return self.ctl is not None and self.ctl.status in ("running", "paused")

    # -- connection handling --------------------------------------------------------

    def _origin_allowed(self, origin: str | None) -> bool:
        """Cross-site protection: accept connections with no Origin header
        (non-browser clients) or an Origin on this server's own host or
        localhost; reject anything else before processing any message."""
        if origin is None:
            return True
        return urlsplit(origin).hostname in {self.host, "localhost",
                                             "127.0.0.1", "::1"}

    async def _handler(self, ws) -> None:
        if not self._origin_allowed(ws.request.headers.get("Origin")):
            await ws.close(code=4403, reason="origin not allowed")
            return
        path = ws.request.path.split("?")[0].rstrip("/") or "/"
        if path == "/ops":
            await self._serve_ops(ws)
        elif path == "/eval":
            await self._serve_eval(ws)
        else:
            await ws.close(code=4404, reason=f"unknown path {path}")

    async def _serve_ops(self, ws) -> None:
        self.ops_clients.add(ws)
        try:
            # Late joiners get the scene + current run state immediately.
            if self.ctl is not None:
                sc = self.scenario
                await self._send(ws, "run_started", {
                    "name": sc.name, "seed": sc.meta["seed"],
                    "eval": sc.meta.get("eval", True),
                })
                await self._send(ws, "scene", self.ctl.scene())
                await self._send(ws, "frame", self.ctl.frame())
                if sc.orchestrator is not None:
                    for payload in sc.orchestrator.pending_requests():
                        await self._send(ws, "auth_request", payload)
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                except (ValueError, TypeError):
                    await self._error(ws, "malformed message (not JSON)")
                    continue
                if not isinstance(msg, dict) or not isinstance(msg.get("data") or {}, dict):
                    await self._error(ws, "malformed message (not a JSON object)")
                    continue
                await self._handle_control(ws, msg.get("type"), msg.get("data") or {})
        finally:
            self.ops_clients.discard(ws)

    async def _serve_eval(self, ws) -> None:
        """Evaluation channel (ICD-002): accepted always in this deployment;
        the production build simply does not run this endpoint."""
        self.eval_clients.add(ws)
        try:
            if self.ctl is not None:
                await self._send(ws, "truth", self.ctl.truth())
            await ws.wait_closed()
        finally:
            self.eval_clients.discard(ws)

    # -- control commands (ICD §3) ------------------------------------------------------

    async def _handle_control(self, ws, msg_type: str, data: dict) -> None:
        if msg_type == "start_run":
            await self._start_run(ws, data)
            return
        ctl, orch = self.ctl, (self.scenario.orchestrator if self.scenario else None)
        if ctl is None:
            await self._error(ws, f"no active run (command '{msg_type}')")
            return
        try:
            if msg_type == "stop_run":
                ctl.stop()
            elif msg_type == "pause":
                ctl.pause()
            elif msg_type == "resume":
                ctl.resume()
            elif msg_type == "set_speed":
                ctl.set_speed(float(data["speed"]))
            elif msg_type == "set_posture":
                ctl.set_posture(str(data["posture"]))
            elif msg_type == "authorize":
                if orch is not None:
                    orch.resolve(int(data["id"]), bool(data["approve"]))
            elif msg_type == "uav_command":
                uav_id, command = data.get("uav_id"), data.get("command")
                if uav_id not in self.scenario.uavs:
                    await self._error(ws, f"unknown uav '{uav_id}'")
                elif command != "rtb":
                    await self._error(ws, f"unknown uav command '{command}'")
                elif orch is not None:
                    orch.uav_command(uav_id, command)
            else:
                await self._error(ws, f"unknown command '{msg_type}'")
        except (KeyError, TypeError, ValueError) as e:
            await self._error(ws, f"bad '{msg_type}' command: {e}")
            return
        if msg_type in ("pause", "resume", "set_speed", "set_posture"):
            # A paused controller emits no frames on its own, so a run-block
            # change made while paused would reach clients only on reconnect
            # (the PAUSE button could never become RESUME). Push the current
            # frame with the updated run block; its events/decisions were
            # already delivered with the original frame, so strip them.
            frame = ctl.frame()
            frame["events"], frame["decisions"] = [], []
            self._broadcast_ops("frame", frame)

    async def _start_run(self, ws, request: dict) -> None:
        if self.active:
            await self._error(ws, "a run is already active — stop it first")
            return
        seed = request.get("seed")
        if seed is None:
            seed = random.randrange(1, 2**31)   # echoed in run_started (HMI-SCN-002)
        try:
            sc = scenario_mod.build_parametric(request, self.preset_cfg, int(seed))
        except (ValueError, KeyError, TypeError) as e:
            await self._error(ws, str(e))       # structured rejection (HMI-SCN-003)
            return
        self.begin(sc)

    def begin(self, sc: Scenario) -> None:
        """Attach a built scenario and start ticking it (also the seam for
        the CLI's auto-started ``run --live`` and for tests)."""
        self.scenario = sc
        self.ctl = RunController(sc)
        self._truth_idx = 0
        self._northbound = []
        if sc.orchestrator is not None:
            sc.orchestrator.set_northbound(
                lambda msg_type, data: self._northbound.append((msg_type, data))
            )
        self._broadcast_ops("run_started", {
            "name": sc.name, "seed": sc.meta["seed"], "eval": sc.meta.get("eval", True),
        })
        self._broadcast_ops("scene", self.ctl.scene())
        self._run_task = asyncio.get_event_loop().create_task(self._run_loop())
        self._run_task.add_done_callback(self._on_run_task_done)

    # -- run loop ---------------------------------------------------------------------------

    async def _run_loop(self) -> None:
        # Capture the controller/scenario at loop start: a stop_run+start_run
        # inside one tick swaps self.ctl, and the old loop must neither tick
        # the new controller nor clear the new run's state on exit.
        ctl, sc = self.ctl, self.scenario
        loop = asyncio.get_event_loop()
        last = loop.time()
        try:
            while ctl.status != "done":
                await asyncio.sleep(TICK_PERIOD_S)
                now = loop.time()
                wall_dt = min(now - last, MAX_TICK_WALL_S)
                last = now
                self._flush(ctl, ctl.tick(wall_dt))
            self._flush(ctl, [])                      # trailing auth resolutions
            self._broadcast_ops("summary", ctl.summary())
        finally:
            if self.ctl is ctl:
                self.ctl = None
                self._run_task = None
            if self.scenario is sc:
                self.scenario = None

    def _on_run_task_done(self, task: asyncio.Task) -> None:
        """Surface run-loop crashes: log and broadcast instead of silently
        dropping the exception of a never-awaited task."""
        if task.cancelled():
            return
        exc = task.exception()
        if exc is not None:
            print(f"run loop crashed: {exc!r}", file=sys.stderr)
            self._broadcast_ops("error", {"message": f"run crashed: {exc}"})

    def _flush(self, ctl: RunController, frames: list[dict]) -> None:
        """Broadcast new frames on /ops, matching truth payloads on /eval,
        and queued orchestrator northbound messages (ICD §2.2/§2.3/§4)."""
        recorder = ctl.recorder
        truths = recorder.truths[self._truth_idx:]
        self._truth_idx = len(recorder.truths)
        for frame in frames:
            self._broadcast_ops("frame", frame)
        for truth in truths:
            self._broadcast(self.eval_clients, "truth", truth)
        northbound, self._northbound = self._northbound, []
        for msg_type, data in northbound:
            self._broadcast_ops(msg_type, data)

    # -- transport helpers ----------------------------------------------------------------------

    def _broadcast_ops(self, msg_type: str, data: dict) -> None:
        self._broadcast(self.ops_clients, msg_type, data)

    @staticmethod
    def _broadcast(clients: set, msg_type: str, data: dict) -> None:
        if not clients:
            return
        import websockets

        websockets.broadcast(clients, json.dumps({"type": msg_type, "data": data}))

    @staticmethod
    async def _send(ws, msg_type: str, data: dict) -> None:
        await ws.send(json.dumps({"type": msg_type, "data": data}))

    @staticmethod
    async def _error(ws, message: str) -> None:
        await ws.send(json.dumps({"type": "error", "data": {"message": message}}))


# ---------------------------------------------------------------------------
# Blocking entry points (CLI)
# ---------------------------------------------------------------------------


def serve(
    preset: str | Path,
    port: int = 8000,
    ws_port: int = 8001,
    auto_start: bool = False,
    seed: int | None = None,
    speed: float | None = None,
    host: str = "127.0.0.1",
) -> None:
    """Run the ICD-RUNTIME backend until interrupted.

    ``auto_start=True`` (the ``coopuavs run --live`` path) builds the preset
    YAML as a scenario and starts it immediately; the server then returns to
    idle and keeps accepting ``start_run`` requests.

    ``host`` binds both the HTTP and websocket servers; the default loopback
    keeps the unauthenticated control channel off the network.
    """
    try:
        import websockets  # noqa: F401  (fail early with a clear message)
    except ImportError as e:  # pragma: no cover
        raise SystemExit("serve mode needs the 'websockets' package "
                         "(pip install coopuavs[viz])") from e

    preset_cfg = yaml.safe_load(Path(preset).read_text())

    async def main() -> None:
        server = CommandServer(preset_cfg, host=host, ws_port=ws_port)
        await server.start()
        _start_http(port, None, host=host, ws_port=server.ws_port)
        print(f"Console:    http://localhost:{port}/")
        print(f"Websocket:  ws://localhost:{server.ws_port}/ops  +  /eval")
        if auto_start:
            sc = scenario_mod.build(copy.deepcopy(preset_cfg), seed=seed)
            if speed is not None:
                sc.meta["speed"] = float(speed)
            server.begin(sc)
            print(f"Auto-started run '{sc.name}' (seed {sc.meta['seed']}, "
                  f"speed {sc.meta.get('speed', 1.0)}x)")
        try:
            await asyncio.Future()                 # run until cancelled
        finally:
            await server.stop()

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
