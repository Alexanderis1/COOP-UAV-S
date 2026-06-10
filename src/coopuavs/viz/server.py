"""Dashboard server.

Two modes, one frontend:

* ``serve_replay``   — static HTTP server for ``viz/web`` plus the recording
  JSON at ``/recording.json``; the page plays it back with a timeline.
* ``serve_live``     — same static server, plus a websocket on ``ws_port``;
  the simulation is stepped in (scaled) real time inside the asyncio loop
  and every frame is broadcast to connected browsers.
"""

from __future__ import annotations

import asyncio
import functools
import http.server
import json
import threading
from pathlib import Path

from ..sim.world import World
from .recorder import Recorder

WEB_DIR = Path(__file__).parent / "web"


class _Handler(http.server.SimpleHTTPRequestHandler):
    recording_path: Path | None = None

    def do_GET(self):  # noqa: N802 (http.server API)
        if self.path.split("?")[0] == "/recording.json" and self.recording_path:
            data = self.recording_path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        super().do_GET()

    def log_message(self, *args):  # quiet
        pass


def _start_http(port: int, recording: Path | None) -> threading.Thread:
    handler = functools.partial(_Handler, directory=str(WEB_DIR))
    _Handler.recording_path = recording
    httpd = http.server.ThreadingHTTPServer(("0.0.0.0", port), handler)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    return thread


def serve_replay(recording: Path, port: int = 8000) -> None:
    _start_http(port, recording)
    print(f"Dashboard (replay): http://localhost:{port}/")
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        pass


def serve_live(
    world: World,
    recorder: Recorder,
    duration: float,
    port: int = 8000,
    ws_port: int = 8001,
    speed: float = 1.0,
) -> dict:
    """Run the sim in scaled real time, streaming frames to the dashboard."""
    try:
        import websockets
    except ImportError as e:  # pragma: no cover
        raise SystemExit("live mode needs the 'websockets' package "
                         "(pip install coopuavs[viz])") from e

    _start_http(port, None)
    print(f"Dashboard (live):  http://localhost:{port}/?live=1")
    print(f"Websocket:         ws://localhost:{ws_port}/")

    clients: set = set()
    scene_msg = json.dumps({"type": "scene", "data": recorder.scene()})

    async def handler(ws):
        clients.add(ws)
        try:
            await ws.send(scene_msg)
            await ws.wait_closed()
        finally:
            clients.discard(ws)

    async def main() -> dict:
        async with websockets.serve(handler, "0.0.0.0", ws_port):
            frame_period = 1.0 / recorder.rate_hz
            steps_per_frame = max(1, int(round(frame_period / world.dt)))
            end = world.t + duration
            while world.t < end:
                for _ in range(steps_per_frame):
                    world.step()
                frame = recorder.snapshot()
                if clients:
                    websockets.broadcast(
                        clients, json.dumps({"type": "frame", "data": frame})
                    )
                await asyncio.sleep(frame_period / speed)
            summary = world.summary()
            if clients:
                websockets.broadcast(
                    clients, json.dumps({"type": "summary", "data": summary})
                )
            await asyncio.sleep(2.0)
            return summary

    return asyncio.run(main())
