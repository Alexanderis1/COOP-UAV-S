"""Frame recorder: serialises the run for the 3D dashboard.

Samples three views of the battle each frame so the dashboard can show them
side by side — ground truth (what happened), the track picture (what the
defence *believed*), and friendly telemetry. Frames go to a JSON recording
for replay, or straight onto a websocket in live mode.
"""

from __future__ import annotations

import json
from pathlib import Path

from ..core.messages import TrackArray, UavState
from ..core.node import Node
from ..sim.world import World


class Recorder(Node):
    def __init__(self, world: World, rate_hz: float = 5.0):
        super().__init__("recorder", world.bus, rate_hz=rate_hz)
        self.world = world
        self.frames: list[dict] = []
        self._tracks: TrackArray | None = None
        self._uavs: dict[str, UavState] = {}
        self._events_emitted = 0
        self.create_subscription("tracks", self._on_tracks)
        self.create_subscription("uav/state", self._on_uav)

    def _on_tracks(self, msg: TrackArray) -> None:
        self._tracks = msg

    def _on_uav(self, msg: UavState) -> None:
        self._uavs[msg.uav_id] = msg

    # -- frames ----------------------------------------------------------------

    def update(self, t: float, dt: float) -> None:
        self.frames.append(self.snapshot())

    def snapshot(self) -> dict:
        w = self.world
        new_events = w.events[self._events_emitted:]
        self._events_emitted = len(w.events)
        return {
            "t": round(w.t, 2),
            "enemies": [
                {
                    "id": e.id,
                    "cls": e.threat_class.value,
                    "pos": [round(float(x), 1) for x in e.position],
                    "alive": e.alive,
                    "killed": e.killed,
                }
                for e in w.enemies.values()
            ],
            "uavs": [
                {
                    "id": u.uav_id,
                    "pos": [round(float(x), 1) for x in u.position],
                    "mode": u.mode.value,
                    "ammo": u.ammo,
                    "battery": round(u.battery, 2),
                }
                for u in self._uavs.values()
            ],
            "tracks": [
                {
                    "id": trk.track_id,
                    "pos": [round(float(x), 1) for x in trk.position],
                    "vel": [round(float(x), 1) for x in trk.velocity],
                    "p_decoy": round(trk.p_decoy, 2),
                }
                for trk in (self._tracks.tracks if self._tracks else [])
            ],
            "wrecks": [
                {"pos": wk["pos"], "zone": wk["zone"].name} for wk in w.wrecks
            ],
            "events": new_events,
        }

    # -- static scene ----------------------------------------------------------

    def scene(self) -> dict:
        env = self.world.env
        rm = env.risk_map
        return {
            "bounds": list(env.bounds),
            "cell_size": rm.cell_size,
            "grid": rm.grid.tolist(),
            "assets": [
                {"name": a.name, "pos": [float(x) for x in a.position], "value": a.value}
                for a in env.assets
            ],
            "buildings": [
                {"rect": list(b.rect), "height": b.height} for b in env.buildings
            ],
        }

    def save(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {"scene": self.scene(), "frames": self.frames,
                   "summary": self.world.summary()}
        path.write_text(json.dumps(payload))
        return path
