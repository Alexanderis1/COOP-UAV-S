"""Frame recorder: serialises the run in the ICD_RUNTIME wire schema.

Produces exactly the §2.1 ``scene`` and §2.2 ``frame`` data objects of
docs/ICD_RUNTIME.md — the serve layer forwards these payloads verbatim on
the ``/ops`` websocket, and ``save()`` writes the replay file the frontend
loads (``{"scene":…, "frames":[…], "truth":[…], "summary":…}``).

The recorder reads the operational topics (``tracks``, ``uav/state``,
``turret/state``) plus the sim-side world for truth-derived display data
(wrecks, strays, env) — it is an evaluation component and may see truth.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from ..core.messages import TrackArray, TurretState, UavState
from ..core.node import Node
from ..sim.world import World

# Maximum believable time-to-impact for the linear ground projection, s.
MAX_PROJECTED_TTI = 600.0


class Recorder(Node):
    def __init__(self, world: World, rate_hz: float = 5.0):
        super().__init__("recorder", world.bus, rate_hz=rate_hz)
        self.world = world
        self.frames: list[dict] = []
        self.truths: list[dict] = []
        # Run presentation state, kept current by the RunController (or left
        # at defaults for plain batch runs).
        self.run_info: dict = {"status": "running", "speed": 1.0,
                               "posture": "human_confirm"}
        self.run_meta: dict = {"name": "", "seed": 0, "duration": 0.0, "eval": True}
        self.eval_tracker = None       # set by scenario.build()

        self._tracks: TrackArray | None = None
        self._uavs: dict[str, UavState] = {}
        self._turrets: dict[str, TurretState] = {}
        self._events_emitted = 0
        self.create_subscription("tracks", self._on_tracks)
        self.create_subscription("uav/state", self._on_uav)
        self.create_subscription("turret/state", self._on_turret)

    def _on_tracks(self, msg: TrackArray) -> None:
        self._tracks = msg

    def _on_uav(self, msg: UavState) -> None:
        self._uavs[msg.uav_id] = msg

    def _on_turret(self, msg: TurretState) -> None:
        self._turrets[msg.turret_id] = msg

    # -- frames (ICD §2.2) -------------------------------------------------------

    def update(self, t: float, dt: float) -> None:
        self.frames.append(self.snapshot())
        if self.eval_tracker is not None:
            self.truths.append(self.eval_tracker.truth_payload())

    def snapshot(self, consume_events: bool = True) -> dict:
        w = self.world
        if consume_events:
            new_events = w.events[self._events_emitted:]
            self._events_emitted = len(w.events)
        else:
            new_events = []
        decisions = [e for e in new_events if e["kind"].startswith("decision")]
        events = [e for e in new_events if not e["kind"].startswith("decision")]

        return {
            "t": round(w.t, 2),
            "run": dict(self.run_info),
            "tracks": [self._track_entry(trk)
                       for trk in (self._tracks.tracks if self._tracks else [])],
            "uavs": [self._uav_entry(u) for u in self._uavs.values()],
            "turrets": [
                {
                    "id": s.turret_id,
                    "az": s.az_deg,
                    "el": s.el_deg,
                    "ammo": s.ammo,
                    "state": s.state,
                    "target": s.target_track,
                }
                for s in self._turrets.values()
            ],
            "wrecks": [
                {
                    "pos": [round(float(wk["pos"][0]), 1), round(float(wk["pos"][1]), 1), 0.0],
                    "zone": wk["zone"].name,
                    "mechanism": wk["effector"],
                }
                for wk in w.wrecks
            ],
            "strays": [
                {"pos": [round(s["pos"][0], 1), round(s["pos"][1], 1), 0.0],
                 "zone": s["zone"].name}
                for s in w.stray_impacts
            ],
            "debris": self._debris_entries(),
            "stations": self._station_entries(),
            "env": w.weather.as_dict(),
            "events": events,
            "decisions": decisions,
        }

    def _uav_entry(self, u: UavState) -> dict:
        entry = {
            "id": u.uav_id,
            "pos": [round(float(x), 1) for x in u.position],
            "vel": [round(float(x), 1) for x in u.velocity],
            "mode": u.mode.value,
            "ammo": u.ammo,
            "battery": round(u.battery, 3),
            "task_id": u.task_id,
            "link": round(float(getattr(u, "link", 1.0)), 3),
            "kind": getattr(u, "kind", "interceptor"),
            "effector": getattr(u, "effector", "") or None,
        }
        # Sitl-mode telemetry, additive only (ICD §2.2 v0.4): keys appear
        # exactly when the platform reports them — a pointmass recording
        # is byte-compatible with v0.3 parsers.
        att = getattr(u, "attitude_q", None)
        if att is not None:
            entry["att"] = [round(float(q), 4) for q in att]
        nav_q = getattr(u, "nav_quality", None)
        if nav_q is not None:
            entry["nav_q"] = round(float(nav_q), 2)
        health = getattr(u, "health", None)
        if health is not None:
            entry["health"] = health
        return entry

    def _debris_entries(self) -> list[dict]:
        """Live falling debris (ICD §2.2 v0.3): truth-derived display data,
        same numbers the debris/state channel carries (SIM-DEB-002)."""
        out = []
        for deb in self.world.debris.values():
            impact = deb.predicted_impact()
            zone = self.world.env.risk_map.zone_at(impact[0], impact[1])
            out.append({
                "id": deb.debris_id,
                "pos": [round(float(x), 1) for x in deb.position],
                "vel": [round(float(x), 1) for x in deb.velocity],
                "impact": [round(float(impact[0]), 1), round(float(impact[1]), 1), 0.0],
                "zone": zone.name,
                "t_impact": round(deb.time_to_impact(), 2),
            })
        return out

    def _station_entries(self) -> list[dict]:
        """Charging-pad occupancy: airframes physically on the pad."""
        out = []
        for st in self.world.env.stations:
            occupied = sum(
                1 for u in self.world.friendlies.values()
                if float(np.linalg.norm(u.position - st.position)) < 30.0
            )
            out.append({"id": st.station_id, "occupied": occupied})
        return out

    def _track_entry(self, trk) -> dict:
        # Predicted ground impact by linear projection of the track velocity
        # (display aid only — the C2's asset-aware prediction is separate).
        impact, tti = None, None
        vz = float(trk.velocity[2])
        if vz < -0.5:
            t_imp = -float(trk.position[2]) / vz
            if 0.0 < t_imp <= MAX_PROJECTED_TTI:
                tti = round(t_imp, 1)
                impact = [
                    round(float(trk.position[0] + trk.velocity[0] * t_imp), 1),
                    round(float(trk.position[1] + trk.velocity[1] * t_imp), 1),
                    0.0,
                ]
        return {
            "id": trk.track_id,
            "pos": [round(float(x), 1) for x in trk.position],
            "vel": [round(float(x), 1) for x in trk.velocity],
            "p_decoy": round(trk.p_decoy, 3),
            "belief": {c.value: round(p, 3) for c, p in trk.class_belief.items()},
            "score": None,    # filled once the C2 publishes assessments northbound
            "impact": impact,
            "tti": tti,
        }

    # -- static scene (ICD §2.1) ----------------------------------------------------

    def scene(self) -> dict:
        from ..sensors.acoustic import AcousticSensor
        from ..sensors.base import Sensor
        from ..sensors.eo_ir import EoIrSensor
        from ..sensors.radar import Radar
        from ..sensors.rf import RfSensor
        from ..sensors.seeker import OnboardSeeker

        type_names = {Radar: "radar", RfSensor: "rf", EoIrSensor: "eo_ir",
                      AcousticSensor: "acoustic"}
        env = self.world.env
        rm = env.risk_map
        sensors = []
        for node in self.world.nodes:
            if isinstance(node, OnboardSeeker) or not isinstance(node, Sensor):
                continue
            if hasattr(node, "_platform"):
                continue   # mounted sentinel payloads move with the airframe
            sensors.append({
                "name": node.name,
                "type": type_names.get(type(node), "radar"),
                "pos": [float(x) for x in node.position],
                "range": float(node.max_range),
            })
        return {
            "bounds": list(env.bounds),
            "cell_size": rm.cell_size,
            "grid": rm.grid.tolist(),
            "assets": [
                {"name": a.name, "pos": [float(x) for x in a.position], "value": a.value}
                for a in env.assets
            ],
            "buildings": [
                {
                    "rect": list(b.rect),
                    "height": b.height,
                    "kind": b.kind.value,
                    "material": b.material.value,
                    "name": b.name,
                }
                for b in env.buildings
            ],
            "sensors": sensors,
            "turrets": [
                {"id": tid, "pos": [float(x) for x in tur.position],
                 "range": float(tur.max_range)}
                for tid, tur in self.world.turrets.items()
            ],
            "homes": [
                {"uav_id": uid, "pos": [float(x) for x in uav.home]}
                for uid, uav in self.world.friendlies.items()
            ],
            "stations": [
                {"id": st.station_id, "pos": [float(x) for x in st.position],
                 "rooftop": st.rooftop}
                for st in env.stations
            ],
            "run": dict(self.run_meta),
        }

    def capture_terminal(self) -> None:
        """Append a final frame if anything happened since the last tick.

        The run can resolve between recorder ticks (the step loops break
        the instant the raid is over): without this, the replay file and
        the live /ops stream cut off before the last kill and the events
        logged since the previous tick. Idempotent — a snapshot adding no
        time and no events is not appended."""
        final = self.snapshot()
        if (not self.frames or final["t"] > self.frames[-1]["t"]
                or final["events"] or final["decisions"]):
            self.frames.append(final)
            if self.eval_tracker is not None:
                self.truths.append(self.eval_tracker.truth_payload())

    def save(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.capture_terminal()
        summary = self.world.summary()
        if self.eval_tracker is not None:
            summary["metrics"] = self.eval_tracker.metrics()
        payload = {"scene": self.scene(), "frames": self.frames,
                   "truth": self.truths or None, "summary": summary}
        path.write_text(json.dumps(payload))
        return path
