"""Evaluation tracker — sim-side ground-truth/track matching and metrics.

Owns the SIM-GT-002/003 bookkeeping: which truth enemies have been
*acquired* by the perception layer (nearest-neighbour gating between truth
positions and fused tracks), per-threat detection latency, attrition,
ammunition economics, collateral and authorisation counters — exactly the
``metrics`` object of ICD_RUNTIME §4, plus the ``truth`` payload the
``/eval`` channel streams to the ghost overlay.

Ground truth stays quarantined (SIM-GT-001): this node reads the world and
the ``tracks`` topic, and nothing tactical reads it back.
"""

from __future__ import annotations

import numpy as np

from ..core.messages import ThreatClass, TrackArray
from ..core.node import Node
from ..risk.zones import ZONE_WEIGHTS
from .world import World

# Event kinds counted as munition releases (one fire event = one shot;
# a turret burst is one trigger pull). Debris intercepts and LOS-blocked
# releases spend ammunition too.
SHOT_EVENT_KINDS = {"kill", "miss", "fire_no_target", "debris_neutralized",
                    "fire_blocked_los"}
AUTH_EVENT_KINDS = {"auth_request", "auth_approved", "auth_denied", "auth_expired"}


class EvalTracker(Node):
    def __init__(self, world: World, rate_hz: float = 5.0, gate_m: float = 250.0):
        super().__init__("eval_tracker", world.bus, rate_hz=rate_hz)
        self.world = world
        self.gate_m = gate_m
        self._tracks: TrackArray | None = None
        self.create_subscription("tracks", self._on_tracks)

    def _on_tracks(self, msg: TrackArray) -> None:
        self._tracks = msg

    # -- acquisition gating (SIM-GT-002) ----------------------------------------

    def update(self, t: float, dt: float) -> None:
        tracks = self._tracks.tracks if self._tracks else []
        if not tracks:
            return
        for enemy in self.world.enemies.values():
            if enemy.acquired or not enemy.alive:
                continue
            best, best_d = None, self.gate_m
            for trk in tracks:
                d = float(np.linalg.norm(trk.position - enemy.position))
                if d < best_d:
                    best, best_d = trk, d
            if best is not None:
                enemy.acquired = True
                enemy.acquired_t = t
                enemy.track_id = best.track_id
                self.world.log_event(
                    "acquired", enemy_id=enemy.id,
                    threat_class=enemy.threat_class.value,
                    track_id=best.track_id,
                    latency=round(t - enemy.spawn_t, 2),
                )

    # -- metrics (ICD §4) ------------------------------------------------------------

    def metrics(self) -> dict:
        enemies = list(self.world.enemies.values())
        events = self.world.events

        latencies = [
            {
                "id": e.id,
                "cls": e.threat_class.value,
                "latency": (
                    round(e.acquired_t - e.spawn_t, 2) if e.acquired_t is not None else None
                ),
            }
            for e in enemies
        ]
        observed = [d["latency"] for d in latencies if d["latency"] is not None]

        attrition: dict[str, dict[str, int]] = {}
        for e in enemies:
            row = attrition.setdefault(
                e.threat_class.value, {"spawned": 0, "killed": 0, "leaked": 0}
            )
            row["spawned"] += 1
            row["killed"] += int(e.killed)
            row["leaked"] += int(e.reached_target)

        decoy_ids = {e.id for e in enemies if e.threat_class == ThreatClass.DECOY}
        shots = sum(ev["kind"] in SHOT_EVENT_KINDS for ev in events)
        kills = sum(e.killed for e in enemies)
        decoy_shots = sum(
            ev["kind"] in ("kill", "miss") and ev.get("enemy_id") in decoy_ids
            for ev in events
        )

        debris_cost = sum(ZONE_WEIGHTS[w["zone"]] for w in self.world.wrecks) \
            + sum(ZONE_WEIGHTS[s["zone"]] for s in self.world.stray_impacts)

        # Authorisation flow events are emitted by the orchestrator layer; a
        # run without one simply reports zeros (tolerated absence).
        auth = {k: [ev for ev in events if ev["kind"] == k] for k in AUTH_EVENT_KINDS}
        auth_latencies = [
            ev["latency"] for ev in auth["auth_approved"] + auth["auth_denied"]
            if "latency" in ev
        ]

        return {
            "detection": {
                "acquired": sum(e.acquired for e in enemies),
                "total": len(enemies),
                "latencies": latencies,
                "mean_latency": round(float(np.mean(observed)), 2) if observed else None,
            },
            "attrition": attrition,
            "economics": {
                "shots": shots,
                "kills": kills,
                "ammo_per_kill": round(shots / kills, 2) if kills else None,
                "decoy_shots": decoy_shots,
            },
            "collateral": {
                "wrecks_by_zone": self.world._wrecks_by_zone(),
                "strays_by_zone": self.world._strays_by_zone(),
                "debris_cost": round(float(debris_cost), 3),
                # Debris interception credit (SIM-DEB-003): zone cost the
                # defence averted by destroying wreckage before impact.
                "debris_intercepts": len(self.world.debris_intercepted),
                "debris_saved_cost": round(float(sum(
                    ZONE_WEIGHTS[d["saved_zone"]]
                    for d in self.world.debris_intercepted
                )), 3),
            },
            "auth": {
                "requests": len(auth["auth_request"]),
                "approved": len(auth["auth_approved"]),
                "denied": len(auth["auth_denied"]),
                "expired": len(auth["auth_expired"]),
                "mean_latency": (
                    round(float(np.mean(auth_latencies)), 2) if auth_latencies else None
                ),
            },
        }

    # -- truth payload (ICD §4 "truth" data) -------------------------------------------

    def truth_payload(self) -> dict:
        return {
            "t": round(self.world.t, 2),
            "enemies": [
                {
                    "id": e.id,
                    "cls": e.threat_class.value,
                    "pos": [round(float(x), 1) for x in e.position],
                    "vel": [round(float(x), 1) for x in e.velocity],
                    "alive": e.alive,
                    "killed": e.killed,
                    "warhead": e.profile.warhead,
                    "target": e.target_name,
                    "acquired": e.acquired,
                    "acquired_t": round(e.acquired_t, 2) if e.acquired_t is not None else None,
                    "track_id": e.track_id,
                }
                for e in self.world.enemies.values()
            ],
            "metrics": self.metrics(),
        }
