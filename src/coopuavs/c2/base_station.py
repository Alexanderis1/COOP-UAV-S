"""Base station C2 node — the brain of the defence.

Closes the TEWA loop at ``rate_hz``:

1. ingest the fused track picture and friendly telemetry;
2. score every track (:mod:`threat_evaluation`);
3. allocate shooters + cooperative support (:mod:`assignment`) and publish
   the task set on ``engagement/tasks``;
4. evaluate every :class:`FireRequest` through the probabilistic ROE and
   forward the request + verdict to the orchestration agent on
   ``c2/roe_evaluation`` (:class:`RoeEvaluation`) — the **Orchestrator**
   owns the autonomy posture and publishes the actual
   ``engagement/clearance`` (SRS ORC-002, SYS-004). The C2 itself never
   clears a shot.

Tracks whose engagement was DENIED (decoy-grade, unsafe geometry) are
excluded from allocation for :data:`~coopuavs.c2.roe.DENIAL_TTL_S` and then
re-evaluated — the verdict was for one instant's geometry and belief, not
for the track's lifetime.
"""

from __future__ import annotations

import logging
from typing import Callable

import numpy as np

from ..core.bus import MessageBus
from ..core.messages import (
    DebrisArray,
    DebrisState,
    EngagementDecision,
    EngagementResult,
    EngagementTask,
    FireRequest,
    Header,
    RoeEvaluation,
    ThreatAssessment,
    Track,
    TrackArray,
    UavMode,
    UavState,
    ZoneClass,
)
from ..core.node import Node
from ..sim.environment import Environment
from ..interceptors.uav import LOW_BATTERY_RTB
from ..risk.debris import DebrisModel
from . import assignment, threat_evaluation
from .roe import DENIAL_TTL_S, RoeConfig, RulesOfEngagement

TASKS_TOPIC = "engagement/tasks"
ROE_TOPIC = "c2/roe_evaluation"
DEBRIS_TOPIC = "debris/state"

# Debris-intercept priority (PHY-GCS-006): a wreck falling on CRITICAL
# ground outranks most threat tracks, DANGEROUS sits mid-queue, SAFE-bound
# debris is never engaged. The zone weights are the objective function:
# minimise expected collateral damage and loss of life.
DEBRIS_SCORE_DANGEROUS = 0.55
DEBRIS_SCORE_CRITICAL = 0.90
# Below this time-to-impact there is no realistic intercept window left.
DEBRIS_MIN_WINDOW_S = 1.5

# Telemetry silent for this long is treated as lost: allocating a platform
# on its last-known state hands the shooter slot to a ghost (the comms
# layer can drop a UAV for minutes under jamming).
UAV_STATE_STALE_S = 5.0
# Kill reports are reconciled against the track picture: a "killed" track
# still absorbing measurements this long after the hit means the munition
# found a different airframe — the engaged threat is still flying. The
# grace covers detections that were already in flight when the kill landed.
KILL_RECONFIRM_GRACE_S = 2.0
TRACK_FRESH_S = 1.0

_log = logging.getLogger("coopuavs.c2")

# A weapon-target allocator: same call shape as ``assignment.allocate``,
# returning the task set for this planning cycle.
Allocator = Callable[..., "list[EngagementTask]"]


class BaseStation(Node):
    def __init__(
        self,
        bus: MessageBus,
        env: Environment,
        debris: DebrisModel,
        uav_speeds: dict[str, float],
        rate_hz: float = 1.0,
        roe_config: RoeConfig | None = None,
        uav_effectors: dict[str, str] | None = None,
        debris_policy: dict | None = None,
        allocator: "Allocator | None" = None,
        allocator_strict: bool = False,
    ):
        super().__init__("base_station", bus, rate_hz=rate_hz)
        self.env = env
        self.uav_speeds = uav_speeds
        # Weapon-target allocation seam (SRS, docs/MARL.md): the classical
        # priority-greedy ``assignment.allocate`` by default, or a swapped-in
        # policy with the same signature — the learned cooperation policy
        # (``c2/learned_allocator.py``) or an env-controlled allocator during
        # MARL training. Always called through ``_allocate`` so a misbehaving
        # policy can never freeze tasking: any exception falls back to the
        # classical allocator and is logged, the raid keeps being defended.
        self.allocator: "Allocator" = allocator or assignment.allocate
        # Strict mode (MARL training): let a policy exception propagate so
        # bugs surface, rather than masking them behind the classical
        # fallback (which is the right behaviour only in deployment).
        self.allocator_strict = bool(allocator_strict)
        self.allocator_fallbacks = 0
        # Effector type per platform (PHY-GCS-006/007): debris tasks go only
        # to projectile carriers, and assignment is envelope-aware.
        self.uav_effectors = uav_effectors or {}
        policy = dict(debris_policy or {})
        self.debris_engage_zones = {
            ZoneClass[z] for z in policy.get("engage_zones",
                                             ["CRITICAL", "DANGEROUS"])
        }
        self.roe = RulesOfEngagement(env.risk_map, debris, roe_config)

        self._tracks: dict[int, object] = {}
        self._assessments: dict[int, ThreatAssessment] = {}
        self._uavs: dict[str, UavState] = {}
        self._debris: dict[str, DebrisState] = {}
        self._denied: dict[int, float] = {}   # track_id -> denial time (TTL)
        self._killed: dict[int, float] = {}   # track_id -> kill report time
        self._shooters: dict[int, str] = {}   # track_id -> incumbent shooter
        self._task_ids: dict[tuple[int, str], int] = {}   # (track, shooter) -> id
        self._t = 0.0

        self._tasks_pub = self.create_publisher(TASKS_TOPIC)
        self._roe_pub = self.create_publisher(ROE_TOPIC)
        self.create_subscription("tracks", self._on_tracks)
        self.create_subscription("uav/state", self._on_uav_state)
        self.create_subscription("engagement/fire_request", self._on_fire_request)
        self.create_subscription("engagement/result", self._on_result)
        self.create_subscription(DEBRIS_TOPIC, self._on_debris)

    # -- subscriptions -------------------------------------------------------

    def _on_tracks(self, msg: TrackArray) -> None:
        self._tracks = {trk.track_id: trk for trk in msg.tracks}

    def _on_uav_state(self, msg: UavState) -> None:
        self._uavs[msg.uav_id] = msg

    def _on_debris(self, msg: DebrisArray) -> None:
        self._debris = {d.debris_id: d for d in msg.debris}

    def _on_result(self, msg: EngagementResult) -> None:
        if msg.hit:
            self._killed[msg.track_id] = msg.header.stamp

    def _on_fire_request(self, msg: FireRequest) -> None:
        """Fire requests are answered immediately, not at the planning rate —
        an in-envelope window against a 55 m/s target lasts a second."""
        if msg.target_kind == "debris":
            # Debris mitigation (SIM-DEB-003): the dedicated ROE branch
            # authorises and logs; no footprint costing needed.
            clearance = self.roe.evaluate_debris(msg, self._t)
            self._roe_pub.publish(
                RoeEvaluation(header=Header(stamp=self._t), request=msg,
                              clearance=clearance)
            )
            return
        track = self._tracks.get(msg.track_id)
        if track is None:
            return
        clearance = self.roe.evaluate(
            request=msg,
            target_velocity=track.velocity,
            effector=msg.effector,
            assessment=self._assessments.get(msg.track_id),
            t=self._t,
        )
        if clearance.decision == EngagementDecision.DENIED:
            self._denied[msg.track_id] = self._t
        self._roe_pub.publish(
            RoeEvaluation(header=Header(stamp=self._t), request=msg, clearance=clearance)
        )

    # -- planning loop ----------------------------------------------------------

    def update(self, t: float, dt: float) -> None:
        self._t = t
        # Reconcile kill claims with the evidence. A hit is reported under
        # the *engaged* track id, but the munition is adjudicated against
        # whatever actually flew at the intercept point — when threats are
        # bunched the wrong airframe can die. A track that keeps absorbing
        # measurements past the grace window is demonstrably alive: drop the
        # kill mark or the armed threat is never engaged again. Marks for
        # vanished tracks are pruned (track ids are never reused).
        for tid, t_kill in list(self._killed.items()):
            trk = self._tracks.get(tid)
            if trk is None:
                del self._killed[tid]
            elif (t - t_kill > KILL_RECONFIRM_GRACE_S
                    and trk.time_since_update < TRACK_FRESH_S):
                del self._killed[tid]
        live = {
            tid: trk for tid, trk in self._tracks.items() if tid not in self._killed
        }
        self._assessments = {
            tid: threat_evaluation.assess(trk, self.env, t) for tid, trk in live.items()
        }
        # Falling debris headed for populated ground enters the same queue
        # as threat tracks (PHY-GCS-006): the zone-derived score makes red
        # debris outrank most threats and yellow debris sit mid-queue.
        debris_tracks, debris_assess, debris_info = self._debris_picture(t)
        live.update(debris_tracks)
        self._assessments.update(debris_assess)
        # A platform is a usable shooter only if it can actually fly the
        # engagement: rounds in the magazine, battery above the RTB floor,
        # not already committed to the recovery/turnaround cycle, and its
        # telemetry recent enough to trust — otherwise the allocator burns
        # its best shooter slot on an airframe that is sitting on the pad
        # (or silent behind a jammer) ignoring its task.
        available = [
            u for u in self._uavs.values()
            if u.ammo > 0
            and u.battery >= LOW_BATTERY_RTB
            and u.mode not in (UavMode.RTB, UavMode.REARM)
            and t - u.header.stamp <= UAV_STATE_STALE_S
        ]
        denied = {tid for tid, t0 in self._denied.items() if t - t0 < DENIAL_TTL_S}
        alloc_args = (
            list(self._assessments.values()),
            live,
            available,
            self.uav_speeds,
            self.env.risk_map,
            t,
        )
        alloc_kwargs = dict(
            denied_tracks=denied,
            incumbents=self._shooters,
            task_ids=self._task_ids,
            debris_info=debris_info,
            uav_effectors=self.uav_effectors,
        )
        try:
            tasks = self.allocator(*alloc_args, **alloc_kwargs)
        except Exception as exc:    # never let a policy fault freeze tasking
            if self.allocator_strict:
                raise
            self.allocator_fallbacks += 1
            _log.warning("allocator failed (%s); falling back to classical "
                         "assignment for this cycle", exc)
            tasks = assignment.allocate(*alloc_args, **alloc_kwargs)
        self._shooters = {task.track_id: task.shooter_id for task in tasks}
        self._task_ids = {
            pairing: tid for pairing, tid in self._task_ids.items()
            if pairing[0] in live
        }
        self._tasks_pub.publish(tasks)

    def _debris_picture(self, t: float):
        """Pseudo-tracks and assessments for interceptable debris
        (SIM-DEB-003): only objects predicted to land in the configured
        engage zones, with enough fall time left to matter."""
        tracks: dict[int, Track] = {}
        assessments: dict[int, ThreatAssessment] = {}
        info: dict[int, str] = {}
        for deb in self._debris.values():
            if deb.impact_zone == ZoneClass.SAFE \
                    or deb.impact_zone not in self.debris_engage_zones:
                continue
            if deb.t_impact < DEBRIS_MIN_WINDOW_S:
                continue
            score = (DEBRIS_SCORE_CRITICAL
                     if deb.impact_zone == ZoneClass.CRITICAL
                     else DEBRIS_SCORE_DANGEROUS)
            tracks[deb.track_ref] = Track(
                # Preserve the reporter's stamp: the position is up to one
                # reporter period old, and downstream extrapolates from stamp.
                header=deb.header,
                track_id=deb.track_ref,
                position=np.asarray(deb.position, dtype=float),
                velocity=np.asarray(deb.velocity, dtype=float),
                p_decoy=0.0,
            )
            assessments[deb.track_ref] = ThreatAssessment(
                header=Header(stamp=t),
                track_id=deb.track_ref,
                threat_score=score,
                time_to_impact=deb.t_impact,
                predicted_impact=np.asarray(deb.predicted_impact, dtype=float),
                impact_zone=deb.impact_zone,
            )
            info[deb.track_ref] = deb.debris_id
        return tracks, assessments, info
