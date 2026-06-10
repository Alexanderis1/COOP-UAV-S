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

from ..core.bus import MessageBus
from ..core.messages import (
    EngagementDecision,
    EngagementResult,
    FireRequest,
    Header,
    RoeEvaluation,
    ThreatAssessment,
    TrackArray,
    UavMode,
    UavState,
)
from ..core.node import Node
from ..sim.environment import Environment
from ..interceptors.uav import LOW_BATTERY_RTB
from ..risk.debris import DebrisModel
from . import assignment, threat_evaluation
from .roe import DENIAL_TTL_S, RoeConfig, RulesOfEngagement

TASKS_TOPIC = "engagement/tasks"
ROE_TOPIC = "c2/roe_evaluation"


class BaseStation(Node):
    def __init__(
        self,
        bus: MessageBus,
        env: Environment,
        debris: DebrisModel,
        uav_speeds: dict[str, float],
        rate_hz: float = 1.0,
        roe_config: RoeConfig | None = None,
    ):
        super().__init__("base_station", bus, rate_hz=rate_hz)
        self.env = env
        self.uav_speeds = uav_speeds
        self.roe = RulesOfEngagement(env.risk_map, debris, roe_config)

        self._tracks: dict[int, object] = {}
        self._assessments: dict[int, ThreatAssessment] = {}
        self._uavs: dict[str, UavState] = {}
        self._denied: dict[int, float] = {}   # track_id -> denial time (TTL)
        self._killed: set[int] = set()
        self._shooters: dict[int, str] = {}   # track_id -> incumbent shooter
        self._task_ids: dict[tuple[int, str], int] = {}   # (track, shooter) -> id
        self._t = 0.0

        self._tasks_pub = self.create_publisher(TASKS_TOPIC)
        self._roe_pub = self.create_publisher(ROE_TOPIC)
        self.create_subscription("tracks", self._on_tracks)
        self.create_subscription("uav/state", self._on_uav_state)
        self.create_subscription("engagement/fire_request", self._on_fire_request)
        self.create_subscription("engagement/result", self._on_result)

    # -- subscriptions -------------------------------------------------------

    def _on_tracks(self, msg: TrackArray) -> None:
        self._tracks = {trk.track_id: trk for trk in msg.tracks}

    def _on_uav_state(self, msg: UavState) -> None:
        self._uavs[msg.uav_id] = msg

    def _on_result(self, msg: EngagementResult) -> None:
        if msg.hit:
            self._killed.add(msg.track_id)

    def _on_fire_request(self, msg: FireRequest) -> None:
        """Fire requests are answered immediately, not at the planning rate —
        an in-envelope window against a 55 m/s target lasts a second."""
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
        live = {
            tid: trk for tid, trk in self._tracks.items() if tid not in self._killed
        }
        self._assessments = {
            tid: threat_evaluation.assess(trk, self.env, t) for tid, trk in live.items()
        }
        # A platform is a usable shooter only if it can actually fly the
        # engagement: rounds in the magazine, battery above the RTB floor,
        # and not already committed to the recovery/turnaround cycle —
        # otherwise the allocator burns its best shooter slot on an
        # airframe that is sitting on the pad ignoring its task.
        available = [
            u for u in self._uavs.values()
            if u.ammo > 0
            and u.battery >= LOW_BATTERY_RTB
            and u.mode not in (UavMode.RTB, UavMode.REARM)
        ]
        denied = {tid for tid, t0 in self._denied.items() if t - t0 < DENIAL_TTL_S}
        tasks = assignment.allocate(
            list(self._assessments.values()),
            live,
            available,
            self.uav_speeds,
            self.env.risk_map,
            t,
            denied_tracks=denied,
            incumbents=self._shooters,
            task_ids=self._task_ids,
        )
        self._shooters = {task.track_id: task.shooter_id for task in tasks}
        self._task_ids = {
            pairing: tid for pairing, tid in self._task_ids.items()
            if pairing[0] in live
        }
        self._tasks_pub.publish(tasks)
