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
from ..interceptors.guidance import intercept_time
from ..interceptors.uav import LOW_BATTERY_RTB
from ..risk.debris import DebrisModel
from . import assignment, threat_evaluation
from .roe import DENIAL_TTL_S, RoeConfig, RulesOfEngagement
from .supervisor import SupervisorPolicy, TacticalSituation, TrackSituation

TASKS_TOPIC = "engagement/tasks"
ROE_TOPIC = "c2/roe_evaluation"

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


class BaseStation(Node):
    def __init__(
        self,
        bus: MessageBus,
        env: Environment,
        debris: DebrisModel,
        uav_speeds: dict[str, float],
        rate_hz: float = 1.0,
        roe_config: RoeConfig | None = None,
        supervisor: SupervisorPolicy | None = None,
        supervisor_period_s: float = 3.0,
        log_event=None,
    ):
        super().__init__("base_station", bus, rate_hz=rate_hz)
        self.env = env
        self.uav_speeds = uav_speeds
        self.roe = RulesOfEngagement(env.risk_map, debris, roe_config)
        # Slow-loop AI supervisor (advise-only): None -> the C2 runs the
        # deterministic fast loop exactly as before. The directive only
        # shapes allocation; the ROE/clearance interlock is untouched.
        self.supervisor = supervisor
        self.supervisor_period_s = supervisor_period_s
        self._directive = None
        self._next_supervise = 0.0
        self._log = log_event or (lambda *a, **k: None)
        self._roe_cap = (roe_config or RoeConfig()).max_expected_collateral

        self._tracks: dict[int, object] = {}
        self._assessments: dict[int, ThreatAssessment] = {}
        self._uavs: dict[str, UavState] = {}
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

    # -- subscriptions -------------------------------------------------------

    def _on_tracks(self, msg: TrackArray) -> None:
        self._tracks = {trk.track_id: trk for trk in msg.tracks}

    def _on_uav_state(self, msg: UavState) -> None:
        self._uavs[msg.uav_id] = msg

    def _on_result(self, msg: EngagementResult) -> None:
        if msg.hit:
            self._killed[msg.track_id] = msg.header.stamp

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

        # Slow loop: refresh the supervisor directive at its own cadence and
        # carry it between ticks. A stale id simply no longer matches a live
        # track, so a held directive can only ever shape, never mis-fire.
        if self.supervisor is not None and t >= self._next_supervise:
            self._next_supervise = t + self.supervisor_period_s
            situation = self._build_situation(t, live, available, denied)
            self._directive = self.supervisor.decide(situation)
            self._log("supervisor", t=round(t, 2),
                      rationale=getattr(self._directive, "rationale", ""))

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
            directive=self._directive,
        )
        self._shooters = {task.track_id: task.shooter_id for task in tasks}
        self._task_ids = {
            pairing: tid for pairing, tid in self._task_ids.items()
            if pairing[0] in live
        }
        self._tasks_pub.publish(tasks)

    # -- supervisor situation (fused estimates only, never ground truth) -------

    def _build_situation(self, t, live, available, denied) -> TacticalSituation:
        speeds = [self.uav_speeds.get(u.uav_id, 30.0) for u in available]
        rows: list[TrackSituation] = []
        for tid, trk in live.items():
            a = self._assessments.get(tid)
            if a is None:
                continue
            # Savability: can any ready shooter reach an intercept solution
            # comfortably before the predicted impact?
            best = None
            for u, sp in zip(available, speeds):
                ti = intercept_time(trk.position - u.position, trk.velocity, sp)
                if ti is not None and (best is None or ti < best):
                    best = ti
            savable = bool(best is not None and best + 5.0 < a.time_to_impact)
            cls = (max(trk.class_belief, key=trk.class_belief.get).value
                   if getattr(trk, "class_belief", None) else "unknown")
            rows.append(TrackSituation(
                track_id=int(tid), threat_class=cls, p_decoy=float(trk.p_decoy),
                speed=float(trk.speed), threat_score=float(a.threat_score),
                time_to_impact=float(a.time_to_impact), savable=savable,
                best_intercept_s=(float(best) if best is not None else None),
                impact_zone=a.impact_zone.name,
            ))
        return TacticalSituation(
            t=t, tracks=rows, n_available_shooters=len(available),
            inventory_rounds=sum(u.ammo for u in available),
            leakers_so_far=0, decoy_shots_so_far=0,
            roe_collateral_cap=self._roe_cap,
        )
