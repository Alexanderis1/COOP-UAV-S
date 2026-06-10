"""Orchestration agent (ORC) — the autonomy-posture authority (SRS §5.7).

The single node between the C2's ROE verdicts and the weapon interlock:
the base station evaluates every :class:`FireRequest` and forwards
request + verdict on ``c2/roe_evaluation``; this agent decides, per the
current autonomy posture (HMI-AUT-003), what becomes of it:

``weapons_hold``
    every request is DENIED ("weapons hold"), resolved by the posture.
``pre_authorized``
    the ROE verdict passes through as the clearance — ROE-authorized shots
    are auto-cleared by the agent (within the human-pre-approved bounds,
    SYS-004).
``human_confirm``
    ROE-authorized shots become northbound ``auth_request`` items
    (ICD_RUNTIME §2.3) raised through the registered callback and the
    clearance is withheld until the operator answers or the authorisation
    window expires in *sim time* (it pauses with the world). ROE
    hold/denied verdicts pass through without bothering the human.
    With no console attached (headless/batch runs) authorized shots fall
    back to ROE pass-through — the ROE config is the pre-approved rule.

Every step is logged through ``log_event``: ``auth_request`` /
``auth_approved`` / ``auth_denied`` / ``auth_expired`` feed the EvalTracker
auth metrics, and ``decision`` entries (actor ``c2`` / ``orc`` /
``operator``, human-readable rationale per ORC-003) feed the frame
decision log. The agent reads only bus messages — never ground truth
(ORC-006): it receives a logging *callable*, not the world.

Operator commands (HMI-AUT-005) are forwarded on ``uav/command`` and
logged as operator decisions.
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass
from typing import Any, Callable

from ..core.bus import MessageBus
from ..core.messages import (
    EngagementDecision,
    FireClearance,
    FireRequest,
    Header,
    RoeEvaluation,
    Track,
    TrackArray,
    UavCommand,
)
from ..core.node import Node

ROE_TOPIC = "c2/roe_evaluation"
CLEARANCE_TOPIC = "engagement/clearance"
COMMAND_TOPIC = "uav/command"

POSTURES = ("human_confirm", "pre_authorized", "weapons_hold")
# Authorisation window (HMI-AUT-002): sim seconds before a pending request
# expires back to HOLD. Measured in sim time, so it pauses with the world.
DEFAULT_AUTH_WINDOW_S = 12.0

Northbound = Callable[[str, dict], None]
LogEvent = Callable[..., None]


@dataclass
class PendingAuth:
    """One northbound authorisation awaiting the human (ICD §2.3)."""

    auth_id: int
    raised_t: float
    expires_t: float
    request: FireRequest
    roe: FireClearance
    payload: dict                      # the auth_request data, for late joiners


class Orchestrator(Node):
    def __init__(
        self,
        bus: MessageBus,
        posture: str = "human_confirm",
        auth_window_s: float = DEFAULT_AUTH_WINDOW_S,
        log_event: LogEvent | None = None,
        rate_hz: float = 10.0,
    ):
        super().__init__("orchestrator", bus, rate_hz=rate_hz)
        if posture not in POSTURES:
            raise ValueError(f"unknown posture '{posture}'; valid: {', '.join(POSTURES)}")
        self.posture = posture
        self.auth_window_s = float(auth_window_s)
        self._log: LogEvent = log_event or (lambda kind, **data: None)
        self._northbound: Northbound | None = None

        self._pending: dict[int, PendingAuth] = {}
        self._auth_ids = itertools.count(1)
        self._tracks: dict[int, Track] = {}
        self._seen_tracks: set[int] = set()   # ever-seen ids, for loss detection
        self._t = 0.0

        self._clearance_pub = self.create_publisher(CLEARANCE_TOPIC)
        self._command_pub = self.create_publisher(COMMAND_TOPIC)
        self.create_subscription(ROE_TOPIC, self._on_roe)
        self.create_subscription("tracks", self._on_tracks)

    # -- northbound wiring (serve layer) ---------------------------------------

    def set_northbound(self, callback: Northbound | None) -> None:
        """Register the console callback receiving ``(type, data)`` for
        ``auth_request`` / ``auth_resolved`` messages (ICD §2.3)."""
        self._northbound = callback

    def pending_requests(self) -> list[dict]:
        """Unresolved auth_request payloads, for /ops late joiners."""
        return [p.payload for p in self._pending.values()]

    # -- subscriptions -----------------------------------------------------------

    def _on_tracks(self, msg: TrackArray) -> None:
        self._tracks = {trk.track_id: trk for trk in msg.tracks}
        self._seen_tracks.update(self._tracks)

    def _track_lost(self, track_id: int) -> bool:
        """True when a track we once saw has dropped out of the picture —
        never true for ids we never saw (a fusion gap is not a loss)."""
        return track_id in self._seen_tracks and track_id not in self._tracks

    def _on_roe(self, msg: RoeEvaluation) -> None:
        """One evaluated fire request: clear, deny, or escalate (ORC-002)."""
        req, roe = msg.request, msg.clearance
        t = max(self._t, msg.header.stamp)
        rationale = self._rationale(req, roe)

        if self.posture == "weapons_hold":
            self._publish_clearance(req, EngagementDecision.DENIED, "weapons hold",
                                    roe.expected_collateral, t)
            self._decision("orc", f"denied (weapons hold) — {rationale}",
                           req, by="posture")
            return

        if self.posture == "pre_authorized":
            self._clearance_pub.publish(roe)
            actor = "orc" if roe.decision == EngagementDecision.AUTHORIZED else "c2"
            verb = ("auto-cleared" if roe.decision == EngagementDecision.AUTHORIZED
                    else roe.decision.value)
            self._decision(actor, f"{verb} (pre-authorized posture) — {rationale}",
                           req, by="orc")
            return

        # human_confirm -------------------------------------------------------
        if roe.decision != EngagementDecision.AUTHORIZED:
            # ROE itself says hold/deny: pass through, don't bother the human.
            self._clearance_pub.publish(roe)
            self._decision("c2", f"{roe.decision.value} by ROE — {rationale}", req)
            return
        if self._northbound is None:
            # Unattended run: the ROE config is the pre-approved rule (SYS-004).
            self._clearance_pub.publish(roe)
            self._decision("orc", f"auto-cleared (unattended) — {rationale}", req)
            return
        if any(p.request.uav_id == req.uav_id and p.request.track_id == req.track_id
               for p in self._pending.values()):
            return                       # already escalated; await the human
        self._raise_auth(req, roe, t, rationale)

    # -- escalation / resolution (ICD §2.3) ------------------------------------------

    def _raise_auth(self, req: FireRequest, roe: FireClearance, t: float,
                    rationale: str) -> None:
        auth_id = next(self._auth_ids)
        expires_t = t + self.auth_window_s
        payload = {
            "id": auth_id,
            "t": round(t, 2),
            "shooter": req.uav_id,
            "track_id": req.track_id,
            "effector": req.effector.value,
            "p_kill": round(float(req.p_kill), 3),
            "roe": {
                "decision": roe.decision.value,
                "reason": roe.reason,
                "expected_collateral": round(float(roe.expected_collateral), 3),
            },
            "rationale": rationale,
            "expires_t": round(expires_t, 2),
        }
        self._pending[auth_id] = PendingAuth(
            auth_id=auth_id, raised_t=t, expires_t=expires_t,
            request=req, roe=roe, payload=payload,
        )
        self._log("auth_request", id=auth_id, uav_id=req.uav_id,
                  track_id=req.track_id, p_kill=round(float(req.p_kill), 3),
                  expires_t=round(expires_t, 2))
        self._decision("orc", f"authorisation #{auth_id} requested — {rationale}", req)
        self._emit("auth_request", payload)

    def resolve(self, auth_id: int, approve: bool, by: str = "operator",
                reason: str | None = None) -> bool:
        """Answer one pending request (operator action or posture change).
        Returns False if the id is unknown or already resolved."""
        pending = self._pending.pop(int(auth_id), None)
        if pending is None:
            return False
        req, roe = pending.request, pending.roe
        t = self._t
        latency = round(max(0.0, t - pending.raised_t), 2)
        if approve and self._track_lost(req.track_id):
            # The world moved on while the human deliberated: the costed
            # track is gone (killed or dropped). An approval must not
            # release a weapon on whatever replaced it.
            self._publish_clearance(req, EngagementDecision.HOLD,
                                    "track lost while pending",
                                    roe.expected_collateral, t)
            self._log("auth_expired", id=pending.auth_id, uav_id=req.uav_id,
                      track_id=req.track_id)
            self._decision("orc", f"authorisation #{pending.auth_id} approved by "
                           f"{by} but track {req.track_id} is gone — "
                           f"{req.uav_id} holds fire", req, by=by)
            self._emit("auth_resolved", {"id": pending.auth_id,
                                         "approved": False, "by": "track_lost"})
            return True
        if approve:
            self._publish_clearance(req, EngagementDecision.AUTHORIZED,
                                    roe.reason, roe.expected_collateral, t)
            self._log("auth_approved", id=pending.auth_id, uav_id=req.uav_id,
                      track_id=req.track_id, latency=latency, by=by)
            self._decision(by if by == "operator" else "orc",
                           f"authorisation #{pending.auth_id} approved by {by} "
                           f"after {latency:.1f} s — release cleared for "
                           f"{req.uav_id} on track {req.track_id}", req, by=by)
        else:
            self._publish_clearance(req, EngagementDecision.DENIED,
                                    reason or "operator denied",
                                    roe.expected_collateral, t)
            self._log("auth_denied", id=pending.auth_id, uav_id=req.uav_id,
                      track_id=req.track_id, latency=latency, by=by)
            self._decision(by if by == "operator" else "orc",
                           f"authorisation #{pending.auth_id} denied by {by} — "
                           f"{reason or 'operator denied'}", req, by=by)
        self._emit("auth_resolved", {"id": pending.auth_id,
                                     "approved": bool(approve), "by": by})
        return True

    def update(self, t: float, dt: float) -> None:
        """Expire pending authorisations in sim time (they pause with the
        world): the engagement window is gone — the shooter gets HOLD."""
        self._t = t
        for auth_id in [i for i, p in self._pending.items() if t >= p.expires_t]:
            pending = self._pending.pop(auth_id)
            req = pending.request
            self._publish_clearance(req, EngagementDecision.HOLD,
                                    "authorization window expired",
                                    pending.roe.expected_collateral, t)
            self._log("auth_expired", id=auth_id, uav_id=req.uav_id,
                      track_id=req.track_id)
            self._decision("orc", f"authorisation #{auth_id} expired unanswered — "
                           f"{req.uav_id} holds fire on track {req.track_id}",
                           req, by="timeout")
            self._emit("auth_resolved", {"id": auth_id, "approved": False,
                                         "by": "timeout"})

    # -- operator controls (HMI-AUT-003/005) -----------------------------------------

    def set_posture(self, posture: str) -> None:
        if posture not in POSTURES:
            raise ValueError(f"unknown posture '{posture}'; valid: {', '.join(POSTURES)}")
        if posture == self.posture:
            return
        self.posture = posture
        self._log("decision", actor="operator",
                  text=f"autonomy posture set to {posture}",
                  track_id=None, uav_id=None)
        if posture == "weapons_hold":
            for auth_id in list(self._pending):
                self.resolve(auth_id, approve=False, by="posture", reason="weapons hold")
        elif posture == "pre_authorized":
            # Pending items were all ROE-authorized: the new posture clears them.
            for auth_id in list(self._pending):
                self.resolve(auth_id, approve=True, by="orc")

    def uav_command(self, uav_id: str, command: str) -> None:
        self._command_pub.publish(
            UavCommand(header=Header(stamp=self._t), uav_id=uav_id, command=command)
        )
        self._log("decision", actor="operator",
                  text=f"operator command: {command} -> {uav_id}",
                  track_id=None, uav_id=uav_id)

    # -- internals ------------------------------------------------------------------------

    def _publish_clearance(self, req: FireRequest, decision: EngagementDecision,
                           reason: str, collateral: float, t: float) -> None:
        self._clearance_pub.publish(FireClearance(
            header=Header(stamp=t), task_id=req.task_id, uav_id=req.uav_id,
            track_id=req.track_id, decision=decision,
            expected_collateral=collateral, reason=reason,
        ))

    def _decision(self, actor: str, text: str, req: FireRequest,
                  by: str | None = None) -> None:
        data: dict[str, Any] = dict(actor=actor, text=text,
                                    track_id=req.track_id, uav_id=req.uav_id)
        if by is not None:
            data["by"] = by
        self._log("decision", **data)

    def _emit(self, msg_type: str, data: dict) -> None:
        if self._northbound is not None:
            self._northbound(msg_type, data)

    def _rationale(self, req: FireRequest, roe: FireClearance) -> str:
        """Human-readable decision context (ORC-003)."""
        context = ""
        trk = self._tracks.get(req.track_id)
        if trk is not None:
            if trk.class_belief:
                cls = max(trk.class_belief, key=trk.class_belief.get).value
            else:
                cls = "unknown"
            context = (f"; threat {cls} at {trk.speed:.0f} m/s, "
                       f"p_decoy {trk.p_decoy:.2f}")
        return (f"{req.uav_id} requests {req.effector.value} release on track "
                f"{req.track_id}: Pk {req.p_kill:.2f}, ROE {roe.decision.value} "
                f"({roe.reason}), expected collateral "
                f"{roe.expected_collateral:.2f}{context}")
