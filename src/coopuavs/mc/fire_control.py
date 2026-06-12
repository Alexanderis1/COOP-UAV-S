"""Shooter-side fire control: the clearance interlock (PHY-UAV-021).

Moved verbatim from ``interceptors/uav.py`` (PLAN_PROBLEM1 P4-3): same
topics, same constants, same check order. BOTH tactical hosts drive
this one state machine — the legacy point-mass ``InterceptorUav`` node
and the SITL ``mc/interceptor_app.py`` on its VirtualMCU — so the
interlock cannot drift between fidelity modes (the clearance-binding
sitl twins pin byte-equivalence of the whole engagement surface).

The safety chain is in the messages, not in trust: no release without
an AUTHORIZED :class:`FireClearance` correlated to the *current*
engagement (track id + freshness), denials abort the task, lost tokens
re-request after a timeout instead of deadlocking (SIM-COM-003).
"""

from __future__ import annotations

import numpy as np

from ..core.messages import EngagementDecision, FireClearance, FireRequest, Header, Track

MIN_PK_TO_RELEASE = 0.30   # abort if geometry collapsed while clearing
# Never below the release floor: a request the release gate would refuse
# consumes the clearance token, aborts, and re-requests every cycle —
# operator auth-spam under human_confirm with no shot ever fired.
MIN_PK_TO_REQUEST = MIN_PK_TO_RELEASE
# Inside this multiple of effector range, guidance switches from PIP lead
# pursuit to terminal pure pursuit so own velocity aligns with the sight
# line — the off-axis Pk gate measures exactly that angle.
TERMINAL_RANGE_FACTOR = 1.5
# No release on a track nobody has measured for this long: a coasted
# prediction is how rounds end up adjudicated as fire_no_target.
STALE_TRACK_FIRE_S = 2.0
# A clearance token lost in transit must not deadlock the interlock
# (SIM-COM-003): if no answer arrives within this window, re-request.
CLEARANCE_TIMEOUT_S = 3.0
# An AUTHORIZED token authorises the geometry the ROE costed *now*, not a
# shot at an arbitrary later time: stale tokens are discarded unconsumed.
CLEARANCE_VALID_S = 3.0

# engage() verdicts for the caller's mode machine.
PURSUING = "pursuing"        # below request floor / reload / hold / stale track
ENGAGING = "engaging"        # in envelope: token pending, consumed, or refused
ABORT_TASK = "abort_task"    # DENIED — C2 will re-task; stop asking


class FireControl:
    """Release-authority state machine for one shooter platform."""

    def __init__(self, uav_id: str, effector):
        self.uav_id = uav_id
        self.effector = effector
        self.clearance: FireClearance | None = None
        self.await_clearance = False
        self._await_until = 0.0
        self._next_fire_ok = 0.0
        self._hold_until = 0.0

    # -- correlation (the H1 interlock) ---------------------------------------

    def reset_engagement(self) -> None:
        """Retasked/untasked: any clearance state belongs to the old
        engagement — the ROE never costed the new one."""
        self.clearance = None
        self.await_clearance = False

    def accept_clearance(self, msg: FireClearance, task) -> None:
        """Accept only tokens correlated to the *current* engagement: a
        clearance answered after a retask authorises a shot whose debris
        footprint was costed for a different track — drop it."""
        if task is None or msg.track_id != task.track_id:
            return
        self.clearance = msg
        self.await_clearance = False

    # -- the release chain ---------------------------------------------------------

    def engage(self, t: float, task, track: Track, tgt_pos: np.ndarray,
               own_pos: np.ndarray, own_vel: np.ndarray,
               request_pub, fire_pub) -> str:
        """One engagement step against the extrapolated picture
        (``tgt_pos`` — the same point guidance is steering at).
        ``request_pub``/``fire_pub`` are publish-callables; the transport
        (bus topic or VirtualMCU outbox) is the host's business."""
        rel = tgt_pos - own_pos
        pk = self.effector.p_kill(rel, own_vel, track.velocity)
        if pk < MIN_PK_TO_REQUEST or t < max(self._next_fire_ok, self._hold_until):
            return PURSUING
        if track.time_since_update > STALE_TRACK_FIRE_S:
            # Coasted estimate: keep pursuing, but a munition released at a
            # prediction nobody has confirmed for seconds is a wasted round
            # (the onboard seeker refreshes the track through the endgame).
            return PURSUING

        if self.clearance is not None:
            clearance = self.clearance
            self.clearance = None
            # Belt-and-braces re-check of the correlation accept_clearance
            # already enforced, plus freshness: a token consumed long after
            # it was issued authorises geometry that no longer exists.
            if (clearance.track_id != track.track_id
                    or t - clearance.header.stamp > CLEARANCE_VALID_S):
                return ENGAGING
            if clearance.decision == EngagementDecision.AUTHORIZED:
                self._fire(task, track, pk, t, own_pos, own_vel, fire_pub)
            elif clearance.decision == EngagementDecision.HOLD:
                self._hold_until = t + 1.5   # geometry unsafe — re-ask shortly
                return ENGAGING
            else:  # DENIED — C2 will re-task us; stop asking for this track
                return ABORT_TASK
            return ENGAGING
        if not self.effector.quality_window(rel, own_vel):
            # In envelope but not in the high-quality core: another beat of
            # closure buys more Pk than this shot is worth — don't request
            # release from degraded geometry. (Tokens already in hand were
            # consumed or discarded above; this only delays new requests.)
            return ENGAGING
        if self.await_clearance and t >= self._await_until:
            # Request or token lost in transit (SIM-COM-003): the interlock
            # held fire the whole time — re-request release authority.
            self.await_clearance = False
        if not self.await_clearance:
            self.await_clearance = True
            self._await_until = t + CLEARANCE_TIMEOUT_S
            # ROE must cost the kill where it will actually happen: the
            # extrapolated target position, not the (stale) track fix.
            request_pub(FireRequest(
                header=Header(stamp=t),
                task_id=task.task_id,
                uav_id=self.uav_id,
                track_id=track.track_id,
                effector=self.effector.type,
                predicted_intercept=tgt_pos + track.velocity * 0.5,
                p_kill=pk,
                target_kind=task.target_kind,
                debris_id=task.debris_id,
            ))
        return ENGAGING

    def _fire(self, task, track: Track, pk: float, t: float,
              own_pos: np.ndarray, own_vel: np.ndarray, fire_pub) -> None:
        # Clearance took a beat — re-check the envelope before releasing.
        tgt_pos = track.position + track.velocity * max(0.0, t - track.header.stamp)
        rel = tgt_pos - own_pos
        pk = self.effector.p_kill(rel, own_vel, track.velocity)
        if pk < MIN_PK_TO_RELEASE:
            return
        self.effector.ammo -= 1
        self._next_fire_ok = t + self.effector.reload_time
        fire_pub(FireRequest(
            header=Header(stamp=t),
            task_id=task.task_id,
            uav_id=self.uav_id,
            track_id=track.track_id,
            effector=self.effector.type,
            # The munition flies at the geometry the Pk was just costed
            # on — the extrapolated fix, not the stale track position.
            predicted_intercept=tgt_pos.copy(),
            p_kill=pk,
            target_kind=task.target_kind,
            debris_id=task.debris_id,
        ))
