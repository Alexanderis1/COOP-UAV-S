"""Friendly interceptor UAV agent.

One node per airframe. The agent owns its point-mass body, flies the role
the C2 assigned it (shooter / blocker / herder), and never releases a
munition without a :class:`FireClearance` — the safety chain is in the
messages, not in trust.

Mode machine
------------
IDLE      hold launch pad, wait for tasking
PURSUIT   shooter: lead-pursuit the assigned track into effector envelope
ENGAGE    shooter: in envelope — request clearance, fire when authorized
BLOCKING  support: hold a cutoff post on the target's predicted corridor
HERDING   support: flank post opposite the kill box
RTB       ammo/battery out: return to pad
REARM     on the pad: recharge + rearm turnaround, then back to IDLE

Energy model (SIM-PHX-002): battery drain is the hover/cruise baseline up
to ``cruise_speed`` and grows quadratically with airspeed above it (induced
+ parasitic drag of the dash regime). Arriving home in RTB starts a
``turnaround_s`` recharge/rearm cycle that restores full battery and the
effector magazine and returns the airframe to availability.
"""

from __future__ import annotations

import numpy as np

from ..core.bus import MessageBus
from ..core.messages import (
    EngagementDecision,
    EngagementTask,
    FireClearance,
    FireRequest,
    Header,
    Track,
    TrackArray,
    UavCommand,
    UavMode,
    UavState,
)
from ..core.node import Node
from ..sim.physics import PointMass
from . import cooperation, guidance
from .effectors import Effector

FIRE_TOPIC = "engagement/fire"
MIN_PK_TO_REQUEST = 0.25   # don't waste ammo on envelope-edge shots
MIN_PK_TO_RELEASE = 0.15   # abort if geometry collapsed while clearing
# A clearance token lost in transit must not deadlock the interlock
# (SIM-COM-003): if no answer arrives within this window, re-request.
CLEARANCE_TIMEOUT_S = 3.0
# An AUTHORIZED token authorises the geometry the ROE costed *now*, not a
# shot at an arbitrary later time: stale tokens are discarded unconsumed.
CLEARANCE_VALID_S = 3.0
# Battery fraction below which the airframe breaks off and recovers.
LOW_BATTERY_RTB = 0.15


class InterceptorUav(Node):
    def __init__(
        self,
        uav_id: str,
        bus: MessageBus,
        home: np.ndarray,
        effector: Effector,
        max_speed: float = 45.0,
        max_accel: float = 20.0,
        rate_hz: float = 10.0,
        battery_minutes: float = 25.0,
        cruise_speed: float | None = None,
        turnaround_s: float = 90.0,
    ):
        super().__init__(uav_id, bus, rate_hz=rate_hz)
        self.uav_id = uav_id
        # All C2/peer traffic rides this airframe's datalink (SIM-COM-001);
        # link_quality is the radio's own telemetry, refreshed by the comms
        # model each step (PHY-UAV-043).
        self.comms_endpoint = uav_id
        self.link_quality = 1.0
        self.home = np.asarray(home, dtype=float)
        self.effector = effector
        self.body = PointMass(self.home.copy(), max_speed=max_speed, max_accel=max_accel)
        self.max_speed = max_speed
        self.cruise_speed = cruise_speed if cruise_speed is not None else 0.6 * max_speed
        self.turnaround_s = turnaround_s
        self.mode = UavMode.IDLE
        self.battery = 1.0
        self._drain_per_s = 1.0 / (battery_minutes * 60.0)
        self._ammo_capacity = effector.ammo
        self._rearm_until: float | None = None

        self._task: EngagementTask | None = None
        self._role: str = "none"               # "shooter" | "support"
        self._tracks: dict[int, Track] = {}
        self._peers: dict[str, UavState] = {}
        self._clearance: FireClearance | None = None
        self._await_clearance = False
        self._await_until = 0.0
        self._next_fire_ok = 0.0
        self._hold_until = 0.0
        self._rtb_ordered = False              # operator RTB (HMI-AUT-005)

        self._state_pub = self.create_publisher("uav/state")
        self._request_pub = self.create_publisher("engagement/fire_request")
        self._fire_pub = self.create_publisher(FIRE_TOPIC)
        self.create_subscription("engagement/tasks", self._on_tasks)
        self.create_subscription("tracks", self._on_tracks)
        self.create_subscription("uav/state", self._on_peer_state)
        self.create_subscription("engagement/clearance", self._on_clearance)
        self.create_subscription("uav/command", self._on_command)

    # -- physical accessors (used by the sim adjudicator) ---------------------

    @property
    def position(self) -> np.ndarray:
        return self.body.position

    @property
    def velocity(self) -> np.ndarray:
        return self.body.velocity

    # -- subscriptions -----------------------------------------------------------

    def _on_tasks(self, tasks: list[EngagementTask]) -> None:
        previous_track = self._task.track_id if self._task else None
        self._task, self._role = None, "none"
        for task in tasks:
            if task.shooter_id == self.uav_id:
                self._task, self._role = task, "shooter"
                break
            if self.uav_id in task.support_ids:
                self._task, self._role = task, "support"
                break
        # Retasked to a different target (or untasked): any clearance state
        # belongs to the old engagement — the ROE never costed the new one.
        new_track = self._task.track_id if self._task else None
        if new_track != previous_track:
            self._clearance = None
            self._await_clearance = False

    def _on_tracks(self, msg: TrackArray) -> None:
        self._tracks = {trk.track_id: trk for trk in msg.tracks}

    def _on_peer_state(self, msg: UavState) -> None:
        if msg.uav_id != self.uav_id:
            self._peers[msg.uav_id] = msg

    def _on_clearance(self, msg: FireClearance) -> None:
        """Accept only tokens correlated to the *current* engagement: a
        clearance answered after a retask authorises a shot whose debris
        footprint was costed for a different track — drop it."""
        if msg.uav_id != self.uav_id:
            return
        if self._task is None or msg.track_id != self._task.track_id:
            return
        self._clearance = msg
        self._await_clearance = False

    def _on_command(self, msg: UavCommand) -> None:
        if msg.uav_id != self.uav_id:
            return
        if msg.command == "rtb":
            self._rtb_ordered = True

    # -- main loop -------------------------------------------------------------------

    def update(self, t: float, dt: float) -> None:
        period = 1.0 / self.rate_hz

        if self.mode == UavMode.REARM:
            if t >= (self._rearm_until or 0.0):
                # Turnaround complete: full battery, full magazine, available.
                self._rearm_until = None
                self.battery = 1.0
                self.effector.ammo = self._ammo_capacity
                self.mode = UavMode.IDLE
            else:
                # On the pad, charging — no drain, no tasking.
                self.body.command_velocity(np.zeros(3))
                self.body.step(period)
                self._publish_state(t)
                return

        track = self._tracks.get(self._task.track_id) if self._task else None

        if self._rtb_ordered:
            # Operator override (HMI-AUT-005): break off, recover, turn around.
            if self._at(self.home):
                self._rtb_ordered = False
                self.mode = UavMode.REARM
                self._rearm_until = t + self.turnaround_s
            else:
                self._fly_to(self.home)
                self.mode = UavMode.RTB
        elif self.battery < LOW_BATTERY_RTB or self.effector.ammo == 0:
            # Empty magazine sends the airframe home whatever its current
            # role: the C2 drops ammo-out platforms from tasking within a
            # cycle, so gating REARM on still *being* the shooter would
            # park it at the pad in IDLE, never turning around.
            if self._at(self.home):
                self.mode = UavMode.REARM
                self._rearm_until = t + self.turnaround_s
            else:
                self._fly_to(self.home)
                self.mode = UavMode.RTB
        elif track is None:
            self._fly_to(self.home)
            self.mode = UavMode.IDLE if self._at(self.home) else UavMode.TRANSIT
        elif self._role == "shooter":
            self._shooter_behaviour(track, t)
        else:
            self._support_behaviour(track)

        # Integrate own flight at the node period (sim-side physics stand-in).
        self.body.step(period)
        self.battery = max(0.0, self.battery - self._drain_rate() * period)
        self._publish_state(t)

    def _drain_rate(self) -> float:
        """Airspeed-dependent battery drain (SIM-PHX-002): baseline up to
        cruise, quadratic penalty in the dash regime."""
        speed = float(np.linalg.norm(self.body.velocity))
        factor = 1.0
        if speed > self.cruise_speed:
            factor += 2.0 * ((speed - self.cruise_speed) / max(self.cruise_speed, 1.0)) ** 2
        return self._drain_per_s * factor

    # -- behaviours -------------------------------------------------------------------

    def _shooter_behaviour(self, track: Track, t: float) -> None:
        self.mode = UavMode.PURSUIT
        # Fire control runs on the track extrapolated to now — a 0.2 s stale
        # track is an 11 m error against an OWA, comparable to the envelope.
        tgt_pos = track.position + track.velocity * max(0.0, t - track.header.stamp)
        v_cmd = guidance.pursuit_velocity(
            self.body.position, tgt_pos, track.velocity, self.max_speed
        )
        self.body.command_velocity(v_cmd)

        rel = tgt_pos - self.body.position
        pk = self.effector.p_kill(rel, self.body.velocity, track.velocity)
        if pk < MIN_PK_TO_REQUEST or t < max(self._next_fire_ok, self._hold_until):
            return

        self.mode = UavMode.ENGAGE
        if self._clearance is not None:
            clearance = self._clearance
            self._clearance = None
            # Belt-and-braces re-check of the correlation _on_clearance
            # already enforced, plus freshness: a token consumed long after
            # it was issued authorises geometry that no longer exists.
            if (clearance.track_id != track.track_id
                    or t - clearance.header.stamp > CLEARANCE_VALID_S):
                return
            if clearance.decision == EngagementDecision.AUTHORIZED:
                self._fire(track, pk, t)
            elif clearance.decision == EngagementDecision.HOLD:
                self._hold_until = t + 1.5   # geometry unsafe — re-ask shortly
            else:  # DENIED — C2 will re-task us; stop asking for this track
                self._task = None
            return
        if self._await_clearance and t >= self._await_until:
            # Request or token lost in transit (SIM-COM-003): the interlock
            # held fire the whole time — re-request release authority.
            self._await_clearance = False
        if not self._await_clearance:
            self._await_clearance = True
            self._await_until = t + CLEARANCE_TIMEOUT_S
            # ROE must cost the kill where it will actually happen: the
            # extrapolated target position, not the (stale) track fix.
            self._request_pub.publish(
                FireRequest(
                    header=Header(stamp=t),
                    task_id=self._task.task_id,
                    uav_id=self.uav_id,
                    track_id=track.track_id,
                    effector=self.effector.type,
                    predicted_intercept=tgt_pos + track.velocity * 0.5,
                    p_kill=pk,
                )
            )

    def _fire(self, track: Track, pk: float, t: float) -> None:
        # Clearance took a beat — re-check the envelope before releasing.
        tgt_pos = track.position + track.velocity * max(0.0, t - track.header.stamp)
        rel = tgt_pos - self.body.position
        pk = self.effector.p_kill(rel, self.body.velocity, track.velocity)
        if pk < MIN_PK_TO_RELEASE:
            return
        self.effector.ammo -= 1
        self._next_fire_ok = t + self.effector.reload_time
        self._fire_pub.publish(
            FireRequest(
                header=Header(stamp=t),
                task_id=self._task.task_id,
                uav_id=self.uav_id,
                track_id=track.track_id,
                effector=self.effector.type,
                predicted_intercept=track.position.copy(),
                p_kill=pk,
            )
        )

    def _support_behaviour(self, track: Track) -> None:
        """Cooperative wingman: cutoff post if the target outruns the
        shooter (relay interception), herding flank otherwise.

        The relay decision keys on the *shooter's* speed (can the assigned
        effector platform win the tail chase?), taken from its telemetry —
        gating on this support's own speed posted blockers for targets the
        shooter handles alone, and herded targets the shooter can never
        catch. Post reachability likewise uses each blocker's own speed.

        Post slots are claimed only among wingmen whose telemetry has been
        heard: substituting our own position for a silent peer (degraded
        link, SIM-COM-001) would both fabricate that peer's reachability
        and shift which post *we* claim — two blockers converge on one
        post and the corridor gaps."""
        support_ids = [
            uid for uid in self._task.support_ids
            if uid == self.uav_id or uid in self._peers
        ]
        my_idx = support_ids.index(self.uav_id) if self.uav_id in support_ids else 0
        positions = [self._peer_position(uid) for uid in support_ids]

        shooter = self._peers.get(self._task.shooter_id)
        shooter_speed = (shooter.max_speed
                         if shooter is not None and shooter.max_speed > 0.0
                         else self.max_speed)
        if track.speed > shooter_speed * 0.95:
            posts = cooperation.cutoff_points(
                track, len(support_ids), positions,
                [self._peer_speed(uid) for uid in support_ids],
            )
            post = posts[min(my_idx, len(posts) - 1)]
            self.mode = UavMode.BLOCKING
        else:
            post = cooperation.herding_post(track, self._task.desired_kill_box)
            self.mode = UavMode.HERDING
        self._fly_to(post)

    # -- helpers ------------------------------------------------------------------------

    def _peer_position(self, uav_id: str) -> np.ndarray:
        if uav_id == self.uav_id:
            return self.body.position
        peer = self._peers.get(uav_id)
        return peer.position if peer is not None else self.body.position

    def _peer_speed(self, uav_id: str) -> float:
        if uav_id == self.uav_id:
            return self.max_speed
        peer = self._peers.get(uav_id)
        if peer is not None and peer.max_speed > 0.0:
            return peer.max_speed
        return self.max_speed

    def _fly_to(self, waypoint: np.ndarray) -> None:
        self.body.command_velocity(
            guidance.goto_velocity(self.body.position, waypoint, self.max_speed)
        )

    def _at(self, point: np.ndarray, radius: float = 25.0) -> bool:
        return bool(np.linalg.norm(self.body.position - point) < radius)

    def _publish_state(self, t: float) -> None:
        self._state_pub.publish(
            UavState(
                header=Header(stamp=t),
                uav_id=self.uav_id,
                position=self.body.position.copy(),
                velocity=self.body.velocity.copy(),
                mode=self.mode,
                battery=self.battery,
                ammo=self.effector.ammo,
                task_id=self._task.task_id if self._task else None,
                link=self.link_quality,
                max_speed=self.max_speed,
            )
        )
