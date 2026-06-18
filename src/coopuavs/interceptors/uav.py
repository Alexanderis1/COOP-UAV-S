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
    DebrisArray,
    DebrisState,
    EngagementTask,
    FireClearance,
    Header,
    Track,
    TrackArray,
    UavCommand,
    UavMode,
    UavState,
)
from ..mc import cooperation, guidance
from ..mc.fire_control import (  # noqa: F401 — re-exported, tests import here
    ABORT_TASK,
    CLEARANCE_TIMEOUT_S,
    CLEARANCE_VALID_S,
    MIN_PK_TO_RELEASE,
    MIN_PK_TO_REQUEST,
    PURSUING,
    STALE_TRACK_FIRE_S,
    TERMINAL_RANGE_FACTOR,
    FireControl,
)
from .airframe import LOW_BATTERY_RTB, UavAirframe
from .effectors import Effector

FIRE_TOPIC = "engagement/fire"
# P5-5 release pairing (SitlShellUav): a staged FireRequest with no FCU
# pulse ack inside this window was refused (or lost on the wire) — the
# round never left the rail, the magazine gets it back. Generous against
# the real transport (~2x link latency + one MC tick << 1 s).
RELEASE_TIMEOUT_S = 1.0


class InterceptorUav(UavAirframe):
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
        super().__init__(
            uav_id, bus, home,
            max_speed=max_speed, max_accel=max_accel, rate_hz=rate_hz,
            battery_minutes=battery_minutes, cruise_speed=cruise_speed,
            turnaround_s=turnaround_s,
        )
        self.effector = effector
        self._ammo_capacity = effector.ammo

        self._task: EngagementTask | None = None
        self._role: str = "none"               # "shooter" | "support"
        self._tracks: dict[int, Track] = {}
        self._debris: dict[str, DebrisState] = {}
        self._peers: dict[str, UavState] = {}
        # Release authority lives in the shared interlock (mc/fire_control,
        # P4-3): one state machine for both fidelity modes, same effector
        # object so ammo bookkeeping cannot fork.
        self._fc = FireControl(uav_id, effector)
        self._rtb_ordered = False              # operator RTB (HMI-AUT-005)

        self._state_pub = self.create_publisher("uav/state")
        self._request_pub = self.create_publisher("engagement/fire_request")
        self._fire_pub = self.create_publisher(FIRE_TOPIC)
        self.create_subscription("engagement/tasks", self._on_tasks)
        self.create_subscription("tracks", self._on_tracks)
        self.create_subscription("debris/state", self._on_debris)
        self.create_subscription("uav/state", self._on_peer_state)
        self.create_subscription("engagement/clearance", self._on_clearance)
        self.create_subscription("uav/command", self._on_command)

    # Interlock state views (tests and tooling peek at these).
    @property
    def _clearance(self) -> FireClearance | None:
        return self._fc.clearance

    @property
    def _await_clearance(self) -> bool:
        return self._fc.await_clearance

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
            self._fc.reset_engagement()

    def _on_tracks(self, msg: TrackArray) -> None:
        self._tracks = {trk.track_id: trk for trk in msg.tracks}

    def _on_debris(self, msg: DebrisArray) -> None:
        self._debris = {d.debris_id: d for d in msg.debris}

    def _on_peer_state(self, msg: UavState) -> None:
        if msg.uav_id != self.uav_id:
            self._peers[msg.uav_id] = msg

    def _on_clearance(self, msg: FireClearance) -> None:
        """Tokens for this platform go to the interlock, which accepts
        only ones correlated to the *current* engagement (mc/fire_control)."""
        if msg.uav_id != self.uav_id:
            return
        self._fc.accept_clearance(msg, self._task)

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

        track = self._target_picture() if self._task else None

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

    # -- behaviours -------------------------------------------------------------------

    def _shooter_behaviour(self, track: Track, t: float) -> None:
        self.mode = UavMode.PURSUIT
        # Fire control runs on the track extrapolated to now — a 0.2 s stale
        # track is an 11 m error against an OWA, comparable to the envelope.
        tgt_pos = track.position + track.velocity * max(0.0, t - track.header.stamp)
        rel = tgt_pos - self.body.position
        if float(np.linalg.norm(rel)) <= TERMINAL_RANGE_FACTOR * self.effector.max_range:
            # Endgame: align own velocity with the sight line (fills the
            # off-axis envelope) instead of flying at the lead point.
            v_cmd = guidance.terminal_pursuit_velocity(
                self.body.position, tgt_pos, track.velocity, self.max_speed
            )
        else:
            v_cmd = guidance.pursuit_velocity(
                self.body.position, tgt_pos, track.velocity, self.max_speed
            )
        self.body.command_velocity(v_cmd)

        action = self._fc.engage(
            t, self._task, track, tgt_pos,
            self.body.position, self.body.velocity,
            self._request_pub.publish, self._fire_pub.publish)
        if action == PURSUING:
            return
        self.mode = UavMode.ENGAGE
        if action == ABORT_TASK:
            self._task = None

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

    def _target_picture(self) -> Track | None:
        """The pursuit picture for the current task: a fused track, or —
        for a debris-intercept task (SIM-DEB-003) — the latest debris state
        wrapped as a pseudo-track so guidance and fire control run
        unchanged. A vanished debris object (landed or neutralized) reads
        as no target and the platform goes home."""
        if self._task.target_kind == "debris":
            deb = self._debris.get(self._task.debris_id)
            if deb is None:
                return None
            return Track(
                header=deb.header,
                track_id=self._task.track_id,
                position=np.asarray(deb.position, dtype=float),
                velocity=np.asarray(deb.velocity, dtype=float),
                p_decoy=0.0,
            )
        return self._tracks.get(self._task.track_id)

    def seeker_cue(self) -> Track | None:
        """Cue source for an onboard gimballed seeker (P2-4
        ``GimbaledSeeker``): the engaged target's fused picture, or None
        when untasked. Estimate-only by construction — the same picture
        guidance and fire control fly on — so the seeker gimbal is slewed
        on track data, never ground truth (SIM-GT-001). P4 moves this call
        onto the modeled FCU<->MC link as the MC's gimbal-cue command; this
        method is that seam."""
        return self._target_picture() if self._task else None

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
                kind="interceptor",
                effector=self.effector.type.value,
            )
        )


class SitlShellUav(InterceptorUav):
    """Thin world-side shell for the P4-3 stage-2 MC split.

    In sitl fidelity the tactical stack runs in
    ``mc/interceptor_app.py`` on a VirtualMCU inside the micro-loop;
    this node only ferries bus traffic across the mailbox boundary —
    subscriptions APPEND to the MCU inboxes (nothing more), and
    ``update`` drains the outboxes onto the bus at node cadence — and
    mirrors ``mode``/``battery`` from the app's telemetry for the
    world-side duck-type. ``body`` and ``effector`` are the app's own
    objects, read-only by convention on this side.

    A crashed MCU (exception fence, SIM-SIL-003) goes silent here: no
    telemetry, no fire traffic — the FCU flies its own link-loss
    failsafe home. ``mc_crashed`` exposes the latch.

    P5-5 (release via FCU): the app's fire traffic is STAGED here and
    published to the bus only when the FCU hard interlock's release
    pulse comes back (``release_ack`` mailbox, fed by the engine); a
    stage that times out (``RELEASE_TIMEOUT_S``) was refused — ammo is
    restored and ``release_refused`` tallies it.
    """

    def __init__(self, uav_id, bus, home, effector, mcu, **kwargs):
        super().__init__(uav_id, bus, home, effector, **kwargs)
        self._mcu = mcu
        self.body = mcu.app.body            # estimate view (read-only)
        self._cue: Track | None = None
        box = mcu.ports.box
        self._to_tasks = box("tasks")
        self._to_tracks = box("tracks")
        self._to_debris = box("debris")
        self._to_peers = box("uav_state_in")
        self._to_clearance = box("clearance")
        self._to_command = box("command")
        self._to_link = box("link_quality")
        self._from_state = box("uav_state")
        self._from_request = box("fire_request")
        self._from_fire = box("fire")
        self._from_cue = box("cue")
        # P5-5 release pairing: fires out of the app are STAGED until
        # the FCU hard interlock's pulse ack (engine-fed mailbox).
        self._from_ack = box("release_ack")
        self._staged: list = []            # (FireRequest, staged_at)
        self.release_refused = 0

    @property
    def mc_crashed(self) -> bool:
        return self._mcu.crashed

    # -- bus -> inboxes (append only, drained at the MC tick) ---------------

    def _on_tasks(self, tasks) -> None:
        self._to_tasks.post(tasks)

    def _on_tracks(self, msg) -> None:
        self._to_tracks.post(msg)

    def _on_debris(self, msg) -> None:
        self._to_debris.post(msg)

    def _on_peer_state(self, msg) -> None:
        if msg.uav_id != self.uav_id:
            self._to_peers.post(msg)

    def _on_clearance(self, msg) -> None:
        if msg.uav_id == self.uav_id:
            self._to_clearance.post(msg)

    def _on_command(self, msg) -> None:
        if msg.uav_id == self.uav_id:
            self._to_command.post(msg)

    # -- outboxes -> bus, telemetry mirror -----------------------------------

    def update(self, t: float, dt: float) -> None:
        self._to_link.post(self.link_quality)
        for msg in self._from_state.drain():
            self.mode = msg.mode
            self.battery = msg.battery
            self._state_pub.publish(msg)
        for msg in self._from_request.drain():
            self._request_pub.publish(msg)
        # P5-5 hard interlock: a FireRequest out of the app is staged,
        # not published — the round leaves the rail only when the FCU
        # pulses (release_ack). Stage before draining acks: by node
        # cadence the ack for a fire staged this same update may
        # already be waiting. One ack = one release.
        for msg in self._from_fire.drain():
            self._staged.append((msg, t))
        for ack in self._from_ack.drain():
            for j, (msg, _) in enumerate(self._staged):
                if msg.track_id == ack[0]:
                    self._fire_pub.publish(self._staged.pop(j)[0])
                    break
        # NACK-by-timeout: no pulse inside the window — the FCU refused
        # (or the command died on the wire). The round never left the
        # rail; the magazine gets it back.
        kept = []
        for msg, t0 in self._staged:
            if t - t0 > RELEASE_TIMEOUT_S:
                # Clamp: a REARM restore while a stage was pending must
                # not overfill the magazine.
                self.effector.ammo = min(self.effector.ammo + 1,
                                         self._ammo_capacity)
                self.release_refused += 1
            else:
                kept.append((msg, t0))
        self._staged = kept
        for cue in self._from_cue.drain():
            self._cue = cue

    def seeker_cue(self) -> Track | None:
        return self._cue
