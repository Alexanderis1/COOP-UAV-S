"""Interceptor mission-computer app — the tactical stack on a VirtualMCU.

P4-3 stage 2 of the strangler (PHY-UAV-010/011): the FSM, guidance,
cooperation and fire control that lived in ``interceptors/uav.py`` run
here as hosted software, near line-for-line — flying on EKF estimates
(``SitlBody`` over the FCU coop-link) and talking to the world ONLY
through mailboxes:

    inboxes   tasks, tracks, debris, uav_state_in (peer telemetry),
              clearance, command, link_quality
    outboxes  uav_state, fire_request, fire, cue (seeker gimbal picture)

The world-side ``interceptors/uav.py SitlShellUav`` node ferries bus
traffic across this boundary at its own cadence; this app drains at its
tick (the §6 step-3 MC slot on the micro clock). The release authority
is the SAME ``mc/fire_control.FireControl`` object the legacy node
drives — the interlock cannot fork between fidelity modes; the
clearance-binding sitl twins pin byte-equivalence of the engagement
surface. Energy stays the legacy drain model until the P4-4 rewire.
"""

from __future__ import annotations

import numpy as np

from ..core.messages import Header, Track, UavMode, UavState
from . import cooperation, guidance
from .fire_control import ABORT_TASK, PURSUING, TERMINAL_RANGE_FACTOR, FireControl
from .fcu_client import SitlBody

# Battery fraction below which the airframe breaks off and recovers
# (the interceptors/airframe.py constant; duplicated by value here to
# keep the import direction mc -> world-side packages closed).
LOW_BATTERY_RTB = 0.15


class InterceptorApp:
    def __init__(self, clock, rng, ports, *, uav_id: str, home, effector,
                 fcu_client, max_speed: float = 45.0, max_accel: float = 20.0,
                 battery_minutes: float = 25.0, cruise_speed: float | None = None,
                 turnaround_s: float = 90.0):
        self.clock = clock
        self.rng = rng
        self.ports = ports
        self.uav_id = uav_id
        self.home = np.asarray(home, dtype=float)
        self.body = SitlBody(fcu_client, self.home, max_speed,
                             clock=lambda: self.clock.now, max_accel=max_accel)
        self.max_speed = max_speed
        self.cruise_speed = cruise_speed if cruise_speed is not None else 0.6 * max_speed
        self.turnaround_s = turnaround_s
        self.mode = UavMode.IDLE
        self.battery = 1.0
        self.link_quality = 1.0
        self._drain_per_s = 1.0 / (battery_minutes * 60.0)
        self._rearm_until: float | None = None

        self.effector = effector
        self._ammo_capacity = effector.ammo
        self._fc = FireControl(uav_id, effector)

        self._task = None
        self._role: str = "none"
        self._tracks: dict[int, Track] = {}
        self._debris: dict[str, object] = {}
        self._peers: dict[str, UavState] = {}
        self._rtb_ordered = False

        box = ports.box
        self._in_tasks = box("tasks")
        self._in_tracks = box("tracks")
        self._in_debris = box("debris")
        self._in_peers = box("uav_state_in")
        self._in_clearance = box("clearance")
        self._in_command = box("command")
        self._in_link = box("link_quality")
        self._out_state = box("uav_state")
        self._out_request = box("fire_request")
        self._out_fire = box("fire")
        self._out_cue = box("cue")

    # ------------------------------------------------------------- inboxes

    def _drain(self) -> None:
        for tasks in self._in_tasks.drain():
            self._on_tasks(tasks)
        for msg in self._in_tracks.drain():
            self._tracks = {trk.track_id: trk for trk in msg.tracks}
        for msg in self._in_debris.drain():
            self._debris = {d.debris_id: d for d in msg.debris}
        for msg in self._in_peers.drain():
            if msg.uav_id != self.uav_id:
                self._peers[msg.uav_id] = msg
        for msg in self._in_clearance.drain():
            if msg.uav_id == self.uav_id:
                self._fc.accept_clearance(msg, self._task)
        for msg in self._in_command.drain():
            if msg.uav_id == self.uav_id and msg.command == "rtb":
                self._rtb_ordered = True
        for q in self._in_link.drain():
            self.link_quality = q

    def _on_tasks(self, tasks) -> None:
        previous_track = self._task.track_id if self._task else None
        self._task, self._role = None, "none"
        for task in tasks:
            if task.shooter_id == self.uav_id:
                self._task, self._role = task, "shooter"
                break
            if self.uav_id in task.support_ids:
                self._task, self._role = task, "support"
                break
        new_track = self._task.track_id if self._task else None
        if new_track != previous_track:
            self._fc.reset_engagement()

    # ----------------------------------------------------------- main loop

    def tick(self, now: float) -> None:
        self._drain()
        self._update(now)
        # Seeker-cue picture for the world-side gimbal shell: the same
        # estimate-only track guidance flies on (SIM-GT-001).
        self._out_cue.post(self._target_picture() if self._task else None)

    def _update(self, t: float) -> None:
        period = 1.0 / self.clock.tick_hz

        if self.mode == UavMode.REARM:
            if t >= (self._rearm_until or 0.0):
                self._rearm_until = None
                self.battery = 1.0
                self.effector.ammo = self._ammo_capacity
                self.mode = UavMode.IDLE
            else:
                self.body.command_velocity(np.zeros(3))
                self.body.step(period)
                self._publish_state(t)
                return

        track = self._target_picture() if self._task else None

        if self._rtb_ordered:
            if self._at(self.home):
                self._rtb_ordered = False
                self.mode = UavMode.REARM
                self._rearm_until = t + self.turnaround_s
            else:
                self._fly_to(self.home)
                self.mode = UavMode.RTB
        elif self.battery < LOW_BATTERY_RTB or self.effector.ammo == 0:
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

        self.body.step(period)
        self.battery = max(0.0, self.battery - self._drain_rate() * period)
        self._publish_state(t)

    # ----------------------------------------------------------- behaviours

    def _shooter_behaviour(self, track: Track, t: float) -> None:
        self.mode = UavMode.PURSUIT
        tgt_pos = track.position + track.velocity * max(0.0, t - track.header.stamp)
        rel = tgt_pos - self.body.position
        if float(np.linalg.norm(rel)) <= TERMINAL_RANGE_FACTOR * self.effector.max_range:
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
            self._out_request.post, self._out_fire.post)
        if action == PURSUING:
            return
        self.mode = UavMode.ENGAGE
        if action == ABORT_TASK:
            self._task = None

    def _support_behaviour(self, track: Track) -> None:
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

    # -------------------------------------------------------------- helpers

    def _target_picture(self) -> Track | None:
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

    def _drain_rate(self) -> float:
        speed = float(np.linalg.norm(self.body.velocity))
        factor = 1.0
        if speed > self.cruise_speed:
            factor += 2.0 * ((speed - self.cruise_speed) / max(self.cruise_speed, 1.0)) ** 2
        return self._drain_per_s * factor

    def _fly_to(self, waypoint: np.ndarray) -> None:
        self.body.command_velocity(
            guidance.goto_velocity(self.body.position, waypoint, self.max_speed)
        )

    def _at(self, point: np.ndarray, radius: float = 25.0) -> bool:
        return bool(np.linalg.norm(self.body.position - point) < radius)

    def _publish_state(self, t: float) -> None:
        self._out_state.post(
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
