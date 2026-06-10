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
        self._next_fire_ok = 0.0
        self._hold_until = 0.0

        self._state_pub = self.create_publisher("uav/state")
        self._request_pub = self.create_publisher("engagement/fire_request")
        self._fire_pub = self.create_publisher(FIRE_TOPIC)
        self.create_subscription("engagement/tasks", self._on_tasks)
        self.create_subscription("tracks", self._on_tracks)
        self.create_subscription("uav/state", self._on_peer_state)
        self.create_subscription("engagement/clearance", self._on_clearance)

    # -- physical accessors (used by the sim adjudicator) ---------------------

    @property
    def position(self) -> np.ndarray:
        return self.body.position

    @property
    def velocity(self) -> np.ndarray:
        return self.body.velocity

    # -- subscriptions -----------------------------------------------------------

    def _on_tasks(self, tasks: list[EngagementTask]) -> None:
        self._task, self._role = None, "none"
        for task in tasks:
            if task.shooter_id == self.uav_id:
                self._task, self._role = task, "shooter"
                return
            if self.uav_id in task.support_ids:
                self._task, self._role = task, "support"
                return

    def _on_tracks(self, msg: TrackArray) -> None:
        self._tracks = {trk.track_id: trk for trk in msg.tracks}

    def _on_peer_state(self, msg: UavState) -> None:
        if msg.uav_id != self.uav_id:
            self._peers[msg.uav_id] = msg

    def _on_clearance(self, msg: FireClearance) -> None:
        if msg.uav_id == self.uav_id:
            self._clearance = msg
            self._await_clearance = False

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

        if self.battery < 0.15 or (self.effector.ammo == 0 and self._role == "shooter"):
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
            decision = self._clearance.decision
            self._clearance = None
            if decision == EngagementDecision.AUTHORIZED:
                self._fire(track, pk, t)
            elif decision == EngagementDecision.HOLD:
                self._hold_until = t + 1.5   # geometry unsafe — re-ask shortly
            else:  # DENIED — C2 will re-task us; stop asking for this track
                self._task = None
            return
        if not self._await_clearance:
            self._await_clearance = True
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
        shooter (relay interception), herding flank otherwise."""
        support_ids = self._task.support_ids
        my_idx = support_ids.index(self.uav_id) if self.uav_id in support_ids else 0
        positions = [self._peer_position(uid) for uid in support_ids]

        if track.speed > self.max_speed * 0.95:
            posts = cooperation.cutoff_points(
                track, len(support_ids), positions, self.max_speed
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
            )
        )
