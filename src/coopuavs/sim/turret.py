"""Ground anti-air gun turret (PHY-TUR, SIM-EFF-003/004).

A remote-controlled gun turret slaved to the fused track picture
(PHY-TUR-003): it never reads ground truth — target selection, lead and
fire-control all run on ``tracks`` messages, and every burst passes the
same clearance interlock as the UAV effectors (PHY-TUR-001). The sim-side
:class:`~coopuavs.sim.adjudicator.EngagementAdjudicator` resolves the truth
outcome of each burst, including stray-round ground impacts.

State machine
-------------
idle      no engageable track — barrel parked
slewing   rate-limited azimuth/elevation drive toward the lead point
tracking  settled on the lead point — clearance requested / awaited
firing    AUTHORIZED clearance in hand — bursts at the rate of fire
empty     magazine exhausted

Deconfliction: turrets publish their claimed track on ``turret/state``;
a turret yields a track already claimed by a lower-id peer, so two turrets
never waste ammunition on the same target.
"""

from __future__ import annotations

import numpy as np

from ..core.messages import (
    DebrisArray,
    EffectorType,
    EngagementDecision,
    FireClearance,
    FireRequest,
    Header,
    Track,
    TrackArray,
    TurretState,
    ZoneClass,
)
from ..core.node import Node
from ..c2.base_station import KILL_RECONFIRM_GRACE_S, TRACK_FRESH_S
from ..c2.roe import DENIAL_TTL_S
from .adjudicator import TURRET_EVASION_FACTOR, TURRET_LETHAL_RADIUS
from .world import World

FIRE_TOPIC = "engagement/fire"
STATE_TOPIC = "turret/state"

# Debris-intercept priority boost over threat tracks (PHY-GCS-006): a
# wreck already falling on populated ground is a certain impact, a track
# is a possible one. CRITICAL-bound debris outranks everything in arc.
DEBRIS_PRIORITY = {ZoneClass.DANGEROUS: 1.2, ZoneClass.CRITICAL: 1.6}


class GroundTurret(Node):
    def __init__(
        self,
        turret_id: str,
        world: World,
        position: np.ndarray,
        slew_az_dps: float = 90.0,        # PHY-TUR-002 reference rates
        slew_el_dps: float = 60.0,
        max_range: float = 1500.0,
        rate_of_fire: float = 10.0,       # rounds/s sustained
        rounds_per_burst: int = 5,
        magazine: int = 300,
        muzzle_velocity: float = 850.0,   # m/s
        dispersion_mrad: float = 3.0,     # 1-sigma per axis
        decoy_threshold: float = 0.7,     # don't spend rounds on likely decoys
        min_pk: float = 0.12,             # don't spray at hopeless geometry
        # (v0.3 hit-rate review: at 0.03 turrets opened fire at ranges where
        # a burst had a 2-5% chance and every miss landed 5 stray rounds on
        # the city; holding to ~0.12 concentrates fire inside ~600 m where
        # bursts actually connect and strays drop proportionally.)
        settle_deg: float = 2.0,
        clearance_window: float = 3.0,    # s an AUTHORIZED token stays valid
        rate_hz: float = 10.0,
    ):
        super().__init__(turret_id, world.bus, rate_hz=rate_hz)
        self.turret_id = turret_id
        # Static survey geometry only (firing-arc masking, SIM-EFF-004) —
        # the turret still never reads ground-truth enemy state.
        self._occlusion = world.occlusion
        self.position = np.asarray(position, dtype=float)
        self.slew_az_dps = slew_az_dps
        self.slew_el_dps = slew_el_dps
        self.max_range = max_range
        self.rate_of_fire = rate_of_fire
        self.rounds_per_burst = rounds_per_burst
        self.magazine = magazine
        self.muzzle_velocity = muzzle_velocity
        self.dispersion_mrad = dispersion_mrad
        self.decoy_threshold = decoy_threshold
        self.min_pk = min_pk
        self.settle_deg = settle_deg
        self.clearance_window = clearance_window

        self.az = 0.0                     # deg, compass bearing of the barrel
        self.el = 0.0                     # deg above horizon
        self.state = "idle"
        self.target_track: int | None = None

        self._tracks: dict[int, Track] = {}
        self._debris_tracks: dict[int, Track] = {}   # pseudo-tracks, ref-keyed
        # track_ref -> (debris_id, impact zone) for fire-message correlation
        # and the zone-driven priority boost.
        self._debris_by_ref: dict[int, tuple[str, ZoneClass]] = {}
        self._peer_claims: dict[str, int | None] = {}
        self._denied: dict[int, float] = {}   # track_id -> denial time (TTL)
        self._killed: dict[int, float] = {}   # track_id -> kill report time
        self._await_until = 0.0           # re-request clearance after this
        self._hold_until = 0.0
        self._cleared_until = 0.0
        self._cleared_track: int | None = None   # the track the token costed
        self._next_burst_ok = 0.0

        self._state_pub = self.create_publisher(STATE_TOPIC)
        self._request_pub = self.create_publisher("engagement/fire_request")
        self._fire_pub = self.create_publisher(FIRE_TOPIC)
        self.create_subscription("tracks", self._on_tracks)
        self.create_subscription("debris/state", self._on_debris)
        self.create_subscription(STATE_TOPIC, self._on_peer_state)
        self.create_subscription("engagement/clearance", self._on_clearance)
        self.create_subscription("engagement/result", self._on_result)

    # -- subscriptions -----------------------------------------------------------

    def _on_tracks(self, msg: TrackArray) -> None:
        self._tracks = {trk.track_id: trk for trk in msg.tracks}

    def _on_debris(self, msg: DebrisArray) -> None:
        """Interceptable debris enters target selection as pseudo-tracks
        (PHY-GCS-006): only wreckage falling toward populated ground —
        SAFE-bound debris is left alone."""
        self._debris_tracks = {}
        self._debris_by_ref = {}
        for d in msg.debris:
            if d.impact_zone == ZoneClass.SAFE:
                continue
            self._debris_tracks[d.track_ref] = Track(
                header=d.header,
                track_id=d.track_ref,
                position=np.asarray(d.position, dtype=float),
                velocity=np.asarray(d.velocity, dtype=float),
                p_decoy=0.0,
            )
            self._debris_by_ref[d.track_ref] = (d.debris_id, d.impact_zone)

    def _on_peer_state(self, msg: TurretState) -> None:
        if msg.turret_id != self.turret_id:
            self._peer_claims[msg.turret_id] = msg.target_track

    def _on_result(self, msg) -> None:
        if msg.hit:
            self._killed[msg.track_id] = msg.header.stamp
            if msg.track_id == self.target_track:
                self.target_track = None
                self._cleared_until = 0.0

    def _on_clearance(self, msg: FireClearance) -> None:
        """Tokens are correlated by ``track_id``: a verdict applies to the
        track the ROE actually costed, not to whatever the turret happens
        to be laying on when the (possibly delayed) answer arrives."""
        if msg.uav_id != self.turret_id:
            return
        if msg.decision == EngagementDecision.DENIED:
            if msg.track_id >= 0:
                self._denied[msg.track_id] = msg.header.stamp
                if msg.track_id == self.target_track:
                    self.target_track = None
                    self._await_until = 0.0
            return
        if msg.track_id != self.target_track:
            return                       # stale token for an abandoned lay
        if msg.decision == EngagementDecision.AUTHORIZED:
            self._cleared_until = msg.header.stamp + self.clearance_window
            self._cleared_track = msg.track_id
        else:  # HOLD
            self._hold_until = msg.header.stamp + 1.5
        self._await_until = 0.0

    # -- main loop -------------------------------------------------------------------

    def update(self, t: float, dt: float) -> None:
        period = 1.0 / self.rate_hz
        if self.magazine <= 0:
            self.state = "empty"
            self.target_track = None
            self._publish_state(t)
            return

        track = self._select_target(t)
        if track is None:
            self.state = "idle"
            self.target_track = None
            self._publish_state(t)
            return
        self.target_track = track.track_id

        # Lead point: track extrapolated to now, then by the round's flight.
        tgt_pos = track.position + track.velocity * max(0.0, t - track.header.stamp)
        dist = float(np.linalg.norm(tgt_pos - self.position))
        tof = dist / self.muzzle_velocity
        aim = tgt_pos + track.velocity * tof

        err = self._slew_toward(aim, period)
        if err > self.settle_deg:
            self.state = "slewing"
            self._publish_state(t)
            return

        if t < self._cleared_until and self._cleared_track == track.track_id:
            self.state = "firing"
            if t >= self._next_burst_ok:
                self._fire_burst(track, aim, dist, t)
        else:
            self.state = "tracking"
            if t >= max(self._await_until, self._hold_until):
                self._request_clearance(track, aim, dist, t)
        self._publish_state(t)

    # -- target selection ----------------------------------------------------------

    def _select_target(self, t: float) -> Track | None:
        """Highest-priority engageable track: in range, credible (p_decoy
        below threshold), not claimed by a lower-id peer turret."""
        # Kill claims are reconciled against the track picture exactly as
        # the C2 does: hits are reported under the engaged track id, but the
        # round is adjudicated against whatever flew at the aim point — a
        # "killed" track still absorbing measurements is alive.
        for tid, t_kill in list(self._killed.items()):
            trk = self._tracks.get(tid)
            if trk is None:
                del self._killed[tid]
            elif (t - t_kill > KILL_RECONFIRM_GRACE_S
                    and trk.time_since_update < TRACK_FRESH_S):
                del self._killed[tid]
        claimed = {
            tid for pid, tid in self._peer_claims.items()
            if tid is not None and pid < self.turret_id
        }
        best, best_priority = None, 0.0
        candidates = list(self._tracks.values()) + list(self._debris_tracks.values())
        for trk in candidates:
            denied_t = self._denied.get(trk.track_id)
            if (denied_t is not None and t - denied_t < DENIAL_TTL_S) \
                    or trk.track_id in claimed or trk.track_id in self._killed:
                continue
            if trk.p_decoy >= self.decoy_threshold:
                continue
            if trk.time_since_update > TRACK_FRESH_S:
                # Coasted estimate: the airframe behind it is unconfirmed
                # (often already dead or crashed) — a burst at it is a
                # guaranteed fire_no_target plus five strays on the city.
                continue
            pos = trk.position + trk.velocity * max(0.0, t - trk.header.stamp)
            dist = float(np.linalg.norm(pos - self.position))
            if dist > self.max_range:
                continue
            if not self._occlusion.clear(self.position, pos):
                # Firing-arc masking (SIM-EFF-004): the static building
                # geometry is survey data a real fire-control computer has;
                # a lay through a rooftop only wastes the magazine.
                continue
            # Ammunition discipline: hold fire until the burst has a real
            # chance — dispersion x range x target speed says when.
            if self._expected_pk(dist, trk.speed) < self.min_pk:
                continue
            priority = (1.0 - trk.p_decoy) * (1.0 - dist / self.max_range)
            if trk.track_id < 0:
                # Falling wreck headed for populated ground: certain impact
                # beats a possible one (PHY-GCS-006, CRITICAL > DANGEROUS).
                _, zone = self._debris_by_ref[trk.track_id]
                priority *= DEBRIS_PRIORITY.get(zone, 1.2)
            if trk.track_id == self.target_track:
                priority *= 1.3            # hysteresis: keep a settled lay
            if priority > best_priority:
                best, best_priority = trk, priority
        return best

    # -- gun laying ------------------------------------------------------------------

    def _slew_toward(self, aim: np.ndarray, period: float) -> float:
        """Rate-limited drive toward the aim point; returns the residual
        angular error in degrees (SIM-EFF-004)."""
        rel = aim - self.position
        az_des = float(np.degrees(np.arctan2(rel[0], rel[1])))   # compass
        el_des = float(np.degrees(np.arctan2(rel[2], np.linalg.norm(rel[:2]) + 1e-9)))

        d_az = (az_des - self.az + 180.0) % 360.0 - 180.0
        d_el = el_des - self.el
        self.az += float(np.clip(d_az, -self.slew_az_dps * period, self.slew_az_dps * period))
        self.az = (self.az + 180.0) % 360.0 - 180.0
        self.el += float(np.clip(d_el, -self.slew_el_dps * period, self.slew_el_dps * period))

        d_az = (az_des - self.az + 180.0) % 360.0 - 180.0
        return float(np.hypot(d_az, el_des - self.el))

    # -- fire control ------------------------------------------------------------------

    def _expected_pk(self, dist: float, target_speed: float) -> float:
        """Predicted per-burst kill probability from dispersion, range, TOF
        and target speed — the same surface the adjudicator rolls against
        (literally: the lethal-radius and evasion constants are imported
        from the adjudicator so the two cannot drift apart)."""
        tof = dist / self.muzzle_velocity
        sigma2 = (self.dispersion_mrad * 1e-3 * dist) ** 2 \
            + (TURRET_EVASION_FACTOR * target_speed * tof) ** 2
        p_round = 1.0 - float(np.exp(-(TURRET_LETHAL_RADIUS**2) / (2.0 * sigma2 + 1e-9)))
        return 1.0 - (1.0 - p_round) ** self.rounds_per_burst

    def _target_kind(self, track_id: int) -> tuple[str, str]:
        """(target_kind, debris_id) for the fire-message correlation: a
        negative id is a debris pseudo-track (SIM-DEB-003)."""
        if track_id < 0:
            debris_id, _ = self._debris_by_ref.get(track_id, ("", None))
            return "debris", debris_id
        return "track", ""

    def _request_clearance(self, track: Track, aim: np.ndarray, dist: float, t: float) -> None:
        self._await_until = t + 2.0
        kind, debris_id = self._target_kind(track.track_id)
        self._request_pub.publish(
            FireRequest(
                header=Header(stamp=t),
                task_id=0,                       # turrets are GCS-slaved, no UAV task
                uav_id=self.turret_id,
                track_id=track.track_id,
                effector=EffectorType.PROJECTILE,
                predicted_intercept=aim.copy(),
                p_kill=self._expected_pk(dist, track.speed),
                target_kind=kind,
                debris_id=debris_id,
            )
        )

    def _fire_burst(self, track: Track, aim: np.ndarray, dist: float, t: float) -> None:
        # The last burst of a magazine may be partial: the adjudicator must
        # roll (and land strays for) the rounds actually fired, not the
        # nominal burst — hence the explicit count on the fire message.
        n = min(self.rounds_per_burst, self.magazine)
        self.magazine -= n
        self._next_burst_ok = t + n / self.rate_of_fire
        kind, debris_id = self._target_kind(track.track_id)
        self._fire_pub.publish(
            FireRequest(
                header=Header(stamp=t),
                task_id=0,
                uav_id=self.turret_id,
                track_id=track.track_id,
                effector=EffectorType.PROJECTILE,
                predicted_intercept=aim.copy(),
                p_kill=self._expected_pk(dist, track.speed),
                rounds=n,
                target_kind=kind,
                debris_id=debris_id,
            )
        )

    # -- telemetry ----------------------------------------------------------------------

    def _publish_state(self, t: float) -> None:
        self._state_pub.publish(
            TurretState(
                header=Header(stamp=t),
                turret_id=self.turret_id,
                position=self.position.copy(),
                az_deg=round(self.az, 2),
                el_deg=round(self.el, 2),
                ammo=self.magazine,
                state=self.state,
                target_track=self.target_track,
            )
        )
