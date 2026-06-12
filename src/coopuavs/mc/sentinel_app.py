"""Sentinel mission-computer app — the patrol stack on a VirtualMCU.

P4-5: the ``interceptors/sentinel.py`` logic ported onto the hosted-MC
pattern (PHY-SNT-001..003 in sitl fidelity). Same shape as
``interceptor_app`` minus the engagement stack: orbit guidance flown on
NAV estimates over the coop-link, battery from FCU telemetry
(voltage-proxy, P4-4), the physical land-dock rearm cycle, mailbox-only
world I/O:

    inboxes   command (operator rtb), link_quality
    outboxes  uav_state

The sensor payload does NOT live here: mounted EO/RF ride the
world-side truth adapter (they are sim-side physics plugins); this app
only flies the platform they are bolted to.

Sitl approximation (documented): the sentinel flies the shared fleet
airframe (interceptor quad) — endurance-class airframe params arrive
with a per-class engine bank if ever needed; the ops envelope is the
MC's ``max_speed``/orbit speed, which is what PHY-SNT pins.
"""

from __future__ import annotations

import zlib

import numpy as np

from coopuavs.coopfc.cbit import CbitEngine
from coopuavs.coopfc.cbit.dictionary import act_rank, word_names

from ..core.messages import Header, UavMode, UavState
from . import guidance
from .fcu_client import SitlBody
from .interceptor_app import (
    BATT_LOW_DEBOUNCE_S, C2_LINK_FLOOR, LOITER_ALT, LOW_BATTERY_RTB,
    REARM_MIN_BATT,
)


class SentinelApp:
    def __init__(self, clock, rng, ports, *, uav_id: str, home, orbit: dict,
                 fcu_client, max_speed: float = 30.0, max_accel: float = 10.0,
                 battery_minutes: float = 40.0, cruise_speed: float | None = None,
                 turnaround_s: float = 90.0):
        self.clock = clock
        self.rng = rng
        self.ports = ports
        self.uav_id = uav_id
        self.home = np.asarray(home, dtype=float)
        self._client = fcu_client
        self.body = SitlBody(fcu_client, self.home, max_speed,
                             clock=lambda: self.clock.now, max_accel=max_accel)
        self.max_speed = max_speed
        self.cruise_speed = cruise_speed if cruise_speed is not None else 0.6 * max_speed
        self.turnaround_s = turnaround_s
        self.mode = UavMode.IDLE
        self.link_quality = 1.0
        self._rearm_until: float | None = None
        self._batt_low_since: float | None = None
        self._loiter = self.home + np.array([0.0, 0.0, LOITER_ALT])
        self._rtb_ordered = False

        self.orbit_center = np.asarray(orbit["center"], dtype=float)[:2]
        self.orbit_radius = float(orbit.get("radius", 800.0))
        self.orbit_alt = float(orbit.get("alt", 350.0))
        self.orbit_speed = float(orbit.get("speed", min(25.0, max_speed)))
        # Deterministic starting phase from the id (crc32 is unsalted).
        self._angle = float(zlib.crc32(uav_id.encode()) % 360) * np.pi / 180.0

        box = ports.box
        self._in_command = box("command")
        self._in_link = box("link_quality")
        self._out_state = box("uav_state")
        # MC-side CBIT (P5-4; interceptor_app rationale).
        self._cbit = CbitEngine()

    @property
    def battery(self) -> float:
        return self._client.batt_frac

    # ----------------------------------------------------------- main loop

    def tick(self, now: float) -> None:
        for msg in self._in_command.drain():
            if msg.uav_id == self.uav_id and msg.command == "rtb":
                self._rtb_ordered = True
        for q in self._in_link.drain():
            self.link_quality = q
        self._cbit.report("LINK_C2_LOSS",
                          self.link_quality < C2_LINK_FLOOR, now)
        self._update(now)

    def _update(self, t: float) -> None:
        period = 1.0 / self.clock.tick_hz

        if self.mode == UavMode.REARM:
            if t >= (self._rearm_until or 0.0) and self.battery >= REARM_MIN_BATT:
                # Same pack-swap declaration gate as the interceptor
                # (interceptor_app.REARM_MIN_BATT rationale).
                self._rearm_until = None
                self._client.request_batt_reset()
                self._client.hold_arm = False
                self._client.desired_mode = "OFFBOARD"
                self.mode = UavMode.IDLE
            else:
                self.body.command_velocity(np.zeros(3))
                self.body.step(period)
                self._publish_state(t)
                return

        if self.battery < LOW_BATTERY_RTB:
            if self._batt_low_since is None:
                self._batt_low_since = t
        else:
            self._batt_low_since = None
        batt_out = (
            (self._batt_low_since is not None
             and t - self._batt_low_since >= BATT_LOW_DEBOUNCE_S)
            or (self._client.state == "ARMED"
                and self._client.failsafe in ("BATT_LOW", "BATT_CRIT")))

        if self._rtb_ordered or batt_out:
            if self._at(self._loiter):
                self._rtb_ordered = False
                self.mode = UavMode.REARM
                self._rearm_until = t + self.turnaround_s
                self._client.hold_arm = True
                self._client.desired_mode = "LAND"
            else:
                self._fly_to(self._loiter)
                self.mode = UavMode.RTB
        else:
            self._patrol(period)

        self.body.step(period)
        self._publish_state(t)

    def _patrol(self, period: float) -> None:
        """Verbatim orbit guidance from interceptors/sentinel.py, flown
        on the NAV estimate."""
        on_station = abs(
            float(np.linalg.norm(self.body.position[:2] - self.orbit_center))
            - self.orbit_radius
        ) < 150.0 and abs(self.body.position[2] - self.orbit_alt) < 60.0
        if on_station:
            self.mode = UavMode.PATROL
            self._angle += (self.orbit_speed / self.orbit_radius) * period
        else:
            self.mode = UavMode.TRANSIT
            rel = self.body.position[:2] - self.orbit_center
            if float(np.linalg.norm(rel)) > 1.0:
                self._angle = float(np.arctan2(rel[0], rel[1]))
        ang = self._angle + 0.15
        waypoint = np.array([
            self.orbit_center[0] + self.orbit_radius * np.sin(ang),
            self.orbit_center[1] + self.orbit_radius * np.cos(ang),
            self.orbit_alt,
        ])
        self._fly_to(waypoint)

    # -------------------------------------------------------------- helpers

    def _fly_to(self, waypoint: np.ndarray) -> None:
        self.body.command_velocity(
            guidance.approach_velocity(self.body.position, waypoint,
                                       self.max_speed)
        )

    def _at(self, point: np.ndarray, radius: float = 25.0) -> bool:
        return bool(np.linalg.norm(self.body.position - point) < radius)

    def _health(self) -> dict:
        """Northbound UavHealth digest (P5-4; interceptor_app rationale)."""
        word = self._client.fault_word | self._cbit.word()
        deg_fcu = self._client.cbit_degraded
        deg_mc = self._cbit.degraded_mode()
        return {
            "faults": word,
            "codes": word_names(word),
            "inhibit_fire": bool(self._client.cbit_inhibit_fire
                                 or self._cbit.inhibit_fire),
            "inhibit_arming": bool(self._client.cbit_inhibit_arming
                                   or self._cbit.inhibit_arming),
            "degraded": (deg_fcu if act_rank(deg_fcu) >= act_rank(deg_mc)
                         else deg_mc),
        }

    def _publish_state(self, t: float) -> None:
        nav, status = self._client.nav, self._client.status
        self._out_state.post(
            UavState(
                header=Header(stamp=t),
                uav_id=self.uav_id,
                position=self.body.position.copy(),
                velocity=self.body.velocity.copy(),
                mode=self.mode,
                battery=self.battery,
                ammo=0,
                task_id=None,
                link=self.link_quality,
                max_speed=self.max_speed,
                kind="sentinel",
                effector="",
                attitude_q=((nav["qw"], nav["qx"], nav["qy"], nav["qz"])
                            if nav is not None else None),
                nav_quality=(status["sigma_pos_h"]
                             if status is not None else None),
                health=self._health(),
            )
        )
