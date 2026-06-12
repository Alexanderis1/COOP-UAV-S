"""Sentinel surveillance UAV (PHY-SNT-001..003).

Unarmed patrol platform: it flies a circular orbit over the city and
carries the sensor payload (one EO/IR ball + one passive RF receiver,
mounted via :func:`coopuavs.sensors.base.mounted`) that feeds the common
fusion picture exactly like the fixed sensor network — its value is the
look angle: an airborne sensor sees down into the building canyons the
ground towers cannot (SIM-SEN-005 applies to its sight lines too).

Mode machine: IDLE → TRANSIT (to the orbit entry) → PATROL (orbit) →
RTB on low battery → REARM (recharge at the charging station) → TRANSIT
back. The orbit phase is a deterministic function of the platform id so
multi-sentinel laydowns spread out without coordination and runs stay
reproducible (SIM-003).
"""

from __future__ import annotations

import zlib

import numpy as np

from ..core.bus import MessageBus
from ..core.messages import Header, UavCommand, UavMode, UavState
from .airframe import LOW_BATTERY_RTB, UavAirframe


class SentinelUav(UavAirframe):
    def __init__(
        self,
        uav_id: str,
        bus: MessageBus,
        home: np.ndarray,
        orbit: dict,
        max_speed: float = 30.0,
        max_accel: float = 10.0,
        rate_hz: float = 10.0,
        battery_minutes: float = 40.0,
        cruise_speed: float | None = None,
        turnaround_s: float = 90.0,
    ):
        super().__init__(
            uav_id, bus, home,
            max_speed=max_speed, max_accel=max_accel, rate_hz=rate_hz,
            battery_minutes=battery_minutes, cruise_speed=cruise_speed,
            turnaround_s=turnaround_s,
        )
        self.orbit_center = np.asarray(orbit["center"], dtype=float)[:2]
        self.orbit_radius = float(orbit.get("radius", 800.0))
        self.orbit_alt = float(orbit.get("alt", 350.0))
        self.orbit_speed = float(orbit.get("speed", min(25.0, max_speed)))
        # Deterministic starting phase from the id: stable across runs and
        # platforms (Python's hash() is salted; crc32 is not).
        self._angle = float(zlib.crc32(uav_id.encode()) % 360) * np.pi / 180.0
        self._rtb_ordered = False

        self._state_pub = self.create_publisher("uav/state")
        self.create_subscription("uav/command", self._on_command)

    def _on_command(self, msg: UavCommand) -> None:
        if msg.uav_id == self.uav_id and msg.command == "rtb":
            self._rtb_ordered = True

    # -- main loop -------------------------------------------------------------------

    def update(self, t: float, dt: float) -> None:
        period = 1.0 / self.rate_hz

        if self.mode == UavMode.REARM:
            if t >= (self._rearm_until or 0.0):
                self._rearm_until = None
                self.battery = 1.0
                self.mode = UavMode.IDLE
            else:
                self.body.command_velocity(np.zeros(3))
                self.body.step(period)
                self._publish_state(t)
                return

        if self._rtb_ordered or self.battery < LOW_BATTERY_RTB:
            if self._at(self.home):
                self._rtb_ordered = False
                self.mode = UavMode.REARM
                self._rearm_until = t + self.turnaround_s
            else:
                self._fly_to(self.home)
                self.mode = UavMode.RTB
        else:
            self._patrol(period)

        self.body.step(period)
        self.battery = max(0.0, self.battery - self._drain_rate() * period)
        self._publish_state(t)

    def _patrol(self, period: float) -> None:
        """Fly the orbit: chase a waypoint advancing along the circle at
        the orbit's angular rate. Far from the circle this is a plain
        transit to the nearest orbit point."""
        on_station = abs(
            float(np.linalg.norm(self.body.position[:2] - self.orbit_center))
            - self.orbit_radius
        ) < 150.0 and abs(self.body.position[2] - self.orbit_alt) < 60.0
        if on_station:
            self.mode = UavMode.PATROL
            self._angle += (self.orbit_speed / self.orbit_radius) * period
        else:
            self.mode = UavMode.TRANSIT
            # Enter the circle at the bearing we already are on.
            rel = self.body.position[:2] - self.orbit_center
            if float(np.linalg.norm(rel)) > 1.0:
                self._angle = float(np.arctan2(rel[0], rel[1]))
        # Lead the waypoint a little so the orbit is flown, not chased.
        ang = self._angle + 0.15
        waypoint = np.array([
            self.orbit_center[0] + self.orbit_radius * np.sin(ang),
            self.orbit_center[1] + self.orbit_radius * np.cos(ang),
            self.orbit_alt,
        ])
        self._fly_to(waypoint)

    def _publish_state(self, t: float) -> None:
        self._state_pub.publish(
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
            )
        )


class SitlShellSentinel(SentinelUav):
    """Thin world-side shell for the sitl sentinel (P4-5): the patrol
    stack runs in ``mc/sentinel_app.py`` on a VirtualMCU inside the
    micro-loop; this node ferries bus traffic across the mailbox
    boundary and mirrors ``mode``/``battery`` from the app's telemetry.
    ``body`` is the app's link-backed estimate body (read-only here);
    the mounted sensor payload rides the FriendlyVehicle TRUTH adapter,
    not this shell."""

    def __init__(self, uav_id, bus, home, orbit, mcu, **kwargs):
        super().__init__(uav_id, bus, home, orbit, **kwargs)
        self._mcu = mcu
        self.body = mcu.app.body
        self._to_command = mcu.ports.box("command")
        self._to_link = mcu.ports.box("link_quality")
        self._from_state = mcu.ports.box("uav_state")

    @property
    def mc_crashed(self) -> bool:
        return self._mcu.crashed

    def _on_command(self, msg: UavCommand) -> None:
        if msg.uav_id == self.uav_id:
            self._to_command.post(msg)

    def update(self, t: float, dt: float) -> None:
        self._to_link.post(self.link_quality)
        for msg in self._from_state.drain():
            self.mode = msg.mode
            self.battery = msg.battery
            self._state_pub.publish(msg)
