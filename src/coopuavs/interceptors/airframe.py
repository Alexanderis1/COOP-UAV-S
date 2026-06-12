"""Shared friendly-UAV airframe base (interceptors and sentinels).

Owns the parts of a friendly platform that are independent of its role:
the point-mass body, the energy model (SIM-PHX-002: baseline drain up to
cruise speed, quadratic dash penalty), the home/charging-station
reference (PHY-CHG-001), the recharge turnaround timer, the datalink
fields the comms layer refreshes (PHY-UAV-043) and the basic fly-to /
arrived helpers. :class:`~coopuavs.interceptors.uav.InterceptorUav` adds
the armed mode machine and fire control on top;
:class:`~coopuavs.interceptors.sentinel.SentinelUav` adds the patrol
orbit (PHY-SNT-*).
"""

from __future__ import annotations

import numpy as np

from ..core.bus import MessageBus
from ..core.messages import UavMode
from ..core.node import Node
from ..sim.physics import PointMass
from . import guidance

# Battery fraction below which an airframe breaks off and recovers.
LOW_BATTERY_RTB = 0.15


class UavAirframe(Node):
    def __init__(
        self,
        uav_id: str,
        bus: MessageBus,
        home: np.ndarray,
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
        self.body = PointMass(self.home.copy(), max_speed=max_speed, max_accel=max_accel)
        self.max_speed = max_speed
        self.cruise_speed = cruise_speed if cruise_speed is not None else 0.6 * max_speed
        self.turnaround_s = turnaround_s
        self.mode = UavMode.IDLE
        self.battery = 1.0
        self._drain_per_s = 1.0 / (battery_minutes * 60.0)
        self._rearm_until: float | None = None

    # -- physical accessors (used by the sim side) -----------------------------

    @property
    def position(self) -> np.ndarray:
        return self.body.position

    @property
    def velocity(self) -> np.ndarray:
        return self.body.velocity

    # -- energy model (SIM-PHX-002) ---------------------------------------------

    def _drain_rate(self) -> float:
        """Airspeed-dependent battery drain: baseline up to cruise,
        quadratic penalty in the dash regime."""
        speed = float(np.linalg.norm(self.body.velocity))
        factor = 1.0
        if speed > self.cruise_speed:
            factor += 2.0 * ((speed - self.cruise_speed) / max(self.cruise_speed, 1.0)) ** 2
        return self._drain_per_s * factor

    # -- flight helpers -----------------------------------------------------------

    def _fly_to(self, waypoint: np.ndarray) -> None:
        self.body.command_velocity(
            guidance.goto_velocity(self.body.position, waypoint, self.max_speed)
        )

    def _at(self, point: np.ndarray, radius: float = 25.0) -> bool:
        return bool(np.linalg.norm(self.body.position - point) < radius)
