"""GNSS driver (10 Hz task).

HAL frame (normative): ``((px, py, pz), (vx, vy, vz), fix_type,
fix_stamp_s)`` — world ENU metres / m/s, u-blox fix type, and the
*measurement* timestamp (delivery minus the modeled 120 ms latency).
Publishes ``gps_fix``; the EKF's OOSM path keys on ``fix_stamp``.

Default stale_after = 3: with a latency longer than one fix period the
first fix legitimately lands two driver ticks after boot, so 2 would
false-alarm during startup.
"""

from __future__ import annotations

from coopuavs.coopfc.core.msgs import GpsMsg
from coopuavs.coopfc.drivers._base import Driver


class GpsDriver(Driver):
    __slots__ = ()

    def __init__(self, port, topics, stale_after: int = 3):
        super().__init__(port, topics.advertise("gps_fix", GpsMsg), stale_after)

    def _convert(self, now: float, frame) -> bool:
        pos, vel, fix_type, fix_stamp = frame
        self._pub.publish(GpsMsg(
            stamp=now,
            fix_stamp=float(fix_stamp),
            pos=(float(pos[0]), float(pos[1]), float(pos[2])),
            vel=(float(vel[0]), float(vel[1]), float(vel[2])),
            fix_type=int(fix_type),
        ))
        return True
