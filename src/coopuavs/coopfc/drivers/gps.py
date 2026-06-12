"""GNSS driver.

HAL frame (normative): ``((px, py, pz), (vx, vy, vz), fix_type,
fix_stamp_s)`` — world ENU metres / m/s, u-blox fix type, and the
*measurement* timestamp (delivery minus the modeled 120 ms latency).
Publishes ``gps_fix``; the EKF's OOSM path keys on ``fix_stamp``.

Poll faster than the 10 Hz fix rate: the EKF ``lag_s`` horizon covers
the device latency but not poll quantization on top, so the FCU runs
this driver at 50 Hz (a fix must reach the EKF before the horizon
passes its stamp). ``stale_after`` is in poll ticks — the default 3
suits a 10 Hz poll (first fix lands two ticks after boot at 120 ms
latency); the FCU passes 15 to keep the same 300 ms window at 50 Hz.
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
