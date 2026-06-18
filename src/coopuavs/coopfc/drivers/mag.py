"""Magnetometer driver (50 Hz task).

HAL frame (normative): ``(bx, by, bz)`` uT, body FLU. SI passthrough
(declination handling and yaw fusion live in the estimator, which knows
the theater field model); publishes ``mag_body``.
"""

from __future__ import annotations

from coopuavs.coopfc.core.msgs import MagMsg
from coopuavs.coopfc.drivers._base import Driver


class MagDriver(Driver):
    __slots__ = ()

    def __init__(self, port, topics, stale_after: int = 2):
        super().__init__(port, topics.advertise("mag_body", MagMsg), stale_after)

    def _convert(self, now: float, frame) -> bool:
        self._pub.publish(MagMsg(
            stamp=now,
            field_ut=(float(frame[0]), float(frame[1]), float(frame[2])),
        ))
        return True
