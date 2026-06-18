"""IMU driver (400 Hz task).

HAL frame (normative): ``((gx, gy, gz), (ax, ay, az))`` — rad/s and
m/s^2 specific force, body FLU, already quantized device-side. The
driver is an SI passthrough plus staleness; it publishes ``imu_raw``.
"""

from __future__ import annotations

from coopuavs.coopfc.core.msgs import ImuSample
from coopuavs.coopfc.drivers._base import Driver


class ImuDriver(Driver):
    __slots__ = ()

    def __init__(self, port, topics, stale_after: int = 2):
        super().__init__(port, topics.advertise("imu_raw", ImuSample), stale_after)

    def _convert(self, now: float, frame) -> bool:
        gyro, accel = frame
        self._pub.publish(ImuSample(
            stamp=now,
            gyro=(float(gyro[0]), float(gyro[1]), float(gyro[2])),
            accel=(float(accel[0]), float(accel[1]), float(accel[2])),
        ))
        return True
