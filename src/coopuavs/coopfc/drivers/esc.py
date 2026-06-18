"""ESC telemetry driver (10 Hz task).

HAL frame (normative): ``((rpm_0..rpm_R-1), v_bus, i_bus)`` — mechanical
shaft rpm per rotor (the hw/esc_telem.py convention; eRPM pole-pair
conversion is already done device-side), bus volts, bus amps. The driver
converts rpm to rad/s (omega = rpm * 2*pi / 60) and publishes
``esc_status`` carrying both.

Garbage frames (non-finite rpm/volts/amps, non-positive bus volts) are
rejected without publishing, like the baro driver: a NaN ``v_bus`` fed
to the battery monitor would sustain the debounce (NaN >= x is False)
into a latched CRITICAL and a forced LAND on a healthy pack.
"""

from __future__ import annotations

import math

from coopuavs.coopfc.core.msgs import EscMsg
from coopuavs.coopfc.drivers._base import Driver

_RPM_TO_RAD_S = math.tau / 60.0


class EscDriver(Driver):
    __slots__ = ()

    def __init__(self, port, topics, stale_after: int = 2):
        super().__init__(port, topics.advertise("esc_status", EscMsg), stale_after)

    def _convert(self, now: float, frame) -> bool:
        rpm, v_bus, i_bus = frame
        rpm_t = tuple(float(r) for r in rpm)
        v, i = float(v_bus), float(i_bus)
        if (not all(math.isfinite(r) for r in rpm_t)
                or not math.isfinite(v) or v <= 0.0
                or not math.isfinite(i)):
            return False
        self._pub.publish(EscMsg(
            stamp=now,
            rpm=rpm_t,
            omega=tuple(r * _RPM_TO_RAD_S for r in rpm_t),
            v_bus=v,
            i_bus=i,
        ))
        return True
