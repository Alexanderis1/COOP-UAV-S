"""Barometer driver (50 Hz task) — pressure (Pa) to ISA altitude (m).

HAL frame (normative): one float, static pressure in Pa. Publishes
``baro_alt`` with both the raw pressure and the converted ISA pressure
altitude. Non-finite or non-positive pressure is rejected (counted in
``bad_frames``, nothing published) — a wedged sensor must not crash the
flight software; the P5 CBIT BARO fault consumes the tally.

The ISA constants are coopfc's own copy (import fence): they must equal
the physics/hw values, which is pinned bit-near by
tests/test_coopfc_drivers.py against hw.baro.altitude_from_pressure.
"""

from __future__ import annotations

import math

from coopuavs.coopfc.core.msgs import BaroMsg
from coopuavs.coopfc.drivers._base import Driver

# ISA troposphere constants — same values as physics/atmosphere.py.
_ISA_T0 = 288.15       # K
_ISA_P0 = 101325.0     # Pa
_ISA_LAPSE = 0.0065    # K/m
_R_AIR = 287.05287     # J/(kg K)
_G0 = 9.80665          # m/s^2

_EXP = _R_AIR * _ISA_LAPSE / _G0


def pressure_to_altitude(p_pa: float) -> float:
    """Exact ISA troposphere inverse: h = (T0/L) (1 - (p/p0)^(R L / g0))."""
    return (_ISA_T0 / _ISA_LAPSE) * (1.0 - (p_pa / _ISA_P0) ** _EXP)


class BaroDriver(Driver):
    __slots__ = ()

    def __init__(self, port, topics, stale_after: int = 2):
        super().__init__(port, topics.advertise("baro_alt", BaroMsg), stale_after)

    def _convert(self, now: float, frame) -> bool:
        p = float(frame)
        if not math.isfinite(p) or p <= 0.0:
            return False
        self._pub.publish(BaroMsg(stamp=now, pressure_pa=p,
                                  alt_m=pressure_to_altitude(p)))
        return True
