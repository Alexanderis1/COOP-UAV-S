"""Pack state-of-charge estimator (P5-1f, user decision 2026-06-12).

OCV-seeded coulomb counting from the ESC telemetry bus reads (10 Hz):

- initialization: while the pack is RESTING (|I| below
  ``rest_current_a``), terminal voltage ~= OCV; average ``rest_samples``
  frames and invert the OCV curve. A pack first seen under load stays
  uninitialized (None) until it rests — guessing SOC from a sagged
  voltage is the exact failure mode this estimator exists to remove.
- counting: SOC -= I dt / (3600 C). The integral drifts with current-
  sensor bias over very long flights; flights here are tens of minutes
  and every pad turnaround re-seeds from rest OCV (``reset()`` rides
  the BATT_RESET pack-swap command), which bounds the drift window.

The OCV calibration table is the flight software's copy of the pack
datasheet curve — deliberately duplicated from physics/battery.py
(coopfc is import-fenced away from the plant); a cross-check test pins
the two tables equal so they cannot drift apart silently.

Charging on the pad is NOT visible in the bus current (the charger
circuit is out of scope, SOC is driven directly — sil/fleet.py pad
note), so a recharge is only learned through the rest-OCV re-seed.
Plain Python on purpose (battery_monitor.py convention).
"""

from __future__ import annotations

from typing import NamedTuple

# LiPo cell OCV(SOC) calibration (= physics/battery.py table; pinned).
OCV_SOC = (0.00, 0.05, 0.10, 0.20, 0.30, 0.40, 0.50,
           0.60, 0.70, 0.80, 0.90, 0.95, 1.00)
OCV_V = (3.27, 3.50, 3.61, 3.69, 3.71, 3.73, 3.75,
         3.79, 3.84, 3.90, 3.97, 4.05, 4.18)


def soc_from_rest_v_cell(v_cell: float) -> float:
    """Invert the OCV curve (linear segments, clamped at the ends)."""
    if v_cell <= OCV_V[0]:
        return 0.0
    for k in range(1, len(OCV_V)):
        if v_cell <= OCV_V[k]:
            f = (v_cell - OCV_V[k - 1]) / (OCV_V[k] - OCV_V[k - 1])
            return OCV_SOC[k - 1] + f * (OCV_SOC[k] - OCV_SOC[k - 1])
    return 1.0


def ocv_v_cell(soc: float) -> float:
    """Forward OCV per cell (the SAG_ANOM monitor's expectation)."""
    if soc <= 0.0:
        return OCV_V[0]
    for k in range(1, len(OCV_SOC)):
        if soc <= OCV_SOC[k]:
            f = (soc - OCV_SOC[k - 1]) / (OCV_SOC[k] - OCV_SOC[k - 1])
            return OCV_V[k - 1] + f * (OCV_V[k] - OCV_V[k - 1])
    return OCV_V[-1]


class SocParams(NamedTuple):
    capacity_ah: float = 16.0
    cells: int = 12
    rest_current_a: float = 2.0
    rest_samples: int = 5
    # Continuous rest recalibration: while resting, blend toward the
    # rest-OCV reading at this rate per frame (~2 s convergence at
    # 10 Hz). A resting terminal voltage IS the charge state — this is
    # how the counter learns about pad charging, whose current never
    # crosses the bus sense (sil/fleet.py pad note). Flight is never
    # resting (hover current >> rest_current_a), so this cannot move
    # the estimate mid-air.
    rest_blend: float = 0.2


class SocEstimator:
    def __init__(self, params: SocParams = SocParams()):
        self.params = params
        self.soc: float | None = None
        self._rest_v_sum = 0.0
        self._rest_n = 0
        self._last_stamp: float | None = None

    def reset(self) -> None:
        """Pack swapped/recharged (BATT_RESET): forget everything and
        re-seed from the next rest window."""
        self.soc = None
        self._rest_v_sum = 0.0
        self._rest_n = 0
        self._last_stamp = None

    def update(self, stamp: float, v_bus: float, i_bus: float) -> None:
        p = self.params
        if self.soc is None:
            if abs(i_bus) < p.rest_current_a:
                self._rest_v_sum += v_bus
                self._rest_n += 1
                if self._rest_n >= p.rest_samples:
                    self.soc = soc_from_rest_v_cell(
                        self._rest_v_sum / self._rest_n / p.cells)
                    self._last_stamp = stamp
            else:
                # Load breaks the rest window: a sagged average would
                # seed a pessimistic SOC — start over.
                self._rest_v_sum = 0.0
                self._rest_n = 0
            return
        if self._last_stamp is not None:
            dt = stamp - self._last_stamp
            if dt > 0.0:
                self.soc = min(max(
                    self.soc - i_bus * dt / (3600.0 * p.capacity_ah),
                    0.0), 1.0)
        if abs(i_bus) < p.rest_current_a:
            rest = soc_from_rest_v_cell(v_bus / p.cells)
            self.soc += p.rest_blend * (rest - self.soc)
        self._last_stamp = stamp
