"""Battery state monitor (10 Hz task): pack volts -> NORMAL/LOW/CRITICAL.

Per-cell voltage from the ESC-telemetry bus reading. Thresholds latch
upward only (NORMAL -> LOW -> CRITICAL, never back: a pack that sagged
to critical under load is not healthy again when the load drops — the
PX4 battery-failsafe convention) and each crossing is debounced: the
voltage must sit below the threshold for ``debounce_s`` of *monitor
time* continuously, so a transient dash-current sag does not trigger an
RTL. Voltage-only on purpose: the pack telemetry is pack-level V/I
until P5 (per-cell + SOC estimation arrive with CELL_IMBALANCE).
"""

from __future__ import annotations

from typing import NamedTuple

NORMAL = "NORMAL"
LOW = "LOW"
CRITICAL = "CRITICAL"


class BattParams(NamedTuple):
    cells: int = 12
    low_v_cell: float = 3.50      # -> RTL
    crit_v_cell: float = 3.30     # -> LAND
    debounce_s: float = 1.0


class BatteryMonitor:
    def __init__(self, params: BattParams = BattParams()):
        self.params = params
        self.state = NORMAL
        self.v_cell = float("nan")
        self._below_low_since: float | None = None
        self._below_crit_since: float | None = None

    def update(self, now: float, v_bus: float) -> str:
        p = self.params
        self.v_cell = v_bus / p.cells

        def debounced(threshold: float, since: float | None
                      ) -> tuple[bool, float | None]:
            if self.v_cell >= threshold:
                return False, None
            if since is None:
                return False, now
            return now - since >= p.debounce_s, since

        crossed, self._below_crit_since = debounced(
            p.crit_v_cell, self._below_crit_since)
        if crossed:
            self.state = CRITICAL
        if self.state != CRITICAL:
            crossed, self._below_low_since = debounced(
                p.low_v_cell, self._below_low_since)
            if crossed and self.state == NORMAL:
                self.state = LOW
        return self.state
