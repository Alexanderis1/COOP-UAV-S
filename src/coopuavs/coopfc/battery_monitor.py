"""Battery state monitor (10 Hz task): volts + SOC -> NORMAL/LOW/CRITICAL.

Per-cell voltage from the ESC-telemetry bus reading. Thresholds latch
upward only (NORMAL -> LOW -> CRITICAL, never back: a pack that sagged
to critical under load is not healthy again when the load drops — the
PX4 battery-failsafe convention) and each crossing is debounced: the
voltage must sit below the threshold for ``debounce_s`` of *monitor
time* continuously, so a transient dash-current sag does not trigger an
RTL.

Voltage AND SOC arbitration (P5-1f, user decision 2026-06-12 — owns the
P4 full-power-climb sag-trip):

- the voltage crossings are VETOED while the pack is demonstrably both
  charged (coulomb SOC above ``soc_guard``) and under load (bus current
  above ``load_current_a``) AND the SOC story is self-consistent: a
  raised BATT_SAG_ANOM (sag beyond what the pack calibration explains)
  means the coulomb estimate itself is suspect — the veto lifts and
  the voltage evidence rules again (a dying pack with a stale coulomb
  count must not be flown into the ground on the strength of that
  count);
- at REST the veto never applies: terminal volts ~= OCV there, so a low
  rest voltage is real charge state no matter what the coulomb count
  claims (broken current sense / self-discharge);
- the coulomb SOC drives its own thresholds (``soc_low``/``soc_crit``)
  through the same upward latch — a smoothly-drained pack trips on
  charge state even while sag-free.

No SOC (estimator unseeded, None) = the exact P4 voltage-only behavior.
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
    full_v_cell: float = 4.20     # li-ion full charge (fraction anchor)
    soc_guard: float = 0.50       # veto voltage trips above this SOC...
    load_current_a: float = 10.0  # ...but only under load (rest = OCV)
    soc_low: float = 0.25         # -> RTL on charge state
    soc_crit: float = 0.10        # -> LAND on charge state


class BatteryMonitor:
    def __init__(self, params: BattParams = BattParams()):
        self.params = params
        self.state = NORMAL
        self.v_cell = float("nan")
        self._below_low_since: float | None = None
        self._below_crit_since: float | None = None

    def fraction(self) -> float:
        """Voltage-proxy battery fraction in [0, 1] for telemetry
        (P4-4 user decision): loaded v_cell mapped linearly
        crit_v_cell -> 0.0 .. full_v_cell -> 1.0. Deliberately
        conservative — sag under load reads as less remaining energy,
        which is exactly when the MC should head home earlier. 1.0
        until the first ESC frame arrives (NaN compares false).
        Real SOC estimation (coulomb counting, per-cell) is P5
        CELL_IMBALANCE scope."""
        p = self.params
        frac = (self.v_cell - p.crit_v_cell) / (p.full_v_cell - p.crit_v_cell)
        if not frac <= 1.0:        # NaN or above full
            return 1.0
        return max(frac, 0.0)

    def reset(self) -> None:
        """Battery swapped/recharged on the pad: clear the upward latch
        and the debounce clocks (the in-flight 'sagged pack is not
        healthy again' doctrine applies per pack, not across a swap)."""
        self.state = NORMAL
        self.v_cell = float("nan")
        self._below_low_since = None
        self._below_crit_since = None

    def update(self, now: float, v_bus: float, soc: float | None = None,
               i_bus: float = 0.0, sag_anom: bool = False) -> str:
        p = self.params
        self.v_cell = v_bus / p.cells
        # Loaded sag on a demonstrably charged pack is I*R physics, not
        # charge state — unless the sag itself is anomalous, which
        # impeaches the SOC estimate (module docstring). The debounce
        # clocks keep running so a veto that lapses does not restart
        # the window.
        veto = (soc is not None and soc > p.soc_guard
                and abs(i_bus) > p.load_current_a and not sag_anom)

        def debounced(threshold: float, since: float | None
                      ) -> tuple[bool, float | None]:
            if self.v_cell >= threshold:
                return False, None
            if since is None:
                return False, now
            return now - since >= p.debounce_s, since

        crossed, self._below_crit_since = debounced(
            p.crit_v_cell, self._below_crit_since)
        if (crossed and not veto) or (soc is not None and soc < p.soc_crit):
            self.state = CRITICAL
        if self.state != CRITICAL:
            crossed, self._below_low_since = debounced(
                p.low_v_cell, self._below_low_since)
            if ((crossed and not veto)
                    or (soc is not None and soc < p.soc_low)) \
                    and self.state == NORMAL:
                self.state = LOW
        return self.state
