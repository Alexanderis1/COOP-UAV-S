"""Thevenin 1-RC equivalent-circuit battery model (ECM), batched per vehicle.

Model [Chen & Rincon-Mora 2006, IEEE Trans. Energy Conversion; see
docs/RESEARCH.md]:

    V_t   = OCV(SOC) - I R0 - V1          (terminal voltage under load)
    V1'   = -V1 / (R1 C1) + I / C1        (single RC polarization branch)
    SOC'  = -I / (3600 * capacity_Ah)     (coulomb counting, I > 0 = discharge)

Discrete update is the exact zero-order-hold solution over one step
(current held constant):

    V1  <- V1 e^{-dt/tau1} + I R1 (1 - e^{-dt/tau1}),  tau1 = R1 C1
    SOC <- SOC - I dt / (3600 capacity_Ah)

so the instant sag is exactly I*R0, recovery is exactly exponential in tau1
and the coulomb integral carries no integration error by construction.

OCV(SOC) is a per-cell LiPo open-circuit-voltage table (typical discharge
curve, interpolated linearly and clamped at the ends) scaled by the series
cell count. SOC is clamped to [0, 1].

The model is pure and unbounded by design: the returned terminal voltage is
NOT clamped to a physical envelope (V_t < 0 under extreme discharge, or
above 4.2 V/cell under forced charge, if the prescribed current demands it)
and R0 is an instantaneous feedthrough, which makes a one-step-lag coupling
with the quasi-static motor model algebraically unstable. Envelope
enforcement — the bus current limit and the 3.0/4.2 V-per-cell bounds — and
the closed-form implicit solve of the motor-battery loop live in
physics/powertrain.py.
"""

from __future__ import annotations

import numpy as np

# Typical LiPo cell OCV vs SOC (rest voltage; knee at low SOC, steep top end).
_OCV_SOC = np.array([0.00, 0.05, 0.10, 0.20, 0.30, 0.40, 0.50,
                     0.60, 0.70, 0.80, 0.90, 0.95, 1.00])
_OCV_V = np.array([3.27, 3.50, 3.61, 3.69, 3.71, 3.73, 3.75,
                   3.79, 3.84, 3.90, 3.97, 4.05, 4.18])


class BatteryEcm:
    """Batched 1-RC ECM pack model; ``step(dt, current) -> terminal voltage``."""

    def __init__(self, n: int, capacity_ah: float, n_series: int, r0: float,
                 r1: float, c1: float, soc0: float = 1.0,
                 ocv_table: tuple[np.ndarray, np.ndarray] | None = None):
        self.n = int(n)
        self.capacity_ah = float(capacity_ah)
        self.n_series = int(n_series)
        self.r0 = float(r0)
        self.r1 = float(r1)
        self.c1 = float(c1)
        self.tau1 = self.r1 * self.c1
        self._ocv_soc, self._ocv_v = ocv_table if ocv_table is not None \
            else (_OCV_SOC, _OCV_V)
        self.soc = np.full(self.n, float(soc0))
        self.v1 = np.zeros(self.n)
        # P5 fault seams (SIM-SIL-003). Both default to the exact
        # pre-P5 arithmetic path — un-faulted packs are bitwise
        # identical to the P4 model (pinned).
        # cell_delta_soc: (n, n_series) per-cell SOC offsets (a weak
        # cell sits at lower SOC than the pack mean — CELL_IMBALANCE).
        # r0_scale: (n,) series-resistance multiplier (aged/cold pack —
        # BATT_SAG_ANOM).
        self.cell_delta_soc: np.ndarray | None = None
        self.r0_scale: np.ndarray | None = None

    def inject_cell_imbalance(self, i: int, deltas) -> None:
        """Offset vehicle ``i``'s per-cell SOCs (length n_series; should
        be ~zero-mean: a spread, not a disguised capacity edit)."""
        deltas = np.asarray(deltas, dtype=float)
        if deltas.shape != (self.n_series,):
            raise ValueError(
                f"deltas must have shape ({self.n_series},), got {deltas.shape}")
        if self.cell_delta_soc is None:
            self.cell_delta_soc = np.zeros((self.n, self.n_series))
        self.cell_delta_soc[i] = deltas

    def inject_r0_scale(self, i: int, scale: float) -> None:
        """Scale vehicle ``i``'s series resistance (aged-pack fault)."""
        if scale <= 0.0:
            raise ValueError(f"r0 scale must be > 0, got {scale!r}")
        if self.r0_scale is None:
            self.r0_scale = np.ones(self.n)
        self.r0_scale[i] = float(scale)

    def _r0(self):
        return self.r0 if self.r0_scale is None else self.r0 * self.r0_scale

    def ocv(self, soc: np.ndarray) -> np.ndarray:
        """Pack open-circuit voltage (V) at the given SOC. With injected
        cell imbalance the pack OCV is the SUM of the per-cell curves
        (cells in series); the un-faulted path keeps the original
        scaled-interp arithmetic bitwise."""
        if self.cell_delta_soc is None:
            return self.n_series * np.interp(soc, self._ocv_soc, self._ocv_v)
        soc = np.asarray(soc)
        per_cell = np.interp(
            np.clip(soc[..., None] + self.cell_delta_soc, 0.0, 1.0),
            self._ocv_soc, self._ocv_v)
        return per_cell.sum(axis=-1)

    def cell_voltages(self, v_bus: np.ndarray) -> np.ndarray:
        """(n, n_series) terminal voltage per cell consistent with the
        pack terminal voltage: equal split plus the per-cell OCV
        deviation (uniform per-cell R/RC split — the load terms divide
        evenly; cell-to-cell resistance variation is out of scope)."""
        base = np.asarray(v_bus, dtype=float)[:, None] / self.n_series
        if self.cell_delta_soc is None:
            return np.repeat(base, self.n_series, axis=1)
        per_cell = np.interp(
            np.clip(self.soc[:, None] + self.cell_delta_soc, 0.0, 1.0),
            self._ocv_soc, self._ocv_v)
        return base + per_cell - per_cell.mean(axis=1, keepdims=True)

    def step(self, dt: float, current_a: np.ndarray) -> np.ndarray:
        """Advance one step under (n,) pack current (A, >0 discharge).

        Returns the terminal voltage (n,) at the end of the step.
        """
        decay = np.exp(-dt / self.tau1)
        self.v1 = self.v1 * decay + current_a * self.r1 * (1.0 - decay)
        self.soc = np.clip(
            self.soc - current_a * dt / (3600.0 * self.capacity_ah), 0.0, 1.0)
        return self.ocv(self.soc) - current_a * self._r0() - self.v1
