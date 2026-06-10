"""Ground-risk map of the defended residential area.

The area is rasterised into a grid of :class:`ZoneClass` cells (SAFE,
DANGEROUS, CRITICAL). The map answers two questions:

* what zone is under a point (``zone_at``), and
* what is the expected collateral cost of a set of probabilistic ground
  impact points (``collateral_cost``).

Zone weights implement the safety policy: debris over SAFE is nearly free,
DANGEROUS is heavily penalised, CRITICAL is effectively forbidden — the ROE
layer compares the resulting expected cost against per-zone thresholds.
This follows the spirit of SORA/JARUS ground-risk modelling while staying
fast enough to evaluate inside the planning loop.
"""

from __future__ import annotations

import numpy as np

from ..core.messages import ZoneClass

# Relative cost of one debris impact in each zone class.
ZONE_WEIGHTS = {
    ZoneClass.SAFE: 0.02,
    ZoneClass.DANGEROUS: 1.0,
    ZoneClass.CRITICAL: 25.0,
}


class RiskMap:
    def __init__(
        self,
        bounds: tuple[float, float, float, float],
        cell_size: float = 50.0,
        default: ZoneClass = ZoneClass.DANGEROUS,
    ):
        """``bounds`` is (xmin, ymin, xmax, ymax) in map-frame metres.

        Default class is DANGEROUS: in a residential scenario, unknown ground
        must be assumed populated.
        """
        self.bounds = bounds
        self.cell_size = cell_size
        xmin, ymin, xmax, ymax = bounds
        self.nx = max(1, int(np.ceil((xmax - xmin) / cell_size)))
        self.ny = max(1, int(np.ceil((ymax - ymin) / cell_size)))
        self.grid = np.full((self.ny, self.nx), int(default), dtype=np.int8)

    # -- authoring -----------------------------------------------------------

    def set_rect(self, rect: tuple[float, float, float, float], zone: ZoneClass) -> None:
        """Mark a rectangle (xmin, ymin, xmax, ymax) with a zone class.

        Only the intersection with the map is painted: ``_index`` clips to
        the grid, so a rectangle authored entirely off-map would otherwise
        silently stamp a stripe of border cells with its class — a stray
        CRITICAL rect in a scenario file must not forbid the map edge.
        """
        xmin, ymin, xmax, ymax = self.bounds
        if rect[2] <= xmin or rect[0] >= xmax or rect[3] <= ymin or rect[1] >= ymax:
            return
        i0, j0 = self._index(rect[0], rect[1])
        i1, j1 = self._index(rect[2], rect[3])
        self.grid[min(j0, j1) : max(j0, j1) + 1, min(i0, i1) : max(i0, i1) + 1] = int(zone)

    # -- queries ---------------------------------------------------------------

    def _index(self, x: float, y: float) -> tuple[int, int]:
        xmin, ymin, _, _ = self.bounds
        i = int(np.clip((x - xmin) / self.cell_size, 0, self.nx - 1))
        j = int(np.clip((y - ymin) / self.cell_size, 0, self.ny - 1))
        return i, j

    def zone_at(self, x: float, y: float) -> ZoneClass:
        i, j = self._index(x, y)
        return ZoneClass(int(self.grid[j, i]))

    def collateral_cost(self, impact_points: np.ndarray) -> float:
        """Expected zone-weighted cost of impact samples, shape (N, 2|3)."""
        if len(impact_points) == 0:
            return 0.0
        costs = [ZONE_WEIGHTS[self.zone_at(p[0], p[1])] for p in impact_points]
        return float(np.mean(costs))

    def critical_hit_probability(self, impact_points: np.ndarray) -> float:
        """Fraction of impact samples landing on CRITICAL cells."""
        if len(impact_points) == 0:
            return 0.0
        hits = [self.zone_at(p[0], p[1]) == ZoneClass.CRITICAL for p in impact_points]
        return float(np.mean(hits))

    def nearest_safe_cell(self, x: float, y: float, max_radius: float = 2000.0) -> np.ndarray:
        """Centre of the closest SAFE cell — used to place the kill box."""
        xmin, ymin, _, _ = self.bounds
        js, is_ = np.where(self.grid == int(ZoneClass.SAFE))
        if len(is_) == 0:
            return np.array([x, y])
        cx = xmin + (is_ + 0.5) * self.cell_size
        cy = ymin + (js + 0.5) * self.cell_size
        d2 = (cx - x) ** 2 + (cy - y) ** 2
        k = int(np.argmin(d2))
        if d2[k] > max_radius**2:
            return np.array([x, y])
        return np.array([cx[k], cy[k]])
