"""Building line-of-sight occlusion (SIM-SEN-005, SIM-EFF-006).

Buildings are 2.5D boxes (footprint rect + height). The grid bins their
footprints once at construction; a sight-line query traverses the bins
along the segment (Amanatides–Woo DDA), slab-tests each candidate
footprint in xy and checks the ray altitude over the crossed interval —
the ray is obstructed only where it passes below the rooftop inside the
footprint.

Each crossing multiplies a per-material, per-channel transmittance into
the result: EO/IR (and seekers) are blocked by any solid structure, radar
and RF partially penetrate light construction, acoustic energy diffracts
around buildings at a flat per-crossing factor. Parks and water
(``Material.NONE``) are not solid and never obstruct.

Pure static geometry — no RNG, deterministic for a given scenario
(SIM-003).
"""

from __future__ import annotations

import numpy as np

# Per-crossing transmittance by material and sensing channel. Plausible
# engineering values (fidelity: representative, see TRACEABILITY
# PHY-SEN-005): radar is two-way in practice — sensors square it.
MATERIAL_TRANSMISSION: dict[str, dict[str, float]] = {
    "concrete":    {"radar": 0.00, "rf": 0.05, "eo_ir": 0.0, "acoustic": 0.6},
    "brick":       {"radar": 0.00, "rf": 0.10, "eo_ir": 0.0, "acoustic": 0.6},
    "glass_steel": {"radar": 0.15, "rf": 0.30, "eo_ir": 0.0, "acoustic": 0.6},
    "light_metal": {"radar": 0.35, "rf": 0.50, "eo_ir": 0.0, "acoustic": 0.6},
    "wood":        {"radar": 0.50, "rf": 0.60, "eo_ir": 0.0, "acoustic": 0.6},
}


class OcclusionGrid:
    def __init__(self, buildings, bounds, bin_size: float = 200.0,
                 enabled: bool = True):
        self.enabled = enabled
        self.bounds = bounds
        self.bin_size = bin_size
        xmin, ymin, xmax, ymax = bounds
        self.nx = max(1, int(np.ceil((xmax - xmin) / bin_size)))
        self.ny = max(1, int(np.ceil((ymax - ymin) / bin_size)))
        # Solid obstructions only: rect, height, material value.
        self._solids: list[tuple[tuple[float, float, float, float], float, str]] = [
            (b.rect, b.height, str(getattr(b.material, "value", b.material)))
            for b in buildings
            if getattr(b, "solid", True) and b.height > 0.0
        ]
        self._bins: dict[tuple[int, int], list[int]] = {}
        for idx, (rect, _, _) in enumerate(self._solids):
            i0, j0 = self._bin_of(rect[0], rect[1])
            i1, j1 = self._bin_of(rect[2], rect[3])
            for j in range(j0, j1 + 1):
                for i in range(i0, i1 + 1):
                    self._bins.setdefault((i, j), []).append(idx)

    def _bin_of(self, x: float, y: float) -> tuple[int, int]:
        xmin, ymin, _, _ = self.bounds
        i = int(np.clip((x - xmin) // self.bin_size, 0, self.nx - 1))
        j = int(np.clip((y - ymin) // self.bin_size, 0, self.ny - 1))
        return i, j

    # -- queries ---------------------------------------------------------------

    def crossings(self, p0, p1) -> list[str]:
        """Material of every solid building the segment p0→p1 passes through
        below rooftop level."""
        if not self.enabled or not self._solids:
            return []
        p0 = np.asarray(p0, dtype=float)
        p1 = np.asarray(p1, dtype=float)
        out: list[str] = []
        for idx in self._candidates(p0, p1):
            rect, height, material = self._solids[idx]
            t_int = _slab_interval(p0, p1, rect)
            if t_int is None:
                continue
            t_enter, t_exit = t_int
            if t_enter <= 1e-9 and p0[2] <= height:
                # The ray starts inside this footprint below the roof: the
                # emitter is *mounted on* the structure (rooftop sensor,
                # turret at its host building) — the host does not occlude
                # its own instrument.
                continue
            z_enter = p0[2] + t_enter * (p1[2] - p0[2])
            z_exit = p0[2] + t_exit * (p1[2] - p0[2])
            if min(z_enter, z_exit) <= height:
                out.append(material)
        return out

    def transmittance(self, p0, p1, channel: str) -> float:
        """Product of per-material transmittances over all crossings; 1.0
        for an unobstructed sight line."""
        trans = 1.0
        for material in self.crossings(p0, p1):
            trans *= MATERIAL_TRANSMISSION[material][channel]
            if trans == 0.0:
                return 0.0
        return trans

    def clear(self, p0, p1) -> bool:
        """Whether the sight line crosses no solid structure at all — the
        fire-LOS gate (SIM-EFF-006)."""
        return not self.crossings(p0, p1)

    # -- internals ---------------------------------------------------------------

    def _candidates(self, p0, p1) -> list[int]:
        """Indices of buildings binned along the segment (deduplicated,
        order preserved)."""
        i, j = self._bin_of(p0[0], p0[1])
        i1, j1 = self._bin_of(p1[0], p1[1])
        seen: set[int] = set()
        out: list[int] = []

        def visit(ci: int, cj: int) -> None:
            for idx in self._bins.get((ci, cj), ()):
                if idx not in seen:
                    seen.add(idx)
                    out.append(idx)

        visit(i, j)
        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        xmin, ymin, _, _ = self.bounds
        step_i = 1 if dx > 0 else -1
        step_j = 1 if dy > 0 else -1
        # Parametric distance to the next bin boundary on each axis.
        t_max_x = np.inf if dx == 0 else (
            (xmin + (i + (step_i > 0)) * self.bin_size - p0[0]) / dx)
        t_max_y = np.inf if dy == 0 else (
            (ymin + (j + (step_j > 0)) * self.bin_size - p0[1]) / dy)
        t_dx = np.inf if dx == 0 else abs(self.bin_size / dx)
        t_dy = np.inf if dy == 0 else abs(self.bin_size / dy)
        while (i, j) != (i1, j1):
            if t_max_x < t_max_y:
                if t_max_x > 1.0:
                    break
                i += step_i
                t_max_x += t_dx
            else:
                if t_max_y > 1.0:
                    break
                j += step_j
                t_max_y += t_dy
            if not (0 <= i < self.nx and 0 <= j < self.ny):
                break
            visit(i, j)
        return out


def _slab_interval(p0, p1, rect) -> tuple[float, float] | None:
    """Parameter interval [t_enter, t_exit] ⊂ [0, 1] where the 2D segment
    p0→p1 lies inside the footprint rect, or None if it misses."""
    t_enter, t_exit = 0.0, 1.0
    for axis, (lo, hi) in enumerate(((rect[0], rect[2]), (rect[1], rect[3]))):
        a, b = p0[axis], p1[axis]
        d = b - a
        if abs(d) < 1e-12:
            if not (lo <= a <= hi):
                return None
            continue
        t0 = (lo - a) / d
        t1 = (hi - a) / d
        if t0 > t1:
            t0, t1 = t1, t0
        t_enter = max(t_enter, t0)
        t_exit = min(t_exit, t1)
        if t_enter >= t_exit:
            return None
    return t_enter, t_exit
