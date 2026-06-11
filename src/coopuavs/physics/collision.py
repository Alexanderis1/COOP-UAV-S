"""Axis-aligned prism (2.5D building) and flat-terrain collision, batched.

Prisms are an (M, 5) float array [xmin, ymin, xmax, ymax, height] with the
ground at z = 0 — the raw form of the sim's ``Building`` contract
(rect + height); world adapters convert Building lists in P4/P6 so this
package stays standalone.

Segment queries use the slab method (Kay-Kajiya; see e.g. Ericson,
"Real-Time Collision Detection", section 5.3.3) vectorized over
N segments x M prisms, so a fast mover cannot tunnel through a building
between steps. Degenerate (axis-parallel) direction components are handled
by substituting a tiny denominator, which drives the slab interval to
+-huge with the correct sign.
"""

from __future__ import annotations

import numpy as np

KIND_NONE = 0
KIND_TERRAIN = 1
KIND_PRISM = 2

_EPS_DIR = 1e-30


def _bounds(prisms: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    lo = np.column_stack([prisms[:, 0], prisms[:, 1], np.zeros(len(prisms))])
    hi = np.column_stack([prisms[:, 2], prisms[:, 3], prisms[:, 4]])
    return lo, hi


def inside_prisms(pos: np.ndarray, prisms: np.ndarray) -> np.ndarray:
    """(N,) bool: point inside any prism volume (closed bounds, 0 <= z <= h)."""
    lo, hi = _bounds(prisms)
    ok = (pos[:, None, :] >= lo[None]) & (pos[:, None, :] <= hi[None])
    return ok.all(axis=2).any(axis=1)


def segment_prisms(p0: np.ndarray, p1: np.ndarray, prisms: np.ndarray
                   ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """First prism hit along each segment p0 -> p1.

    Returns (hit (N,) bool, t (N,) in [0,1] or +inf, point (N,3)).
    """
    n = p0.shape[0]
    if len(prisms) == 0:
        return np.zeros(n, bool), np.full(n, np.inf), p0.copy()
    lo, hi = _bounds(prisms)
    d = p1 - p0
    d_safe = np.where(np.abs(d) < _EPS_DIR, _EPS_DIR, d)[:, None, :]
    t0 = (lo[None] - p0[:, None, :]) / d_safe
    t1 = (hi[None] - p0[:, None, :]) / d_safe
    tlo = np.minimum(t0, t1).max(axis=2)            # (N, M) slab entry
    thi = np.maximum(t0, t1).min(axis=2)            # (N, M) slab exit
    valid = (thi >= np.maximum(tlo, 0.0)) & (tlo <= 1.0) & (thi >= 0.0)
    t_entry = np.where(valid, np.maximum(tlo, 0.0), np.inf)
    t = t_entry.min(axis=1)
    hit = np.isfinite(t)
    point = p0 + np.where(hit, t, 0.0)[:, None] * d
    return hit, t, point


def segment_terrain(p0: np.ndarray, p1: np.ndarray, ground_z: float = 0.0
                    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Flat-ground crossing along each segment (already-underground => t=0)."""
    z0, z1 = p0[:, 2], p1[:, 2]
    below = z0 <= ground_z
    dz = z1 - z0
    dz_safe = np.where(np.abs(dz) < _EPS_DIR, _EPS_DIR, dz)
    t_cross = (ground_z - z0) / dz_safe
    crossing = (~below) & (z1 <= ground_z)
    t = np.where(below, 0.0, np.where(crossing, t_cross, np.inf))
    hit = below | crossing
    point = p0 + np.where(hit, t, 0.0)[:, None] * (p1 - p0)
    return hit, t, point


def first_collision(p0: np.ndarray, p1: np.ndarray, prisms: np.ndarray,
                    ground_z: float = 0.0
                    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Earliest of terrain/prism hit per segment.

    Returns (kind (N,) int: KIND_NONE/TERRAIN/PRISM, t (N,) or +inf,
    point (N,3)).
    """
    hit_p, t_p, pt_p = segment_prisms(p0, p1, prisms)
    hit_g, t_g, pt_g = segment_terrain(p0, p1, ground_z)
    prism_first = hit_p & (t_p <= np.where(hit_g, t_g, np.inf))
    kind = np.where(prism_first, KIND_PRISM,
                    np.where(hit_g, KIND_TERRAIN, KIND_NONE)).astype(np.int8)
    t = np.where(prism_first, t_p, np.where(hit_g, t_g, np.inf))
    point = np.where(prism_first[:, None], pt_p, pt_g)
    return kind, t, point
