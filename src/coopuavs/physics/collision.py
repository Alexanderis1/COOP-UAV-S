"""Axis-aligned prism (2.5D building) and flat-terrain collision, batched.

Prisms are an (M, 5) float array [xmin, ymin, xmax, ymax, height] with the
prism base rooted at the terrain datum ``ground_z`` (default 0, one
consistent datum with ``segment_terrain``/``first_collision``) and the roof
at ``ground_z + height`` — the raw form of the sim's ``Building`` contract
(rect + height above ground); world adapters convert Building lists in
P4/P6 so this package stays standalone. Every public entry point validates
the prism array and raises ValueError when any row has xmax <= xmin,
ymax <= ymin, or height <= 0; an empty (0, 5) set is valid (open field).

Segment queries use the slab method (Kay-Kajiya; see e.g. Ericson,
"Real-Time Collision Detection", section 5.3.3) vectorized over
N segments x M prisms, so a fast mover cannot tunnel through a building
between steps. Degenerate (axis-parallel) direction components are handled
by substituting a tiny sign-preserving denominator (dz == 0 counts as
positive), which drives the slab interval to +-huge with the correct sign.
"""

from __future__ import annotations

import numpy as np

KIND_NONE = 0
KIND_TERRAIN = 1
KIND_PRISM = 2

_EPS_DIR = 1e-30


def _validate_prisms(prisms: np.ndarray) -> None:
    """ValueError on malformed rows; an empty (0, 5) array is valid.

    An inverted slab would otherwise answer inconsistently: solid to
    crossing segments (order-agnostic min/max below) but empty to point
    and degenerate-axis queries (closed lo <= p <= hi).
    """
    if len(prisms) == 0:
        return
    bad = ((prisms[:, 2] <= prisms[:, 0])
           | (prisms[:, 3] <= prisms[:, 1])
           | (prisms[:, 4] <= 0.0))
    if np.any(bad):
        raise ValueError(
            "malformed prisms (need xmin < xmax, ymin < ymax, height > 0) "
            f"at rows {np.flatnonzero(bad).tolist()}")


def _bounds(prisms: np.ndarray, ground_z: float = 0.0
            ) -> tuple[np.ndarray, np.ndarray]:
    base = np.full(len(prisms), ground_z)
    lo = np.column_stack([prisms[:, 0], prisms[:, 1], base])
    hi = np.column_stack([prisms[:, 2], prisms[:, 3], prisms[:, 4] + base])
    return lo, hi


def inside_prisms(pos: np.ndarray, prisms: np.ndarray,
                  ground_z: float = 0.0) -> np.ndarray:
    """(N,) bool: point inside any prism volume (closed bounds,
    ground_z <= z <= ground_z + h)."""
    _validate_prisms(prisms)
    lo, hi = _bounds(prisms, ground_z)
    ok = (pos[:, None, :] >= lo[None]) & (pos[:, None, :] <= hi[None])
    return ok.all(axis=2).any(axis=1)


def segment_prisms(p0: np.ndarray, p1: np.ndarray, prisms: np.ndarray,
                   ground_z: float = 0.0
                   ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """First prism hit along each segment p0 -> p1.

    Returns (hit (N,) bool, t (N,) in [0,1] or +inf, point (N,3)).
    """
    _validate_prisms(prisms)
    n = p0.shape[0]
    if len(prisms) == 0:
        return np.zeros(n, bool), np.full(n, np.inf), p0.copy()
    lo, hi = _bounds(prisms, ground_z)
    d = p1 - p0
    deg = np.abs(d) < _EPS_DIR                      # (N, 3) degenerate axes
    d_safe = np.where(deg, _EPS_DIR, d)[:, None, :]
    t0 = (lo[None] - p0[:, None, :]) / d_safe
    t1 = (hi[None] - p0[:, None, :]) / d_safe
    tlo_ax = np.minimum(t0, t1)
    thi_ax = np.maximum(t0, t1)
    if deg.any():
        # A degenerate axis is no constraint when p0 lies inside the CLOSED
        # slab (matches inside_prisms; a signed-epsilon division would make
        # exact hi-face grazes miss while lo-face grazes hit) and rules the
        # prism out entirely when outside it.
        inside0 = (p0[:, None, :] >= lo[None]) & (p0[:, None, :] <= hi[None])
        deg_m = np.broadcast_to(deg[:, None, :], inside0.shape)
        tlo_ax = np.where(deg_m, np.where(inside0, -np.inf, np.inf), tlo_ax)
        thi_ax = np.where(deg_m, np.where(inside0, np.inf, -np.inf), thi_ax)
    tlo = tlo_ax.max(axis=2)                        # (N, M) slab entry
    thi = thi_ax.min(axis=2)                        # (N, M) slab exit
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
    # sign-preserving epsilon (dz == 0 counts as positive): an unsigned
    # substitute flipped t_cross negative for near-flat downward skims
    dz_safe = np.where(np.abs(dz) < _EPS_DIR,
                       np.where(dz < 0.0, -_EPS_DIR, _EPS_DIR), dz)
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

    ``ground_z`` is the single datum for both queries: terrain crossing
    AND prism bases (buildings stand on the ground, roofs at
    ground_z + height). Returns (kind (N,) int: KIND_NONE/TERRAIN/PRISM,
    t (N,) or +inf, point (N,3)).
    """
    _validate_prisms(prisms)
    hit_p, t_p, pt_p = segment_prisms(p0, p1, prisms, ground_z)
    hit_g, t_g, pt_g = segment_terrain(p0, p1, ground_z)
    prism_first = hit_p & (t_p <= np.where(hit_g, t_g, np.inf))
    kind = np.where(prism_first, KIND_PRISM,
                    np.where(hit_g, KIND_TERRAIN, KIND_NONE)).astype(np.int8)
    t = np.where(prism_first, t_p, np.where(hit_g, t_g, np.inf))
    point = np.where(prism_first[:, None], pt_p, pt_g)
    return kind, t, point
