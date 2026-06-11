"""P1-6: prism (2.5D building) + flat-terrain collision queries.

Prisms are (M, 5) arrays [xmin, ymin, xmax, ymax, height] matching the
sim's Building contract (rect + height, ground at z=0); the slab-method
segment test catches tunneling at macro-step speeds. Pins: analytic
wall/roof/terrain hits, nearest-hit selection, start-inside, axis-parallel
degenerate segments, and vectorized == scalar-reference within 1e-12.
"""

from __future__ import annotations

import numpy as np

from coopuavs.physics import collision as col

PRISM = np.array([[0.0, 0.0, 10.0, 10.0, 30.0]])


def seg(p0, p1, prisms=PRISM):
    return col.segment_prisms(np.atleast_2d(np.asarray(p0, float)),
                              np.atleast_2d(np.asarray(p1, float)), prisms)


# ---------------------------------------------------------------- point queries


def test_inside_prism_basic():
    pos = np.array([
        [5.0, 5.0, 10.0],     # inside
        [5.0, 5.0, 35.0],     # above roof
        [-1.0, 5.0, 10.0],    # outside xy
        [5.0, 5.0, -0.5],     # below ground
        [10.0, 10.0, 30.0],   # on the corner/roof boundary (closed)
    ])
    np.testing.assert_array_equal(col.inside_prisms(pos, PRISM),
                                  [True, False, False, False, True])


# -------------------------------------------------------------- segment queries


def test_segment_hits_wall_at_half():
    hit, t, point = seg([-5.0, 5.0, 10.0], [5.0, 5.0, 10.0])
    assert hit[0] and abs(t[0] - 0.5) < 1e-12
    np.testing.assert_allclose(point[0], [0.0, 5.0, 10.0], atol=1e-12)


def test_segment_over_roof_misses():
    hit, _, _ = seg([-5.0, 5.0, 35.0], [15.0, 5.0, 35.0])
    assert not hit[0]


def test_segment_descends_through_roof():
    hit, t, point = seg([5.0, 5.0, 40.0], [5.0, 5.0, 20.0])
    assert hit[0] and abs(t[0] - 0.5) < 1e-12
    np.testing.assert_allclose(point[0], [5.0, 5.0, 30.0], atol=1e-12)


def test_segment_starts_inside_hits_at_zero():
    hit, t, _ = seg([5.0, 5.0, 10.0], [20.0, 5.0, 10.0])
    assert hit[0] and t[0] == 0.0


def test_segment_axis_parallel_outside_misses():
    # moves parallel to the x slab, fixed y outside the rect; degenerate dy=0
    hit, _, _ = seg([-5.0, 20.0, 10.0], [15.0, 20.0, 10.0])
    assert not hit[0]


def test_terrain_crossing():
    hit, t, point = col.segment_terrain(np.array([[0.0, 0.0, 5.0]]),
                                        np.array([[0.0, 0.0, -5.0]]))
    assert hit[0] and abs(t[0] - 0.5) < 1e-12
    np.testing.assert_allclose(point[0], [0.0, 0.0, 0.0], atol=1e-12)
    hit, t, _ = col.segment_terrain(np.array([[0.0, 0.0, -1.0]]),
                                    np.array([[0.0, 0.0, 5.0]]))
    assert hit[0] and t[0] == 0.0          # already underground
    hit, _, _ = col.segment_terrain(np.array([[0.0, 0.0, 5.0]]),
                                    np.array([[0.0, 0.0, 1.0]]))
    assert not hit[0]


def test_first_collision_picks_nearest():
    prisms = np.array([
        [20.0, -5.0, 30.0, 5.0, 50.0],
        [40.0, -5.0, 50.0, 5.0, 50.0],
    ])
    p0 = np.array([[0.0, 0.0, 10.0]])
    p1 = np.array([[100.0, 0.0, 10.0]])
    kind, t, point = col.first_collision(p0, p1, prisms)
    assert kind[0] == col.KIND_PRISM and abs(t[0] - 0.2) < 1e-12
    # diving path hits the ground before reaching the far prism
    p1 = np.array([[100.0, 0.0, -90.0]])
    kind, t, _ = col.first_collision(p0, p1, prisms)
    assert kind[0] == col.KIND_TERRAIN and abs(t[0] - 0.1) < 1e-12
    # clear sky
    kind, t, _ = col.first_collision(np.array([[0.0, 0.0, 100.0]]),
                                     np.array([[100.0, 0.0, 100.0]]), prisms)
    assert kind[0] == col.KIND_NONE and np.isinf(t[0])


# ------------------------------------------------------------- batch == scalar


def _segment_prism_scalar(p0, p1, prism):
    """Reference slab method, one segment vs one prism."""
    lo = [prism[0], prism[1], 0.0]
    hi = [prism[2], prism[3], prism[4]]
    tmin, tmax = -np.inf, np.inf
    for ax in range(3):
        d = p1[ax] - p0[ax]
        if abs(d) < 1e-30:
            if p0[ax] < lo[ax] or p0[ax] > hi[ax]:
                return None
            continue
        t0, t1 = (lo[ax] - p0[ax]) / d, (hi[ax] - p0[ax]) / d
        if t0 > t1:
            t0, t1 = t1, t0
        tmin, tmax = max(tmin, t0), min(tmax, t1)
    if tmax < max(tmin, 0.0) or tmin > 1.0 or tmax < 0.0:
        return None
    return max(tmin, 0.0)


def test_batch_equals_scalar_reference():
    rng = np.random.default_rng(66)
    n, m = 60, 12
    prisms = np.empty((m, 5))
    xy0 = rng.uniform(-100, 100, size=(m, 2))
    prisms[:, 0:2] = xy0
    prisms[:, 2:4] = xy0 + rng.uniform(5, 40, size=(m, 2))
    prisms[:, 4] = rng.uniform(5, 60, size=m)
    p0 = rng.uniform(-120, 120, size=(n, 3))
    p1 = p0 + rng.normal(scale=40, size=(n, 3))
    p0[:, 2] = rng.uniform(1, 80, size=n)
    p1[:, 2] = p0[:, 2] + rng.normal(scale=20, size=n)
    p1[: n // 6, 1] = p0[: n // 6, 1]          # degenerate dy = 0 cases

    hit, t, _ = col.segment_prisms(p0, p1, prisms)
    for i in range(n):
        ts = [_segment_prism_scalar(p0[i], p1[i], prisms[j]) for j in range(m)]
        ts = [x for x in ts if x is not None]
        if ts:
            assert hit[i]
            assert abs(t[i] - min(ts)) < 1e-12
        else:
            assert not hit[i]
