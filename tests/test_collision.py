"""P1-6: prism (2.5D building) + flat-terrain collision queries.

Prisms are (M, 5) arrays [xmin, ymin, xmax, ymax, height] matching the
sim's Building contract (rect + height, base at the shared ground_z datum,
default 0); the slab-method segment test catches tunneling at macro-step
speeds. Pins: analytic wall/roof/terrain hits, nearest-hit selection with
point output, start-inside, axis-parallel degenerate segments, closed
lo/hi-face and t=0/t=1 bounds, malformed-prism ValueError, empty prism
sets, nonzero ground_z datum, and vectorized == scalar-reference within
1e-12.
"""

from __future__ import annotations

import numpy as np
import pytest

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


def test_segment_grazing_hi_face_in_plane_hits():
    """Gate-review pin: closed bounds on hi faces too — a graze exactly in
    the y=ymax or z=height plane hits, consistent with inside_prisms (the
    old signed-epsilon trick hit lo-face grazes but missed hi-face ones)."""
    hit, t, _ = seg([-5.0, 10.0, 10.0], [5.0, 10.0, 10.0])   # y = ymax plane
    assert hit[0] and abs(t[0] - 0.5) < 1e-12
    hit, t, _ = seg([-5.0, 0.0, 10.0], [5.0, 0.0, 10.0])     # y = ymin plane
    assert hit[0] and abs(t[0] - 0.5) < 1e-12
    hit, t, _ = seg([-5.0, 5.0, 30.0], [5.0, 5.0, 30.0])     # roof plane
    assert hit[0] and abs(t[0] - 0.5) < 1e-12


def test_stationary_point_queries():
    hit, t, _ = seg([5.0, 5.0, 10.0], [5.0, 5.0, 10.0])      # hovering inside
    assert hit[0] and t[0] == 0.0
    hit, _, _ = seg([20.0, 5.0, 10.0], [20.0, 5.0, 10.0])    # hovering outside
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


def test_inside_prism_lo_faces_closed():
    """Review pin: lo faces are closed too (kills `>=` -> `>` at the point
    query), symmetric with the hi-corner row in test_inside_prism_basic."""
    pos = np.array([
        [0.0, 5.0, 10.0],     # exactly on the x = xmin face
        [5.0, 0.0, 10.0],     # exactly on the y = ymin face
        [5.0, 5.0, 0.0],      # exactly on the ground (z = 0) face
        [0.0, 0.0, 0.0],      # lo corner
    ])
    np.testing.assert_array_equal(col.inside_prisms(pos, PRISM),
                                  [True, True, True, True])


def test_segment_endpoint_on_face_hits_at_t_one():
    """Review pin: contact exactly at t = 1 is a hit (closed `tlo <= 1.0`)."""
    hit, t, point = seg([-10.0, 5.0, 10.0], [0.0, 5.0, 10.0])
    assert hit[0] and abs(t[0] - 1.0) < 1e-12
    np.testing.assert_allclose(point[0], [0.0, 5.0, 10.0], atol=1e-12)


def test_terrain_endpoint_exactly_on_ground_hits():
    """Review pin: z1 == ground_z exactly is a crossing (closed `<=`)."""
    hit, t, point = col.segment_terrain(np.array([[0.0, 0.0, 5.0]]),
                                        np.array([[0.0, 0.0, 0.0]]))
    assert hit[0] and abs(t[0] - 1.0) < 1e-12
    np.testing.assert_allclose(point[0], [0.0, 0.0, 0.0], atol=1e-12)


def test_terrain_degenerate_dz_keeps_t_in_unit_interval():
    """Review pin: the |dz| < eps substitution preserves dz's sign, so a
    near-flat skim onto the ground reports t in [0, 1] (was negative)."""
    hit, t, _ = col.segment_terrain(np.array([[0.0, 0.0, 5e-324]]),
                                    np.array([[0.0, 0.0, 0.0]]))
    assert hit[0] and 0.0 <= t[0] <= 1.0
    # normal (non-denormal) doubles below _EPS_DIR trigger it too
    hit, t, _ = col.segment_terrain(np.array([[0.0, 0.0, 5e-31]]),
                                    np.array([[0.0, 0.0, -4e-31]]))
    assert hit[0] and 0.0 <= t[0] <= 1.0


# ----------------------------------------------------------- prism validation


BAD_PRISMS = {
    "xmax_lt_xmin": [10.0, 0.0, 0.0, 10.0, 30.0],
    "xmax_eq_xmin": [5.0, 0.0, 5.0, 10.0, 30.0],
    "ymax_lt_ymin": [0.0, 10.0, 10.0, 0.0, 30.0],
    "height_zero": [0.0, 0.0, 10.0, 10.0, 0.0],
    "height_negative": [0.0, 0.0, 10.0, 10.0, -30.0],
}


@pytest.mark.parametrize("bad", BAD_PRISMS.values(), ids=BAD_PRISMS.keys())
def test_malformed_prisms_raise_in_every_entry_point(bad):
    """Review pin: inverted/degenerate prisms used to answer three different
    ways (solid to crossing segments, empty to point and degenerate-axis
    queries); now every public entry point raises ValueError."""
    prisms = np.array([bad])
    pos = np.array([[5.0, 5.0, 10.0]])
    p0 = np.array([[-5.0, 5.0, 10.0]])
    p1 = np.array([[15.0, 5.0, 10.0]])
    with pytest.raises(ValueError):
        col.inside_prisms(pos, prisms)
    with pytest.raises(ValueError):
        col.segment_prisms(p0, p1, prisms)
    with pytest.raises(ValueError):
        col.first_collision(p0, p1, prisms)


def test_empty_prism_set_all_entry_points():
    """Review pin: (0, 5) prisms (open-field world) are valid for all three
    entry points; first_collision falls through to terrain."""
    empty = np.empty((0, 5))
    pos = np.array([[5.0, 5.0, 10.0]])
    np.testing.assert_array_equal(col.inside_prisms(pos, empty), [False])
    p0 = np.array([[0.0, 0.0, 10.0], [0.0, 0.0, 10.0]])
    p1 = np.array([[100.0, 0.0, 10.0], [100.0, 0.0, -10.0]])
    hit, t, point = col.segment_prisms(p0, p1, empty)
    assert not hit.any()
    assert np.isinf(t).all()
    np.testing.assert_array_equal(point, p0)
    kind, t, point = col.first_collision(p0, p1, empty)
    assert kind[0] == col.KIND_NONE and np.isinf(t[0])
    assert kind[1] == col.KIND_TERRAIN and abs(t[1] - 0.5) < 1e-12
    np.testing.assert_allclose(point[1], [50.0, 0.0, 0.0], atol=1e-12)


# ----------------------------------------------------- first_collision points


def test_first_collision_point_output():
    """Review pin: the returned POINT follows the winning branch (kills the
    pt_p/pt_g selection swap that kind/t alone cannot see)."""
    prisms = np.array([
        [20.0, -5.0, 30.0, 5.0, 50.0],
        [40.0, -5.0, 50.0, 5.0, 50.0],
    ])
    p0 = np.array([[0.0, 0.0, 10.0]])
    # (a) prism face before terrain
    kind, t, point = col.first_collision(p0, np.array([[100.0, 0.0, 10.0]]),
                                         prisms)
    assert kind[0] == col.KIND_PRISM and abs(t[0] - 0.2) < 1e-12
    np.testing.assert_allclose(point[0], [20.0, 0.0, 10.0], atol=1e-12)
    # (b) terrain before prism (diving path)
    kind, t, point = col.first_collision(p0, np.array([[100.0, 0.0, -90.0]]),
                                         prisms)
    assert kind[0] == col.KIND_TERRAIN and abs(t[0] - 0.1) < 1e-12
    np.testing.assert_allclose(point[0], [10.0, 0.0, 0.0], atol=1e-12)


# ------------------------------------------------------------ nonzero ground_z


def test_nonzero_ground_z_terrain_and_prisms_share_datum():
    """Review pin: ground_z threads through terrain AND prism queries, so a
    building roots on the raised ground (base z = ground_z, roof at
    ground_z + h) instead of being buried at z = 0."""
    gz = 5.0
    # terrain crossing happens at z = ground_z, not z = 0
    hit, t, point = col.segment_terrain(np.array([[0.0, 0.0, 15.0]]),
                                        np.array([[0.0, 0.0, -5.0]]),
                                        ground_z=gz)
    assert hit[0] and abs(t[0] - 0.5) < 1e-12
    np.testing.assert_allclose(point[0], [0.0, 0.0, 5.0], atol=1e-12)
    # a point between z = 0 and ground_z is OUTSIDE the prism volume
    pos = np.array([
        [5.0, 5.0, 2.0],      # below the raised ground -> outside
        [5.0, 5.0, 5.0],      # exactly on the raised base face -> inside
        [5.0, 5.0, 34.0],     # under the raised roof (gz + 30 = 35) -> inside
        [5.0, 5.0, 36.0],     # above the raised roof -> outside
    ])
    np.testing.assert_array_equal(
        col.inside_prisms(pos, PRISM, ground_z=gz),
        [False, True, True, False])
    # roof of the raised building sits at gz + h = 35
    hit, t, point = col.segment_prisms(np.array([[5.0, 5.0, 45.0]]),
                                       np.array([[5.0, 5.0, 25.0]]),
                                       PRISM, ground_z=gz)
    assert hit[0] and abs(t[0] - 0.5) < 1e-12
    np.testing.assert_allclose(point[0], [5.0, 5.0, 35.0], atol=1e-12)


def test_first_collision_nonzero_ground_z_one_datum():
    """Review pin: first_collision passes ground_z into the prism query."""
    gz = 5.0
    # level flight 1 m above the raised ground hits the wall at x = 0
    kind, t, point = col.first_collision(np.array([[-5.0, 5.0, 6.0]]),
                                         np.array([[15.0, 5.0, 6.0]]),
                                         PRISM, ground_z=gz)
    assert kind[0] == col.KIND_PRISM and abs(t[0] - 0.25) < 1e-12
    np.testing.assert_allclose(point[0], [0.0, 5.0, 6.0], atol=1e-12)
    # flight at z = 32 is above the z = 0 roof (30) but below the raised one
    # (35): only a ground_z-threaded prism query reports the wall hit
    kind, t, point = col.first_collision(np.array([[-5.0, 5.0, 32.0]]),
                                         np.array([[15.0, 5.0, 32.0]]),
                                         PRISM, ground_z=gz)
    assert kind[0] == col.KIND_PRISM and abs(t[0] - 0.25) < 1e-12
    np.testing.assert_allclose(point[0], [0.0, 5.0, 32.0], atol=1e-12)
    # a path between z = 0 and ground_z starts underground: terrain at t = 0
    kind, t, _ = col.first_collision(np.array([[-5.0, 5.0, 2.0]]),
                                     np.array([[15.0, 5.0, 2.0]]),
                                     PRISM, ground_z=gz)
    assert kind[0] == col.KIND_TERRAIN and t[0] == 0.0
    # diving path strikes the raised terrain before reaching the prism wall
    kind, t, point = col.first_collision(np.array([[-20.0, 5.0, 8.0]]),
                                         np.array([[20.0, 5.0, 0.0]]),
                                         PRISM, ground_z=gz)
    assert kind[0] == col.KIND_TERRAIN and abs(t[0] - 0.375) < 1e-12
    np.testing.assert_allclose(point[0], [-5.0, 5.0, 5.0], atol=1e-12)


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
