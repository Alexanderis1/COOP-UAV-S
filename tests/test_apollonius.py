"""Apollonius-circle cooperative interception geometry (RESEARCH.md §1)."""

import numpy as np

from coopuavs.core.messages import Header, Track
from coopuavs.mc import apollonius


def _track(pos, vel):
    return Track(header=Header(stamp=0.0), track_id=1,
                 position=np.asarray(pos, float), velocity=np.asarray(vel, float))


# -- the circle ------------------------------------------------------------

def test_circle_ratio_invariant():
    """Every point on the returned circle has |x-E|/|x-P| == gamma."""
    E, P = np.array([0.0, 0.0]), np.array([1000.0, 0.0])
    for gamma in (0.5, 0.8, 1.5, 2.5):
        center, radius = apollonius.apollonius_circle(E, P, gamma)
        for theta in np.linspace(0, 2 * np.pi, 16, endpoint=False):
            x = center + radius * np.array([np.cos(theta), np.sin(theta)])
            ratio = np.linalg.norm(x - E) / np.linalg.norm(x - P)
            assert abs(ratio - gamma) < 1e-6, f"gamma={gamma} ratio={ratio}"


def test_circle_degenerate_at_equal_speed():
    assert apollonius.apollonius_circle([0, 0], [10, 0], 1.0) is None


def test_slower_pursuer_dominance_disk_encloses_pursuer():
    """gamma>1 (slower pursuer): the dominance region is a finite disk around
    the pursuer; the evader dominates far away."""
    E, P, v_e, v_p = np.array([0.0, 0, 0]), np.array([1000.0, 0, 0]), 100.0, 60.0
    assert apollonius.dominates(P, E, P, v_e, v_p)           # at the pursuer
    assert not apollonius.dominates(E, E, P, v_e, v_p)       # at the evader
    far = np.array([5000.0, 0, 0])
    assert not apollonius.dominates(far, E, P, v_e, v_p)     # evader's ground


# -- the intercept (rendezvous) --------------------------------------------

def test_intercept_is_a_simultaneous_arrival_point():
    """The intercept point on the corridor is where blocker and target arrive
    at the same instant: s/v_e == |Q-P|/v_p."""
    E = np.array([0.0, 3000.0, 800.0])
    v_e = 100.0
    u = np.array([0.0, -1.0, 0.0])               # heading -y (toward origin)
    for P in (np.array([0.0, 1500.0, 800.0]), np.array([200.0, 1000.0, 400.0])):
        for v_p in (80.0, 120.0):
            s = apollonius.intercept_depth(E, u, P, v_e / v_p, max_depth=5000.0)
            assert s is not None, f"P={P} v_p={v_p}: expected a rendezvous"
            Q = E + s * u
            t_evader = s / v_e
            t_pursuer = float(np.linalg.norm(Q - P)) / v_p
            assert abs(t_evader - t_pursuer) < 1e-3, "not simultaneous arrival"


def test_unreachable_blocker_returns_none():
    """A near-stationary blocker off the corridor cannot rendezvous."""
    E = np.array([0.0, 3000.0, 800.0])
    u = np.array([0.0, -1.0, 0.0])
    P = np.array([4000.0, 3000.0, 0.0])          # far to the side, on the ground
    assert apollonius.intercept_depth(E, u, P, 100.0 / 1.0, max_depth=4000.0) is None


def test_altitude_gap_reduces_reachability():
    """A blocker forced to climb to the corridor meets the target deeper
    down-corridor than a co-altitude one (3D ranges). Use a faster pursuer so
    both can rendezvous and the altitude penalty shows as a later meeting."""
    E = np.array([0.0, 3000.0, 3000.0]); u = np.array([0.0, -1.0, 0.0]); v_e = 100.0
    low = np.array([0.0, 1500.0, 200.0])
    co_alt = np.array([0.0, 1500.0, 3000.0])
    s_low = apollonius.intercept_depth(E, u, low, v_e / 150.0, 6000.0)
    s_hi = apollonius.intercept_depth(E, u, co_alt, v_e / 150.0, 6000.0)
    assert s_low is not None and s_hi is not None
    assert s_low > s_hi      # climbing -> deeper (later) interception
    # And a slower pursuer well below the corridor cannot reach it at all.
    assert apollonius.intercept_depth(E, u, low, v_e / 80.0, 6000.0) is None


# -- relay posts (cutoff_points delegate) ----------------------------------

def test_containment_posts_relay_and_fallback():
    trk = _track([0, 3000, 800], [0, -100, 0])
    pos = np.array([0.0, 1500.0, 800.0])
    fast = apollonius.containment_posts(trk, 1, [pos], [120.0])
    slow = apollonius.containment_posts(trk, 1, [pos], [1.0])
    # the catchable fast blocker posts on the corridor near the target; the
    # near-stationary one cannot rendezvous and falls back far down-corridor
    assert (np.linalg.norm(fast[0] - trk.position)
            < np.linalg.norm(slow[0] - trk.position))
    # two blockers do not stack on the same point (relay spread)
    two = apollonius.containment_posts(trk, 2, [pos, pos.copy()], [120.0, 120.0])
    assert np.linalg.norm(two[0] - two[1]) > 1.0


# -- containment / escape-set diagnostics ----------------------------------

def test_safe_fraction_bounds_and_monotonicity():
    E, goal, v_e = np.array([0.0, 0, 0]), np.array([0.0, 3000, 0]), 100.0
    open_field = apollonius.evader_safe_fraction(E, v_e, [], [], goal)
    assert open_field == 1.0                      # no pursuers -> fully open
    # a blocker planted on the path closes part of the escape arc
    blocked = apollonius.evader_safe_fraction(
        E, v_e, [np.array([0.0, 1200, 0])], [90.0], goal)
    assert 0.0 <= blocked < open_field


def test_containment_arc_closes_the_gap():
    """Posting blockers on the Apollonius containment arc lowers the evader's
    escape fraction vs leaving them behind the evader."""
    trk = _track([0, 0, 0], [0, 100, 0])          # heading +y
    goal = np.array([0.0, 4000.0, 0.0])
    speeds = [70.0, 70.0, 70.0]
    arc = apollonius.containment_arc(trk, 3, speeds, goal=goal)
    behind = [np.array([0.0, -1500.0, 0.0])] * 3   # trailing the evader
    f_arc = apollonius.evader_safe_fraction(trk.position, 100.0, arc, speeds, goal)
    f_behind = apollonius.evader_safe_fraction(trk.position, 100.0, behind, speeds, goal)
    assert f_arc < f_behind, f"arc {f_arc} should contain better than behind {f_behind}"
    assert len(arc) == 3
