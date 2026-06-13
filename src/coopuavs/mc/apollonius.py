"""Apollonius-circle cooperative interception geometry (RESEARCH.md §1).

The proper geometric core behind the project's first innovation pillar —
beating a *faster*, non-cooperating target with a slower team. The
**Apollonius circle** of an evader ``E`` and a pursuer ``P`` is the locus

    { x : |x - E| / |x - P| = gamma },   gamma = v_E / v_P

i.e. the points both reach at the same instant. It splits the plane into the
pursuer's **dominance region** (it arrives first) and the evader's. For a
slower pursuer (``gamma > 1``) the dominance region is a finite disk around
the pursuer; a single slow pursuer therefore dominates only locally and a
smart faster evader escapes — capture is inherently a *cooperative* problem
(Isaacs; Garcia/Von Moll/Pachter, AFRL).

Two uses, matched to the threat (RESEARCH.md §1 caveat — a Shahed/jet OWA is
*not* an optimal evader; it flies a pre-planned, lightly-manoeuvring route
with a terminal dive, so the game degenerates to rendezvous):

* :func:`containment_posts` — **rendezvous relay** for ballistic threats: the
  exact Apollonius intercept point of each blocker on the target's predicted
  corridor (closed form, replacing the v0.1 time-stepping search), with a
  down-corridor relay so a miss at one post hands the target to the next.
  This is the operative path for the fast jet OWA and is what
  ``cooperation.cutoff_points`` now uses.
* :func:`containment_arc` + :func:`evader_safe_fraction` — the
  game-theoretic **containment** machinery for a genuinely manoeuvring
  evader: distribute blockers across the approach so the union of their
  dominance disks closes the escape gap (no Apollonius gap), and measure the
  remaining escape (the area-minimisation objective of the 2025 AFRL line).

Geometry is 3D where it matters (a low blocker must climb to the corridor,
so reachability uses full 3D ranges); the planar :func:`apollonius_circle`
and the safe-set diagnostics work in the horizontal plane.
"""

from __future__ import annotations

import numpy as np

_EPS = 1e-9


def apollonius_circle(evader, pursuer, gamma: float):
    """Planar Apollonius circle ``(center_xy, radius)`` for ratio ``gamma =
    v_E/v_P``. Returns ``None`` when ``gamma == 1`` (the locus degenerates to
    the perpendicular bisector — a line, not a circle)."""
    E = np.asarray(evader, dtype=float)[:2]
    P = np.asarray(pursuer, dtype=float)[:2]
    if abs(gamma - 1.0) < 1e-6:
        return None
    denom = 1.0 - gamma * gamma
    center = (E - gamma * gamma * P) / denom
    radius = gamma * float(np.linalg.norm(E - P)) / abs(denom)
    return center, radius


def dominates(x, evader, pursuer, v_e: float, v_p: float) -> bool:
    """True if the pursuer reaches ``x`` no later than the evader (i.e. ``x``
    is in the pursuer's dominance region)."""
    x = np.asarray(x, dtype=float)
    te = float(np.linalg.norm(x - np.asarray(evader, dtype=float))) / max(v_e, _EPS)
    tp = float(np.linalg.norm(x - np.asarray(pursuer, dtype=float))) / max(v_p, _EPS)
    return tp <= te


def intercept_depth(evader, heading, pursuer, gamma: float,
                    max_depth: float) -> float | None:
    """Distance ``s`` (metres, in ``[0, max_depth]``) along unit ``heading``
    from ``evader`` to the earliest point where the pursuer can rendezvous —
    where the evader's straight path crosses the Apollonius surface (both
    arrive simultaneously). ``None`` if the path never enters the pursuer's
    dominance region within ``max_depth`` (the pursuer cannot intercept on
    the predicted corridor). 3D.
    """
    E = np.asarray(evader, dtype=float)
    P = np.asarray(pursuer, dtype=float)
    u = np.asarray(heading, dtype=float)
    w = E - P
    wu = float(w @ u)
    ww = float(w @ w)
    g2 = gamma * gamma
    a = 1.0 - g2
    if abs(a) < 1e-9:                       # gamma == 1: linear in s
        if abs(wu) < _EPS:
            return None
        s = -ww / (2.0 * wu)
        return s if 0.0 < s <= max_depth + 1e-6 else None
    b = -2.0 * g2 * wu
    c = -g2 * ww
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return None
    sq = float(np.sqrt(disc))
    roots = sorted(s for s in ((-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a))
                   if s > 1e-6 and s <= max_depth + 1e-6)
    return roots[0] if roots else None      # earliest simultaneous-arrival point


def intercept_point(evader, heading, pursuer, gamma: float,
                    max_depth: float):
    """The rendezvous point ``E + s*heading`` (or ``None``) for
    :func:`intercept_depth`."""
    s = intercept_depth(evader, heading, pursuer, gamma, max_depth)
    if s is None:
        return None
    return np.asarray(evader, dtype=float) + s * np.asarray(heading, dtype=float)


def containment_posts(track, n_blockers: int, blocker_positions, blocker_speeds,
                      horizon: float = 120.0, spacing: float = 15.0):
    """Apollonius rendezvous-relay posts (drop-in for the v0.1
    ``cutoff_points``).

    For each blocker (in order, so ``posts[i]`` is blocker ``i``'s) take its
    exact Apollonius intercept point on the target's predicted corridor; push
    it down-corridor to the next free slot (``spacing`` seconds apart) so the
    blockers form a relay rather than stacking on one point. A blocker that
    cannot rendezvous on the corridor falls back to the deepest free
    down-corridor slot — pressure / a later relay leg — like the heuristic it
    replaces. ``blocker_speeds`` is one speed per blocker (a scalar is
    broadcast): reachability is tested with each airframe's own capability.
    """
    p = np.asarray(track.position, dtype=float)
    v = np.asarray(track.velocity, dtype=float)
    v_e = float(np.linalg.norm(v))
    if isinstance(blocker_speeds, (int, float)):
        blocker_speeds = [float(blocker_speeds)] * n_blockers
    if v_e < 1.0:                           # no heading: hold on the target
        return [p.copy() for _ in range(n_blockers)]
    u = v / v_e
    max_depth = horizon * v_e               # corridor depth budget, metres
    spacing_m = spacing * v_e               # min post separation, metres

    posts: list[np.ndarray] = []
    claimed: list[float] = []
    for own, v_p in zip(blocker_positions[:n_blockers], blocker_speeds):
        gamma = v_e / max(float(v_p), _EPS)
        s = intercept_depth(p, u, np.asarray(own, dtype=float), gamma, max_depth)
        if s is None:
            # cannot meet on the corridor: take the deepest free slot.
            s = min(max_depth, (claimed[-1] + spacing_m) if claimed else max_depth)
        else:
            # relay: avoid stacking two blockers on the same point.
            while any(abs(s - c) < spacing_m for c in claimed) and s < max_depth:
                s += spacing_m
            s = min(s, max_depth)
        claimed.append(s)
        posts.append(p + u * s)
    return posts


# --- game-theoretic containment (manoeuvring evader) -----------------------

def _perp(u: np.ndarray) -> np.ndarray:
    """Horizontal unit vector perpendicular to heading ``u``."""
    perp = np.array([-u[1], u[0], 0.0])
    n = float(np.linalg.norm(perp))
    return perp / n if n > _EPS else np.array([1.0, 0.0, 0.0])


def _p3(x) -> np.ndarray:
    """Coerce a 2D/3D position to a 3D vector."""
    a = np.asarray(x, dtype=float)
    return a if a.size >= 3 else np.array([a[0], a[1], 0.0])


def containment_arc(track, n_blockers: int, blocker_speeds,
                    goal=None, horizon: float = 60.0):
    """Blocker posts on an arc across the approach corridor, sized so the
    union of their dominance disks closes the lateral escape gap — the
    *containment phase* for a manoeuvring evader (FPV/Lancet) rather than the
    ballistic relay. Posts are spread symmetrically about the evader->goal
    axis at a common reachable barrier depth.

    Returns ``n_blockers`` posts. This is the cooperative-mode tool exposed
    for the evasive-threat branch and for seeding the learned policy; the
    default blocking path (:func:`containment_posts`) handles the
    non-reactive threats. ``goal`` defaults to the down-corridor heading point.
    """
    p = np.asarray(track.position, dtype=float)
    v = np.asarray(track.velocity, dtype=float)
    v_e = float(np.linalg.norm(v))
    if isinstance(blocker_speeds, (int, float)):
        blocker_speeds = [float(blocker_speeds)] * n_blockers
    speeds = list(blocker_speeds[:n_blockers])
    if v_e < 1.0 or not speeds:
        return [p.copy() for _ in range(n_blockers)]
    u = v / v_e
    # Barrier depth: bounded by the slowest blocker's reach so every post is
    # makeable before the evader arrives.
    slow = min(speeds)
    depth = v_e * horizon * min(1.0, slow / max(v_e, _EPS))
    barrier = p + u * depth
    perp = _perp(u)
    posts = []
    for k, v_p in enumerate(speeds):
        gamma = v_e / max(float(v_p), _EPS)
        rank = (k + 1) // 2
        side = 1.0 if k % 2 == 1 else -1.0           # 0, +1, -1, +2, -2, ...
        posts.append(barrier + perp * side * rank * _arc_spacing(gamma, depth))
    return posts


def _arc_spacing(gamma: float, depth: float) -> float:
    """Lateral slot spacing ~ the dominance-disk size at the barrier, so
    adjacent blockers' Apollonius disks abut (no escape gap). Bounded."""
    margin = max(1e-3, abs(1.0 - 1.0 / max(gamma, _EPS)))
    return float(np.clip(depth * margin, 120.0, 600.0))


def evader_safe_fraction(evader, v_e: float, pursuers, pursuer_speeds,
                         goal, n_samples: int = 72) -> float:
    """Fraction of the evader's forward escape directions (sampled +/-90 deg
    about the bearing to ``goal``) on which the evader reaches the test radius
    before *every* pursuer — the size of the un-closed Apollonius gap. ``0.0``
    means fully contained. The scalar area-minimisation objective (lower is
    better) a containment controller or a learned policy descends.
    """
    E = _p3(evader)
    goal = _p3(goal)
    to_goal = goal - E
    base = (float(np.arctan2(to_goal[0], to_goal[1]))
            if float(np.linalg.norm(to_goal)) > _EPS else 0.0)
    radius = float(np.linalg.norm(to_goal)) or 1000.0
    if isinstance(pursuer_speeds, (int, float)):
        pursuer_speeds = [float(pursuer_speeds)] * len(pursuers)
    pur = [(_p3(P), max(float(vp), _EPS)) for P, vp in zip(pursuers, pursuer_speeds)]
    te = radius / max(v_e, _EPS)
    free = 0
    for k in range(n_samples):
        ang = base + (k / n_samples - 0.5) * np.pi
        x = E + radius * np.array([np.sin(ang), np.cos(ang), 0.0])
        if all(float(np.linalg.norm(x - P)) / vp > te for P, vp in pur):
            free += 1
    return free / n_samples
