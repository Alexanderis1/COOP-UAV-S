"""Patrol-pattern geometry for surveillance platforms (PHY-SNT-001..004).

The MC owns the tactical math (PLAN_PROBLEM1 P4-3), so the patrol geometry
lives here alongside ``mc/guidance.py`` and ``mc/cooperation.py``; the
world-side ``interceptors/patrol.py`` is a thin re-export shim. Keeping it in
``coopuavs.mc`` is also what lets ``mc/sentinel_app.py`` use it without
reaching across the MC import fence into the simulator
(``tests/test_coopfc_fence.py``).

Two deterministic loop patterns, both parameterised by a single phase in
``[0, 2*pi)`` that advances at the platform's orbit speed so the patrol is
*flown*, not chased, and so multi-sentinel laydowns spread out from a
per-id phase offset without coordination (SIM-003):

``circle``
    The v0.1 surveillance orbit: a ring of ``radius`` about ``center`` at
    ``alt``. ``orbit_waypoint('circle', ...)`` and ``path_offset('circle',
    ...)`` reproduce the exact formulae used in
    :mod:`coopuavs.interceptors.sentinel` since v0.1, so existing scenarios
    stay byte-reproducible.

``racetrack``
    A barrier CAP: a stadium (two straight legs of length ``leg`` joined by
    two ``radius`` half-circle caps) oriented along ``heading_deg``. This is
    the doctrinal forward picket — a long straight leg *across* a threat
    axis keeps the airborne radar's beam on the approach corridor far longer
    per lap than a tight circle, which is what buys the time margin against
    a high-altitude diver (PHY-SNT-004, ``scenarios/high_diver_raid.yaml``).

The phase is loop-length-normalised: advancing it by
``2*pi * orbit_speed * dt / loop_length`` moves the waypoint ``orbit_speed *
dt`` metres along the path for either pattern. For ``circle`` the loop
length is ``2*pi*radius`` so the advance reduces to the v0.1
``orbit_speed/radius * dt`` on the angle directly.
"""

from __future__ import annotations

import numpy as np

PATTERNS = ("circle", "racetrack")


def _heading_axes(heading_deg: float) -> tuple[np.ndarray, np.ndarray]:
    """Forward (along the legs) and right (cap offset) unit axes in ENU.

    ``heading_deg`` is a compass-style bearing: 0 deg points +Y (north),
    90 deg points +X (east), matching the rest of the sim's bearing
    convention (``arctan2(x, y)``)."""
    h = np.deg2rad(heading_deg)
    fwd = np.array([np.sin(h), np.cos(h)])      # leg direction
    right = np.array([np.cos(h), -np.sin(h)])   # perpendicular (cap offset)
    return fwd, right


def loop_length(pattern: str, radius: float, leg: float) -> float:
    if pattern == "racetrack":
        return 2.0 * leg + 2.0 * np.pi * radius
    return 2.0 * np.pi * radius


def orbit_waypoint(
    pattern: str,
    center: np.ndarray,
    radius: float,
    alt: float,
    phase: float,
    *,
    heading_deg: float = 0.0,
    leg: float = 0.0,
    lead: float = 0.15,
) -> np.ndarray:
    """Patrol waypoint at ``phase`` (+ a small ``lead`` so the loop is led,
    not chased), as an ENU ``[x, y, alt]`` point."""
    center = np.asarray(center, dtype=float)[:2]
    if pattern == "circle":
        ang = phase + lead
        return np.array([
            center[0] + radius * np.sin(ang),
            center[1] + radius * np.cos(ang),
            alt,
        ])

    # racetrack: map the lead-adjusted phase to an arc length along the loop.
    L = loop_length("racetrack", radius, leg)
    s = ((phase + lead) / (2.0 * np.pi)) % 1.0 * L
    cap = np.pi * radius
    half = 0.5 * leg
    if s < leg:                                   # straight A (v = +radius)
        u, v = -half + s, radius
    elif s < leg + cap:                           # cap at +u
        phi = (s - leg) / radius
        u, v = half + radius * np.sin(phi), radius * np.cos(phi)
    elif s < 2.0 * leg + cap:                      # straight B (v = -radius)
        u, v = half - (s - leg - cap), -radius
    else:                                          # cap at -u
        phi = (s - 2.0 * leg - cap) / radius
        u, v = -half - radius * np.sin(phi), -radius * np.cos(phi)
    fwd, right = _heading_axes(heading_deg)
    xy = center + u * fwd + v * right
    return np.array([xy[0], xy[1], alt])


def path_offset(
    pattern: str,
    center: np.ndarray,
    radius: float,
    pos_xy: np.ndarray,
    *,
    heading_deg: float = 0.0,
    leg: float = 0.0,
) -> float:
    """Horizontal distance from ``pos_xy`` to the nearest point on the
    patrol loop — the on-station test's range term."""
    center = np.asarray(center, dtype=float)[:2]
    pos_xy = np.asarray(pos_xy, dtype=float)[:2]
    if pattern == "circle":
        return abs(float(np.linalg.norm(pos_xy - center)) - radius)

    # racetrack: distance to the stadium boundary in local (u, v) coords.
    fwd, right = _heading_axes(heading_deg)
    rel = pos_xy - center
    u = float(rel @ fwd)
    v = float(rel @ right)
    half = 0.5 * leg
    if abs(u) <= half:                       # alongside a straight leg
        return abs(abs(v) - radius)
    cu = np.sign(u) * half                    # nearest cap centre
    return abs(float(np.hypot(u - cu, v)) - radius)
