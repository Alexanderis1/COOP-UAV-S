"""Cooperative interception geometry.

This module is the project's first innovation pillar: how a team of
interceptors beats a *faster*, non-cooperating target. Two primitives:

Relay/cutoff ("cornering")
    A single slower interceptor loses a tail chase, but the target's mission
    constrains it to a predictable corridor toward its asset. Blockers are
    therefore posted at future points of the predicted corridor (Apollonius
    logic: pick the corridor points each blocker can reach *before* the
    target). The target flies into the engagement instead of being chased.

Herding bias
    The ROE prefers wrecks over SAFE ground. Support UAVs position on the
    flank of the target opposite the designated kill box; the planned
    upgrade is reactive evader models where pressure actually displaces the
    trajectory. With non-reactive targets, flank posts still put a second
    shooter in envelope on the kill-box side.

Both return *desired positions*; the UAV agent flies them with goto/pursuit
guidance. Upgrades (differential-game encirclement, learned policies) slot
in behind the same interface — see docs/RESEARCH.md §1.
"""

from __future__ import annotations

import numpy as np

from ..core.messages import Track
from . import apollonius
from .guidance import intercept_time


def cutoff_points(
    track: Track,
    n_blockers: int,
    blocker_positions: list[np.ndarray],
    blocker_speeds: list[float] | float,
    horizon: float = 120.0,
    spacing: float = 15.0,
) -> list[np.ndarray]:
    """Reachable future points on the target's predicted corridor — the
    cooperative-relay blocker posts.

    Now computed with proper **Apollonius-circle** rendezvous geometry
    (:func:`coopuavs.mc.apollonius.containment_posts`): each blocker's post is
    its exact closed-form intercept point on the target's predicted corridor
    — the point where blocker and target arrive simultaneously, i.e. where
    the corridor crosses the blocker's Apollonius surface — replacing the
    v0.1 fixed-step ``tau`` search. Blockers are spread down-corridor so a
    miss at one post hands the target to the next (the relay), and a blocker
    that cannot rendezvous on the corridor falls back to the deepest free
    down-corridor slot. ``blocker_speeds`` is one speed per blocker (a scalar
    is broadcast); reachability uses each airframe's own capability.

    For a genuinely *manoeuvring* evader (FPV/Lancet), the area-minimising
    containment arc (:func:`coopuavs.mc.apollonius.containment_arc`) closes
    the lateral escape gap instead — see docs/RESEARCH.md §1.
    """
    return apollonius.containment_posts(
        track, n_blockers, blocker_positions, blocker_speeds, horizon, spacing)


def herding_post(
    track: Track,
    kill_box: np.ndarray,
    standoff: float = 250.0,
) -> np.ndarray:
    """Flank position opposite the kill box, slightly behind the target."""
    away = track.position[:2] - kill_box[:2]
    n = np.linalg.norm(away)
    away = away / n if n > 1e-6 else np.array([1.0, 0.0])
    v = track.velocity[:2]
    speed = np.linalg.norm(v)
    back = -v / speed if speed > 1e-6 else np.zeros(2)
    post_xy = track.position[:2] + away * standoff + back * 0.4 * standoff
    return np.array([post_xy[0], post_xy[1], track.position[2]])


def catchable(track: Track, own_pos: np.ndarray, own_speed: float) -> bool:
    return intercept_time(track.position - own_pos, track.velocity, own_speed) is not None
