"""Guidance laws for drone-on-drone interception.

The base law is lead pursuit against the predicted intercept point (PIP):
solve the constant-velocity intercept triangle for time-to-go, aim at where
the target will be. Equivalent to proportional navigation in the
small-manoeuvre limit but maps directly onto a velocity-command point-mass
airframe. True PN with lateral-acceleration commands arrives with the
higher-fidelity dynamics (see ROADMAP).
"""

from __future__ import annotations

import numpy as np


def intercept_time(rel_pos: np.ndarray, target_vel: np.ndarray, own_speed: float) -> float | None:
    """Smallest t >= 0 with |rel_pos + target_vel*t| = own_speed*t.

    None means the target cannot be caught from here at this speed — the
    geometric fact cooperative blocking exists to overcome.
    """
    a = float(target_vel @ target_vel) - own_speed**2
    b = 2.0 * float(rel_pos @ target_vel)
    c = float(rel_pos @ rel_pos)
    if abs(a) < 1e-9:
        if abs(b) < 1e-9:
            return None
        t = -c / b
        return t if t >= 0.0 else None
    disc = b * b - 4 * a * c
    if disc < 0.0:
        return None
    sq = np.sqrt(disc)
    candidates = [t for t in ((-b - sq) / (2 * a), (-b + sq) / (2 * a)) if t >= 0.0]
    return min(candidates) if candidates else None


def predicted_intercept_point(
    own_pos: np.ndarray, target_pos: np.ndarray, target_vel: np.ndarray, own_speed: float
) -> tuple[np.ndarray, float | None]:
    t_go = intercept_time(target_pos - own_pos, target_vel, own_speed)
    if t_go is None:
        # Uncatchable head-on chase: aim ahead of the target anyway (tail
        # chase / wait for blockers).
        return target_pos + target_vel * 2.0, None
    return target_pos + target_vel * t_go, t_go


def pursuit_velocity(
    own_pos: np.ndarray, target_pos: np.ndarray, target_vel: np.ndarray, own_speed: float
) -> np.ndarray:
    aim, _ = predicted_intercept_point(own_pos, target_pos, target_vel, own_speed)
    direction = aim - own_pos
    n = np.linalg.norm(direction)
    if n < 1e-6:
        return np.zeros(3)
    return direction / n * own_speed


def terminal_pursuit_velocity(
    own_pos: np.ndarray, target_pos: np.ndarray, target_vel: np.ndarray, own_speed: float
) -> np.ndarray:
    """Terminal-phase pure pursuit: steer at the target itself, not the PIP.

    The effector's off-axis gate measures the angle between *own velocity*
    and the target line of sight — flying at the lead point keeps that
    angle large through the endgame and hard-zeroes Pk exactly when the
    shot should happen. Inside effector range, pointing the velocity
    vector down the sight line (with a small kinematic lead so a crossing
    target doesn't outwalk the closure) is what fills the envelope.
    """
    rel = target_pos - own_pos
    rng = float(np.linalg.norm(rel))
    if rng < 1e-6:
        return np.zeros(3)
    lead_t = 0.2 * rng / max(own_speed, 1.0)
    direction = rel + target_vel * lead_t
    n = float(np.linalg.norm(direction))
    if n < 1e-6:
        return np.zeros(3)
    return direction / n * own_speed


def goto_velocity(own_pos: np.ndarray, waypoint: np.ndarray, speed: float, arrive_radius: float = 20.0) -> np.ndarray:
    direction = waypoint - own_pos
    dist = float(np.linalg.norm(direction))
    if dist < 1e-6:
        return np.zeros(3)
    v = speed if dist > arrive_radius else speed * dist / arrive_radius
    return direction / dist * v
