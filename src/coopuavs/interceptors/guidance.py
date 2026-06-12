"""Guidance laws for drone-on-drone interception.

The base law is lead pursuit against the predicted intercept point (PIP):
solve the constant-velocity intercept triangle for time-to-go, aim at where
the target will be. Equivalent to proportional navigation in the
small-manoeuvre limit but maps directly onto a velocity-command point-mass
airframe.

:func:`pro_nav_accel` is true PN — a lateral-acceleration command
``a = N * Vc * lambda_dot`` perpendicular to the line of sight — for
airframes flown through ``command_acceleration`` (the load-factor body).
Against a manoeuvring target it corrects continuously from the LOS rate
where PIP lead pursuit re-aims through the velocity-command lag.
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


def pro_nav_accel(
    own_pos: np.ndarray,
    own_vel: np.ndarray,
    target_pos: np.ndarray,
    target_vel: np.ndarray,
    nav_gain: float = 4.0,
) -> np.ndarray:
    """True proportional navigation: ``a = N * Vc * lambda_dot``, applied
    perpendicular to the line of sight (3D vector form).

    Returns the zero vector when the geometry is degenerate (zero range)
    or opening — PN only steers a closing engagement; re-acquisition is
    pursuit's job.
    """
    r = target_pos - own_pos
    rr = float(r @ r)
    if rr < 1e-9:
        return np.zeros(3)
    r_hat = r / np.sqrt(rr)
    v_rel = target_vel - own_vel
    closing = -float(v_rel @ r_hat)
    if closing <= 0.0:
        return np.zeros(3)
    omega = np.cross(r, v_rel) / rr        # LOS rotation-rate vector
    return nav_gain * closing * np.cross(omega, r_hat)


def goto_velocity(own_pos: np.ndarray, waypoint: np.ndarray, speed: float, arrive_radius: float = 20.0) -> np.ndarray:
    direction = waypoint - own_pos
    dist = float(np.linalg.norm(direction))
    if dist < 1e-6:
        return np.zeros(3)
    v = speed if dist > arrive_radius else speed * dist / arrive_radius
    return direction / dist * v
