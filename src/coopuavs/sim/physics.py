"""Flight dynamics for the time-stepped simulation.

Two bodies behind one seam (:class:`AirframeBody`):

* :class:`PointMass` — physics-lite kinematics with isotropic speed and
  acceleration limits, the v0.1 baseline and the default everywhere;
* :class:`LoadFactorBody` — 3-DOF airframe whose lateral acceleration is
  bounded by a structural load factor (``n_max`` g), so turn rate falls
  with airspeed (``omega = n_max * g / v``) exactly as SIM-PHX-001
  requires. Deliberately attitude-free: not stiff, runs at the world
  ``dt``, keeps Monte-Carlo throughput.

Agents talk to either body through the same four members — command in,
state out — which is also the adapter boundary a future PX4 SITL body
implements (SIM-PHX-005).
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

import numpy as np

GRAVITY = 9.81


@runtime_checkable
class AirframeBody(Protocol):
    """The seam every vehicle agent flies through.

    ``command_velocity`` and ``command_acceleration`` are mutually
    exclusive: the most recent call defines the command consumed by the
    next ``step``.
    """

    position: np.ndarray
    velocity: np.ndarray

    def command_velocity(self, v_cmd: np.ndarray) -> None: ...

    def command_acceleration(self, a_cmd: np.ndarray) -> None: ...

    def step(self, dt: float) -> None: ...


class PointMass:
    """Point-mass vehicle with saturated acceleration toward a commanded
    velocity vector."""

    def __init__(
        self,
        position: np.ndarray,
        velocity: np.ndarray | None = None,
        max_speed: float = 30.0,
        max_accel: float = 10.0,
    ):
        self.position = np.asarray(position, dtype=float).copy()
        self.velocity = (
            np.asarray(velocity, dtype=float).copy() if velocity is not None else np.zeros(3)
        )
        self.max_speed = max_speed
        self.max_accel = max_accel
        self.cmd_velocity = self.velocity.copy()
        self._a_cmd: np.ndarray | None = None

    def command_velocity(self, v_cmd: np.ndarray) -> None:
        speed = np.linalg.norm(v_cmd)
        if speed > self.max_speed:
            v_cmd = v_cmd * (self.max_speed / speed)
        self.cmd_velocity = np.asarray(v_cmd, dtype=float)
        self._a_cmd = None

    def command_acceleration(self, a_cmd: np.ndarray) -> None:
        self._a_cmd = np.asarray(a_cmd, dtype=float)

    def step(self, dt: float) -> None:
        if self._a_cmd is not None:
            a = self._a_cmd
            a_norm = float(np.linalg.norm(a))
            if a_norm > self.max_accel:
                a = a * (self.max_accel / a_norm)
            self.velocity = self.velocity + a * dt
            speed = float(np.linalg.norm(self.velocity))
            if speed > self.max_speed:
                self.velocity = self.velocity * (self.max_speed / speed)
            self.position = self.position + self.velocity * dt
            return
        dv = self.cmd_velocity - self.velocity
        dv_norm = np.linalg.norm(dv)
        max_dv = self.max_accel * dt
        if dv_norm > max_dv:
            dv = dv * (max_dv / dv_norm)
        self.velocity = self.velocity + dv
        self.position = self.position + self.velocity * dt


class LoadFactorBody:
    """3-DOF airframe with a structural load-factor limit (SIM-PHX-001).

    The commanded acceleration is split along the velocity tangent:
    longitudinal accel/decel are bounded by thrust/brake limits, lateral
    acceleration by ``n_max`` g — so achievable turn rate degrades with
    airspeed, which is the physical fact the point mass cannot express.
    No attitude state: the model is not stiff and integrates at the world
    ``dt``.

    ``min_speed`` is a stall floor for fixed-wing airframes; the default
    of 0 keeps hover-capable behaviour (pad hold, REARM) intact.
    """

    def __init__(
        self,
        position: np.ndarray,
        velocity: np.ndarray | None = None,
        max_speed: float = 30.0,
        n_max: float = 4.0,
        max_long_accel: float = 10.0,
        max_long_decel: float | None = None,
        min_speed: float = 0.0,
        vel_cmd_tau: float = 0.3,
    ):
        self.position = np.asarray(position, dtype=float).copy()
        self.velocity = (
            np.asarray(velocity, dtype=float).copy() if velocity is not None else np.zeros(3)
        )
        self.max_speed = max_speed
        self.n_max = n_max
        self.max_long_accel = max_long_accel
        self.max_long_decel = max_long_decel if max_long_decel is not None else max_long_accel
        self.min_speed = min_speed
        self.vel_cmd_tau = vel_cmd_tau
        self.cmd_velocity = self.velocity.copy()
        self._a_cmd: np.ndarray | None = None

    def command_velocity(self, v_cmd: np.ndarray) -> None:
        speed = np.linalg.norm(v_cmd)
        if speed > self.max_speed:
            v_cmd = v_cmd * (self.max_speed / speed)
        self.cmd_velocity = np.asarray(v_cmd, dtype=float)
        self._a_cmd = None

    def command_acceleration(self, a_cmd: np.ndarray) -> None:
        self._a_cmd = np.asarray(a_cmd, dtype=float)

    def step(self, dt: float) -> None:
        if self._a_cmd is not None:
            a_des = self._a_cmd
        else:
            a_des = (self.cmd_velocity - self.velocity) / self.vel_cmd_tau

        speed = float(np.linalg.norm(self.velocity))
        if speed < 1e-6:
            # Launch bootstrap: no tangent direction yet, the whole demand
            # is longitudinal.
            a_norm = float(np.linalg.norm(a_des))
            a = a_des * (self.max_long_accel / a_norm) if a_norm > self.max_long_accel else a_des
        else:
            t_hat = self.velocity / speed
            a_long = float(a_des @ t_hat)
            a_long = min(max(a_long, -self.max_long_decel), self.max_long_accel)
            a_lat = a_des - float(a_des @ t_hat) * t_hat
            lat_norm = float(np.linalg.norm(a_lat))
            lat_max = self.n_max * GRAVITY
            if lat_norm > lat_max:
                a_lat = a_lat * (lat_max / lat_norm)
            a = a_long * t_hat + a_lat

        self.position = self.position + self.velocity * dt + 0.5 * a * dt**2
        self.velocity = self.velocity + a * dt
        speed = float(np.linalg.norm(self.velocity))
        if speed > self.max_speed:
            self.velocity = self.velocity * (self.max_speed / speed)
        elif self.min_speed > 0.0 and 1e-6 < speed < self.min_speed:
            self.velocity = self.velocity * (self.min_speed / speed)


BODY_KINDS = ("point_mass", "load_factor")


def make_body(
    kind: str,
    position: np.ndarray,
    velocity: np.ndarray | None = None,
    *,
    max_speed: float = 30.0,
    max_accel: float = 10.0,
    n_max: float = 4.0,
    **params,
) -> AirframeBody:
    """Build an airframe body by scenario-config kind.

    ``max_accel`` doubles as the load-factor body's longitudinal
    accel/decel bound so a fleet entry can switch ``airframe`` without
    re-specifying its performance numbers."""
    if kind == "point_mass":
        return PointMass(position, velocity, max_speed=max_speed, max_accel=max_accel)
    if kind == "load_factor":
        return LoadFactorBody(
            position, velocity, max_speed=max_speed, n_max=n_max,
            max_long_accel=max_accel, **params,
        )
    raise ValueError(f"unknown airframe kind '{kind}'; valid kinds: {', '.join(BODY_KINDS)}")


def time_to_go(p_rel: np.ndarray, v_rel: np.ndarray) -> float:
    """Closest-point-of-approach time for constant velocities (>= 0)."""
    vv = float(v_rel @ v_rel)
    if vv < 1e-9:
        return 0.0
    return max(0.0, -float(p_rel @ v_rel) / vv)
