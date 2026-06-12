"""LoadFactorBody and the AirframeBody seam (SIM-PHX-001/005).

The load-factor body's contract is kinematic and exact: lateral
acceleration never exceeds n_max * g, so turn rate at speed v never
exceeds n_max * g / v. The tests measure achieved turn rate from the
integrated trajectory, not from the commanded values.
"""

import numpy as np
import pytest

from coopuavs.sim.physics import (
    GRAVITY,
    AirframeBody,
    LoadFactorBody,
    PointMass,
    make_body,
)

DT = 0.05


def heading(v: np.ndarray) -> float:
    return float(np.arctan2(v[1], v[0]))


def heading_delta(h1: float, h0: float) -> float:
    """Smallest signed angle difference, immune to the +/-pi wrap."""
    return float(np.arctan2(np.sin(h1 - h0), np.cos(h1 - h0)))


def test_protocol_conformance():
    pm = PointMass(np.zeros(3))
    lf = LoadFactorBody(np.zeros(3))
    assert isinstance(pm, AirframeBody)
    assert isinstance(lf, AirframeBody)


def test_factory_kinds_and_unknown_rejected():
    assert isinstance(make_body("point_mass", np.zeros(3)), PointMass)
    body = make_body("load_factor", np.zeros(3), max_speed=80.0, max_accel=20.0, n_max=6.0)
    assert isinstance(body, LoadFactorBody)
    assert body.max_long_accel == 20.0 and body.n_max == 6.0
    with pytest.raises(ValueError, match="quaternion.*point_mass"):
        make_body("quaternion", np.zeros(3))


def test_turn_rate_bounded_by_load_factor():
    # Full-deflection lateral command at speed: achieved turn rate must
    # never exceed n_max * g / v (the whole point of the model).
    n_max, speed = 4.0, 60.0
    body = LoadFactorBody(
        np.zeros(3), np.array([speed, 0.0, 0.0]), max_speed=speed, n_max=n_max
    )
    for _ in range(200):
        v = body.velocity
        s = float(np.linalg.norm(v))
        # Hard-left lateral demand, far above the structural limit.
        lateral = np.array([-v[1], v[0], 0.0]) / s
        h0 = heading(v)
        body.command_acceleration(lateral * 500.0)
        body.step(DT)
        omega = abs(heading_delta(heading(body.velocity), h0)) / DT
        omega_max = n_max * GRAVITY / s
        assert omega <= omega_max * 1.01
        assert float(np.linalg.norm(body.velocity)) <= speed * 1.0 + 1e-9


def test_longitudinal_accel_bounded_and_reaches_max_speed():
    body = LoadFactorBody(
        np.zeros(3), np.array([5.0, 0.0, 0.0]), max_speed=80.0, max_long_accel=10.0
    )
    speeds = []
    body_cmd = np.array([80.0, 0.0, 0.0])
    for _ in range(400):
        s0 = float(np.linalg.norm(body.velocity))
        body.command_velocity(body_cmd)
        body.step(DT)
        s1 = float(np.linalg.norm(body.velocity))
        assert (s1 - s0) / DT <= 10.0 * 1.01
        speeds.append(s1)
    # 75 m/s of headroom at 10 m/s^2 is 7.5 s; generous margin for the tau lag.
    assert speeds[-1] == pytest.approx(80.0, abs=0.5)
    assert max(speeds) <= 80.0 + 1e-9


def test_launch_from_rest_and_hover_hold():
    # From rest the body must come up cleanly (no NaN, no stall floor
    # fighting the pad hold), and a zero command must let it hover.
    body = LoadFactorBody(np.array([0.0, 0.0, 400.0]), max_speed=50.0)
    body.command_velocity(np.array([30.0, 0.0, 0.0]))
    for _ in range(100):
        body.step(DT)
        assert np.all(np.isfinite(body.velocity))
    assert float(np.linalg.norm(body.velocity)) > 20.0
    body.command_velocity(np.zeros(3))
    for _ in range(600):
        body.step(DT)
    assert float(np.linalg.norm(body.velocity)) < 0.5


def test_stall_floor_holds_for_fixed_wing():
    body = LoadFactorBody(
        np.zeros(3), np.array([40.0, 0.0, 0.0]), max_speed=80.0, min_speed=15.0
    )
    body.command_velocity(np.zeros(3))   # demand a stop the airframe cannot fly
    for _ in range(400):
        body.step(DT)
        assert float(np.linalg.norm(body.velocity)) >= 15.0 - 1e-9


def test_deterministic_trajectories():
    def fly():
        body = LoadFactorBody(np.zeros(3), np.array([40.0, 0.0, 0.0]), max_speed=60.0)
        out = []
        for k in range(300):
            if k % 2:
                body.command_acceleration(np.array([0.0, 20.0, 1.0]))
            else:
                body.command_velocity(np.array([50.0, 10.0, 0.0]))
            body.step(DT)
            out.append(body.position.copy())
        return np.array(out)

    assert np.array_equal(fly(), fly())


def test_point_mass_acceleration_command():
    # The baseline body honours the new seam too: accel clamped at
    # max_accel, speed at max_speed, and command_velocity overrides it.
    pm = PointMass(np.zeros(3), np.array([10.0, 0.0, 0.0]), max_speed=30.0, max_accel=5.0)
    pm.command_acceleration(np.array([100.0, 0.0, 0.0]))
    pm.step(1.0)
    assert float(np.linalg.norm(pm.velocity)) == pytest.approx(15.0)
    for _ in range(10):
        pm.step(1.0)
    assert float(np.linalg.norm(pm.velocity)) == pytest.approx(30.0)
    pm.command_velocity(np.zeros(3))
    pm.step(1.0)
    assert float(np.linalg.norm(pm.velocity)) == pytest.approx(25.0)
