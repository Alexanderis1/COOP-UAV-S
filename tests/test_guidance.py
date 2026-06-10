import numpy as np

from coopuavs.interceptors.guidance import intercept_time, pursuit_velocity
from coopuavs.sim.physics import PointMass


def test_intercept_time_closing_target():
    # Target 1000 m east, flying west at 20 m/s; we fly 40 m/s.
    rel = np.array([1000.0, 0.0, 0.0])
    v_t = np.array([-20.0, 0.0, 0.0])
    t = intercept_time(rel, v_t, 40.0)
    assert t is not None
    # Closing speed 60 m/s over 1000 m.
    assert abs(t - 1000.0 / 60.0) < 1e-6


def test_intercept_time_uncatchable():
    # Target receding faster than we can fly.
    rel = np.array([1000.0, 0.0, 0.0])
    v_t = np.array([80.0, 0.0, 0.0])
    assert intercept_time(rel, v_t, 40.0) is None


def test_pursuit_converges():
    own = PointMass(np.array([0.0, 0.0, 100.0]), max_speed=50.0, max_accel=30.0)
    target_pos = np.array([2000.0, 500.0, 300.0])
    target_vel = np.array([-30.0, 0.0, 0.0])
    dt = 0.05
    closest = 1e9
    for _ in range(2000):
        own.command_velocity(
            pursuit_velocity(own.position, target_pos, target_vel, 50.0)
        )
        own.step(dt)
        target_pos = target_pos + target_vel * dt
        closest = min(closest, float(np.linalg.norm(target_pos - own.position)))
    assert closest < 10.0
