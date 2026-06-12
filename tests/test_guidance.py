import numpy as np

from coopuavs.interceptors.guidance import (
    intercept_time,
    pro_nav_accel,
    pursuit_velocity,
)
from coopuavs.sim.physics import LoadFactorBody, PointMass


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


# -- true proportional navigation ---------------------------------------------


def test_pn_zero_on_collision_course():
    # Constant-bearing decreasing-range geometry: LOS rate is zero, so PN
    # must not command (the definition of the law).
    own_pos = np.array([0.0, 0.0, 100.0])
    own_vel = np.array([60.0, 0.0, 0.0])
    tgt_pos = np.array([1000.0, 0.0, 100.0])
    tgt_vel = np.array([-40.0, 0.0, 0.0])
    a = pro_nav_accel(own_pos, own_vel, tgt_pos, tgt_vel)
    assert float(np.linalg.norm(a)) < 1e-9


def test_pn_command_perpendicular_to_los_and_corrective():
    own_pos = np.array([0.0, 0.0, 100.0])
    own_vel = np.array([60.0, 0.0, 0.0])
    tgt_pos = np.array([1000.0, 0.0, 100.0])
    tgt_vel = np.array([-40.0, 30.0, 0.0])    # crossing left
    a = pro_nav_accel(own_pos, own_vel, tgt_pos, tgt_vel)
    los = tgt_pos - own_pos
    assert abs(float(a @ los)) < 1e-6 * float(np.linalg.norm(a)) * float(np.linalg.norm(los))
    # The target drifts +y, the LOS rotates that way — PN must pull +y.
    assert a[1] > 0.0


def test_pn_degenerate_and_opening_geometry_returns_zero():
    p = np.array([0.0, 0.0, 100.0])
    assert np.array_equal(pro_nav_accel(p, np.zeros(3), p, np.zeros(3)), np.zeros(3))
    receding = pro_nav_accel(
        p, np.array([10.0, 0.0, 0.0]),
        np.array([500.0, 0.0, 100.0]), np.array([80.0, 20.0, 0.0]),
    )
    assert np.array_equal(receding, np.zeros(3))


def _terminal_miss(use_pn: bool) -> float:
    """Closed loop on the load-factor body against a weaving target."""
    dt = 0.02
    own = LoadFactorBody(
        np.array([0.0, 0.0, 250.0]), np.array([70.0, 10.0, 5.0]),
        max_speed=70.0, n_max=5.0,
    )
    tgt_pos = np.array([900.0, 250.0, 300.0])
    closest = np.inf
    for k in range(1500):
        t = k * dt
        bearing = np.pi + 0.7 * np.sin(0.9 * t)     # ~3.5 m/s^2 weave at 50 m/s
        tgt_vel = np.array([50.0 * np.cos(bearing), 50.0 * np.sin(bearing), 0.0])
        if use_pn:
            own.command_acceleration(
                pro_nav_accel(own.position, own.velocity, tgt_pos, tgt_vel)
            )
        else:
            own.command_velocity(
                pursuit_velocity(own.position, tgt_pos, tgt_vel, 70.0)
            )
        own.step(dt)
        tgt_pos = tgt_pos + tgt_vel * dt
        closest = min(closest, float(np.linalg.norm(tgt_pos - own.position)))
    return closest


def test_pn_intercepts_weaving_target_on_load_factor_body():
    # Acceptance gate: closed-loop PN on the load-factor airframe defeats
    # a weaving target well inside both effector envelopes. (With perfect
    # target state both PN and re-aimed lead pursuit reach sub-metre miss
    # here, so no comparative claim between the laws is asserted.)
    assert _terminal_miss(use_pn=True) < 5.0
    assert _terminal_miss(use_pn=False) < 5.0
