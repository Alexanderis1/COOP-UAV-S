"""P1-5: Beard-McLain fixed-wing aero plant + shahed_fw / jet_owa_fw / fpv_quad params.

B&M ch.4 equations are implemented verbatim in FRD and flipped to the package
FLU/ENU convention by M = diag(1,-1,-1). Pins: straight-level trim at cruise
with wrench residual < 1e-3 mg (both airframes); static pitch stability
C_m_alpha < 0 at parameter and wrench level; post-stall lift bounded by the
blended flat-plate model; lateral symmetry, weathervane and roll-damping
signs; throttle/airspeed thrust behavior per prop vs jet model;
batch==scalar; fpv_quad loads as a consistent multirotor.
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import fsolve

from coopuavs.physics import GRAVITY
from coopuavs.physics import rigid_body as rb
from coopuavs.physics.fixedwing import (
    FixedwingParams,
    FixedwingPlant,
    lift_coefficient,
    stall_blend,
)
from coopuavs.physics.multirotor import MultirotorParams
from coopuavs.physics.params import load_airframe

CRUISE = {"shahed_fw": 50.0, "jet_owa_fw": 120.0}


def make_plant(name, n=1):
    params = FixedwingParams.from_dict(load_airframe(name))
    return params, FixedwingPlant(params, n)


def level_state(va, alpha, n=1):
    """Level flight along +x East at angle of attack alpha (nose-up pitch)."""
    state = np.zeros((n, rb.STATE_DIM))
    state[:, 2] = 200.0
    state[:, 3] = va
    state[:, rb.QUAT] = rb.quat_from_axis_angle(
        np.array([[0.0, 1.0, 0.0]]), np.array([-alpha]))  # FLU: nose-up = -y rotation
    return state


def solve_trim(name):
    params, plant = make_plant(name)
    va = CRUISE[name]

    def residual(x):
        alpha, de, dt = x
        state = level_state(va, alpha)
        controls = np.array([[de, 0.0, 0.0, dt]])
        force, torque = plant.wrench(state, controls, np.zeros((1, 3)), 1.225)
        return [force[0, 0], force[0, 2], torque[0, 1]]

    x, _, ier, _ = fsolve(residual, x0=[0.05, -0.05, 0.5], full_output=True)
    return params, plant, va, x, ier, residual(x)


# ------------------------------------------------------------------------- trim


def test_trim_at_cruise_shahed():
    params, plant, va, (alpha, de, dt), ier, res = solve_trim("shahed_fw")
    assert ier == 1
    assert np.abs(res).max() < 1e-3 * params.mass * GRAVITY
    assert 0.0 < alpha < np.deg2rad(10.0)
    assert 0.05 < dt < 0.95
    assert abs(de) < np.deg2rad(20.0)
    # lateral channel identically zero at zero sideslip / zero lateral controls
    state = level_state(va, alpha)
    force, torque = plant.wrench(state, np.array([[de, 0.0, 0.0, dt]]),
                                 np.zeros((1, 3)), 1.225)
    assert abs(force[0, 1]) < 1e-9
    assert abs(torque[0, 0]) < 1e-9 and abs(torque[0, 2]) < 1e-9


def test_trim_at_cruise_jet_owa():
    params, plant, va, (alpha, de, dt), ier, res = solve_trim("jet_owa_fw")
    assert ier == 1
    assert np.abs(res).max() < 1e-3 * params.mass * GRAVITY
    assert 0.0 < alpha < np.deg2rad(8.0)
    assert 0.05 < dt < 0.95


# -------------------------------------------------------------------- stability


def test_cm_alpha_negative_param_and_wrench_level():
    for name in ("shahed_fw", "jet_owa_fw"):
        params, plant, va, (alpha, de, dt), _, _ = solve_trim(name)
        assert params.aero["Cm_alpha"] < 0.0
        controls = np.array([[de, 0.0, 0.0, dt]])

        def pitch_up_moment(a):
            force, torque = plant.wrench(level_state(va, a), controls,
                                         np.zeros((1, 3)), 1.225)
            return -torque[0, 1]            # FRD m (nose-up positive) = -tau_flu_y

        d_alpha = np.deg2rad(2.0)
        assert pitch_up_moment(alpha + d_alpha) < pitch_up_moment(alpha)  # restoring


def test_stall_bounded_blended_lift():
    params, _ = make_plant("shahed_fw")
    alphas = np.linspace(-np.pi / 2, np.pi / 2, 721)
    cl = lift_coefficient(params, alphas)
    assert np.all(np.isfinite(cl))
    assert np.abs(cl).max() < 2.0
    sigma = stall_blend(params, alphas)
    assert np.all((sigma >= 0.0) & (sigma <= 1.0))
    # lift collapses past stall instead of growing linearly forever
    a0 = params.alpha0
    assert lift_coefficient(params, np.array([a0 + 0.26]))[0] < \
        lift_coefficient(params, np.array([a0 - 0.05]))[0]
    # deep post-stall tends to flat plate, not the linear extrapolation
    linear = params.aero["CL0"] + params.aero["CL_alpha"] * (a0 + 0.5)
    assert lift_coefficient(params, np.array([a0 + 0.5]))[0] < 0.5 * linear


def test_weathervane_and_side_force_signs():
    params, plant = make_plant("shahed_fw")
    va = CRUISE["shahed_fw"]
    state = level_state(va, 0.05)
    state[0, 4] = -3.0                      # drifting toward -y world = slip right
    force, torque = plant.wrench(state, np.array([[0.0, 0.0, 0.0, 0.5]]),
                                 np.zeros((1, 3)), 1.225)
    assert torque[0, 2] < 0.0               # nose yaws right (toward the slip)
    assert force[0, 1] > 0.0                # side force opposes the slip


def test_roll_and_pitch_damping_signs():
    params, plant = make_plant("shahed_fw")
    va = CRUISE["shahed_fw"]
    controls = np.array([[0.0, 0.0, 0.0, 0.5]])

    state = level_state(va, 0.05)
    state[0, 10] = 1.0                      # +p roll rate (left side up)
    _, torque = plant.wrench(state, controls, np.zeros((1, 3)), 1.225)
    assert torque[0, 0] < 0.0               # Cl_p < 0 damps it

    state0 = level_state(va, 0.05)
    _, torque0 = plant.wrench(state0, controls, np.zeros((1, 3)), 1.225)
    state1 = level_state(va, 0.05)
    state1[0, 11] = -1.0                    # FLU -y rate = FRD +q (nose-up rate)
    _, torque1 = plant.wrench(state1, controls, np.zeros((1, 3)), 1.225)
    assert -torque1[0, 1] < -torque0[0, 1]  # Cm_q < 0 opposes the nose-up rate


def test_control_surface_polarity_pins():
    """Gate-review pin: da/dr were exercised by no behavioral test and a de
    sign flip just mirrors the fsolve trim. Finite-difference polarity of
    every control channel at trim, in B&M FRD terms (l, m, n) =
    (tau_x, -tau_y, -tau_z); fy_frd = -delta force_y (pitch-only attitude)."""
    params, plant, va, (alpha, de, dt), _, _ = solve_trim("shahed_fw")
    state = level_state(va, alpha)

    def wrench_of(controls):
        force, tau = plant.wrench(state, np.array([controls]),
                                  np.zeros((1, 3)), 1.225)
        return force[0], tau[0]

    h = 0.05
    f0, t0 = wrench_of([de, 0.0, 0.0, dt])

    f1, t1 = wrench_of([de + h, 0.0, 0.0, dt])
    assert -(t1[1] - t0[1]) < 0.0           # Cm_de < 0: +elevator pitches down

    f1, t1 = wrench_of([de, h, 0.0, dt])
    assert (t1[0] - t0[0]) > 0.0            # Cl_da > 0: +aileron rolls right (FRD l+)
    assert -(t1[2] - t0[2]) < 0.0           # Cn_da < 0: adverse yaw

    f1, t1 = wrench_of([de, 0.0, h, dt])
    assert -(t1[2] - t0[2]) < 0.0           # Cn_dr < 0 with +rudder
    assert -(f1[1] - f0[1]) > 0.0           # CY_dr > 0: +rudder side force (FRD +y)


def test_throttle_thrust_prop_vs_jet():
    _, prop_plant = make_plant("shahed_fw")
    _, jet_plant = make_plant("jet_owa_fw")

    def fx(plant, va, dt):
        state = level_state(va, 0.0)
        force, _ = plant.wrench(state, np.array([[0.0, 0.0, 0.0, dt]]),
                                np.zeros((1, 3)), 1.225)
        return force[0, 0]

    assert fx(prop_plant, 40.0, 0.9) > fx(prop_plant, 40.0, 0.5)
    assert fx(jet_plant, 100.0, 0.9) > fx(jet_plant, 100.0, 0.5)
    # prop thrust washes out with airspeed; jet model does not
    prop_gain = fx(prop_plant, 60.0, 0.8) - fx(prop_plant, 40.0, 0.8)
    jet_drag_40 = fx(jet_plant, 40.0, 0.0)
    jet_drag_60 = fx(jet_plant, 60.0, 0.0)
    jet_gain = (fx(jet_plant, 60.0, 0.8) - jet_drag_60) - \
               (fx(jet_plant, 40.0, 0.8) - jet_drag_40)
    assert prop_gain < 0.0                  # extra drag AND less prop thrust
    assert abs(jet_gain) < 1e-9             # pure throttle-proportional thrust


def test_fixedwing_batch_equals_scalar():
    rng = np.random.default_rng(55)
    n = 5
    params, plant = make_plant("shahed_fw", n)
    state = rng.normal(size=(n, rb.STATE_DIM))
    state[:, 2] += 300.0
    state[:, 3] += 50.0
    state[:, rb.QUAT] = rb.quat_normalize(state[:, rb.QUAT])
    controls = rng.uniform(-0.2, 0.2, size=(n, 4))
    controls[:, 3] = rng.uniform(0.1, 0.9, size=n)
    wind = rng.normal(scale=4.0, size=(n, 3))
    f_b, t_b = plant.wrench(state, controls, wind, 1.2)
    for i in range(n):
        _, single = make_plant("shahed_fw", 1)
        f_s, t_s = single.wrench(state[i:i + 1], controls[i:i + 1], wind[i:i + 1], 1.2)
        np.testing.assert_allclose(f_b[i], f_s[0], rtol=0, atol=1e-12)
        np.testing.assert_allclose(t_b[i], t_s[0], rtol=0, atol=1e-12)


def test_zero_airspeed_no_aero_blowup():
    params, plant = make_plant("shahed_fw")
    state = np.zeros((1, rb.STATE_DIM))
    state[0, 2] = 100.0
    state[0, rb.QUAT] = [1, 0, 0, 0]
    force, torque = plant.wrench(state, np.zeros((1, 4)), np.zeros((1, 3)), 1.225)
    assert np.isfinite(force).all() and np.isfinite(torque).all()
    np.testing.assert_allclose(force[0], [0.0, 0.0, -params.mass * GRAVITY], atol=1e-9)


def test_inertia_frd_to_flu_mapping():
    params, _ = make_plant("shahed_fw")
    jxz = load_airframe("shahed_fw")["inertia_frd"]["jxz"]
    # J_flu = M J_frd M with M = diag(1,-1,-1): off-diagonal xz flips sign
    assert params.inertia[0, 2] == jxz
    assert params.inertia[2, 0] == jxz
    assert np.allclose(params.inertia, params.inertia.T)


# ---------------------------------------------------------------- fpv_quad params


def test_fpv_quad_loads_as_consistent_multirotor():
    params = MultirotorParams.from_dict(load_airframe("fpv_quad"))
    assert params.n_rotors == 4
    assert int(np.sum(params.rotor_spin)) == 0
    w_h = np.sqrt(params.mass * GRAVITY / (params.n_rotors * params.kf))
    ke = 60.0 / (2.0 * np.pi * params.motor["kv_rpm_per_v"])
    a = params.km * params.motor["r_w"] / ke
    v_bus = params.battery["n_series"] * 3.7
    w_max = (-ke + np.sqrt(ke * ke + 4.0 * a * v_bus)) / (2.0 * a)
    assert 0.3 < w_h / w_max < 0.8
    t_w = params.n_rotors * params.kf * w_max**2 / (params.mass * GRAVITY)
    assert 2.0 < t_w < 4.5, t_w
