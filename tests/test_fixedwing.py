"""P1-5: Beard-McLain fixed-wing aero plant + shahed_fw / jet_owa_fw / fpv_quad params.

B&M ch.4 equations are implemented verbatim in FRD and flipped to the package
FLU/ENU convention by M = diag(1,-1,-1). Pins: straight-level trim at cruise
with wrench residual < 1e-3 mg (both airframes); static pitch stability
C_m_alpha < 0 at parameter and wrench level; post-stall lift bounded by the
blended flat-plate model; lateral symmetry, weathervane and roll-damping
signs; throttle/airspeed thrust behavior per prop vs jet model;
batch==scalar; fpv_quad loads as a consistent multirotor.

PR-8 review pins (mutation-killing, hand-computed literals from the YAML):
pure yaw-rate lateral wrench (kills FRD r sign flip, Cl_r/Cn_r drops);
pitch-rate c/(2Va) nondimensionalization + CL_q in the 4.19 rotation (kills
chord/span swap); prop washout isolated at < -400 N; FixedwingPlant.step
closed-loop trim hold + batch==scalar; aileron-step Jxz roll-yaw inertial
coupling (kills diagonal-inertia mutant of the plant wiring).
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


def test_yaw_rate_lateral_wrench_quantitative_pin():
    """PR-8 review pin: the FRD yaw-rate channel was completely unpinned (an
    rr sign-flip mutant survived the full suite). Pure body yaw rate at level
    cruise; lateral wrench must equal the hand-computed B&M r-derivative
    terms with the SPAN factor b/(2Va) to 1e-10, as literals independent of
    the implementation:

      qbar*S = 0.5 * 1.225 * 50^2 * 1.6 = 2450.0;  b/(2Va) = 2.5/100 = 0.025
      FLU omega_z = +1  ->  FRD r = -1 (M = diag(1,-1,-1))
      l_frd  = 2450 * 2.5 * (Cl_r = 0.10) * 0.025 * (-1) = -15.3125
      n_frd  = 2450 * 2.5 * (Cn_r = -0.08) * 0.025 * (-1) = +12.25
      fy_frd = 2450 * (CY_r = 0.0) * 0.025 * (-1)         = 0.0
      m_frd  = 2450 * 0.65 * (Cm0 = 0.02)                 = 31.85 (q = 0)
      torque_flu = [l, -m, -n]

    Kills: rr = +om[:, 2] mutant (sign of every term flips), Cl_r / Cn_r
    term drops. CY_r is 0.0 in the YAML so the fy pin is exact-zero."""
    _, plant = make_plant("shahed_fw")
    state = level_state(50.0, 0.0)
    state[0, 12] = 1.0                      # FLU +z yaw rate -> FRD r = -1
    # delta_t = Va/k_motor = 0.625 zeroes the prop term ((k dt)^2 = Va^2)
    controls = np.array([[0.0, 0.0, 0.0, 0.625]])
    force, torque = plant.wrench(state, controls, np.zeros((1, 3)), 1.225)
    np.testing.assert_allclose(torque[0, 0], -15.3125, rtol=0, atol=1e-10)
    np.testing.assert_allclose(torque[0, 1], -31.85, rtol=0, atol=1e-10)
    np.testing.assert_allclose(torque[0, 2], -12.25, rtol=0, atol=1e-10)
    np.testing.assert_allclose(force[0, 1], 0.0, rtol=0, atol=1e-10)


def test_pitch_rate_terms_quantitative_pin():
    """PR-8 review pin: rate-term nondimensionalization was sign-only pinned,
    so a classic c/(2Va) -> b/(2Va) swap on the pitch-damping term (3.85x for
    shahed_fw) and a CL_q drop from the eq. 4.19 rotation survived. Pin the
    pure-q wrench DELTA (baseline at the same air data cancels exactly)
    against literals with the CHORD factor c/(2Va) = 0.65/100 = 0.0065
    (span would be b/(2Va) = 0.025).

    Part A, alpha = 0, FLU omega_y = -1 -> FRD q = +1:
      d m_frd  = 2450 * 0.65 * (Cm_q = -20.0) * 0.0065 = -207.025
                 -> d torque_flu_y = +207.025
      d fz_frd = 2450 * (cz_q = -CD_q*sin a - CL_q*cos a = -5.0) * 0.0065
               = -79.625  -> d force_world_z = +79.625 (identity attitude)
      d fx_frd = 0 (cx_q = -CD_q*cos a + CL_q*sin a, CD_q = 0, sin a = 0)

    Part B, alpha = 0.1 via descending velocity at identity attitude (world
    axes == FLU axes), pins CL_q inside cx_q (line untouched by part A):
      d fx = 2450 * (5.0 * sin 0.1) * 0.0065 = 7.94923580...
    """
    _, plant = make_plant("shahed_fw")
    controls = np.zeros((1, 4))
    wind = np.zeros((1, 3))

    base = level_state(50.0, 0.0)
    rate = level_state(50.0, 0.0)
    rate[0, 11] = -1.0                      # FLU -y rate = FRD q = +1
    f0, t0 = plant.wrench(base, controls, wind, 1.225)
    f1, t1 = plant.wrench(rate, controls, wind, 1.225)
    np.testing.assert_allclose(t1[0, 1] - t0[0, 1], 207.025, rtol=0, atol=1e-10)
    np.testing.assert_allclose(f1[0, 2] - f0[0, 2], 79.625, rtol=0, atol=1e-10)
    np.testing.assert_allclose(f1[0, 0] - f0[0, 0], 0.0, rtol=0, atol=1e-10)

    alpha = 0.1
    base = np.zeros((1, rb.STATE_DIM))
    base[0, 2] = 200.0
    base[0, 3] = 50.0 * np.cos(alpha)       # nose level, descending: alpha = 0.1
    base[0, 5] = -50.0 * np.sin(alpha)
    base[0, rb.QUAT] = [1.0, 0.0, 0.0, 0.0]
    rate = base.copy()
    rate[0, 11] = -1.0
    f0, _ = plant.wrench(base, controls, wind, 1.225)
    f1, _ = plant.wrench(rate, controls, wind, 1.225)
    expected_dfx = 2450.0 * (5.0 * np.sin(0.1)) * 0.0065
    np.testing.assert_allclose(f1[0, 0] - f0[0, 0], expected_dfx, rtol=0, atol=1e-9)


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
    # PR-8 review pin: < 0.0 was satisfied by airframe drag alone (~-99 N
    # from 40->60 m/s), so a mutant deleting the B&M 4.15 washout term
    # (-Va^2) survived. Real prop_gain ~= -613 N = washout (-514.5 N =
    # 0.5*1.225*0.42*(40^2 - 60^2)) + drag delta (~-99 N); -400 N splits
    # the two cleanly. (A dt=0-drag-corrected gain would NOT work here: the
    # washout term is throttle-independent and cancels in that difference.)
    assert prop_gain < -400.0               # extra drag AND less prop thrust
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


# ------------------------------------------------------------ step / closed loop


def test_step_closed_loop_trim_hold_bounded():
    """PR-8 review pin: FixedwingPlant.step / wrench_fn were never executed
    by any test (the fixed-wing/rigid-body seam was dead code under test).
    Fly the shahed from its solved trim point for 5 s at dt = 1e-3 with
    frozen trim controls and a steady 10 m/s tailwind (initial ground speed
    raised to match, so the air-relative state is exactly the trim point and
    a wind sign error through step() breaks the envelope).

    Trim (omega = 0, lateral identically zero by symmetry) is an exact fixed
    point of the dynamics, so deviations come only from the fsolve residual
    (< 1e-3 mg) and roundoff. Envelope deliberately loose but meaningful --
    the phugoid (Lanchester period ~ pi*sqrt(2)*Va/g ~ 23 s) is undamped
    enough that a perturbed start oscillates, but it cannot move airspeed by
    5 m/s or altitude by 30 m in 5 s from an on-trim start: finite state,
    |Va_air - 50| < 5 m/s, |dz| < 30 m, pitch within 10 deg of trim alpha,
    lateral channel (q_x, q_z, v_y) untouched, body rates < 0.2 rad/s."""
    params, plant, va, (alpha, de, dthr), ier, _ = solve_trim("shahed_fw")
    assert ier == 1
    wind = np.array([[10.0, 0.0, 0.0]])
    controls = np.array([[de, 0.0, 0.0, dthr]])
    state = level_state(va, alpha)
    state[0, 3] += wind[0, 0]               # ground speed = trim airspeed + wind
    for _ in range(5000):
        state = plant.step(state, 1e-3, controls, wind, 1.225)
    assert np.isfinite(state).all()
    va_air = np.linalg.norm(state[0, 3:6] - wind[0])
    assert abs(va_air - va) < 5.0
    assert abs(state[0, 2] - 200.0) < 30.0
    pitch = -2.0 * np.arctan2(state[0, 8], state[0, 6])   # pure-y quat: nose-up +
    assert abs(pitch - alpha) < np.deg2rad(10.0)
    assert abs(state[0, 7]) < 1e-9 and abs(state[0, 9]) < 1e-9   # no roll/yaw
    assert abs(state[0, 4]) < 1e-9                               # no sideslip
    assert np.abs(state[0, 10:13]).max() < 0.2


def test_step_batch_equals_scalar():
    """Batched step() must equal per-row scalar step() to 1e-12 (the wrench
    batch==scalar pin did not cover the step/integrator path)."""
    rng = np.random.default_rng(56)
    n = 4
    _, plant = make_plant("shahed_fw", n)
    state = rng.normal(size=(n, rb.STATE_DIM))
    state[:, 2] += 300.0
    state[:, 3] += 50.0
    state[:, rb.QUAT] = rb.quat_normalize(state[:, rb.QUAT])
    controls = rng.uniform(-0.2, 0.2, size=(n, 4))
    controls[:, 3] = rng.uniform(0.1, 0.9, size=n)
    wind = rng.normal(scale=4.0, size=(n, 3))
    out_b = plant.step(state, 1e-3, controls, wind, 1.2)
    for i in range(n):
        _, single = make_plant("shahed_fw", 1)
        out_s = single.step(state[i:i + 1], 1e-3, controls[i:i + 1],
                            wind[i:i + 1], 1.2)
        np.testing.assert_allclose(out_b[i], out_s[0], rtol=0, atol=1e-12)


def test_aileron_step_jxz_roll_yaw_coupling():
    """PR-8 review pin: the Jxz product-of-inertia path was never integrated
    (only the matrix-level mapping was pinned). Aileron-only step from trim:
    with J_flu = [[31, 0, 3], [0, 82, 0], [3, 0, 100]] (Jxz_flu = +3 from
    the FRD->FLU flip) the roll torque couples through the inertia x-z
    product and the initial FLU yaw acceleration is NEGATIVE (FRD r_dot > 0,
    nose toward the roll), overpowering the adverse-yaw Cn_da term:

      tau_flu = [qbar*S*b*Cl_da*da, 0, -qbar*S*b*Cn_da*da] = [91.875, 0, 6.125]
      omdot_z = (-3*91.875 + 31*6.125) / (31*100 - 3^2) = -0.027742 rad/s^2

    A diagonal-inertia mutant of the plant wiring gives omdot_z =
    +6.125/100 = +0.06125 -- opposite sign AND wrong magnitude. J is built
    here from the YAML literals, independent of FixedwingPlant.__init__."""
    params, plant, va, (alpha, de, dthr), ier, _ = solve_trim("shahed_fw")
    assert ier == 1
    state = level_state(va, alpha)
    controls = np.array([[de, 0.1, 0.0, dthr]])
    _, tau = plant.wrench(state, controls, np.zeros((1, 3)), 1.225)
    j_flu = np.array([[31.0, 0.0, 3.0], [0.0, 82.0, 0.0], [3.0, 0.0, 100.0]])
    expected_omdot = np.linalg.solve(j_flu, tau[0])
    assert expected_omdot[0] > 0.0          # rolls toward +aileron (FRD l > 0)
    assert expected_omdot[2] < 0.0          # Jxz coupling beats adverse yaw

    h = 1e-4
    out = plant.step(state, h, controls, np.zeros((1, 3)), 1.225)
    omega = out[0, 10:13]
    assert omega[0] > 0.0
    assert omega[2] < 0.0                   # diag-J mutant: +6.1e-6 here
    np.testing.assert_allclose(omega[0] / h, expected_omdot[0], rtol=2e-3)
    np.testing.assert_allclose(omega[2] / h, expected_omdot[2], rtol=2e-3)
    assert abs(omega[1] / h) < 1e-6         # pitch channel stays trimmed


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
