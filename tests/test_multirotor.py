"""P1-4: batched multirotor plant + interceptor_quad airframe params.

Pins: hover trim sum(kf w^2) = mg within 0.1%; Cheeseman-Bennett ground
effect exactly 1/(1-(R/4z)^2) at z/R in {0.6, 1, 2}; terminal dash speed
80 +- 5 m/s at 65 deg tilt (this test pins the invented airframe drag
numbers); Faessler drag signs (opposes airspeed, zero at rest, dissipative);
rotor torque allocation signs for quad-X FLU; batch==scalar; params load
from the packaged YAML with motor-consistent thrust/weight ~ 3.6.
"""

from __future__ import annotations

import dataclasses

import numpy as np
from scipy.optimize import fsolve

from coopuavs.physics import GRAVITY, atmosphere as atm
from coopuavs.physics import rigid_body as rb
from coopuavs.physics.motor import MotorEsc
from coopuavs.physics.battery import BatteryEcm
from coopuavs.physics.multirotor import MultirotorParams, MultirotorPlant
from coopuavs.physics.params import load_airframe


def make_plant(n=1):
    params = MultirotorParams.from_dict(load_airframe("interceptor_quad"))
    return params, MultirotorPlant(params, n)


def hover_state(n=1, z=100.0):
    state = np.zeros((n, rb.STATE_DIM))
    state[:, 2] = z
    state[:, rb.QUAT] = [1.0, 0.0, 0.0, 0.0]
    return state


def hover_omega(params):
    return np.sqrt(params.mass * GRAVITY / (params.n_rotors * params.kf))


def motor_omega_max(params, v_bus):
    """Full-throttle steady state: Kt (V - Ke w)/R_w = km w^2."""
    ke = 60.0 / (2.0 * np.pi * params.motor["kv_rpm_per_v"])
    a = params.km * params.motor["r_w"] / ke
    return (-ke + np.sqrt(ke * ke + 4.0 * a * v_bus)) / (2.0 * a)


# ------------------------------------------------------------------ hover trim


def test_hover_trim_within_0p1_percent():
    params, plant = make_plant()
    w_h = hover_omega(params)
    rotor = np.full((1, params.n_rotors), w_h)
    force, torque = plant.wrench(hover_state(), rotor, np.zeros((1, 3)), 1.225)
    assert np.abs(force).max() < 1e-3 * params.mass * GRAVITY
    assert np.abs(torque).max() < 1e-9


def test_hover_omega_headroom():
    params, _ = make_plant()
    ratio = hover_omega(params) / motor_omega_max(params, 44.4)
    assert 0.4 < ratio < 0.7, ratio


def test_thrust_to_weight_about_3p6():
    params, _ = make_plant()
    w_max = motor_omega_max(params, 44.4)
    t_w = params.n_rotors * params.kf * w_max**2 / (params.mass * GRAVITY)
    assert 3.3 < t_w < 3.9, t_w


# --------------------------------------------------------------- ground effect


def test_cheeseman_bennett_ground_effect_curve():
    params, plant = make_plant()
    w_h = hover_omega(params)
    rotor = np.full((1, params.n_rotors), w_h)
    oge_force, _ = plant.wrench(hover_state(z=1000.0), rotor, np.zeros((1, 3)), 1.225)
    thrust_oge = oge_force[0, 2] + params.mass * GRAVITY
    for z_over_r in (0.6, 1.0, 2.0):
        z = z_over_r * params.rotor_radius
        force, _ = plant.wrench(hover_state(z=z), rotor, np.zeros((1, 3)), 1.225)
        thrust = force[0, 2] + params.mass * GRAVITY
        expected = 1.0 / (1.0 - (1.0 / (4.0 * z_over_r)) ** 2)
        np.testing.assert_allclose(thrust / thrust_oge, expected, rtol=1e-6)


def test_ground_effect_gain_clamped_near_ground():
    params, plant = make_plant()
    w_h = hover_omega(params)
    rotor = np.full((1, params.n_rotors), w_h)
    force, _ = plant.wrench(hover_state(z=0.01), rotor, np.zeros((1, 3)), 1.225)
    thrust = force[0, 2] + params.mass * GRAVITY
    assert thrust <= params.ground_effect_max_gain * params.mass * GRAVITY * 1.001
    assert np.isfinite(force).all()


def test_ground_effect_max_gain_clip_in_singular_band():
    """Deep-inspection pin: for z in (R/4 ~= 0.0445, ~0.077] m the raw
    Cheeseman-Bennett gain 1/denom is finite but > max_gain (4.81 at
    z=0.05, 2.22 at z=0.06), so the np.clip upper bound -- not the
    np.where singularity fallback probed at z=0.01 -- is load-bearing.
    Kills the np.clip(gain, 1.0, np.inf) mutant that survives the suite."""
    params, plant = make_plant()
    w_h = hover_omega(params)
    rotor = np.full((1, params.n_rotors), w_h)
    expected = params.ground_effect_max_gain * params.kf * np.sum(rotor**2)
    for z in (0.05, 0.06):
        force, _ = plant.wrench(hover_state(z=z), rotor, np.zeros((1, 3)), 1.225)
        thrust = force[0, 2] + params.mass * GRAVITY
        np.testing.assert_allclose(thrust, expected, rtol=0, atol=1e-12)


# ------------------------------------------------------------ terminal speed pin


def test_terminal_speed_80ms_at_65deg_tilt():
    """Force equilibrium at fixed 65 deg tilt pins the airframe drag numbers."""
    params, plant = make_plant()
    tilt = np.deg2rad(65.0)
    q = rb.quat_from_axis_angle(np.array([[0.0, 1.0, 0.0]]), np.array([tilt]))
    rho = float(atm.density(200.0))

    def residual(x):
        w_rot, v = x
        state = hover_state(z=200.0)
        state[0, rb.QUAT] = q[0]
        state[0, 3] = v
        rotor = np.full((1, params.n_rotors), w_rot)
        force, _ = plant.wrench(state, rotor, np.zeros((1, 3)), rho)
        return [force[0, 0], force[0, 2]]

    (w_sol, v_sol), info, ier, _ = fsolve(residual, x0=[1200.0, 75.0], full_output=True)
    assert ier == 1
    assert 75.0 <= v_sol <= 85.0, f"terminal speed {v_sol:.1f} m/s"
    # feasible within the motor envelope at nominal bus voltage
    assert 0.0 < w_sol < motor_omega_max(params, 44.4)


# ----------------------------------------------------------------- drag model


def test_faessler_drag_signs_and_dissipation():
    params, plant = make_plant()
    no_rotor = np.zeros((1, params.n_rotors))
    grav = np.array([0.0, 0.0, -params.mass * GRAVITY])

    state = hover_state()
    drag0, _ = plant.wrench(state, no_rotor, np.zeros((1, 3)), 1.225)
    np.testing.assert_allclose(drag0[0], grav, atol=1e-12)  # no airspeed -> no drag

    for v_world in ([5.0, 0, 0], [0, 5.0, 0], [0, 0, 5.0], [-3.0, 4.0, -2.0]):
        state = hover_state()
        state[0, rb.VEL] = v_world
        force, _ = plant.wrench(state, no_rotor, np.zeros((1, 3)), 1.225)
        drag = force[0] - grav
        assert drag @ np.asarray(v_world) < 0.0  # dissipative

    # wind only: drag pushes the hovering vehicle downwind
    state = hover_state()
    force, _ = plant.wrench(state, no_rotor, np.array([[8.0, 0.0, 0.0]]), 1.225)
    assert (force[0] - grav)[0] > 0.0


def test_faessler_drag_applied_in_body_frame():
    """Gate-review pin: D = diag(dx, dy, dz) acts in the BODY frame
    [Faessler 2018] — force - gravity == -(R D R^T) v_air at arbitrary
    attitude. Kills the world-frame-drag mutant, which survives the
    terminal-speed band (that equilibrium projects out Dy/Dz/frame)."""
    base = MultirotorParams.from_dict(load_airframe("interceptor_quad"))
    params = dataclasses.replace(base, cda_iso=0.0)
    plant = MultirotorPlant(params, 1)
    no_rotor = np.zeros((1, params.n_rotors))
    grav = np.array([0.0, 0.0, -params.mass * GRAVITY])
    rng = np.random.default_rng(46)
    for _ in range(5):
        q = rb.quat_normalize(rng.normal(size=(1, 4)))
        v_air = rng.normal(scale=15.0, size=3)
        state = hover_state()
        state[0, rb.QUAT] = q[0]
        state[0, rb.VEL] = v_air
        force, _ = plant.wrench(state, no_rotor, np.zeros((1, 3)), 1.225)
        r = rb.quat_to_rotmat(q)[0]
        expected = -(r @ np.diag(params.drag_linear_diag) @ r.T) @ v_air
        np.testing.assert_allclose(force[0] - grav, expected, atol=1e-12)

    # rolled 90 deg about x: world-z motion must see body-LATERAL dy, not dz
    q = rb.quat_from_axis_angle(np.array([[1.0, 0.0, 0.0]]), np.array([np.pi / 2]))
    state = hover_state()
    state[0, rb.QUAT] = q[0]
    state[0, rb.VEL] = [0.0, 0.0, 5.0]
    force, _ = plant.wrench(state, no_rotor, np.zeros((1, 3)), 1.225)
    np.testing.assert_allclose((force[0] - grav)[2],
                               -params.drag_linear_diag[1] * 5.0, rtol=1e-9)
    # and the latched step() path sees the same body-frame drag
    plant_drag = MultirotorPlant(params, 1)
    new = plant_drag.step(state, 1e-3, no_rotor, np.zeros((1, 3)), 1.225)
    dvz_drag = new[0, 5] - state[0, 5] + GRAVITY * 1e-3
    # drag acts on the within-step mean vz (gravity pulls it down g*dt/2)
    vz_mean = 5.0 - GRAVITY * 1e-3 / 2.0
    expected_dvz = -params.drag_linear_diag[1] * vz_mean / params.mass * 1e-3
    np.testing.assert_allclose(dvz_drag, expected_dvz, rtol=1e-4)


def test_rotor_torque_allocation_signs():
    params, plant = make_plant()
    w_h = hover_omega(params)
    pos, spin = params.rotor_positions, params.rotor_spin
    state = hover_state()

    def torque(boost_mask):
        rotor = np.full((1, params.n_rotors), w_h)
        rotor[0, boost_mask] *= 1.1
        _, tau = plant.wrench(state, rotor, np.zeros((1, 3)), 1.225)
        return tau[0]

    tau = torque(pos[:, 1] < 0)        # boost right side (y<0 in FLU)
    assert tau[0] < -1e-3              # right side up = negative roll about +x fwd
    tau = torque(pos[:, 0] > 0)        # boost front rotors
    assert tau[1] < -1e-3              # nose up = negative pitch about +y left
    tau = torque(spin > 0)             # boost CCW rotors
    assert tau[2] < -1e-6              # reaction spins body CW (negative yaw)


def test_rotor_moment_magnitudes_pinned():
    """Deep-inspection pin: literal moment magnitudes from interceptor_quad
    geometry (kf=5.4e-5, km=1e-6, arms +-0.318 m FLU, spin [1,1,-1,-1]) kill
    moment-arm scale mutants that the sign-only allocation test admits.
    z=1e6 m makes the ground-effect gain 1 to <2e-15 so the literals hold.

    omega = [900, 800, 700, 600] rad/s on [FR, BL, FL, BR]:
      T_i = kf w_i^2          = [43.74, 34.56, 26.46, 19.44] N
      tau_x = sum(y_i T_i)    = 0.318*(-43.74+34.56+26.46-19.44) = -0.68688
      tau_y = -sum(x_i T_i)   = -0.318*(43.74-34.56+26.46-19.44) = -5.1516
      tau_z = -km sum(s w^2)  = -1e-6*(810000+640000-490000-360000) = -0.6
    """
    _, plant = make_plant()
    rotor = np.array([[900.0, 800.0, 700.0, 600.0]])
    _, tau = plant.wrench(hover_state(z=1.0e6), rotor, np.zeros((1, 3)), 1.225)
    np.testing.assert_allclose(tau[0], [-0.68688, -5.1516, -0.6],
                               rtol=0, atol=1e-10)


def test_parasitic_drag_scales_with_air_density():
    """Deep-inspection pin: the rho argument reaches the quadratic parasitic
    term. The wrench delta between rho=1.225 and rho=0.9 (same state, hover
    rotor speeds) cancels gravity, thrust and the rho-free Faessler term
    exactly, leaving -0.5*(rho_hi-rho_lo)*CdA*|v|v; a hardcoded-sea-level-rho
    mutant makes the delta zero. Kills the mutant the 75-85 m/s terminal
    band cannot (rho(200 m) is only 2% off 1.225)."""
    params, plant = make_plant()
    w_h = hover_omega(params)
    rotor = np.full((1, params.n_rotors), w_h)
    v = np.array([30.0, -10.0, 5.0])
    state = hover_state()
    state[0, rb.VEL] = v
    f_hi, _ = plant.wrench(state, rotor, np.zeros((1, 3)), 1.225)
    f_lo, _ = plant.wrench(state, rotor, np.zeros((1, 3)), 0.9)
    expected = -0.5 * (1.225 - 0.9) * params.cda_iso * np.linalg.norm(v) * v
    np.testing.assert_allclose(f_hi[0] - f_lo[0], expected, rtol=0, atol=1e-12)


# ------------------------------------------------------------------ batch + io


def test_multirotor_batch_equals_scalar():
    rng = np.random.default_rng(44)
    n = 5
    params, plant = make_plant(n)
    state = rng.normal(size=(n, rb.STATE_DIM))
    state[:, 2] += 50.0
    state[:, rb.QUAT] = rb.quat_normalize(state[:, rb.QUAT])
    rotor = rng.uniform(300.0, 1200.0, size=(n, params.n_rotors))
    wind = rng.normal(scale=5.0, size=(n, 3))
    f_b, t_b = plant.wrench(state, rotor, wind, 1.2)
    for i in range(n):
        _, single = make_plant(1)
        f_s, t_s = single.wrench(state[i:i + 1], rotor[i:i + 1], wind[i:i + 1], 1.2)
        np.testing.assert_allclose(f_b[i], f_s[0], rtol=0, atol=1e-12)
        np.testing.assert_allclose(t_b[i], t_s[0], rtol=0, atol=1e-12)


def test_step_integrates_with_rigid_body():
    params, plant = make_plant()
    w_h = hover_omega(params)
    rotor = np.full((1, params.n_rotors), w_h)
    state = hover_state()
    for _ in range(800):  # 1 s at 800 Hz, perfect hover trim
        state = plant.step(state, 1.0 / 800.0, rotor, np.zeros((1, 3)), 1.225)
    # residual ground effect at z/R ~ 560 leaves ~2e-7 mg excess thrust
    assert abs(state[0, 2] - 100.0) < 5e-6
    assert np.abs(state[0, rb.VEL]).max() < 5e-6


def test_step_latched_matches_live_wrench_path():
    """step() latches thrust/GE/moments per step; away from ground effect this
    must match the fully-live wrench integration to integrator round-off."""
    rng = np.random.default_rng(45)
    params, plant = make_plant(3)
    state = rng.normal(size=(3, rb.STATE_DIM))
    state[:, 2] = rng.uniform(50.0, 150.0, size=3)        # OGE: GE gain ~ 1
    state[:, rb.QUAT] = rb.quat_normalize(state[:, rb.QUAT])
    rotor = rng.uniform(400.0, 1200.0, size=(3, params.n_rotors))
    wind = rng.normal(scale=4.0, size=(3, 3))
    latched = plant.step(state, 1 / 800, rotor, wind, 1.225)
    live = rb.rk4_step(state, 1 / 800, plant.wrench_fn(rotor, wind, 1.225),
                       plant.mass, plant.inertia, plant.inertia_inv)
    np.testing.assert_allclose(latched, live, rtol=0, atol=1e-10)


def test_params_yaml_loads_and_is_consistent():
    cfg = load_airframe("interceptor_quad")
    params = MultirotorParams.from_dict(cfg)
    assert params.mass == 12.0
    assert params.n_rotors == 4
    assert params.rotor_positions.shape == (4, 3)
    assert int(np.sum(params.rotor_spin)) == 0           # balanced CCW/CW
    assert params.kf > 0 and params.km > 0
    assert params.motor["k_q"] == params.km          # same prop loads motor and yaw
    # motor/battery blocks wired for P1-3 models
    motor = MotorEsc(1, params.n_rotors, **params.motor)
    batt = BatteryEcm(1, **params.battery)
    assert motor.omega.shape == (1, 4)
    assert batt.ocv(np.array([1.0]))[0] > 49.0
