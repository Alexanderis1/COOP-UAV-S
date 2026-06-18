"""P1-3: motor/ESC first-order-lag rotor dynamics + Thevenin 1-RC battery ECM.

Motor: ESC average-value chopper (V_m = throttle * V_bus), DC-motor electrics
i = (V_m - Ke w)/R_w with Ke = Kt = 1/KV_rad, prop load k_q w^2, J_r w_dot.
Pins: step time constant inside the 15-50 ms band, full-throttle speed
ceiling tracks a sagging bus voltage, steady-state torque balance,
batch==scalar.

Battery: V_t = OCV(SOC) - I R0 - V1, V1' = -V1/(R1 C1) + I/C1 with the exact
zero-order-hold discrete update. Pins: instant sag = I*R0, recovery follows
exp(-t/tau1), coulomb integral exact, OCV monotone in SOC.

Powertrain: closed-form implicit solve of the motor-battery algebraic bus
loop (one-step-lag composition has loop gain R0 sum(theta^2)/R_w > 1 above
~hover throttle and diverges at any dt). Pins: explicit composition diverges
while Powertrain stays bounded, fixed point satisfies both component
equations, inrush clamped at i_bus_max_a, bus voltage held to 3.0-4.2 V/cell,
10 s closed loop finite with SOC monotone, batch==scalar, YAML limit values.
"""

from __future__ import annotations

import numpy as np
import pytest

from coopuavs.physics.battery import BatteryEcm
from coopuavs.physics.motor import MotorEsc
from coopuavs.physics.params import load_airframe
from coopuavs.physics.powertrain import V_CELL_MAX, V_CELL_MIN, Powertrain

# Interceptor-class motor (invented-but-self-consistent, pinned here and in P1-4):
# 320 KV, 12S, ~14" prop. Ke = 60/(2 pi 320) ~ 0.0298 V s/rad.
KV = 320.0
R_W = 0.040          # ohm
J_R = 4.5e-4         # kg m^2 rotor+prop
K_Q = 1.0e-6         # N m / (rad/s)^2 prop drag torque coefficient
V_BUS = 44.4         # 12S nominal
KE = 60.0 / (2.0 * np.pi * KV)


def make_motor(n=1, rotors=4):
    return MotorEsc(n, rotors, kv_rpm_per_v=KV, r_w=R_W, j_r=J_R, k_q=K_Q)


def settle(motor, throttle, v_bus, t=1.5, dt=1e-4):
    for _ in range(round(t / dt)):
        omega, _ = motor.step(dt, throttle, v_bus)
    return omega


# ------------------------------------------------------------------------ motor


def test_step_response_tau_in_15_50ms_band():
    motor = make_motor()
    throttle = np.full((1, 4), 0.6)
    v_bus = np.array([V_BUS])
    omega_ss = settle(make_motor(), throttle, v_bus)[0, 0]
    dt, t, target = 1e-4, 0.0, 0.632 * omega_ss
    while motor.omega[0, 0] < target:
        motor.step(dt, throttle, v_bus)
        t += dt
        assert t < 0.2, "step response did not rise"
    assert 0.015 <= t <= 0.050, f"tau {t * 1e3:.1f} ms outside 15-50 ms"


def test_omega_ceiling_tracks_sagging_voltage():
    ceilings = []
    for v in (50.4, 44.4, 39.6):  # 12S: full / nominal / sagged
        omega = settle(make_motor(), np.ones((1, 4)), np.array([v]))
        ceilings.append(omega[0, 0])
        assert omega[0, 0] < v / KE  # back-EMF bound
    assert ceilings[0] > ceilings[1] > ceilings[2]
    # sag ratio roughly follows voltage ratio (quadratic load -> sublinear)
    assert ceilings[2] / ceilings[0] > 0.7


def test_steady_state_torque_balance():
    motor = make_motor()
    throttle = np.full((1, 4), 0.5)
    v_bus = np.array([V_BUS])
    omega = settle(motor, throttle, v_bus)[0, 0]
    i = (0.5 * V_BUS - KE * omega) / R_W
    assert abs(KE * i - K_Q * omega**2) < 1e-3 * K_Q * omega**2 + 1e-9


def test_bus_current_positive_under_load_and_scales():
    motor = make_motor()
    throttle = np.full((1, 4), 0.6)
    v_bus = np.array([V_BUS])
    settle(motor, throttle, v_bus)
    _, i_bus = motor.step(1e-4, throttle, v_bus)
    assert i_bus.shape == (1,)
    assert i_bus[0] > 0.0
    # 4 rotors at identical state -> bus current = 4 * throttle * motor current
    omega = motor.omega[0, 0]
    i_m = (0.6 * V_BUS - KE * omega) / R_W
    np.testing.assert_allclose(i_bus[0], 4 * 0.6 * i_m, rtol=1e-6)


def test_zero_throttle_spins_down_to_rest():
    motor = make_motor()
    settle(motor, np.full((1, 4), 0.7), np.array([V_BUS]))
    omega = settle(motor, np.zeros((1, 4)), np.array([V_BUS]), t=2.0)
    assert np.all(omega >= 0.0)
    assert np.all(omega < 1.0)


def test_motor_batch_equals_scalar():
    rng = np.random.default_rng(33)
    n = 5
    throttle = rng.uniform(0.2, 0.9, size=(n, 4))
    v_bus = rng.uniform(40.0, 50.0, size=n)
    batched = make_motor(n)
    for _ in range(200):
        batched.step(1e-3, throttle, v_bus)
    for i in range(n):
        single = make_motor(1)
        for _ in range(200):
            single.step(1e-3, throttle[i:i + 1], v_bus[i:i + 1])
        np.testing.assert_allclose(batched.omega[i], single.omega[0], rtol=0, atol=1e-12)


# ---------------------------------------------------------------------- battery


def make_batt(n=1, soc0=0.9):
    # 12S 16 Ah pack: R0 36 mOhm, R1 18 mOhm, tau1 = 15 s
    return BatteryEcm(n, capacity_ah=16.0, n_series=12, r0=0.036, r1=0.018,
                      c1=15.0 / 0.018, soc0=soc0)


def test_instant_sag_equals_i_r0():
    batt = make_batt()
    ocv = batt.ocv(batt.soc)[0]
    v = batt.step(1e-6, np.array([60.0]))[0]
    assert abs((ocv - v) - 60.0 * 0.036) < 1e-3  # V1 contribution ~ I*dt/C1 ~ 7e-5 V


def test_rc_recovery_time_constant():
    batt = make_batt()
    for _ in range(round(300.0 / 0.05)):           # 300 s = 20 tau1: V1 settled
        batt.step(0.05, np.array([40.0]))
    v1_0 = batt.v1[0]
    assert abs(v1_0 - 40.0 * 0.018) < 1e-6         # settled V1 = I R1
    for _ in range(round(15.0 / 0.05)):            # rest exactly tau1 seconds
        batt.step(0.05, np.array([0.0]))
    assert abs(batt.v1[0] - v1_0 * np.exp(-1.0)) < 1e-9 * v1_0 + 1e-12


def test_coulomb_integral_exact():
    batt = make_batt(soc0=1.0)
    for _ in range(7200):                          # 360 s at 20 A, dt = 0.05
        batt.step(0.05, np.array([20.0]))
    expected = 1.0 - 20.0 * 360.0 / (3600.0 * 16.0)
    assert abs(batt.soc[0] - expected) < 1e-12


def test_ocv_monotone_and_12s_range():
    batt = make_batt()
    soc = np.linspace(0.0, 1.0, 101)
    ocv = batt.ocv(soc)
    assert np.all(np.diff(ocv) >= 0.0)
    assert 38.0 < ocv[0] < 41.0                    # 12 cells empty-ish
    assert 49.0 < ocv[-1] < 51.0                   # 12 cells full


def test_battery_batch_independent():
    batt = make_batt(n=3, soc0=1.0)
    current = np.array([0.0, 20.0, 60.0])
    for _ in range(2000):
        v = batt.step(0.05, current)
    assert batt.soc[0] == 1.0
    assert batt.soc[0] > batt.soc[1] > batt.soc[2]
    assert v[0] > v[1] > v[2]


def test_soc_clamped_at_zero():
    batt = BatteryEcm(1, capacity_ah=0.1, n_series=12, r0=0.036, r1=0.018,
                      c1=100.0, soc0=0.05)
    for _ in range(2000):
        batt.step(1.0, np.array([50.0]))
    assert batt.soc[0] == 0.0
    assert np.isfinite(batt.step(0.05, np.array([50.0]))[0])


def test_soc_charge_side_clamped_at_full():
    """Regen (negative current) into a full pack must not push SOC above 1.

    Mutation pin: the upper bound of the SOC clip (battery.py) was previously
    exercised by no test because no test applied charge current.
    """
    batt = make_batt(soc0=1.0)
    v = batt.step(0.05, np.array([-30.0]))
    for _ in range(200):
        v = batt.step(0.05, np.array([-30.0]))
    assert batt.soc[0] == 1.0
    assert v[0] > batt.ocv(np.array([1.0]))[0]   # charging raises terminal V


def test_throttle_clipped_to_unit_interval():
    """Mutation pin: np.clip(throttle, 0, 1) at both motor.step() sites.

    throttle > 1 must behave exactly like 1.0 (ESC duty saturates) and
    throttle < 0 exactly like 0.0; previously no test left [0, 1].
    """
    over, one = make_motor(), make_motor()
    v_bus = np.array([V_BUS])
    for _ in range(300):
        w_over, i_over = over.step(1e-4, np.full((1, 4), 1.2), v_bus)
        w_one, i_one = one.step(1e-4, np.ones((1, 4)), v_bus)
    np.testing.assert_array_equal(w_over, w_one)
    np.testing.assert_array_equal(i_over, i_one)

    neg, zero = make_motor(), make_motor()
    settle(neg, np.full((1, 4), 0.7), v_bus)
    settle(zero, np.full((1, 4), 0.7), v_bus)
    for _ in range(300):
        w_neg, i_neg = neg.step(1e-4, np.full((1, 4), -0.3), v_bus)
        w_zero, i_zero = zero.step(1e-4, np.zeros((1, 4)), v_bus)
    np.testing.assert_array_equal(w_neg, w_zero)
    np.testing.assert_array_equal(i_neg, i_zero)


# ------------------------------------------------------------------- powertrain
#
# Quasi-static armature + instantaneous R0 feedthrough form an algebraic loop
# with gain R0 sum(theta^2)/R_w = 3.6 theta^2 for this motor/pack set: any
# one-step-lag composition of MotorEsc and BatteryEcm diverges above
# theta ~ 0.53 at ANY dt. Powertrain solves the loop in closed form and
# enforces the bus current limit and the 3.0/4.2 V-per-cell bounds.

I_BUS_MAX = 350.0    # A, pinned == params/interceptor_quad.yaml powertrain block


def make_powertrain(n=1, soc0=1.0):
    return Powertrain(make_motor(n), make_batt(n, soc0=soc0),
                      i_bus_max_a=I_BUS_MAX)


def powertrain_from_yaml(name, n=1):
    cfg = load_airframe(name)
    motor_kwargs = dict(cfg["motor"])
    motor_kwargs["k_q"] = float(cfg["rotors"]["km"])
    motor = MotorEsc(n, int(cfg["rotors"]["count"]), **motor_kwargs)
    batt = BatteryEcm(n, **cfg["battery"])
    return Powertrain(motor, batt, **cfg["powertrain"])


def test_explicit_lagged_composition_diverges_powertrain_does_not():
    """The naive wiring the module docstrings describe is algebraically
    unstable: at theta = 0.6 the loop gain is 1.296 > 1, so v_bus/i_bus
    oscillate with geometric growth regardless of dt. Documents WHY the
    implicit solve exists; the same condition through Powertrain is bounded.
    """
    throttle = np.full((1, 4), 0.6)
    motor, batt = make_motor(), make_batt(soc0=1.0)
    v_bus = np.array([V_BUS])
    diverged = False
    for _ in range(2000):                          # 0.2 s at dt = 1e-4
        _, i_bus = motor.step(1e-4, throttle, v_bus)
        v_bus = batt.step(1e-4, i_bus)
        if not np.all(np.isfinite(v_bus)) or abs(v_bus[0]) > 1e3:
            diverged = True
            break
    assert diverged, "explicit one-step-lag composition unexpectedly stable"

    pt = make_powertrain()
    for _ in range(2000):
        omega, v, i = pt.step(1e-4, throttle)
    assert np.all(np.isfinite(omega))
    assert pt.v_bus_min <= v[0] <= pt.v_bus_max
    assert 40.0 <= v[0] <= 50.4                    # mild sag, not rail-pinned
    assert 0.0 < i[0] < I_BUS_MAX


def test_bus_fixed_point_satisfies_both_component_equations():
    """solve_bus() output satisfies the armature/chopper equation AND the
    battery terminal equation simultaneously (rtol 1e-12): the loop is solved
    implicitly, not iterated.
    """
    pt = make_powertrain()
    theta = np.full((1, 4), 0.6)
    for _ in range(5000):                          # settle 0.5 s (tau ~ 30 ms)
        pt.step(1e-4, theta)
    v, i = pt.solve_bus(theta)
    s = np.sum(theta * theta, axis=1)
    bemf = KE * np.sum(theta * pt.motor.omega, axis=1)
    np.testing.assert_allclose(i, (s * v - bemf) / R_W, rtol=1e-12, atol=0)
    np.testing.assert_allclose(
        v, pt.battery.ocv(pt.battery.soc) - i * 0.036 - pt.battery.v1,
        rtol=1e-12, atol=0)
    # unclamped step applies exactly the solved pair
    _, v_step, i_step = pt.step(1e-4, theta)
    np.testing.assert_allclose(v_step, v, rtol=1e-12)
    np.testing.assert_allclose(i_step, i, rtol=1e-12)


def test_closed_loop_10s_bounded_and_soc_monotone():
    """10 s closed loop at dt = 1e-4 for theta in {0.3, 0.6, 1.0} (batched):
    no NaN/inf ever, v_bus inside the cell-voltage bounds every step, SOC
    monotone non-increasing, pack draw monotone in throttle.
    """
    theta = np.array([[0.3] * 4, [0.6] * 4, [1.0] * 4])
    pt = make_powertrain(n=3)
    v_lo = np.full(3, np.inf)
    v_hi = np.full(3, -np.inf)
    soc_hist = [pt.battery.soc.copy()]
    for k in range(100_000):                       # 10 s
        omega, v_bus, i_bus = pt.step(1e-4, theta)
        v_lo = np.minimum(v_lo, v_bus)
        v_hi = np.maximum(v_hi, v_bus)
        if k % 2000 == 1999:
            assert np.all(np.isfinite(omega))
            assert np.all(np.isfinite(v_bus)) and np.all(np.isfinite(i_bus))
            soc_hist.append(pt.battery.soc.copy())
    assert np.all(v_lo >= V_CELL_MIN * 12) and np.all(v_hi <= V_CELL_MAX * 12)
    soc = np.array(soc_hist)
    assert np.all(np.diff(soc, axis=0) <= 0.0)     # monotone non-increasing
    assert np.all(soc[-1] < soc[0])                # actually discharging
    assert 0.0 < i_bus[0] < i_bus[1] < i_bus[2] < I_BUS_MAX
    assert 150.0 < i_bus[2]                        # near the steady ~230 A draw


def test_inrush_clamped_at_spinup_from_rest():
    """Full-throttle spin-up from rest: the unconstrained algebraic solution
    is ~1090 A (would be 4440 A on a rigid bus); the bus limit clamps it and
    v_bus = OCV - V1 - R0 * i_clamped.
    """
    pt = make_powertrain()
    _, i_star = pt.solve_bus(np.ones((1, 4)))
    assert i_star[0] > 1000.0                      # unconstrained inrush
    ocv0 = pt.battery.ocv(pt.battery.soc)[0]       # V1 = 0 at rest
    _, v_bus, i_bus = pt.step(1e-4, np.ones((1, 4)))
    assert i_bus[0] == I_BUS_MAX
    assert abs(v_bus[0] - (ocv0 - 0.036 * I_BUS_MAX)) < 1e-9
    assert v_bus[0] >= pt.v_bus_min


def test_bus_voltage_floor_and_ceiling():
    # Floor: near-empty pack under clamped full-throttle load would sag to
    # ~29 V; the powertrain holds the bus at the 3.0 V/cell cutoff.
    pt = make_powertrain(soc0=0.05)
    _, v_bus, i_bus = pt.step(1e-4, np.ones((1, 4)))
    assert i_bus[0] == I_BUS_MAX
    assert v_bus[0] == V_CELL_MIN * 12

    # Ceiling: throttle chop at speed regenerates into a near-full pack; the
    # raw terminal voltage exceeds 4.2 V/cell and is clamped.
    pt = make_powertrain(soc0=1.0)
    for _ in range(10_000):                        # settle at full throttle
        pt.step(1e-4, np.ones((1, 4)))
    _, v_bus, i_bus = pt.step(1e-4, np.full((1, 4), 0.05))
    assert i_bus[0] < -50.0                        # regen into the pack
    assert v_bus[0] == V_CELL_MAX * 12
    assert pt.battery.soc[0] <= 1.0


def test_powertrain_batch_equals_scalar():
    rng = np.random.default_rng(7)
    theta = rng.uniform(0.2, 1.0, size=(3, 4))
    pt = make_powertrain(n=3)
    for _ in range(500):
        _, v, i = pt.step(1e-4, theta)
    for k in range(3):
        single = make_powertrain()
        for _ in range(500):
            _, v1, i1 = single.step(1e-4, theta[k:k + 1])
        np.testing.assert_allclose(pt.motor.omega[k], single.motor.omega[0],
                                   rtol=0, atol=1e-12)
        np.testing.assert_allclose(v[k], v1[0], rtol=0, atol=1e-12)
        np.testing.assert_allclose(i[k], i1[0], rtol=0, atol=1e-12)
        np.testing.assert_allclose(pt.battery.soc[k], single.battery.soc[0],
                                   rtol=0, atol=1e-15)


def test_powertrain_yaml_limits_pinned_and_sized():
    """i_bus_max_a in both airframe YAMLs is pinned and sized ~1.5x the
    self-consistent steady full-throttle pack draw (steady draw lands around
    2/3 of the limit for both airframes).
    """
    for name, limit, n_series in (("interceptor_quad", 350.0, 12),
                                  ("fpv_quad", 125.0, 6)):
        pt = powertrain_from_yaml(name)
        assert pt.i_bus_max_a == limit
        assert pt.v_bus_min == V_CELL_MIN * n_series
        assert pt.v_bus_max == V_CELL_MAX * n_series
        for _ in range(10_000):                    # 1 s >> motor tau
            _, v_bus, i_bus = pt.step(1e-4, np.ones((1, 4)))
        assert 0.55 * limit < i_bus[0] < 0.80 * limit
        assert pt.v_bus_min <= v_bus[0] <= pt.v_bus_max


def test_powertrain_rejects_mismatched_batch_size():
    with pytest.raises(ValueError):
        Powertrain(make_motor(2), make_batt(1), i_bus_max_a=I_BUS_MAX)
