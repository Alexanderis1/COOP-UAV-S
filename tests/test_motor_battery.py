"""P1-3: motor/ESC first-order-lag rotor dynamics + Thevenin 1-RC battery ECM.

Motor: ESC average-value chopper (V_m = throttle * V_bus), DC-motor electrics
i = (V_m - Ke w)/R_w with Ke = Kt = 1/KV_rad, prop load k_q w^2, J_r w_dot.
Pins: step time constant inside the 15-50 ms band, full-throttle speed
ceiling tracks a sagging bus voltage, steady-state torque balance,
batch==scalar.

Battery: V_t = OCV(SOC) - I R0 - V1, V1' = -V1/(R1 C1) + I/C1 with the exact
zero-order-hold discrete update. Pins: instant sag = I*R0, recovery follows
exp(-t/tau1), coulomb integral exact, OCV monotone in SOC.
"""

from __future__ import annotations

import numpy as np

from coopuavs.physics.battery import BatteryEcm
from coopuavs.physics.motor import MotorEsc

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
