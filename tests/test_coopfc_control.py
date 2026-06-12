"""P3-5: cascade control + mixer acceptance against the REAL plant.

Closed loop = coopfc controllers (plain-float, truth-fed: controller
acceptance only — the EKF joins at the P3-8 bench) -> QuadXMixer ->
P1 Powertrain (motor lag ~20 ms, implicit DC bus) -> MultirotorPlant
RK4 at 800 Hz, control at 400 Hz, velocity loop at 50 Hz.

Plan tolerances (tests-first spec, P3-5):
- roll/pitch rate step: 10-90% rise < 60 ms, overshoot < 20%
- yaw rate step 0.5 rad/s: settle < 0.4 s (yaw authority is ~30x
  weaker — drag-torque actuation, ~10 rad/s^2 per unit demand — so the
  60 ms figure is physically a roll/pitch spec)
- 30 deg attitude step: settle into +-2 deg < 0.5 s
- velocity: zero steady-state error (calm + 5 m/s steady wind)
- anti-windup: integrator frozen under saturation (white-box pin) and
  bounded recovery after a 2.5 s saturated dash command
- mixer desaturation: exact-arithmetic priority pins (rp > thrust > yaw)
- determinism: run-twice bit-identical closed loop
"""

from __future__ import annotations

import math

import numpy as np

from coopuavs.coopfc.control import (
    AttCtl, MixFlags, QuadXMixer, RateCtl, VelCtl, VelParams,
)
from coopuavs.coopfc.core import vec
from coopuavs.physics.battery import BatteryEcm
from coopuavs.physics.motor import MotorEsc
from coopuavs.physics.multirotor import MultirotorParams, MultirotorPlant
from coopuavs.physics.params import load_airframe
from coopuavs.physics.powertrain import Powertrain

PHYS_HZ = 800
CTL_EVERY = 2          # rate/attitude at 400 Hz
VEL_EVERY = 16         # velocity at 50 Hz
DT = 1.0 / PHYS_HZ
RHO = 1.225


class Bench:
    """One interceptor_quad hovering at 50 m, trimmed (rotors pre-spun)."""

    def __init__(self):
        cfg = MultirotorParams.from_dict(load_airframe("interceptor_quad"))
        self.cfg = cfg
        self.plant = MultirotorPlant(cfg, 1)
        motor = MotorEsc(1, cfg.n_rotors, **cfg.motor)
        battery = BatteryEcm(1, **cfg.battery)
        self.pt = Powertrain(motor, battery, i_bus_max_a=350.0)
        w_h = math.sqrt(cfg.mass * 9.81 / (cfg.n_rotors * cfg.kf))
        motor.omega[:] = w_h
        self.state = np.zeros((1, 13))
        self.state[0, 2] = 50.0
        self.state[0, 6] = 1.0
        self.rate = RateCtl()
        self.att = AttCtl()
        self.vel = VelCtl()
        self.mixer = QuadXMixer()
        self.throttle = np.full((1, cfg.n_rotors), VelParams().u_hover)
        self.sat = (0, 0, 0)
        self.k = 0
        self._q_sp = None
        self._thrust = None

    @property
    def t(self) -> float:
        return self.k * DT

    # one 800 Hz physics step; control refreshed on its own divisors
    def step(self, *, v_sp=None, q_sp=None, rate_sp=None, thrust=None,
             yaw_sp=0.0, wind=(0.0, 0.0, 0.0)):
        if self.k % CTL_EVERY == 0:
            s = self.state[0]
            q = (s[6], s[7], s[8], s[9])
            omega = (s[10], s[11], s[12])
            if v_sp is not None and (self.k % VEL_EVERY == 0
                                     or self._q_sp is None):
                self._q_sp, self._thrust = self.vel.update(
                    v_sp, (s[3], s[4], s[5]), yaw_sp, VEL_EVERY * DT)
            if v_sp is not None:
                q_sp, thrust = self._q_sp, self._thrust
            if q_sp is not None:
                rate_sp = self.att.update(q_sp, q)
            torque = self.rate.update(rate_sp, omega, CTL_EVERY * DT,
                                      self.sat)
            u, flags = self.mixer.mix(thrust, torque)
            self.sat = flags.axis_sat
            self.throttle[0, :] = u
        omega_r, _, _ = self.pt.step(DT, self.throttle)
        wind_w = np.array([wind])
        self.state = self.plant.step(self.state, DT, omega_r, wind_w, RHO)
        self.k += 1

    def run(self, t_span: float, **kw):
        for _ in range(round(t_span * PHYS_HZ)):
            self.step(**kw)


def euler(state) -> tuple[float, float, float]:
    s = state[0]
    return vec.quat_to_euler((s[6], s[7], s[8], s[9]))


# ------------------------------------------------------------- rate loop


def _rate_step_metrics(axis: int, sp: float, t_span: float):
    b = Bench()
    sp_v = [0.0, 0.0, 0.0]
    sp_v[axis] = sp
    u_h = VelParams().u_hover
    hist = []
    for _ in range(round(t_span * PHYS_HZ)):
        b.step(rate_sp=tuple(sp_v), thrust=u_h)
        hist.append((b.t, b.state[0, 10 + axis]))
    t10 = next(t for t, w in hist if w >= 0.1 * sp)
    t90 = next(t for t, w in hist if w >= 0.9 * sp)
    peak = max(w for _, w in hist)
    return t90 - t10, peak / sp - 1.0


def test_rate_step_roll_rise_and_overshoot():
    rise, overshoot = _rate_step_metrics(0, 2.0, 0.30)
    assert rise < 0.060, f"roll rate rise {rise * 1e3:.1f} ms"
    assert overshoot < 0.20, f"roll rate overshoot {overshoot:.2%}"


def test_rate_step_pitch_rise_and_overshoot():
    rise, overshoot = _rate_step_metrics(1, 2.0, 0.30)
    assert rise < 0.060, f"pitch rate rise {rise * 1e3:.1f} ms"
    assert overshoot < 0.20, f"pitch rate overshoot {overshoot:.2%}"


def test_rate_step_yaw_settles():
    # Yaw authority is ~30x weaker than roll/pitch (drag-torque
    # actuation), so the plan's 60 ms rise figure is a roll/pitch spec.
    # User decision 2026-06-12 (P3 gate review): yaw is gated as a
    # REGRESSION gate at 0.20 s settle — measured 0.138 s, deterministic
    # truth-fed bench, +45% headroom — replacing the unstamped 0.40 s
    # whose 2.9x headroom would have passed a tripled settle time.
    b = Bench()
    u_h = VelParams().u_hover
    hist = []
    for _ in range(round(0.8 * PHYS_HZ)):
        b.step(rate_sp=(0.0, 0.0, 0.5), thrust=u_h)
        hist.append((b.t, b.state[0, 12]))
    inside = [t for t, w in hist if abs(w - 0.5) > 0.05]
    t_settle = max(inside) if inside else 0.0
    assert t_settle < 0.20, f"yaw rate settle {t_settle:.2f} s"
    peak = max(w for _, w in hist)
    assert peak / 0.5 - 1.0 < 0.20, "yaw rate overshoot"


# --------------------------------------------------------- attitude loop


def test_attitude_step_30deg_settles_in_half_second():
    b = Bench()
    target = math.radians(30.0)
    q_sp = vec.quat_from_euler(target, 0.0, 0.0)
    thrust = VelParams().u_hover / math.cos(target)
    bad = 0.0
    for _ in range(round(1.0 * PHYS_HZ)):
        b.step(q_sp=q_sp, thrust=thrust)
        roll, _, _ = euler(b.state)
        if abs(roll - target) > math.radians(2.0):
            bad = b.t
    assert bad < 0.5, f"attitude settle {bad:.2f} s"


# --------------------------------------------------------- velocity loop


def test_velocity_step_zero_steady_state_error():
    b = Bench()
    errs = []
    for _ in range(round(6.0 * PHYS_HZ)):
        b.step(v_sp=(3.0, 0.0, 0.0))
        if b.t > 5.0:
            s = b.state[0]
            errs.append(math.hypot(s[3] - 3.0, s[4]) + abs(s[5]))
    assert max(errs) < 0.05, f"vel SS error {max(errs):.3f} m/s"


def test_velocity_hold_zero_ss_error_in_steady_wind():
    b = Bench()
    errs = []
    for _ in range(round(6.0 * PHYS_HZ)):
        b.step(v_sp=(0.0, 0.0, 0.0), wind=(5.0, 0.0, 0.0))
        if b.t > 5.0:
            s = b.state[0]
            errs.append(math.hypot(s[3], s[4]) + abs(s[5]))
    assert max(errs) < 0.05, f"wind-hold SS error {max(errs):.3f} m/s"
    roll, pitch, _ = euler(b.state)
    assert abs(pitch) > math.radians(0.2)  # trimmed into the wind


def test_hover_hold_stays_level():
    b = Bench()
    for _ in range(round(5.0 * PHYS_HZ)):
        b.step(v_sp=(0.0, 0.0, 0.0))
    s = b.state[0]
    assert math.hypot(s[3], s[4]) + abs(s[5]) < 0.02
    roll, pitch, _ = euler(b.state)
    assert max(abs(roll), abs(pitch)) < math.radians(0.5)
    assert abs(s[2] - 50.0) < 0.5


def test_vertical_brake_survives_horizontal_chatter():
    """P4 gate-review finding (user decision 2026-06-12, fidelity-first):
    braking a fast climb means low specific force (fz = g - a_down), so
    the tilt cone lets ANY cone-saturating horizontal demand command
    ±45° — and a sign-flipping horizontal error then flips the attitude
    setpoint at 50 Hz. The rate loop slams torque chasing steps no
    airframe can follow, the mixer's rp-priority desat drags average
    collective back to hover, and the vertical brake is LOST (~90 m
    overshoot measured in the fleet engine). The attitude-setpoint slew
    limit makes the setpoint followable; vertical priority then holds
    end-to-end."""
    b = Bench()
    b.state[0, 5] = 15.0                  # climbing hard
    sign = 1.0
    for k in range(round(3.0 * PHYS_HZ)):
        if k % VEL_EVERY == 0:
            sign = -sign                  # worst case: cone-saturating
        b.step(v_sp=(3.0 * sign, 0.0, -20.0))
    vz = b.state[0, 5]
    assert vz < -5.0, f"vz {vz:+.1f} m/s after 3 s — brake lost to flailing"
    roll, pitch, _ = euler(b.state)
    assert max(abs(roll), abs(pitch)) < math.radians(50.0)


# ------------------------------------------------------------ anti-windup


def test_rate_integrator_frozen_while_saturated():
    ctl = RateCtl()
    for _ in range(400):  # 1 s at 400 Hz, mixer reports high-saturated
        ctl.update((0.5, 0.0, 0.0), (0.0, 0.0, 0.0), 1 / 400, sat=(1, 0, 0))
    assert ctl.i[0] == 0.0  # conditional integration: frozen, not clamped
    ctl.update((0.5, 0.0, 0.0), (0.0, 0.0, 0.0), 1 / 400, sat=(0, 0, 0))
    assert ctl.i[0] > 0.0   # resumes the tick saturation clears


def test_antiwindup_dash_recovery():
    b = Bench()
    for _ in range(round(2.5 * PHYS_HZ)):   # tilt-limit saturated dash
        b.step(v_sp=(25.0, 0.0, 0.0))
    crossed = None
    vmin = math.inf
    errs = []
    for _ in range(round(4.0 * PHYS_HZ)):
        b.step(v_sp=(5.0, 0.0, 0.0))
        vx = b.state[0, 3]
        if crossed is None and vx <= 5.5:
            crossed = b.t
        if crossed is not None:
            vmin = min(vmin, vx)
        if b.t > 2.5 + 3.0:
            errs.append(abs(vx - 5.0))
    assert crossed is not None and crossed - 2.5 < 2.0, "slow reversal"
    assert vmin > 3.5, f"windup undershoot to {vmin:.2f} m/s"
    assert max(errs) < 0.2, f"post-recovery error {max(errs):.2f} m/s"


# ----------------------------------------------------------------- mixer


def test_mixer_clean_mix_exact():
    u, flags = QuadXMixer().mix(0.5, (0.1, 0.0, 0.0))
    assert u == (0.4, 0.6, 0.6, 0.4)        # +roll raises left pair
    assert flags == MixFlags(False, False, False, False, (0, 0, 0))
    u, _ = QuadXMixer().mix(0.5, (0.0, 0.1, 0.0))
    assert u == (0.4, 0.6, 0.4, 0.6)        # +pitch raises rear pair
    u, _ = QuadXMixer().mix(0.5, (0.0, 0.0, 0.1))
    assert u == (0.4, 0.4, 0.6, 0.6)        # +yaw raises CW pair


def test_mixer_collective_shift_preserves_roll():
    u, flags = QuadXMixer().mix(0.95, (0.1, 0.0, 0.0))
    assert abs(max(u) - 1.0) < 1e-12 and abs(u[1] - u[0] - 0.2) < 1e-12
    assert flags.sat_hi and not flags.yaw_scaled and not flags.rp_scaled


def test_mixer_sheds_yaw_before_roll():
    u, flags = QuadXMixer().mix(0.5, (0.45, 0.0, 0.3))
    assert flags.yaw_scaled and not flags.rp_scaled
    # roll fully preserved: left minus right differential = 2*0.45
    assert abs((u[1] + u[2]) - (u[0] + u[3]) - 4 * 0.45) < 1e-12
    assert min(u) >= 0.0 and max(u) <= 1.0
    # analytic: k = 1/6 -> y -> 0.05; FR/BL lose it, FL/BR gain it
    expect = (0.0, 0.9, 1.0, 0.1)
    assert all(abs(a - b) < 1e-12 for a, b in zip(u, expect))


def test_mixer_scales_roll_pitch_last():
    u, flags = QuadXMixer().mix(0.5, (0.8, 0.0, 0.0))
    assert flags.rp_scaled
    assert u == (0.0, 1.0, 1.0, 0.0)        # span exactly fits the band


# ----------------------------------------------------------- determinism


def test_closed_loop_run_twice_bit_identical():
    def run():
        b = Bench()
        b.run(1.0, v_sp=(2.0, -1.0, 0.5))
        return b.state.tobytes()

    assert run() == run()
