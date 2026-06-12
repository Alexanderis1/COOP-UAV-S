"""P3-6: FCU boot/PBIT/arming/modes + battery monitor + failsafes.

Two harnesses:

- SynthHost: still, perfect sensor frames at the device rates (no
  physics) — drives the FSM/PBIT/failsafe timeline pins. The vehicle
  never moves; actuator output is ignored.
- FlightHost (integration): the P3-5 plant/powertrain bench behind the
  HAL — sensor frames synthesized from plant truth (perfect, zero
  latency: device errors are the P2/P3-8 suites' business; this test
  is the FCU pipeline + EKF + cascade flying a real airframe). Plant
  frozen until arming (bench convention), motors pre-spun at hover.

Plan pins: PBIT-blocks-arming; OFFBOARD setpoint-timeout -> POS_HOLD;
link-loss -> RTL timeline (tick-exact at the 50 Hz nav task); RTL home
from 2 km under wind (@slow; a short RTL flight stays in the fast
suite); battery LOW -> RTL, CRITICAL -> LAND with debounce, and
CRITICAL beating link-loss.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from coopuavs.coopfc.core import vec
from coopuavs.coopfc.drivers.baro import (
    _EXP, _ISA_LAPSE, _ISA_P0, _ISA_T0,
)
from coopuavs.coopfc.fcu import (
    ARMED, BOOT, LAND, OFFBOARD, POS_HOLD, RTL, STANDBY, TICK_HZ, Fcu,
)
from coopuavs.coopfc.hal import HalIO
from coopuavs.physics.battery import BatteryEcm
from coopuavs.physics.motor import MotorEsc
from coopuavs.physics.multirotor import MultirotorParams, MultirotorPlant
from coopuavs.physics.params import load_airframe
from coopuavs.physics.powertrain import Powertrain

DT = 1.0 / TICK_HZ
G = 9.81

# Theater field (EkfParams defaults), ENU.
_D, _I = math.radians(4.0), math.radians(63.0)
B_ENU = (50.0 * math.cos(_I) * math.sin(_D),
         50.0 * math.cos(_I) * math.cos(_D),
         -50.0 * math.sin(_I))


def pressure_at(alt_m: float) -> float:
    return _ISA_P0 * (1.0 - _ISA_LAPSE * alt_m / _ISA_T0) ** (1.0 / _EXP)


class SynthHost:
    """Still vehicle at `pos`, perfect frames, no physics."""

    def __init__(self, pos=(0.0, 0.0, 50.0), v_cell=4.0):
        self.hal = HalIO()
        self.fcu = Fcu(self.hal)
        self.pos = pos
        self.v_cell = v_cell
        self.mag_on = True
        self.k = 0

    @property
    def now(self) -> float:
        return self.k * DT

    def run(self, t_span: float, vibrate=0.0, hb_every=None):
        for _ in range(round(t_span * TICK_HZ)):
            k, now = self.k, self.now
            if k % 2 == 0:
                # 1e-6 rad/s non-repeating dither: live MEMS noise never
                # repeats a sample exactly, and the P5 GYRO_STUCK monitor
                # (correctly) reads a perfectly-constant stream as a
                # stuck sensor and inhibits arming. Far below every
                # alignment/EKF tolerance in this suite.
                g = (vibrate if (k // 2) % 2 == 0 else -vibrate) \
                    + 1e-6 * math.sin(0.7 * k)
                self.hal.port("imu").write(((g, -g, g), (0.0, 0.0, G)))
            if k % 80 == 0:
                self.hal.port("gps").write(
                    (self.pos, (0.0, 0.0, 0.0), 3, now))
            if k % 16 == 0:
                self.hal.port("baro").write(pressure_at(self.pos[2]))
                if self.mag_on:
                    self.hal.port("mag").write(B_ENU)
            if k % 80 == 0:
                # 5 A avionics load: above the SOC estimator's rest
                # window, below the arbitration's load threshold — the
                # P5 SOC machinery stays fully dormant and these tests
                # keep pinning the pure voltage-path timelines.
                self.hal.port("esc").write(
                    ((0.0,) * 4, self.v_cell * 12, 5.0,
                     (self.v_cell,) * 12))
            if hb_every is not None and k % round(hb_every * TICK_HZ) == 0:
                self.fcu.on_heartbeat()
            self.fcu.run_tick()
            self.k += 1

    def boot_and_arm(self):
        """Arm, then move home far away: the synthetic vehicle never
        moves, so RTL/LAND stay observable (RTL over the arming spot
        would touch down on the home datum within one nav tick)."""
        self.run(2.6, hb_every=0.1)
        ok, why = self.fcu.cmd_arm()
        assert ok, why
        self.fcu.cmd_set_home((500.0, 0.0, 0.0))


# ----------------------------------------------------------- boot / PBIT


def test_boot_aligns_to_standby_and_pbit_passes():
    h = SynthHost(pos=(5.0, -3.0, 20.0))
    h.run(1.9)
    assert h.fcu.state == BOOT
    ok, why = h.fcu.cmd_arm()
    assert not ok and "STANDBY" in why
    h.run(0.7)
    assert h.fcu.state == STANDBY
    assert h.fcu.pbit_ok, h.fcu.pbit_reasons
    nav = h.fcu.nav
    assert max(abs(nav.pos[0] - 5.0), abs(nav.pos[1] + 3.0),
               abs(nav.pos[2] - 20.0)) < 0.5


def test_pbit_blocks_arming_on_stale_sensor_then_recovers():
    h = SynthHost()
    h.run(2.6)
    assert h.fcu.pbit_ok
    h.mag_on = False
    h.run(0.5)
    ok, why = h.fcu.cmd_arm()
    assert not ok and "MAG_STALE" in why
    h.mag_on = True
    h.run(0.3)
    ok, why = h.fcu.cmd_arm()
    assert ok, why
    assert h.fcu.state == ARMED and h.fcu.mode == POS_HOLD


def test_alignment_retries_on_vibration():
    h = SynthHost()
    h.run(2.6, vibrate=0.05)               # rocking: variance gate trips
    assert h.fcu.state == BOOT             # still retrying, not aligned
    # the 0.6 s of vibration that leaked into the second 2 s window
    # fails it too; the third, fully-still window aligns at ~6.1 s
    h.run(4.0)
    assert h.fcu.state == STANDBY


# -------------------------------------------------------------- failsafes


def test_offboard_setpoint_timeout_falls_back_to_pos_hold():
    h = SynthHost()
    h.boot_and_arm()
    h.fcu.cmd_velocity((1.0, 0.0, 0.0))
    ok, why = h.fcu.cmd_set_mode(OFFBOARD)
    assert ok, why
    for _ in range(10):                    # fresh setpoints for 1 s
        h.fcu.cmd_velocity((1.0, 0.0, 0.0))
        h.run(0.1, hb_every=0.1)
    assert h.fcu.mode == OFFBOARD
    t_stop = h.now
    h.run(0.7, hb_every=0.1)               # setpoints stop, link alive
    assert h.fcu.mode == POS_HOLD
    assert h.fcu.failsafe == "OFFBOARD_TIMEOUT"
    # fell back within one nav period after the 0.5 s timeout
    status = h.fcu.topics.subscribe("fcu_status").read()
    assert status.mode == POS_HOLD
    assert t_stop + 0.5 <= h.now           # sanity on the window


def test_link_loss_triggers_rtl_on_the_documented_timeline():
    h = SynthHost()
    h.boot_and_arm()
    h.run(0.5, hb_every=0.1)
    t0 = h.now                             # heartbeats stop here
    hist = []
    for _ in range(round(3.0 / 0.02)):
        h.run(0.02)
        hist.append((h.now, h.fcu.mode))
    t_rtl = next(t for t, m in hist if m == RTL)
    # detected at the first 50 Hz nav tick after t0 + link_loss_s (2.0):
    # the last heartbeat landed at most 0.1 s before t0.
    assert t0 + 2.0 - 0.1 <= t_rtl <= t0 + 2.0 + 0.03
    assert h.fcu.failsafe == "LINK_LOSS"


def test_battery_low_rtl_then_critical_land_with_debounce():
    h = SynthHost()
    h.boot_and_arm()
    h.v_cell = 3.45                        # below LOW, above CRITICAL
    h.run(0.9, hb_every=0.1)
    assert h.fcu.mode == POS_HOLD          # debounce (1 s) still running
    h.run(0.4, hb_every=0.1)
    assert h.fcu.mode == RTL and h.fcu.failsafe == "BATT_LOW"
    h.v_cell = 3.25
    h.run(1.3, hb_every=0.1)
    assert h.fcu.mode == LAND
    assert h.fcu.failsafe == "BATT_LOW"    # first reason stays latched
    assert h.fcu.batt.state == "CRITICAL"


def test_critical_battery_beats_link_loss():
    h = SynthHost()
    h.boot_and_arm()
    h.v_cell = 3.25
    h.run(3.0)                             # no heartbeats AND critical batt
    assert h.fcu.mode == LAND              # LAND won the priority fight
    assert h.fcu.failsafe in ("BATT_CRIT", "BATT_LOW", "LINK_LOSS")
    assert h.fcu.batt.state == "CRITICAL"


def test_link_loss_failsafe_armed_even_without_any_heartbeat():
    # P3 review F5: arming seeds the heartbeat clock — a link that never
    # delivered a single heartbeat must still trigger LINK_LOSS -> RTL
    # (the `_last_hb is None` guard used to disable the failsafe
    # structurally for the whole flight).
    h = SynthHost()
    h.run(2.6)                              # boot with NO heartbeats
    ok, why = h.fcu.cmd_arm()
    assert ok, why
    h.fcu.cmd_set_home((500.0, 0.0, 0.0))   # keep RTL observable
    h.run(2.1)
    assert h.fcu.failsafe == "LINK_LOSS"
    assert h.fcu.mode == RTL


def test_rearm_resets_attitude_setpoint_state():
    # P3 review F7: the previous flight's terminal _q_sp/_thrust/_sat
    # must not drive the 400 Hz rate loop between re-arm and the first
    # 50 Hz nav tick (up to 15 ticks of stale descent commands).
    h = SynthHost()
    h.boot_and_arm()
    h.run(0.2, hb_every=0.1)
    h.fcu._q_sp = vec.quat_from_euler(0.5, -0.4, 1.0)  # descent-end state
    h.fcu._thrust = 0.9
    h.fcu._sat = (1, -1, 0)
    h.fcu.cmd_disarm()
    ok, why = h.fcu.cmd_arm()
    assert ok, why
    assert h.fcu._q_sp == (1.0, 0.0, 0.0, 0.0)
    assert h.fcu._thrust == 0.0
    assert h.fcu._sat == (0, 0, 0)


def test_set_home_at_or_above_current_altitude_refused_while_armed():
    # P3 review F2: home z is the touchdown datum (bench placeholder) —
    # accepting a home at/above the vehicle would latch touchdown
    # mid-air on LAND entry and cut the motors.
    h = SynthHost()
    h.boot_and_arm()                        # helper parks home at z=0
    ok, why = h.fcu.cmd_set_home((100.0, 0.0, 50.0))  # at current alt
    assert not ok and "datum" in why
    ok, _ = h.fcu.cmd_set_home((100.0, 0.0, 10.0))
    assert ok
    assert h.fcu.home == (100.0, 0.0, 10.0)


def test_land_datum_frozen_at_land_entry():
    # P3 review F2: moving home during the descent must not move the
    # touchdown datum out from under the LAND controller.
    h = SynthHost()
    h.boot_and_arm()                        # home at z=0
    ok, why = h.fcu.cmd_set_mode(LAND)
    assert ok, why
    ok, _ = h.fcu.cmd_set_home((500.0, 0.0, 30.0))    # mid-descent
    assert ok
    h.run(0.2, hb_every=0.1)
    assert h.fcu._land_datum == 0.0         # pinned at LAND entry
    assert h.fcu.state == ARMED and not h.fcu.touchdown


def test_ekf_seeded_from_first_fix_far_from_origin():
    # P3 review F3: the EKF pos prior used to start at the world origin
    # (sigma 100 m) — a vehicle spawned beyond ~870 m had every GPS
    # fusion chi-square-gated out and could never pass PBIT. The first
    # fix now seeds the nominal position.
    h = SynthHost(pos=(4000.0, -2500.0, 30.0))
    h.run(2.6, hb_every=0.1)
    assert h.fcu.pbit_ok, h.fcu.pbit_reasons
    ok, why = h.fcu.cmd_arm()
    assert ok, why
    nav = h.fcu.nav
    assert max(abs(nav.pos[0] - 4000.0), abs(nav.pos[1] + 2500.0),
               abs(nav.pos[2] - 30.0)) < 0.5


def test_overlay_mag_declination_reaches_both_aligner_and_ekf():
    # P3 review F6: ONE knob — the param table — must feed alignment AND
    # the in-flight EKF mag fusion (split defaults = a persistent
    # heading bias the suite cannot see).
    from coopuavs.coopfc.estimation.ekf import EkfParams

    f = Fcu(HalIO(), overlay={"fcu.mag_declination_deg": 12.0})
    assert f._ekf_params.mag_declination_deg == 12.0
    assert f._aligner.decl == math.radians(12.0)
    # an explicit ekf_params still wins (documented precedence)
    f2 = Fcu(HalIO(), overlay={"fcu.mag_declination_deg": 12.0},
             ekf_params=EkfParams(mag_declination_deg=7.0))
    assert f2._ekf_params.mag_declination_deg == 7.0


def test_disarmed_actuators_are_zero():
    h = SynthHost()
    h.run(2.6)
    seq, frame = h.hal.port("actuators").read()
    assert seq > 0 and frame == (0.0, 0.0, 0.0, 0.0)


# ------------------------------------------------------------ integration


class FlightHost:
    """Plant + powertrain behind the HAL; perfect frames from truth."""

    def __init__(self, start=(0.0, 0.0, 50.0)):
        self.hal = HalIO()
        self.fcu = Fcu(self.hal)
        cfg = MultirotorParams.from_dict(load_airframe("interceptor_quad"))
        self.cfg = cfg
        self.plant = MultirotorPlant(cfg, 1)
        self.motor = MotorEsc(1, cfg.n_rotors, **cfg.motor)
        battery = BatteryEcm(1, **cfg.battery)
        self.pt = Powertrain(self.motor, battery, i_bus_max_a=350.0)
        self.state = np.zeros((1, 13))
        self.state[0, 0:3] = start
        self.state[0, 6] = 1.0
        self._v_prev = np.zeros(3)
        self._esc = ((0.0,) * 4, 50.0, 0.0, (50.0 / 12,) * 12)
        self.k = 0

    @property
    def now(self) -> float:
        return self.k * DT

    def _frames(self, flying: bool):
        s = self.state[0]
        q = (s[6], s[7], s[8], s[9])
        k, now = self.k, self.now
        if k % 2 == 0:
            if flying:
                a_w = (np.array(s[3:6]) - self._v_prev) / DT
                f_w = (a_w[0], a_w[1], a_w[2] + G)
                accel = vec.quat_rotate_inv(q, f_w)
                gyro = (s[10], s[11], s[12])
            else:
                # Same GYRO_STUCK honesty dither as SynthHost.
                d = 1e-6 * math.sin(0.7 * k)
                accel, gyro = (0.0, 0.0, G), (d, -d, d)
            self.hal.port("imu").write((gyro, accel))
        if k % 80 == 0:
            pos = (s[0], s[1], s[2])
            velo = (s[3], s[4], s[5]) if flying else (0.0, 0.0, 0.0)
            self.hal.port("gps").write((pos, velo, 3, now))
            self.hal.port("esc").write(self._esc)
        if k % 16 == 0:
            self.hal.port("baro").write(pressure_at(s[2]))
            self.hal.port("mag").write(vec.quat_rotate_inv(q, B_ENU))

    def run(self, t_span: float, wind=(0.0, 0.0, 0.0), hb_every=0.1,
            until=None):
        wind_w = np.array([wind])
        for _ in range(round(t_span * TICK_HZ)):
            flying = self.fcu.state == ARMED
            self._frames(flying)
            if hb_every and self.k % round(hb_every * TICK_HZ) == 0:
                self.fcu.on_heartbeat()
            self.fcu.run_tick()
            if flying:
                _, u = self.hal.port("actuators").read()
                self._v_prev = self.state[0, 3:6].copy()
                omega_r, v_bus, i_bus = self.pt.step(
                    DT, np.array([u], dtype=float))
                self._esc = (tuple(o * 60.0 / math.tau for o in omega_r[0]),
                             float(v_bus[0]), float(i_bus[0]),
                             (float(v_bus[0]) / 12,) * 12)
                self.state = self.plant.step(self.state, DT, omega_r,
                                             wind_w, 1.225)
            self.k += 1
            if until is not None and until(self):
                return True
        return False

    def boot_and_arm(self):
        self.run(2.6)
        ok, why = self.fcu.cmd_arm()
        assert ok, why
        w_h = math.sqrt(self.cfg.mass * G / (self.cfg.n_rotors * self.cfg.kf))
        self.motor.omega[:] = w_h          # bench: pre-spun at hover


def _fly_rtl_home(home, t_max, wind):
    h = FlightHost(start=(0.0, 0.0, 50.0))
    h.boot_and_arm()
    h.run(1.0, wind=wind)                  # settle in POS_HOLD
    h.fcu.cmd_set_home(home)
    ok, why = h.fcu.cmd_set_mode(RTL)
    assert ok, why
    done = h.run(t_max, wind=wind, until=lambda hh: hh.fcu.touchdown)
    assert done, f"no touchdown within {t_max} s (mode={h.fcu.mode})"
    s = h.state[0]
    assert math.hypot(s[0] - home[0], s[1] - home[1]) < 5.0
    assert abs(s[2] - (home[2] + 0.5)) < 2.0   # touchdown datum (bench)
    assert h.fcu.state == STANDBY              # disarmed after touchdown
    return h


def test_rtl_flies_home_and_lands_short():
    _fly_rtl_home(home=(-120.0, 0.0, 40.0), t_max=30.0, wind=(0.0, 3.0, 0.0))


@pytest.mark.slow
def test_rtl_home_from_2km_under_wind():
    h = _fly_rtl_home(home=(-2000.0, 0.0, 30.0), t_max=200.0,
                      wind=(0.0, 6.0, 0.0))
    assert h.now < 190.0                   # 2 km + descent, sane timeline
