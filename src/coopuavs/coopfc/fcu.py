"""FCU application: boot/alignment, PBIT, arming, modes, failsafes.

One ``Fcu`` per vehicle. The host (VirtualMCU in P4, the bench in
P3-6/8 tests) writes raw device frames into HAL ports and calls
``run_tick()`` at the 800 Hz tick rate; the FCU writes normalized motor
commands ``(u0..u3)`` to the ``actuators`` port. Scheduler registration
order IS the within-tick pipeline (the sched.py determinism contract):

    drivers -> est_intake(400) -> est_update(50) -> batt(10) ->
    pbit(10) -> nav(50: failsafes + guidance + velocity loop) ->
    rate_mix(400: attitude P + rate PID + mixer -> actuators)

State machine: BOOT (static alignment; auto-retry on the variance
gate) -> STANDBY (PBIT runs) -> ARMED in mode OFFBOARD / POS_HOLD /
RTL / LAND -> STANDBY again after touchdown or disarm.

Failsafe priority (evaluated highest first, each latches its reason):
battery CRITICAL -> LAND beats link-loss -> RTL beats battery LOW ->
RTL beats OFFBOARD setpoint-timeout -> POS_HOLD. Link loss = no
heartbeat for ``fcu.link_loss_s`` (the heartbeat source is the MC
coop-link, P3-7; tests drive ``on_heartbeat`` directly).

Rate-loop feedback is the latest gyro sample minus the EKF gyro bias
(PX4 convention — NavState refreshes at 50 Hz, far too slow for the
400 Hz rate PID); attitude/velocity feedback is the 50 Hz NavState.

Touchdown (bench placeholder, documented): LAND descends at
``fcu.land_speed`` and the FCU latches touchdown when the estimated
altitude reaches the arming datum + ``fcu.touchdown_alt`` — physical
ground contact/land-detector logic arrives with the P4 world wiring.
"""

from __future__ import annotations

from typing import NamedTuple

from coopuavs.coopfc.battery_monitor import (
    CRITICAL, LOW, NORMAL, BatteryMonitor, BattParams,
)
from coopuavs.coopfc.control import (
    AttCtl, QuadXMixer, RateCtl, VelCtl,
)
from coopuavs.coopfc.control.position import PosCtl, PosParams
from coopuavs.coopfc.core import vec
from coopuavs.coopfc.core.topics import TopicStore
from coopuavs.coopfc.drivers.baro import BaroDriver
from coopuavs.coopfc.drivers.esc import EscDriver
from coopuavs.coopfc.drivers.gps import GpsDriver
from coopuavs.coopfc.drivers.imu import ImuDriver
from coopuavs.coopfc.drivers.mag import MagDriver
from coopuavs.coopfc.estimation.alignment import Aligner
from coopuavs.coopfc.estimation.ekf import Ekf, EkfParams
from coopuavs.coopfc.params import ParamTable
from coopuavs.coopfc.sched import Scheduler

TICK_HZ = 800

# FCU-owned parameter defaults (overlay via Fcu(..., overlay=...)).
FCU_DEFAULTS = {
    "fcu.offboard_timeout_s": 0.5,
    "fcu.link_loss_s": 2.0,
    "fcu.batt_cells": 12,
    "fcu.batt_low_v_cell": 3.50,
    "fcu.batt_crit_v_cell": 3.30,
    "fcu.batt_debounce_s": 1.0,
    "fcu.align_n_imu": 800,
    "fcu.mag_declination_deg": 4.0,
    "fcu.rtl_accept_radius_m": 2.0,
    "fcu.land_speed": 1.5,
    "fcu.touchdown_alt": 0.5,
    "fcu.pos_kp": 1.0,
    "fcu.vel_max_h": 15.0,
    "fcu.vel_max_up": 5.0,
    "fcu.vel_max_down": 3.0,
}

BOOT = "BOOT"
STANDBY = "STANDBY"
ARMED = "ARMED"

OFFBOARD = "OFFBOARD"
POS_HOLD = "POS_HOLD"
RTL = "RTL"
LAND = "LAND"


class FcuStatus(NamedTuple):
    stamp: float
    state: str            # BOOT / STANDBY / ARMED
    mode: str             # OFFBOARD / POS_HOLD / RTL / LAND ("" if not armed)
    failsafe: str         # latched reason ("" = none)
    pbit_ok: bool
    pbit_reasons: tuple   # of str, empty when ok
    batt_state: str
    sigma_pos_h: float


class Fcu:
    def __init__(self, hal, overlay: dict | None = None,
                 ekf_params: EkfParams = EkfParams()):
        self.params = ParamTable(FCU_DEFAULTS, overlay)
        p = self.params.get
        self.topics = TopicStore()
        self.sched = Scheduler(TICK_HZ)
        self._ekf_params = ekf_params

        self.imu_drv = ImuDriver(hal.port("imu"), self.topics)
        self.gps_drv = GpsDriver(hal.port("gps"), self.topics)
        self.baro_drv = BaroDriver(hal.port("baro"), self.topics)
        self.mag_drv = MagDriver(hal.port("mag"), self.topics)
        self.esc_drv = EscDriver(hal.port("esc"), self.topics)
        self.actuators = hal.port("actuators")

        self._sub_imu = self.topics.subscribe("imu_raw")
        self._sub_gps = self.topics.subscribe("gps_fix")
        self._sub_baro = self.topics.subscribe("baro_alt")
        self._sub_mag = self.topics.subscribe("mag_body")
        self._sub_esc = self.topics.subscribe("esc_status")
        self._pub_nav = self.topics.advertise("nav_state")
        self._pub_status = self.topics.advertise("fcu_status", FcuStatus)
        self._pub_att_sp = self.topics.advertise("att_sp")

        self.batt = BatteryMonitor(BattParams(
            cells=p("fcu.batt_cells"), low_v_cell=p("fcu.batt_low_v_cell"),
            crit_v_cell=p("fcu.batt_crit_v_cell"),
            debounce_s=p("fcu.batt_debounce_s")))
        self.pos_ctl = PosCtl(PosParams(
            kp=p("fcu.pos_kp"), vel_max_h=p("fcu.vel_max_h"),
            vel_max_up=p("fcu.vel_max_up"),
            vel_max_down=p("fcu.vel_max_down")))
        self.vel_ctl = VelCtl()
        self.att_ctl = AttCtl()
        self.rate_ctl = RateCtl()
        self.mixer = QuadXMixer()

        self.state = BOOT
        self.mode = ""
        self.failsafe = ""
        self.pbit_ok = False
        self.pbit_reasons: tuple = ("BOOT",)
        self.ekf: Ekf | None = None
        self.align_result = None
        self._aligner = self._new_aligner()
        self.nav = None              # latest NavState
        self.home: vec.Vec3 | None = None
        self._hold_pos: vec.Vec3 | None = None
        self._rtl_target: vec.Vec3 | None = None
        self._sp_vel: vec.Vec3 = (0.0, 0.0, 0.0)
        self._sp_yaw = 0.0
        self._sp_stamp: float | None = None
        self._last_hb: float | None = None
        self._q_sp = (1.0, 0.0, 0.0, 0.0)
        self._thrust = 0.0
        self._sat = (0, 0, 0)
        self._last_imu = None
        self.touchdown = False

        s = self.sched
        s.add("imu_drv", 400, self.imu_drv.tick)
        s.add("gps_drv", 10, self.gps_drv.tick)
        s.add("baro_drv", 50, self.baro_drv.tick)
        s.add("mag_drv", 50, self.mag_drv.tick)
        s.add("esc_drv", 10, self.esc_drv.tick)
        s.add("est_intake", 400, self._est_intake)
        s.add("est_update", 50, self._est_update)
        s.add("batt_mon", 10, self._batt_task)
        s.add("pbit", 10, self._pbit_task)
        s.add("nav", 50, self._nav_task)
        s.add("rate_mix", 400, self._rate_mix_task)

    # ------------------------------------------------------------- host API

    def run_tick(self) -> None:
        self.sched.run_tick()

    # ---------------------------------------------------------- command API
    # (MC-side coop_link drives these from P3-7 on; tests call directly.)

    def cmd_arm(self) -> tuple[bool, str]:
        if self.state != STANDBY:
            return False, f"not in STANDBY (state={self.state})"
        if not self.pbit_ok:
            return False, "PBIT: " + ",".join(self.pbit_reasons)
        nav = self.nav
        if nav is None:
            return False, "no nav solution yet"
        self.home = nav.pos
        self.state = ARMED
        self.mode = POS_HOLD
        self._hold_pos = nav.pos
        self.failsafe = ""
        self.touchdown = False
        self.vel_ctl.reset()
        self.rate_ctl.reset()
        return True, ""

    def cmd_disarm(self) -> None:
        self.state = STANDBY
        self.mode = ""

    def cmd_set_mode(self, mode: str) -> tuple[bool, str]:
        if self.state != ARMED:
            return False, "not armed"
        if mode == OFFBOARD:
            now = self.sched.now
            if self._sp_stamp is None or (
                    now - self._sp_stamp
                    > self.params.get("fcu.offboard_timeout_s")):
                return False, "no fresh offboard setpoint"
        elif mode == POS_HOLD:
            self._hold_pos = self.nav.pos
        elif mode == RTL:
            self._enter_rtl()
        elif mode == LAND:
            self._enter_land()
        else:
            return False, f"unknown mode {mode!r}"
        self.mode = mode
        return True, ""

    def cmd_velocity(self, v_sp: vec.Vec3, yaw_sp: float = 0.0) -> None:
        """OFFBOARD velocity setpoint (stamped on the FCU clock)."""
        self._sp_vel = v_sp
        self._sp_yaw = yaw_sp
        self._sp_stamp = self.sched.now

    def cmd_set_home(self, pos: vec.Vec3) -> None:
        """Override the RTL home (GCS / launch-site handoff convention;
        default home is the arming position)."""
        self.home = pos

    def on_heartbeat(self) -> None:
        self._last_hb = self.sched.now

    # ------------------------------------------------------------ alignment

    def _new_aligner(self) -> Aligner:
        return Aligner(
            n_imu=self.params.get("fcu.align_n_imu"),
            mag_declination_deg=self.params.get("fcu.mag_declination_deg"))

    # ----------------------------------------------------------- estimator

    def _est_intake(self, now: float) -> None:
        if self._sub_imu.updated:
            imu = self._sub_imu.read()
            self._last_imu = imu
            if self.ekf is None:
                self._aligner.add_imu(imu.gyro, imu.accel)
                if self._sub_mag.updated:
                    self._aligner.add_mag(self._sub_mag.read().field_ut)
                res = self._aligner.result()
                if res is not None:
                    if res.ok:
                        self.align_result = res
                        self.ekf = Ekf(res, self._ekf_params)
                        self.state = STANDBY
                    else:           # moved/vibrating: retry from scratch
                        self._aligner = self._new_aligner()
                return
            self.ekf.on_imu(imu.stamp, imu.gyro, imu.accel)
        if self.ekf is None:
            return
        if self._sub_gps.updated:
            g = self._sub_gps.read()
            self.ekf.on_gps(g.fix_stamp, g.pos, g.vel, g.fix_type)
        if self._sub_baro.updated:
            b = self._sub_baro.read()
            self.ekf.on_baro(b.stamp, b.alt_m)
        if self._sub_mag.updated:
            m = self._sub_mag.read()
            self.ekf.on_mag(m.stamp, m.field_ut)

    def _est_update(self, now: float) -> None:
        if self.ekf is None:
            return
        self.nav = self.ekf.update(now)
        self._pub_nav.publish(self.nav)

    # ------------------------------------------------------------ monitors

    def _batt_task(self, now: float) -> None:
        if self._sub_esc.updated:
            self.batt.update(now, self._sub_esc.read().v_bus)

    def _pbit_task(self, now: float) -> None:
        if self.state == BOOT:
            self.pbit_ok, self.pbit_reasons = False, ("BOOT",)
            return
        reasons = []
        if self.align_result is None or not self.align_result.ok:
            reasons.append("ALIGN")
        for name, drv in (("IMU", self.imu_drv), ("GPS", self.gps_drv),
                          ("BARO", self.baro_drv), ("MAG", self.mag_drv),
                          ("ESC", self.esc_drv)):
            if drv.stale:
                reasons.append(f"{name}_STALE")
        if self.ekf is None or self.ekf.diverged:
            reasons.append("EKF")
        elif self.ekf.last_gps_fuse is None:
            reasons.append("NO_GPS_FUSION")
        if self.batt.state != NORMAL:
            reasons.append("BATTERY")
        if not self.params.verify():
            reasons.append("PARAM_CRC")
        if self.sched.faults():
            reasons.append("SCHED_FAULT")
        self.pbit_ok = not reasons
        self.pbit_reasons = tuple(reasons)

    # ------------------------------------------------------------ guidance

    def _enter_rtl(self) -> None:
        # Return at the current altitude, then LAND over home.
        self._rtl_target = (self.home[0], self.home[1], self.nav.pos[2])

    def _enter_land(self) -> None:
        self._hold_pos = self.nav.pos

    def _failsafes(self, now: float) -> None:
        p = self.params.get
        if self.batt.state == CRITICAL and self.mode != LAND:
            self._enter_land()
            self.mode = LAND
            self.failsafe = self.failsafe or "BATT_CRIT"
            return
        if self.mode in (RTL, LAND):
            return
        if (self._last_hb is not None
                and now - self._last_hb > p("fcu.link_loss_s")):
            self._enter_rtl()
            self.mode = RTL
            self.failsafe = self.failsafe or "LINK_LOSS"
            return
        if self.batt.state == LOW:
            self._enter_rtl()
            self.mode = RTL
            self.failsafe = self.failsafe or "BATT_LOW"
            return
        if (self.mode == OFFBOARD
                and now - self._sp_stamp > p("fcu.offboard_timeout_s")):
            self._hold_pos = self.nav.pos
            self.mode = POS_HOLD
            self.failsafe = self.failsafe or "OFFBOARD_TIMEOUT"

    def _nav_task(self, now: float) -> None:
        nav = self.nav
        if nav is not None:
            self._pub_status.publish(FcuStatus(
                stamp=now, state=self.state, mode=self.mode,
                failsafe=self.failsafe, pbit_ok=self.pbit_ok,
                pbit_reasons=self.pbit_reasons, batt_state=self.batt.state,
                sigma_pos_h=nav.sigma_pos_h))
        if self.state != ARMED or nav is None:
            return
        self._failsafes(now)
        p = self.params.get
        yaw_sp = self._sp_yaw if self.mode == OFFBOARD else 0.0
        if self.mode == OFFBOARD:
            v_sp = self._sp_vel
        elif self.mode == POS_HOLD:
            v_sp = self.pos_ctl.update(self._hold_pos, nav.pos)
        elif self.mode == RTL:
            v_sp = self.pos_ctl.update(self._rtl_target, nav.pos)
            dx = self._rtl_target[0] - nav.pos[0]
            dy = self._rtl_target[1] - nav.pos[1]
            if dx * dx + dy * dy < p("fcu.rtl_accept_radius_m") ** 2:
                self._enter_land()
                self.mode = LAND
        else:  # LAND
            hp = self._hold_pos
            v_h = self.pos_ctl.update((hp[0], hp[1], nav.pos[2]), nav.pos)
            v_sp = (v_h[0], v_h[1], -p("fcu.land_speed"))
            if nav.pos[2] <= self.home[2] + p("fcu.touchdown_alt"):
                self.touchdown = True
                self.cmd_disarm()
                return
        self._q_sp, self._thrust = self.vel_ctl.update(
            v_sp, nav.vel, yaw_sp, 1.0 / 50.0)
        self._pub_att_sp.publish((self._q_sp, self._thrust))

    def _rate_mix_task(self, now: float) -> None:
        if self.state != ARMED or self.nav is None:
            self.actuators.write((0.0, 0.0, 0.0, 0.0))
            return
        # Rate feedback: latest gyro minus EKF bias (NavState is 50 Hz).
        imu = self._last_imu
        b_g = self.ekf.b_g
        omega = (imu.gyro[0] - b_g[0], imu.gyro[1] - b_g[1],
                 imu.gyro[2] - b_g[2])
        rate_sp = self.att_ctl.update(self._q_sp, self.nav.q)
        torque = self.rate_ctl.update(rate_sp, omega, 1.0 / 400.0, self._sat)
        u, flags = self.mixer.mix(self._thrust, torque)
        self._sat = flags.axis_sat
        self.actuators.write(u)
