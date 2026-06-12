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
altitude reaches the touchdown datum + ``fcu.touchdown_alt`` — physical
ground contact/land-detector logic arrives with the P4 world wiring.
The datum is home z, frozen at LAND entry; ``cmd_set_home`` refuses a
home at/above the vehicle while armed (either hole would latch
touchdown mid-air and cut the motors).
"""

from __future__ import annotations

import math
from typing import NamedTuple

from coopuavs.coopfc.battery_monitor import (
    CRITICAL, LOW, NORMAL, BatteryMonitor, BattParams,
)
from coopuavs.coopfc.cbit import CbitEngine
from coopuavs.coopfc.cbit.monitors import FcuMonitors
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
from coopuavs.coopfc.soc import SocEstimator, SocParams

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
    # CBIT (P5-1b): IMU full-scale limits for the range monitor (defaults
    # match the interceptor_devices ICM-42688 class: 2000 dps / 16 g) and
    # the dead-reckoning position-sigma budget (DR_BUDGET_LOW).
    "fcu.gyro_range_rad_s": 34.9,
    "fcu.accel_range_m_s2": 157.0,
    "fcu.dr_sigma_budget_m": 8.0,
    # FAILSAFE_ATT collective: just under the ~0.55 hover point so the
    # rate-damped descent is gentle, not ballistic.
    "fcu.fs_att_thrust": 0.45,
    # Pack calibration (P5-1f; defaults = the interceptor_quad pack —
    # per-airframe values are overlaid by the host, e.g. the fleet
    # engine reads them off each vehicle's airframe class).
    "fcu.batt_capacity_ah": 16.0,
    "fcu.batt_r0": 0.036,
    "fcu.batt_r1": 0.018,
    # Hard interlock token freshness (P5-5): must equal the MC-side
    # mc/fire_control.CLEARANCE_VALID_S — the two ends of one window
    # (cross-checked by test_sitl_release.py).
    "fcu.release_token_valid_s": 3.0,
}

BOOT = "BOOT"
STANDBY = "STANDBY"
ARMED = "ARMED"

OFFBOARD = "OFFBOARD"
POS_HOLD = "POS_HOLD"
RTL = "RTL"
LAND = "LAND"
# Rate-only flight (P5-1c, the nav-loss degraded mode): on a diverged
# estimator the position/velocity loops fly a fiction — gyro rate
# damping at a fixed sub-hover thrust is what an attitude failsafe can
# honestly promise (level-ish controlled descent, no nav required).
FAILSAFE_ATT = "FAILSAFE_ATT"


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
                 ekf_params: EkfParams | None = None):
        self.params = ParamTable(FCU_DEFAULTS, overlay)
        p = self.params.get
        self.topics = TopicStore()
        self.sched = Scheduler(TICK_HZ)
        if ekf_params is None:
            # One knob: the param-table declination feeds BOTH the
            # aligner and the EKF mag fusion (split defaults would be a
            # persistent heading bias). An explicit ekf_params wins.
            ekf_params = EkfParams()._replace(
                mag_declination_deg=p("fcu.mag_declination_deg"))
        self._ekf_params = ekf_params

        self.imu_drv = ImuDriver(hal.port("imu"), self.topics)
        # GPS polled at 50 Hz, NOT the 10 Hz fix rate: the EKF lag_s
        # horizon covers the device latency (120 ms) but not a 100 ms
        # poll quantization on top — a 10 Hz poll off-phase from fix
        # delivery hands the EKF fixes already behind the horizon
        # (fused stale; the ekf.late_meas seam counts it, pinned at
        # zero by test_coopfc_bench). stale_after 15 keeps the
        # staleness window at 300 ms of poll ticks.
        self.gps_drv = GpsDriver(hal.port("gps"), self.topics,
                                 stale_after=15)
        self.baro_drv = BaroDriver(hal.port("baro"), self.topics)
        self.mag_drv = MagDriver(hal.port("mag"), self.topics)
        self.esc_drv = EscDriver(hal.port("esc"), self.topics)
        self.actuators = hal.port("actuators")
        # Release pulse output (P5-5): one write per authorized release,
        # frame (track_id, stamp) — the host pairs it with the staged
        # FireRequest (fleet engine -> world-side shell).
        self.effector_port = hal.port("effector")

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
        self.soc_est = SocEstimator(SocParams(
            capacity_ah=p("fcu.batt_capacity_ah"),
            cells=p("fcu.batt_cells")))
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
        self._land_datum: float | None = None   # frozen at LAND entry
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
        # CBIT observation counters (P5-1b; read by cbit/monitors.py):
        # consecutive 400 Hz rate ticks with a blocked mixer axis, the
        # latest mixer output, and alignment restarts this power-up.
        self._sat_streak = 0
        self._u_last = (0.0, 0.0, 0.0, 0.0)
        self._align_retries = 0
        self.cbit = CbitEngine()
        self._cbit_mon = FcuMonitors(self, self.cbit)
        self._fs_att_thrust = p("fcu.fs_att_thrust")
        # Hard interlock state (P5-5): the latest mirrored clearance
        # token (track_id, issued — MC clock domain) and the tallies.
        self._release_token: tuple[int, float] | None = None
        self.releases = 0
        self.release_refusals: dict[str, int] = {}

        s = self.sched
        s.add("imu_drv", 400, self.imu_drv.tick)
        s.add("gps_drv", 50, self.gps_drv.tick)
        s.add("baro_drv", 50, self.baro_drv.tick)
        s.add("mag_drv", 50, self.mag_drv.tick)
        s.add("esc_drv", 10, self.esc_drv.tick)
        s.add("est_intake", 400, self._est_intake)
        s.add("est_update", 50, self._est_update)
        s.add("batt_mon", 10, self._batt_task)
        s.add("pbit", 10, self._pbit_task)
        s.add("nav", 50, self._nav_task)
        s.add("rate_mix", 400, self._rate_mix_task)
        # ORDERING pipeline slot: "... mixer -> PWM -> CBIT -> link" —
        # the monitors observe the tick the pipeline just produced.
        s.add("cbit_fast", 50, self._cbit_mon.fast)
        s.add("cbit_slow", 1, self._cbit_mon.slow)

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
        inhibitors = self.cbit.arming_inhibitors()
        if inhibitors:
            # Latched CBIT faults survive their condition clearing (a
            # repaired-looking sensor is not a repaired sensor); PBIT
            # alone would re-arm through them.
            return False, "CBIT: " + ",".join(inhibitors)
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
        # The previous flight's terminal setpoints must not drive the
        # 400 Hz rate loop before the first nav tick refreshes them.
        self._q_sp = (1.0, 0.0, 0.0, 0.0)
        self._thrust = 0.0
        self._sat = (0, 0, 0)
        # Heartbeat clock armed from arm time: a link that never
        # delivered a heartbeat still gets the LINK_LOSS failsafe.
        if self._last_hb is None:
            self._last_hb = self.sched.now
        return True, ""

    def cmd_disarm(self) -> None:
        self.state = STANDBY
        self.mode = ""

    def cmd_set_mode(self, mode: str) -> tuple[bool, str]:
        if self.state != ARMED:
            return False, "not armed"
        if (self.mode == FAILSAFE_ATT
                and self.cbit.degraded()[0] == FAILSAFE_ATT):
            # The nav-loss fault is still raised: every other mode would
            # fly the diverged estimate. Hard refusal, not etiquette.
            return False, "FAILSAFE_ATT: nav-loss fault active"
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

    def cmd_set_home(self, pos: vec.Vec3) -> tuple[bool, str]:
        """Override the RTL home (GCS / launch-site handoff convention;
        default home is the arming position). While armed, a home at or
        above the vehicle is refused: home z is the touchdown datum
        (bench placeholder, module docstring) and accepting it would
        latch touchdown mid-air on LAND entry and cut the motors."""
        if (self.state == ARMED and self.nav is not None
                and pos[2] > self.nav.pos[2]
                - self.params.get("fcu.touchdown_alt")):
            return False, "home z at/above vehicle: touchdown datum refused"
        self.home = pos
        return True, ""

    def on_heartbeat(self) -> None:
        self._last_hb = self.sched.now

    def cmd_batt_reset(self) -> tuple[bool, str]:
        """Battery swapped/recharged on the pad (P4-4 rearm cycle):
        clear the upward-latched monitor. Refused while armed — an
        in-flight 'reset' would defeat the sag-latch failsafe doctrine."""
        if self.state == ARMED:
            return False, "armed: battery reset is a ground operation"
        self.batt.reset()
        self.soc_est.reset()      # re-seed from the new pack's rest OCV
        # Battery-family CBIT latches belong to the swapped-out pack.
        self.cbit.clear("BATT_SAG_ANOM")
        self.cbit.clear("CELL_IMBALANCE")
        return True, ""

    def cmd_clearance_token(self, track_id: int, issued: float) -> None:
        """MC-side clearance token mirrored over the wire (P5-5):
        latest wins. ``issued`` is the clearance stamp in the MC clock
        domain — cmd_weapon_release compares the release stamp against
        it ONLY (never sched.now: the FCU clock is boot-relative)."""
        self._release_token = (int(track_id), float(issued))

    def cmd_weapon_release(self, stamp: float, track_id: int) -> tuple[bool, str]:
        """The FCU-side hard fire interlock (P5-5, PHY-UAV-021/033):
        release only ARMED, CBIT-clean, against the mirrored token for
        THIS track inside its freshness window. Success consumes the
        token (one token = one release) and pulses the effector port;
        refusals are tallied by reason."""
        if self.state != ARMED:
            ok, why = False, "NOT_ARMED"
        elif self.cbit.inhibit_fire:
            ok, why = False, "CBIT_INHIBIT"
        elif self._release_token is None:
            ok, why = False, "NO_TOKEN"
        elif self._release_token[0] != int(track_id):
            ok, why = False, "TOKEN_MISMATCH"
        elif (stamp - self._release_token[1]
                > self.params.get("fcu.release_token_valid_s")):
            ok, why = False, "TOKEN_STALE"
        else:
            ok, why = True, ""
        if not ok:
            self.release_refusals[why] = self.release_refusals.get(why, 0) + 1
            return False, why
        self._release_token = None
        self.effector_port.write((int(track_id), float(stamp)))
        self.releases += 1
        return True, ""

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
                        # Seed the nominal position from the latest fix:
                        # starting at the world origin under the wide
                        # alignment prior leaves any spawn beyond ~870 m
                        # permanently chi-square-gated out of GPS fusion
                        # (PBIT NO_GPS_FUSION, can never arm). The read
                        # consumes the topic flag, so this fix is the
                        # prior, not also a measurement.
                        g = self._sub_gps.read()
                        if g is not None and g.fix_type >= 3:
                            self.ekf = Ekf(res, self._ekf_params,
                                           pos0=g.pos, vel0=g.vel)
                        else:
                            self.ekf = Ekf(res, self._ekf_params)
                        self.state = STANDBY
                    else:           # moved/vibrating: retry from scratch
                        self._aligner = self._new_aligner()
                        self._align_retries += 1
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
        if self.ekf.mag_trusted and self.cbit.raised("MAG_FAULT"):
            # P5-1d: exclude the corrupted yaw source, latched per
            # flight (re-applied here to any rebuilt post-touchdown
            # EKF while the fault stays latched).
            self.ekf.mag_trusted = False
        self.nav = self.ekf.update(now)
        self._pub_nav.publish(self.nav)

    # ------------------------------------------------------------ monitors

    def _batt_task(self, now: float) -> None:
        if self._sub_esc.updated:
            msg = self._sub_esc.read()
            self.soc_est.update(msg.stamp, msg.v_bus, msg.i_bus)
            self.batt.update(now, msg.v_bus, soc=self.soc_est.soc,
                             i_bus=msg.i_bus,
                             sag_anom=self.cbit.raised("BATT_SAG_ANOM"))

    def battery_fraction(self) -> float:
        """Telemetry fraction (STATUS batt_frac): the real coulomb SOC
        once seeded, the conservative voltage proxy until then."""
        soc = self.soc_est.soc
        return soc if soc is not None else self.batt.fraction()

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
        # Touchdown datum frozen at LAND entry: a later cmd_set_home
        # must not move it out from under the descent (mid-air disarm).
        self._land_datum = self.home[2]

    def _failsafes(self, now: float) -> None:
        """Priority chain, pinned (PLAN_PROBLEM1 P5): FAILSAFE_ATT
        (nav-loss) > BATT_CRIT > CBIT LAND > LINK_LOSS > BATT_LOW >
        CBIT RTL > OFFBOARD_TIMEOUT. The CBIT slots command only for
        faults the legacy chain does not own (mirror rows excluded by
        the engine), so no-fault flights are bit-identical to P4. The
        failsafe REASON latches first-come (P3 contract) even when a
        later fault escalates the mode."""
        p = self.params.get
        act, cause = self.cbit.degraded()
        if act == FAILSAFE_ATT:
            # Position/velocity control is meaningless on a diverged
            # estimator — outranks every position-tracking response.
            if self.mode != FAILSAFE_ATT:
                self.mode = FAILSAFE_ATT
                self.failsafe = self.failsafe or cause
            return
        if self.mode == FAILSAFE_ATT:
            return    # fault cleared in flight: hold until a command
        if self.batt.state == CRITICAL and self.mode != LAND:
            self._enter_land()
            self.mode = LAND
            self.failsafe = self.failsafe or "BATT_CRIT"
            return
        if act == LAND and self.mode != LAND:
            # Get-down-now class (motor/IMU damage): preempts RTL.
            self._enter_land()
            self.mode = LAND
            self.failsafe = self.failsafe or cause
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
        if act == RTL:
            self._enter_rtl()
            self.mode = RTL
            self.failsafe = self.failsafe or cause
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
        if self.mode == FAILSAFE_ATT:
            return    # rate-only: the 400 Hz task flies, no setpoints
        p = self.params.get
        yaw_sp = self._sp_yaw if self.mode == OFFBOARD else 0.0
        if self.mode == OFFBOARD:
            # PX4 convention (P4-4): offboard setpoints obey the same
            # velocity limits as the internal modes — an MC cannot
            # command the airframe past its envelope params.
            vx, vy, vz = self._sp_vel
            h = math.hypot(vx, vy)
            max_h = p("fcu.vel_max_h")
            if h > max_h:
                vx *= max_h / h
                vy *= max_h / h
            vz = min(max(vz, -p("fcu.vel_max_down")), p("fcu.vel_max_up"))
            v_sp = (vx, vy, vz)
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
            if nav.pos[2] <= self._land_datum + p("fcu.touchdown_alt"):
                self.touchdown = True
                self.cmd_disarm()
                # Ground recalibration (P4-4): touchdown stops the
                # airframe faster than the IMU stream can express
                # (contact dynamics are not modeled), leaving the EKF
                # with a velocity belief its chi-gates then defend
                # against every GPS/baro correction — free-running
                # drift on the pad. Re-run the static alignment from
                # scratch (the BOOT machinery, ~2 s on the stand,
                # GPS-seeded): PBIT holds re-arming until it is green.
                self.ekf = None
                self.align_result = None
                self._aligner = self._new_aligner()
                return
        self._q_sp, self._thrust = self.vel_ctl.update(
            v_sp, nav.vel, yaw_sp, 1.0 / 50.0)
        self._pub_att_sp.publish((self._q_sp, self._thrust))

    def _rate_mix_task(self, now: float) -> None:
        if self.state != ARMED or self.nav is None:
            self.actuators.write((0.0, 0.0, 0.0, 0.0))
            self._sat_streak = 0
            return
        # Rate feedback: latest gyro minus EKF bias (NavState is 50 Hz).
        imu = self._last_imu
        b_g = self.ekf.b_g
        omega = (imu.gyro[0] - b_g[0], imu.gyro[1] - b_g[1],
                 imu.gyro[2] - b_g[2])
        if self.mode == FAILSAFE_ATT:
            # Nav-loss flight: damp all rotation, fixed sub-hover
            # collective — no attitude estimate consulted.
            rate_sp = (0.0, 0.0, 0.0)
            thrust = self._fs_att_thrust
        else:
            rate_sp = self.att_ctl.update(self._q_sp, self.nav.q)
            thrust = self._thrust
        torque = self.rate_ctl.update(rate_sp, omega, 1.0 / 400.0, self._sat)
        u, flags = self.mixer.mix(thrust, torque)
        self._sat = flags.axis_sat
        # Persistent-saturation observable (CBIT): motors pinned at the
        # rails. The per-axis flags oscillate with the anti-windup
        # (demand backs off the tick after it pins), but railed outputs
        # are the steady signature of unachievable demand. Margin, not
        # equality: desaturation arithmetic lands epsilon inside the
        # clip on some ticks (measured 5e-7) — still pinned.
        self._sat_streak = (self._sat_streak + 1
                            if any(ui <= 1e-3 or ui >= 0.999 for ui in u)
                            else 0)
        self._u_last = u
        self.actuators.write(u)
