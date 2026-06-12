"""FCU CBIT monitors: dictionary rows wired to the existing health seams.

Two scheduler tasks (P5-1b), registered by the Fcu AFTER ``rate_mix`` —
the ORDERING pipeline slot ("... mixer -> PWM -> CBIT -> link"):

- ``fast`` (50 Hz): IMU family, GPS family, EKF family, motor response,
  saturation persistence, MC-link / battery mirrors;
- ``slow`` (1 Hz): baro/mag frame health, param CRC, scheduler faults,
  alignment retries, and the watchdog cross-check.

Monitors only OBSERVE in P5-1b: they read seams the earlier phases
planted (driver ``stale``/``bad_frames``, ``ekf.rejected``/``diverged``/
``last_gps_fuse``, ``sched.faults()``, ``params.verify()``, the battery
monitor) plus three cheap counters the Fcu now maintains for them
(``_sat_streak``, ``_u_last``, ``_align_retries``). Command authority
(degraded modes, arming/fire inhibits) arrives with P5-1c — wiring the
monitors first keeps every behavior change reviewable against a
fault-reporting baseline.

This module is coopfc-internal: it reaches into ``Fcu`` private state by
contract (same package, the seams are listed above); thresholds live
here as constants except the DR budget, which is a flight parameter
(``fcu.dr_sigma_budget_m``) because degraded-nav scenarios tune it.

Watchdog honesty: WDOG_MISS is a *cross*-check (slow verifies fast fire
counts, fast verifies slow recency) — it detects a wedged monitor task,
not a wedged processor; a dead MCU is the host exception fence's
business (SIM-SIL-003).
"""

from __future__ import annotations

import math

from coopuavs.coopfc.battery_monitor import CRITICAL, LOW
from coopuavs.coopfc.core import vec
from coopuavs.coopfc.estimation.alignment import mag_yaw
from coopuavs.coopfc.soc import ocv_v_cell

from .engine import CbitEngine

# coopfc.fcu imports this package (cycle), so its state vocabulary
# cannot be imported here; the string is cross-checked against the fcu
# module by the monitor tests.
_ARMED = "ARMED"

# IMU range monitor: flag samples at/above this fraction of full scale
# (the quantizer top code is inside the range — hw/stoch full-scale
# convention — so exact-equality would miss real clipping).
RANGE_FRAC = 0.98
# Vibration proxy: EMA of the summed |delta accel| between consecutive
# 50 Hz reads. Nominal sensor noise contributes ~0.05 m/s^2; honest
# maneuvering ~0.3; the IMU_NOISE injection class (P5-2) sits well above.
VIBE_ALPHA = 0.2
VIBE_THRESH = 1.2          # m/s^2
# GPS: no successful position fuse for this long = loss (10 Hz fix rate,
# so 10 consecutive missed/ignored fixes).
GPS_FUSE_TIMEOUT_S = 1.0
# EKF innovation consistency: reject streaks in >= 2 sensor families at
# once (a single-family streak is that sensor's own fault code).
INNOV_FAMILIES_MIN = 2
# Per-slow-window reject minima before a frame-healthy sensor is called
# faulty (chi-gates reject the odd honest outlier; streaks are faults).
# GPS sits low: a multipath-style fault rejects only SOME fixes (the
# interleaved accepts keep resetting an instantaneous-streak condition,
# so the window count is what makes the debounce completable).
BARO_REJ_MIN = 3           # of 50 fuse opportunities / s
MAG_REJ_MIN = 10
GPS_REJ_MIN = 3            # of 20 (pos+vel at 10 Hz) / s
# Mag consistency: mag-derived yaw vs EKF yaw, checked OUTSIDE the
# fusion path — the EKF's yaw information floor stops mag fusion once
# converged (colored hard-iron honesty, estimation/ekf.py), so a field
# fault raises no innovation rejects until yaw variance regrows; the
# direct comparison sees it immediately. Same tilt guard as the fusion.
MAG_YAW_ERR_MAX = 0.5      # rad (~29 deg; hard-iron residual is ~2.3 deg)
MAG_TILT_GUARD = 0.3       # |cos(pitch)| below this: leveling degenerate
# Motor response: a fault is a command/rpm SPREAD the other side cannot
# explain. Naive per-rotor share ratios false-fire on healthy trimmed
# flight: the motor curve is affine (omega ~ a*u - droop), so a banked
# orbit's honest u asymmetry yields a LARGER relative rpm asymmetry
# (elasticity (omega+droop)/omega in [1, ~2]) — measured on the sentinel
# orbit suite. The two real fault signatures:
#   (1) rpm spread with no commanding u spread (pre-trim omega deficit,
#       stuck/stopped rotor) -> flag the slowest rotor;
#   (2) u spread with no responding rpm spread (post-trim ESC-gain/prop
#       fault: the controller equalizes omega by over-commanding the
#       weak rotor; healthy physics NEVER yields equal rpm at unequal u)
#       -> flag the most-commanded rotor.
# EXPLAIN_RATIO 0.4: rule (1) false-fires only at elasticity > 2.5
# (beyond the deepest healthy droop), rule (2) only below 0.4 (healthy
# is always >= 1). Pack brownout scales every rotor together — spreads
# preserved, no fault: the battery monitor owns that reason.
# Judged ONLY at quasi-steady points: u moves at 400 Hz while the ESC
# frame reports omega ~100 ms behind (frame period + motor lag), so
# honest aggressive maneuvering disagrees wildly (measured: u_0 = 0.0
# with rotor 0 still spinning down at 8000 rpm). Unsteady windows
# report healthy — faults are judged at the next steady stretch.
MOTOR_SPREAD_MIN = 0.20    # relative (max-min)/mean spread to accuse
MOTOR_EXPLAIN_RATIO = 0.4  # other side's spread below this fraction = fault
MOTOR_U_MIN = 0.15         # mean commanded u below this = idle, skip
MOTOR_RPM_MIN = 500.0      # mean rpm below this = no credible telemetry
MOTOR_U_STEADY = 0.08      # max per-rotor |u - u at previous frame|
# Mixer saturation: axis blocked for at least this many 400 Hz rate
# ticks before the 50 Hz observer calls the instant "saturated" (the
# 1 s dictionary debounce sits on top).
SAT_STREAK_TICKS = 8
# Battery family (P5-1f). CELL_IMBALANCE: tap spread (max-min) above
# this — taps carry sigma 0.005 + lsb 0.01, so 80 mV is ~8x the noise
# floor and ~1/3 of a healthy pack's full usable OCV span.
CELL_SPREAD_V = 0.08
# BATT_SAG_ANOM: measured bus volts below the SOC-implied expectation
# OCV(soc) - I*(r0 + r1) by more than this margin per cell (the RC
# branch lags toward I*r1 — using the settled value over-predicts sag
# during load transients, which only makes the monitor conservative).
SAG_MARGIN_V_CELL = 0.10
# Watchdog cross-check.
WDOG_FAST_MIN = 45         # fast fires per slow period (nominal 50)
WDOG_SLOW_AGE_S = 2.5      # fast-side bound on slow-task recency
WDOG_GRACE_S = 2.0         # no verdicts while the boot transient settles
ALIGN_RETRIES_MAX = 3


class FcuMonitors:
    def __init__(self, fcu, engine: CbitEngine):
        self.fcu = fcu
        self.eng = engine
        # Own subscription: the battery task consumes the Fcu's
        # esc_status subscription; per-subscription generations make a
        # second reader free (topics.py contract).
        self._sub_esc = fcu.topics.subscribe("esc_status")
        self._sub_mag = fcu.topics.subscribe("mag_body")
        self._esc_msg = None
        self._u_at_frame = None
        self._mag_err = False
        self._prev_gyro = None
        self._vibe = 0.0
        self._prev_accel = None
        # Reject-streak state: totals snapshotted at the last successful
        # GPS fuse / at each slow window roll.
        self._gps_rej_at_fuse = 0
        self._last_gps_fuse_seen = None
        self._rej_at_slow = dict.fromkeys(("baro", "mag", "gps"), 0)
        self._baro_bad_at_slow = 0
        self._mag_bad_at_slow = 0
        self._baro_recent = False
        self._mag_recent = False
        self._mag_frames_bad = False
        self._gps_recent = False
        self._fast_fires = 0
        self._fast_at_slow = None
        self._last_slow = None

    # ------------------------------------------------------------ 50 Hz

    def fast(self, now: float) -> None:
        fcu = self.fcu
        eng = self.eng
        p = fcu.params.get
        self._fast_fires += 1

        # -- IMU family ------------------------------------------------
        eng.report("IMU_STALE", fcu.imu_drv.stale, now)
        imu = fcu._last_imu
        if imu is not None:
            g_lim = RANGE_FRAC * p("fcu.gyro_range_rad_s")
            a_lim = RANGE_FRAC * p("fcu.accel_range_m_s2")
            in_range = (max(abs(imu.gyro[0]), abs(imu.gyro[1]),
                            abs(imu.gyro[2])) >= g_lim
                        or max(abs(imu.accel[0]), abs(imu.accel[1]),
                               abs(imu.accel[2])) >= a_lim)
            eng.report("IMU_RANGE", in_range, now)
            # Stuck = fresh frames carrying an exactly frozen gyro (live
            # MEMS noise never repeats a sample; a stale stream is
            # IMU_STALE's finding, not this one's).
            eng.report("GYRO_STUCK",
                       (self._prev_gyro is not None
                        and imu.gyro == self._prev_gyro
                        and not fcu.imu_drv.stale), now)
            self._prev_gyro = imu.gyro
            if self._prev_accel is not None:
                d = (abs(imu.accel[0] - self._prev_accel[0])
                     + abs(imu.accel[1] - self._prev_accel[1])
                     + abs(imu.accel[2] - self._prev_accel[2]))
                self._vibe += VIBE_ALPHA * (d - self._vibe)
            self._prev_accel = imu.accel
            noisy = self._vibe > VIBE_THRESH
            eng.report("IMU_NOISE", noisy, now,
                       detail=f"vibe {self._vibe:.2f}" if noisy else "")

        # -- GPS / EKF family -------------------------------------------
        ekf = fcu.ekf
        if ekf is not None:
            rej = ekf.rejected
            gps_rej = rej["gps_pos"] + rej["gps_vel"]
            if ekf.last_gps_fuse != self._last_gps_fuse_seen:
                self._last_gps_fuse_seen = ekf.last_gps_fuse
                self._gps_rej_at_fuse = gps_rej
            fuse_stale = (ekf.last_gps_fuse is None
                          or ekf.state_time - ekf.last_gps_fuse
                          > GPS_FUSE_TIMEOUT_S)
            eng.report("GPS_LOSS", fcu.gps_drv.stale or fuse_stale, now)
            gps_bad = (gps_rej > self._gps_rej_at_fuse) or self._gps_recent
            eng.report("GPS_DEGRADED", gps_bad and not fuse_stale, now)
            nav = fcu.nav
            if self._sub_mag.updated and nav is not None:
                m = self._sub_mag.read()
                roll, pitch, yaw_est = vec.quat_to_euler(nav.q)
                if abs(math.cos(pitch)) >= MAG_TILT_GUARD:
                    ym = mag_yaw(roll, pitch, m.field_ut, math.radians(
                        fcu.params.get("fcu.mag_declination_deg")))
                    self._mag_err = (abs(vec.wrap_pi(ym - yaw_est))
                                     > MAG_YAW_ERR_MAX)
            mag_bad = self._mag_recent or self._mag_err
            eng.report("MAG_FAULT",
                       (fcu.mag_drv.stale or self._mag_frames_bad
                        or mag_bad), now)
            families = (int(gps_bad) + int(self._baro_recent)
                        + int(mag_bad))
            eng.report("EKF_INNOV", families >= INNOV_FAMILIES_MIN, now)
            eng.report("EKF_DIVERGED", ekf.diverged, now)
            if nav is not None:
                over = nav.sigma_pos_h > p("fcu.dr_sigma_budget_m")
                eng.report("DR_BUDGET_LOW", over, now,
                           detail=f"sigma {nav.sigma_pos_h:.1f} m"
                           if over else "")

        # -- actuation family (motor check per ESC frame, 10 Hz: the
        # engine holds state between reports, the debounce clock runs
        # on report times) ----------------------------------------------
        if self._sub_esc.updated:
            self._esc_msg = self._sub_esc.read()
            self._motor_response(now)
            self._battery_family(now)
        eng.report("SAT_PERSIST", fcu._sat_streak >= SAT_STREAK_TICKS, now)

        # -- mirrors (legacy failsafe chain keeps command authority) ----
        eng.report("LINK_MC_LOSS",
                   (fcu._last_hb is not None
                    and now - fcu._last_hb > p("fcu.link_loss_s")), now)
        eng.report("BATT_LOW", fcu.batt.state == LOW, now)
        eng.report("BATT_CRIT", fcu.batt.state == CRITICAL, now)

        # -- watchdog: is the slow task alive? ---------------------------
        eng.report("WDOG_MISS",
                   (now > WDOG_GRACE_S and self._last_slow is not None
                    and now - self._last_slow > WDOG_SLOW_AGE_S), now)

    def _motor_response(self, now: float) -> None:
        msg = self._esc_msg
        fcu = self.fcu
        if msg is None or fcu.state != _ARMED:
            self._u_at_frame = None
            return
        u = fcu._u_last
        prev, self._u_at_frame = self._u_at_frame, u
        if prev is None:
            return
        if max(abs(ui - pi) for ui, pi in zip(u, prev)) > MOTOR_U_STEADY:
            # Transient: shares carry no information (docstring above);
            # explicit healthy report so a maneuvering blip never parks
            # a half-finished debounce for a later spurious completion.
            self.eng.report("MOTOR_RESPONSE", False, now)
            return
        rpm = msg.rpm
        mean_u = sum(u) / len(u)
        mean_rpm = sum(rpm) / len(rpm)
        if mean_u < MOTOR_U_MIN or mean_rpm < MOTOR_RPM_MIN:
            return
        spread_u = (max(u) - min(u)) / mean_u
        spread_rpm = (max(rpm) - min(rpm)) / mean_rpm
        bad, rotor = False, -1
        if (spread_rpm > MOTOR_SPREAD_MIN
                and spread_u < MOTOR_EXPLAIN_RATIO * spread_rpm):
            bad, rotor = True, rpm.index(min(rpm))
        elif (spread_u > MOTOR_SPREAD_MIN
                and spread_rpm < MOTOR_EXPLAIN_RATIO * spread_u):
            bad, rotor = True, u.index(max(u))
        self.eng.report("MOTOR_RESPONSE", bad, now,
                        detail=f"rotor {rotor}" if bad else "")

    def _battery_family(self, now: float) -> None:
        """Per ESC frame (10 Hz): cell-tap spread + SOC-implied sag."""
        msg = self._esc_msg
        eng = self.eng
        spread = max(msg.cells) - min(msg.cells)
        eng.report("CELL_IMBALANCE", spread > CELL_SPREAD_V, now,
                   detail=f"spread {spread:.2f} V"
                   if spread > CELL_SPREAD_V else "")
        soc = self.fcu.soc_est.soc
        if soc is None:
            return
        p = self.fcu.params.get
        cells = p("fcu.batt_cells")
        # Full ECM expectation incl. the tracked RC relaxation (the
        # estimator integrates v1 from every frame; batt_mon runs before
        # cbit_fast, sched order): the settled-sag shortcut i*(r0+r1)
        # false-fired for ~3*tau1 after every sustained load DECREASE
        # (dash -> hover), latching a fault that then disabled the SOC
        # veto for the rest of the flight.
        expected = (ocv_v_cell(soc) * cells
                    - msg.i_bus * p("fcu.batt_r0") - self.fcu.soc_est.v1)
        anom = msg.v_bus < expected - SAG_MARGIN_V_CELL * cells
        eng.report("BATT_SAG_ANOM", anom, now,
                   detail=f"{(expected - msg.v_bus) / cells:.2f} V/cell"
                   if anom else "")

    # ------------------------------------------------------------- 1 Hz

    def slow(self, now: float) -> None:
        fcu = self.fcu
        eng = self.eng

        # Per-window reject/bad-frame deltas (window = the 1 s slow
        # period; the fast task folds the family flags into EKF_INNOV).
        rej = fcu.ekf.rejected if fcu.ekf is not None else None
        baro_bad = fcu.baro_drv.bad_frames - self._baro_bad_at_slow
        mag_bad = fcu.mag_drv.bad_frames - self._mag_bad_at_slow
        baro_rej = (rej["baro"] - self._rej_at_slow["baro"]) if rej else 0
        mag_rej = (rej["mag"] - self._rej_at_slow["mag"]) if rej else 0
        gps_rej = ((rej["gps_pos"] + rej["gps_vel"]
                    - self._rej_at_slow["gps"]) if rej else 0)
        self._baro_bad_at_slow = fcu.baro_drv.bad_frames
        self._mag_bad_at_slow = fcu.mag_drv.bad_frames
        if rej:
            self._rej_at_slow["baro"] = rej["baro"]
            self._rej_at_slow["mag"] = rej["mag"]
            self._rej_at_slow["gps"] = rej["gps_pos"] + rej["gps_vel"]
        # Same per-window minimum as BARO_FAULT itself: the chi-gates
        # reject the odd honest outlier — one must not count a family.
        self._baro_recent = baro_rej >= BARO_REJ_MIN
        self._mag_recent = mag_rej >= MAG_REJ_MIN
        self._mag_frames_bad = mag_bad > 0
        self._gps_recent = gps_rej >= GPS_REJ_MIN

        eng.report("BARO_FAULT",
                   (fcu.baro_drv.stale or baro_bad > 0
                    or baro_rej >= BARO_REJ_MIN), now)
        eng.report("ESC_STALE", fcu.esc_drv.stale, now)
        # MAG_FAULT itself is reported by the fast task (the direct
        # yaw-consistency check lives there); this window just feeds it.

        eng.report("PARAM_CRC", not fcu.params.verify(), now)
        faults = fcu.sched.faults()
        eng.report("SCHED_OVERRUN", bool(faults), now,
                   detail=",".join(faults))
        eng.report("ALIGN_FAIL",
                   (fcu.ekf is None
                    and fcu._align_retries >= ALIGN_RETRIES_MAX), now,
                   detail=f"retries {fcu._align_retries}")

        # Watchdog: did the fast task fire ~50x since the last window?
        if self._fast_at_slow is not None and now > WDOG_GRACE_S:
            eng.report("WDOG_MISS",
                       self._fast_fires - self._fast_at_slow
                       < WDOG_FAST_MIN, now)
        self._fast_at_slow = self._fast_fires
        self._last_slow = now
