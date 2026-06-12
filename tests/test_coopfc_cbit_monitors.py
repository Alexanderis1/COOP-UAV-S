"""P5-1b: FCU CBIT monitors wired to the real seams — one test per fault.

Harness: ``CbitHost`` (the test_coopfc_fcu SynthHost pattern with fault
knobs). One physical honesty note: the nominal gyro stream carries a
tiny alternating dither — live MEMS noise never repeats a sample
exactly, and a byte-identical repeating stream IS the GYRO_STUCK
signature, so perfectly-constant synthetic frames would (correctly)
read as a stuck gyro.

P5-1b is observation-only: monitors raise faults and inhibit flags;
command authority (degraded modes, arming gates) lands in P5-1c, so
every legacy failsafe pin is asserted unchanged alongside its mirror
fault here.
"""

from __future__ import annotations

import math

from coopuavs.coopfc.drivers.baro import _EXP, _ISA_LAPSE, _ISA_P0, _ISA_T0
from coopuavs.coopfc.fcu import ARMED, LAND, OFFBOARD, RTL, STANDBY, TICK_HZ, Fcu
from coopuavs.coopfc.hal import HalIO

DT = 1.0 / TICK_HZ
G = 9.81

_D, _I = math.radians(4.0), math.radians(63.0)
B_ENU = (50.0 * math.cos(_I) * math.sin(_D),
         50.0 * math.cos(_I) * math.cos(_D),
         -50.0 * math.sin(_I))
B_SWAPPED = (B_ENU[1], B_ENU[0], B_ENU[2])     # ~86 deg rotated: mag fault


def pressure_at(alt_m: float) -> float:
    return _ISA_P0 * (1.0 - _ISA_LAPSE * alt_m / _ISA_T0) ** (1.0 / _EXP)


class CbitHost:
    """Still vehicle, perfect-but-dithered frames, per-channel fault knobs."""

    def __init__(self, pos=(0.0, 0.0, 50.0), overlay=None):
        self.hal = HalIO()
        self.fcu = Fcu(self.hal, overlay=overlay)
        self.pos = pos
        self.k = 0
        # knobs
        self.imu_on = True
        self.gps_on = True
        self.baro_on = True
        self.mag_on = True
        self.heartbeats = True
        self.v_cell = 4.0
        self.gyro_amp = 1e-6          # alternating dither (see module doc)
        self.accel_amp = 0.0          # IMU_NOISE injection
        self.gyro_freeze = None       # exact tuple -> GYRO_STUCK
        self.gyro_big = None          # scalar -> IMU_RANGE
        self.gps_offset_every_other = 0.0   # m on odd fixes -> GPS_DEGRADED
        self.baro_nan = False
        self.mag_swapped = False
        self.esc_rpm = (6000.0,) * 4
        self._fix_i = 0

    @property
    def now(self) -> float:
        return self.k * DT

    def run(self, t_span: float, offboard_v=None):
        for _ in range(round(t_span * TICK_HZ)):
            k = self.k
            if self.imu_on and k % 2 == 0:
                # Non-repeating dither phase: a square wave synchronous
                # with the 50 Hz monitor reads the same value every read
                # — indistinguishable from a stuck sensor (correctly!).
                s = math.sin(0.7 * k)
                if self.gyro_freeze is not None:
                    gyro = self.gyro_freeze
                elif self.gyro_big is not None:
                    gyro = (self.gyro_big, 0.0, 0.0)
                else:
                    d = s * self.gyro_amp
                    gyro = (d, -d, d)
                accel = (0.0, 0.0, G + s * self.accel_amp)
                self.hal.port("imu").write((gyro, accel))
            if self.gps_on and k % 80 == 0:
                off = (self.gps_offset_every_other
                       if self._fix_i % 2 else 0.0)
                self._fix_i += 1
                self.hal.port("gps").write(
                    ((self.pos[0] + off, self.pos[1], self.pos[2]),
                     (0.0, 0.0, 0.0), 3, self.now))
            if self.baro_on and k % 16 == 0:
                self.hal.port("baro").write(
                    float("nan") if self.baro_nan
                    else pressure_at(self.pos[2]))
            if self.mag_on and k % 16 == 0:
                self.hal.port("mag").write(
                    B_SWAPPED if self.mag_swapped else B_ENU)
            if k % 80 == 0:
                self.hal.port("esc").write(
                    (self.esc_rpm, self.v_cell * 12, 0.0))
            if self.heartbeats and k % 80 == 0:
                self.fcu.on_heartbeat()
            if offboard_v is not None and k % 160 == 0:   # keep sp fresh
                self.fcu.cmd_velocity(offboard_v)
            self.fcu.run_tick()
            self.k += 1

    def boot_and_arm(self):
        self.run(2.6)
        ok, why = self.fcu.cmd_arm()
        assert ok, why
        # Home away + below: RTL/LAND responses stay observable on a
        # vehicle that never moves (the SynthHost convention).
        self.fcu.cmd_set_home((500.0, 0.0, 0.0))


def armed_host(**kw) -> CbitHost:
    h = CbitHost(**kw)
    h.boot_and_arm()
    return h


# --------------------------------------------------------------- baseline

def test_no_fault_baseline_word_zero():
    h = armed_host()
    h.run(3.0)
    assert h.fcu.cbit.faults() == []
    assert h.fcu.cbit.word() == 0
    assert not h.fcu.cbit.inhibit_fire and not h.fcu.cbit.inhibit_arming
    assert h.fcu.failsafe == "" and h.fcu.state == ARMED


# -------------------------------------------------------------- IMU family

def test_imu_stale_raises_and_recovers():
    h = armed_host()
    h.imu_on = False
    h.run(0.3)
    assert h.fcu.cbit.raised("IMU_STALE")
    assert h.fcu.cbit.inhibit_fire and h.fcu.cbit.inhibit_arming
    h.imu_on = True
    h.run(0.3)
    assert not h.fcu.cbit.raised("IMU_STALE")


def test_imu_range_on_clipped_gyro():
    h = armed_host()
    h.gyro_big = 34.5                 # >= 0.98 * 34.9 rad/s full scale
    h.run(0.3)
    assert h.fcu.cbit.raised("IMU_RANGE")


def test_gyro_stuck_on_frozen_stream():
    h = armed_host()
    h.gyro_freeze = (0.001, 0.002, -0.001)
    h.run(0.3)
    assert h.fcu.cbit.raised("GYRO_STUCK")
    assert not h.fcu.cbit.raised("IMU_STALE")      # frames are fresh


def test_imu_noise_vibration_proxy():
    h = armed_host()
    h.accel_amp = 2.0
    h.run(1.0)
    assert h.fcu.cbit.raised("IMU_NOISE")
    assert "vibe" in h.fcu.cbit.snapshot()["IMU_NOISE"]["detail"]
    h.accel_amp = 0.0
    h.run(1.5)
    assert not h.fcu.cbit.raised("IMU_NOISE")


# -------------------------------------------------------------- GPS family

def test_gps_loss_on_silence_and_recovery():
    h = armed_host()
    h.gps_on = False
    h.run(2.5)
    assert h.fcu.cbit.raised("GPS_LOSS")
    h.gps_on = True
    h.run(1.0)
    assert not h.fcu.cbit.raised("GPS_LOSS")


def test_gps_degraded_on_multipath_rejects():
    h = armed_host()
    h.gps_offset_every_other = 12.0   # odd fixes chi-rejected, evens fuse
    h.run(3.0)
    assert h.fcu.cbit.raised("GPS_DEGRADED")
    assert not h.fcu.cbit.raised("GPS_LOSS")       # accepts keep flowing
    h.gps_offset_every_other = 0.0
    h.run(2.5)
    assert not h.fcu.cbit.raised("GPS_DEGRADED")


def test_dr_budget_low_under_denial():
    h = armed_host(overlay={"fcu.dr_sigma_budget_m": 1.5})
    h.gps_on = False
    h.run(25.0)
    assert h.fcu.cbit.raised("DR_BUDGET_LOW")
    assert h.fcu.cbit.raised("GPS_LOSS")
    assert "sigma" in h.fcu.cbit.snapshot()["DR_BUDGET_LOW"]["detail"]


# -------------------------------------------------------------- EKF family

def test_ekf_innov_needs_two_families():
    h = armed_host()
    h.mag_swapped = True              # one family rejecting
    h.run(3.0)
    assert h.fcu.cbit.raised("MAG_FAULT")
    assert not h.fcu.cbit.raised("EKF_INNOV")
    h.gps_offset_every_other = 12.0   # second family
    h.run(3.0)
    assert h.fcu.cbit.raised("EKF_INNOV")


def test_ekf_diverged_wiring():
    h = armed_host()
    h.fcu.ekf.diverged = True
    h.run(0.1)
    assert h.fcu.cbit.raised("EKF_DIVERGED")
    assert h.fcu.cbit.inhibit_fire and h.fcu.cbit.inhibit_arming


# ------------------------------------------------------------- baro / mag

def test_baro_fault_on_garbage_frames():
    h = armed_host()
    h.baro_nan = True
    h.run(2.5)
    assert h.fcu.cbit.raised("BARO_FAULT")
    h.baro_nan = False
    h.run(2.5)
    assert not h.fcu.cbit.raised("BARO_FAULT")


def test_mag_fault_latches_per_flight():
    h = armed_host()
    h.mag_swapped = True
    h.run(3.5)
    assert h.fcu.cbit.raised("MAG_FAULT")
    h.mag_swapped = False
    h.run(3.0)
    assert h.fcu.cbit.raised("MAG_FAULT")          # latched (yaw-source switch)
    h.fcu.cbit.clear("MAG_FAULT")
    assert not h.fcu.cbit.raised("MAG_FAULT")


# -------------------------------------------------------------- actuation

def test_motor_response_deficit_detail_and_latch():
    h = armed_host()
    h.run(1.0)                        # mixer output settled
    h.esc_rpm = (6000.0, 6000.0, 4500.0, 6000.0)   # rotor 2: 25% share down
    h.run(1.5)
    assert h.fcu.cbit.raised("MOTOR_RESPONSE")
    assert h.fcu.cbit.snapshot()["MOTOR_RESPONSE"]["detail"] == "rotor 2"
    h.esc_rpm = (6000.0,) * 4
    h.run(1.0)
    assert h.fcu.cbit.raised("MOTOR_RESPONSE")     # damage latches
    assert h.fcu.cbit.inhibit_fire and h.fcu.cbit.inhibit_arming


def test_sat_persist_under_unachievable_demand():
    h = armed_host()
    ok, why = h.fcu.cmd_velocity((15.0, 0.0, 0.0)), None
    ok, why = h.fcu.cmd_set_mode(OFFBOARD)
    assert ok, why
    # Static truth: the gyro never confirms any rotation, so the rate
    # loop winds up and the motors pin at the rails — exactly the
    # persistent-saturation signature (3 s debounce: shorter railed
    # stretches are legitimate max-performance transients).
    h.run(4.5, offboard_v=(15.0, 0.0, 0.0))
    assert h.fcu.cbit.raised("SAT_PERSIST")


# ----------------------------------------------------- mirrors (no authority)

def test_link_mc_loss_mirror_keeps_legacy_failsafe():
    h = armed_host()
    h.heartbeats = False
    h.run(2.5)
    assert h.fcu.cbit.raised("LINK_MC_LOSS")
    assert h.fcu.failsafe == "LINK_LOSS"           # legacy chain untouched
    assert h.fcu.mode in (RTL, LAND)
    assert h.fcu.cbit.degraded_mode() == ""        # mirror never commands


def test_batt_mirrors_follow_monitor_and_legacy():
    h = armed_host()
    h.v_cell = 3.45
    h.run(1.5)
    assert h.fcu.cbit.raised("BATT_LOW")
    assert h.fcu.failsafe == "BATT_LOW" and h.fcu.mode in (RTL, LAND)
    h.v_cell = 3.20
    h.run(1.5)
    assert h.fcu.cbit.raised("BATT_CRIT")
    assert h.fcu.mode == LAND


# ------------------------------------------------------------ housekeeping

def test_param_crc_on_table_corruption():
    h = armed_host()
    h.fcu.params._values["fcu.pos_kp"] = 999.0     # simulated bit-rot
    h.run(1.5)
    assert h.fcu.cbit.raised("PARAM_CRC")
    assert h.fcu.cbit.inhibit_fire and h.fcu.cbit.inhibit_arming


def test_sched_overrun_names_the_task():
    h = armed_host()
    h.fcu.sched._by_name["nav"].cost_ticks = 100   # modeled CPU overload
    h.run(1.5)
    assert h.fcu.cbit.raised("SCHED_OVERRUN")
    assert "nav" in h.fcu.cbit.snapshot()["SCHED_OVERRUN"]["detail"]


def test_align_fail_counts_retries():
    h = CbitHost()
    h.gyro_amp = 0.5                  # vibration: variance gate fails
    h.run(7.5)
    assert h.fcu.state != ARMED and h.fcu.ekf is None
    assert h.fcu.cbit.raised("ALIGN_FAIL")
    assert h.fcu.cbit.inhibit_arming


def test_wdog_miss_on_wedged_fast_monitor():
    h = armed_host()
    h.fcu.sched._by_name["cbit_fast"].cost_ticks = 80_000   # wedged busy
    h.run(2.5)
    assert h.fcu.cbit.raised("WDOG_MISS")


# ---------------------------------------------------------- vocabulary pin

def test_monitor_state_vocabulary_matches_fcu():
    # monitors.py cannot import coopfc.fcu (cycle); pin the literals.
    from coopuavs.coopfc.cbit import monitors
    assert monitors._ARMED == ARMED
    assert STANDBY == "STANDBY"       # used implicitly via batt mirrors
