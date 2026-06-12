"""P3-8: bench acceptance flights — real devices, real noise, one FCU.

Hover gate semantics (user decision 2026-06-12, fidelity-first split;
RESEARCH.md "P3 CoopFC flight stack"):

- CONTROL error = |EKF estimate - hold setpoint|: what the cascade can
  actually control. Plan numbers apply here: RMS < 0.15 m calm,
  < 1.0 m in 8 m/s mean wind + MIL-8785C Dryden (w20 = 8 m/s).
- TRUTH error = |truth - truth at hold capture|: bounded below by the
  navigation error a GNSS-class suite cannot remove (GPS GM wander
  sigma_h 1.2 m, tau 60 s, drags the estimate and hence the vehicle).
  Gate 2.0 m RMS — the documented device budget (measured 0.5-0.9 m
  over seeds; ~1-1.5 m is the published GNSS position-hold class,
  centimeter hover needs RTK which this suite deliberately is not).

Waypoint square: 200 m sides at 10 m/s, MC-role OFFBOARD velocity
guidance from NAV telemetry at 10 Hz; TRUTH cross-track < 2 m on the
straight segments (plan number holds at face value here: the GM error
is mostly along-track-invariant; measured ~1 m class).

Perf (@perf, separate process per repo convention; >= 4 sim-s reps for
the Windows process_time quantum):

- 1-vehicle RTF >= 3x. The original ">= 20x" figure predates P1: the
  plant RK4 costs ~0.2 s CPU/sim-s INDEPENDENT of N (numpy small-batch
  overhead, the P1/P2 same-bound-both-N evidence), which alone caps a
  1-vehicle bench at ~5x. Re-scoped on the measured profile (plant+
  powertrain ~76%, FCU ~15%, devices ~8%) — user-approved 2026-06-12.
- 20-instance projection >= 1x, measured not assumed, per the P4 fleet
  architecture (ONE batched plant + device suite, N python FCUs):
  C20 = C_phys+dev(N=20, batched, no FCU) + 20 * C_fcu, with C_fcu
  measured DIRECTLY on a synthetic-frame FCU host (armed POS_HOLD, full
  pipeline: drivers/EKF/cascade/mixer) — the earlier bench-minus-physics
  subtraction amplified timer noise into +-40% swings; the direct
  measurement repeats to 3 decimal places. min-of-3 reps throughout
  (least-interrupted run estimates true cost). Enabling work: the EKF
  fusion path is selection-indexed (sha256-verified value-identical
  refactor, 2026-06-12) — see estimation/ekf.py `_fuse_sel`.
"""

from __future__ import annotations

import math
import time

import numpy as np
import pytest

from coopuavs.sil.bench import DT, RHO, TICK_HZ, Bench

CRUISE = 10.0
WP_RADIUS = 5.0


def _hover_rms(seed: int, wind, w20, t_meas: float = 30.0):
    b = Bench(seed=seed, wind_mean=wind, dryden_wind20=w20)
    b.boot_and_arm()
    hold = b.fcu._hold_pos
    truth0 = b.state[0, 0:3].copy()
    b.run(5.0)                                  # transient out
    ce, te = [], []
    for _ in range(round(t_meas * TICK_HZ)):
        b.tick()
        if b.k % 16 == 0:
            n = b.fcu.nav
            ce.append((n.pos[0] - hold[0]) ** 2 + (n.pos[1] - hold[1]) ** 2)
            s = b.state[0]
            te.append((s[0] - truth0[0]) ** 2 + (s[1] - truth0[1]) ** 2)
    return math.sqrt(np.mean(ce)), math.sqrt(np.mean(te))


def test_hover_calm_control_and_truth_rms():
    ctl, truth = _hover_rms(0, (0.0, 0.0, 0.0), None)
    assert ctl < 0.15, f"calm control RMS {ctl:.3f} m"
    assert truth < 2.0, f"calm truth RMS {truth:.3f} m (GNSS budget)"


@pytest.mark.slow
def test_hover_wind_dryden_control_and_truth_rms():
    worst_ctl = worst_truth = 0.0
    for seed in range(3):
        ctl, truth = _hover_rms(seed, (8.0, 0.0, 0.0), 8.0)
        worst_ctl, worst_truth = max(worst_ctl, ctl), max(worst_truth, truth)
    assert worst_ctl < 1.0, f"wind control RMS {worst_ctl:.3f} m"
    assert worst_truth < 2.0, f"wind truth RMS {worst_truth:.3f} m"


# ------------------------------------------------------- waypoint square


def _seg_cross_track(p, a, b) -> float:
    ab = (b[0] - a[0], b[1] - a[1])
    n = math.hypot(*ab)
    return abs((p[0] - a[0]) * ab[1] - (p[1] - a[1]) * ab[0]) / n


@pytest.mark.slow
def test_waypoint_square_cross_track():
    b = Bench(seed=2)
    b.boot_and_arm()
    b.run(3.0)
    nav_sub = b.fcu.topics.subscribe("nav_state")
    start = b.fcu.nav.pos
    z_hold = start[2]
    corners = [(start[0] + 200.0, start[1]), (start[0] + 200.0, start[1] + 200.0),
               (start[0], start[1] + 200.0), (start[0], start[1])]
    b.fcu.cmd_velocity((0.0, 0.0, 0.0))
    ok, why = b.fcu.cmd_set_mode("OFFBOARD")
    assert ok, why

    prev = (start[0], start[1])
    xt = []
    for wp in corners:
        def guide(bb, wp=wp, prev=prev):
            if bb.k % 80 == 0:                  # MC at 10 Hz on telemetry
                nav = nav_sub.read()
                dx, dy = wp[0] - nav.pos[0], wp[1] - nav.pos[1]
                d = math.hypot(dx, dy)
                sp = min(CRUISE, max(1.0, d))   # slow into the corner
                bb.fcu.cmd_velocity((sp * dx / d, sp * dy / d,
                                     1.0 * (z_hold - nav.pos[2])))
                s = bb.state[0]
                if (math.hypot(s[0] - prev[0], s[1] - prev[1]) > 15.0
                        and d > 15.0):
                    xt.append(_seg_cross_track((s[0], s[1]), prev, wp))
                return d < WP_RADIUS
            return False
        reached = b.run(60.0, until=guide)
        assert reached, f"waypoint {wp} not reached (truth {b.state[0, :2]})"
        prev = wp
    worst = max(xt)
    assert worst < 2.0, f"cross-track {worst:.2f} m"
    assert not b.fcu.failsafe, b.fcu.failsafe


def test_no_late_measurements_through_real_device_timing():
    # P3 review F1: GPS device latency (120 ms) plus the driver poll
    # quantization must stay inside the EKF lag_s horizon — every
    # measurement fuses at its own stamp. A 10 Hz GPS poll off-phase
    # from the 120 ms fix delivery used to hand the EKF every fix
    # 200 ms old (60 ms behind the horizon): a silent, systematic
    # along-track bias at speed. ekf.late_meas is the CBIT seam.
    b = Bench(seed=1)
    b.boot_and_arm()
    b.run(3.0)
    ekf = b.fcu.ekf
    assert ekf.late_meas == {"gps": 0, "baro": 0, "mag": 0}
    assert ekf.last_gps_fuse is not None
    assert ekf.nis["gps_pos"][1] > 0        # fixes actually fused


# ----------------------------------------------------------- determinism


def test_bench_run_twice_bit_identical():
    def run():
        b = Bench(seed=7, wind_mean=(4.0, 1.0, 0.0), dryden_wind20=4.0)
        b.boot_and_arm()
        b.run(4.0)
        n = b.fcu.nav
        return (b.state.tobytes(), n.pos, n.q,
                b.hal.port("actuators").read())

    assert run() == run()


# ------------------------------------------------------------------ perf


@pytest.mark.perf
def test_bench_rtf_and_fleet_projection():
    from coopuavs.hw import params as hw_params
    from coopuavs.hw.baro import Baro, BaroParams
    from coopuavs.hw.esc_telem import EscTelem, EscTelemParams
    from coopuavs.hw.gps import Gps, GpsParams
    from coopuavs.hw.imu import Imu, ImuParams
    from coopuavs.hw.mag import Mag, MagParams
    from coopuavs.physics.battery import BatteryEcm
    from coopuavs.physics.motor import MotorEsc
    from coopuavs.physics.multirotor import MultirotorParams, MultirotorPlant
    from coopuavs.physics.params import load_airframe
    from coopuavs.physics.powertrain import Powertrain

    import sys
    sys.path.insert(0, "tests")
    from test_coopfc_fcu import SynthHost

    t_span = 8.0     # >> 15.625 ms Windows process_time quantum

    def best_of(fn, reps: int = 3) -> float:
        return min(fn() for _ in range(reps))

    def cost_bench1() -> float:
        b = Bench(seed=0)
        b.boot_and_arm()
        b.run(1.0)
        t0 = time.process_time()
        b.run(t_span)
        return (time.process_time() - t0) / t_span

    def cost_fcu() -> float:
        h = SynthHost()
        h.boot_and_arm()
        h.run(1.0, hb_every=0.1)
        t0 = time.process_time()
        h.run(t_span, hb_every=0.1)
        return (time.process_time() - t0) / t_span

    def cost_phys_dev(n: int) -> float:
        cfg = MultirotorParams.from_dict(load_airframe("interceptor_quad"))
        dev = hw_params.load_devices("interceptor_devices")
        rng = np.random.default_rng(0)
        imu = Imu(ImuParams.from_dict(dev["imu"]), n, rng.spawn(1)[0])
        gps = Gps(GpsParams.from_dict(dev["gps"]), n, rng.spawn(1)[0],
                  clock_hz=TICK_HZ)
        baro = Baro(BaroParams.from_dict(dev["baro"]), n, rng.spawn(1)[0])
        mag = Mag(MagParams.from_dict(dev["mag"]), n, rng.spawn(1)[0])
        esc = EscTelem(EscTelemParams.from_dict(dev["esc_telem"]), n,
                       cfg.n_rotors, rng.spawn(1)[0])
        plant = MultirotorPlant(cfg, n)
        motor = MotorEsc(n, cfg.n_rotors, **cfg.motor)
        pt = Powertrain(motor, BatteryEcm(n, **cfg.battery), 350.0)
        w_h = math.sqrt(cfg.mass * 9.81 / (cfg.n_rotors * cfg.kf))
        motor.omega[:] = w_h
        state = np.zeros((n, 13))
        state[:, 2] = 50.0
        state[:, 6] = 1.0
        throttle = np.full((n, cfg.n_rotors), 0.463)
        wind = np.zeros((n, 3))
        a_w = np.zeros((n, 3))

        def loop(steps: int, k0: int = 0):
            nonlocal state
            for k in range(k0, k0 + steps):
                if k % 2 == 0:
                    imu.sample(state[:, 6:10], state[:, 10:13], a_w)
                gps.tick(state[:, 0:3], state[:, 3:6])
                if k % 16 == 0:
                    baro.sample(state[:, 2])
                    mag.sample(state[:, 6:10])
                omega_r, v_bus, i_bus = pt.step(DT, throttle)
                if k % 80 == 0:
                    esc.sample(omega_r, v_bus, i_bus)
                state = plant.step(state, DT, omega_r, wind, RHO)

        loop(round(1.0 * TICK_HZ))                       # warm-up
        t0 = time.process_time()
        loop(round(t_span * TICK_HZ), k0=TICK_HZ)
        return (time.process_time() - t0) / t_span

    c_bench = best_of(cost_bench1)
    c_fcu = best_of(cost_fcu)
    c_phys20 = best_of(lambda: cost_phys_dev(20))
    c_20 = c_phys20 + 20.0 * c_fcu
    rtf1 = 1.0 / c_bench
    rtf20 = 1.0 / c_20
    print(f"\nbench 1-vehicle {c_bench:.3f} s/sim-s (RTF {rtf1:.1f}x); "
          f"phys+dev N=20 {c_phys20:.3f}; FCU stack (direct) {c_fcu:.4f}; "
          f"20-instance projection {c_20:.3f} s/sim-s (RTF {rtf20:.2f}x)")
    assert rtf1 >= 3.0, f"1-vehicle RTF {rtf1:.1f}x < 3x"
    assert rtf20 >= 1.0, f"20-instance projection RTF {rtf20:.2f}x < 1x"
