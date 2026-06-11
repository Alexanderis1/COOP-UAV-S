"""P2 gate-review fix: absolute draw-layout pins for every stochastic hw
device.

Each device docstring declares its draw layout FROZEN (which init/tick
standard-normal columns feed which error component). The statistical and
determinism suites cannot enforce that contract: a consistent column swap
(e.g. accel white noise reading the gyro columns, or the RW fed from the
white columns) leaves every marginal statistic, the bit-exact
generate==sample pin, and even the @slow Allan identification inside its
gates — proven by surviving mutants in the 2026-06-11 adversarial review.

These tests reconstruct the documented draws test-side from an
identically-seeded parent (same spawn children, same draw counts) and
assert each device output equals the documented composition bit-for-bit.
Any column reassignment, init/tick mix-up, or cross-channel reuse fails.
"""

import numpy as np

from coopuavs.hw.baro import Baro, BaroParams
from coopuavs.hw.esc_telem import RPM_PER_RAD_S, EscTelem, EscTelemParams
from coopuavs.hw.gps import Gps, GpsParams
from coopuavs.hw.imu import Imu, ImuParams
from coopuavs.hw.mag import Mag, MagParams, theater_field_enu
from coopuavs.physics import GRAVITY
from coopuavs.physics import rigid_body as rb

SEED = 4242
N = 3


def _draws(init_cols: int, tick_cols: int, ticks: int = 1):
    """Replay the device's documented stream consumption: spawn children
    from an identically-seeded parent, then per child one init draw block
    and `ticks` tick draw blocks."""
    children = np.random.default_rng(SEED).spawn(N)
    init = np.stack([g.standard_normal(init_cols) for g in children])
    eps = np.stack([g.standard_normal((ticks, tick_cols)) for g in children],
                   axis=1)                        # (ticks, N, tick_cols)
    return init, eps


def _gm_first(sigma, tau_s, dt, eps0, eps1):
    """First output of the exact-ZOH GM: phi*(sigma*eps0) + sigma*sqrt(1-phi^2)*eps1,
    with the same float ops as stoch.GaussMarkov."""
    phi = float(np.exp(-dt / tau_s))
    s = sigma * float(np.sqrt(1.0 - phi * phi))
    return phi * (sigma * eps0) + s * eps1


def test_imu_draw_layout_is_exactly_as_documented():
    # init: [0:3] gyro turn-on, [3:6] accel turn-on, [6:9] gyro GM, [9:12]
    # accel GM; tick: [0:3] gyro wn, [3:6] accel wn, [6:9] gyro RW,
    # [9:12] accel RW, [12:15] gyro GM, [15:18] accel GM.
    p = ImuParams(
        rate_hz=400.0,
        gyro_noise_density=1e-3, gyro_gm_sigma=4e-3, gyro_gm_tau_s=20.0,
        gyro_rw_sigma=2e-4, gyro_turn_on_sigma=3e-3, gyro_lsb=0.0,
        gyro_range=np.inf,
        accel_noise_density=2e-3, accel_gm_sigma=8e-3, accel_gm_tau_s=10.0,
        accel_rw_sigma=4e-4, accel_turn_on_sigma=2e-1, accel_lsb=0.0,
        accel_range=np.inf, fifo_depth=4)
    imu = Imu(p, N, np.random.default_rng(SEED))
    quat = np.zeros((N, 4))
    quat[:, 0] = 1.0
    gyro, accel = imu.sample(quat, np.zeros((N, 3)),
                             np.broadcast_to([0.0, 0.0, -GRAVITY], (N, 3)))
    init, eps = _draws(12, 18)
    dt = 1.0 / 400.0
    e = eps[0]

    expect_g = np.zeros((N, 3)) + p.gyro_turn_on_sigma * init[:, 0:3]
    expect_g += np.zeros((N, 3)) + e[:, 6:9] * (p.gyro_rw_sigma * float(np.sqrt(dt)))
    expect_g += _gm_first(p.gyro_gm_sigma, p.gyro_gm_tau_s, dt,
                          init[:, 6:9], e[:, 12:15])
    expect_g += e[:, 0:3] * (p.gyro_noise_density / np.sqrt(dt))
    np.testing.assert_array_equal(gyro, expect_g)

    expect_a = np.zeros((N, 3)) + p.accel_turn_on_sigma * init[:, 3:6]
    expect_a += np.zeros((N, 3)) + e[:, 9:12] * (p.accel_rw_sigma * float(np.sqrt(dt)))
    expect_a += _gm_first(p.accel_gm_sigma, p.accel_gm_tau_s, dt,
                          init[:, 9:12], e[:, 15:18])
    expect_a += e[:, 3:6] * (p.accel_noise_density / np.sqrt(dt))
    np.testing.assert_array_equal(accel, expect_a)


def test_gps_draw_layout_is_exactly_as_documented():
    # init: [0:3] GM cold start; per sample: [0:3] GM, [3:6] pos white,
    # [6:9] vel white.
    p = GpsParams(rate_hz=10.0, latency_s=0.0, sigma_pos_h=0.4,
                  sigma_pos_v=0.8, gm_sigma_h=1.2, gm_sigma_v=2.4,
                  gm_tau_s=60.0, sigma_vel=0.1)
    gps = Gps(p, N, np.random.default_rng(SEED), 800)
    pos = np.tile([100.0, -50.0, 80.0], (N, 1))
    vel = np.tile([10.0, -5.0, 0.0], (N, 1))
    fix = gps.tick(pos, vel)                      # latency 0: same tick
    init, eps = _draws(3, 9)
    e = eps[0]
    gm_scale = np.array([1.2, 1.2, 2.4])
    wn_pos = np.array([0.4, 0.4, 0.8])
    expect_pos = pos + gm_scale * _gm_first(1.0, 60.0, 0.1, init, e[:, 0:3])
    expect_pos += e[:, 3:6] * wn_pos
    np.testing.assert_array_equal(fix.pos, expect_pos)
    np.testing.assert_array_equal(fix.vel, vel + e[:, 6:9] * 0.1)


def test_baro_draw_layout_is_exactly_as_documented():
    # init: [0] GM cold start; per sample: [0] GM, [1] white.
    from coopuavs.physics import atmosphere
    p = BaroParams(rate_hz=50.0, sigma_pa=3.0, gm_sigma_pa=15.0,
                   gm_tau_s=600.0, lsb_pa=0.0)
    baro = Baro(p, N, np.random.default_rng(SEED))
    alt = np.array([10.0, 120.0, 900.0])
    out = baro.sample(alt)
    init, eps = _draws(1, 2)
    e = eps[0]
    expect = atmosphere.pressure(alt) + _gm_first(15.0, 600.0, 1.0 / 50.0,
                                                  init[:, 0], e[:, 0])
    expect += e[:, 1] * 3.0
    np.testing.assert_array_equal(out, expect)


def test_mag_draw_layout_is_exactly_as_documented():
    # init: [0:3] hard iron, [3:6] GM cold start; per sample: [0:3] GM,
    # [3:6] white.
    p = MagParams(rate_hz=50.0, magnitude_ut=50.0, declination_deg=4.0,
                  inclination_deg=63.0, sigma_ut=0.3, gm_sigma_ut=0.5,
                  gm_tau_s=300.0, hard_iron_sigma_ut=2.0, lsb_ut=0.0)
    mag = Mag(p, N, np.random.default_rng(SEED))
    rng = np.random.default_rng(1)
    axis = rng.standard_normal((N, 3))
    axis /= np.linalg.norm(axis, axis=1, keepdims=True)
    quat = rb.quat_from_axis_angle(axis, rng.uniform(-np.pi, np.pi, N))
    out = mag.sample(quat)
    init, eps = _draws(6, 6)
    e = eps[0]
    b = np.broadcast_to(theater_field_enu(50.0, 4.0, 63.0), (N, 3))
    expect = rb.quat_rotate_inv(quat, b)
    expect += 2.0 * init[:, 0:3]
    expect += _gm_first(0.5, 300.0, 1.0 / 50.0, init[:, 3:6], e[:, 0:3])
    expect += e[:, 3:6] * 0.3
    np.testing.assert_array_equal(out, expect)


def test_esc_telem_draw_layout_is_exactly_as_documented():
    # per sample: [0:r] rpm white, [r] voltage white, [r+1] current white.
    r = 4
    p = EscTelemParams(rate_hz=10.0, sigma_rpm=5.0, sigma_v=0.02,
                       sigma_i=0.1, rpm_lsb=0.0, v_lsb=0.0, i_lsb=0.0)
    telem = EscTelem(p, N, r, np.random.default_rng(SEED))
    omega = np.full((N, r), 900.0)
    v_bus = np.full(N, 44.4)
    i_bus = np.full(N, 120.0)
    frame = telem.sample(omega, v_bus, i_bus)
    _, eps = _draws(0, r + 2)
    e = eps[0]
    np.testing.assert_array_equal(frame.rpm,
                                  omega * RPM_PER_RAD_S + e[:, :r] * 5.0)
    np.testing.assert_array_equal(frame.voltage, v_bus + e[:, r] * 0.02)
    np.testing.assert_array_equal(frame.current, i_bus + e[:, r + 1] * 0.1)
