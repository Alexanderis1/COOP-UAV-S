"""P4-1: SitlEngine — the fleet SITL micro-loop behind ``World.micro``.

N vehicles of physics + P2 hw devices + one CoopFC FCU each, the
``sil/bench.py`` shape fleet-vectorized. Pins here:

- the frozen ORDERING §6 micro-tick order (devices → per-vehicle FCU →
  latch → Dryden → powertrain → ONE batched RK4);
- registry-named device streams (``sensor/*`` parents, ``dryden``) and
  the fleet-size invariance they buy: vehicle u1 flies bit-identically
  whether it is alone or one of two;
- run-twice bit-identical determinism (truth + nav), wind + gusts on;
- IMU acceleration = the exact wrench ``force_world / m`` (the P3 dv/dt
  finite difference was a documented bench placeholder; user decision
  2026-06-12 closes it in P4-1);
- stand convention: a non-ARMED row is frozen truth (devices keep
  sampling it with real noise — alignment runs on honest data);
- wind enters as a plant FORCE: the world's truth-side displacement
  skips SITL friendlies (and still applies to legacy bodies);
- ``ekf.late_meas == 0`` through real fleet device timing (the P3-R F1
  CBIT seam contract any P4 host must keep).
"""

from __future__ import annotations

import numpy as np
import pytest

from coopuavs.core.rng import RngRegistry
from coopuavs.sil.fleet import SitlEngine
from coopuavs.sil.vehicle import FriendlyVehicle
from coopuavs.sim.environment import Environment
from coopuavs.sim.weather import WeatherState
from coopuavs.sim.world import World

DT = 0.05
STARTS = [("u1", (0.0, 0.0, 50.0)), ("u2", (30.0, 0.0, 50.0))]


def _weather(seed=0, wind=0.0):
    return WeatherState(RngRegistry(seed).stream("weather"),
                        wind_speed=wind, wind_dir_deg=270.0)


def _engine(vehicles=STARTS, seed=3, wind=0.0):
    weather = _weather(seed, wind) if wind else None
    return SitlEngine(vehicles, RngRegistry(seed), weather=weather,
                      world_dt=DT)


def _run(eng, t_span, t0=0.0):
    steps = round(t_span / DT)
    for m in range(steps):
        eng.run_macro_step(t0 + m * DT, DT)
    return t0 + steps * DT


def _boot_and_arm(eng, t_max=8.0):
    t = 0.0
    while t < t_max and not all(f.pbit_ok for f in eng.fcus):
        t = _run(eng, DT, t)
    assert all(f.pbit_ok for f in eng.fcus), \
        [f.pbit_reasons for f in eng.fcus]
    for f in eng.fcus:
        ok, why = f.cmd_arm()
        assert ok, why
    return t


# ---------------------------------------------------------------- validation

def test_base_hz_must_match_fcu_tick_rate():
    with pytest.raises(ValueError, match="base_hz"):
        SitlEngine(STARTS, RngRegistry(0), world_dt=DT, base_hz=400)


def test_duplicate_ids_rejected():
    with pytest.raises(ValueError, match="duplicate"):
        SitlEngine([("u1", (0, 0, 50)), ("u1", (1, 0, 50))],
                   RngRegistry(0), world_dt=DT)


def test_macro_dt_mismatch_rejected():
    eng = _engine()
    with pytest.raises(ValueError, match="dt"):
        eng.run_macro_step(0.0, 0.1)


# ------------------------------------------------------------ §6 tick order

def _label(seq, name, fn):
    def wrapped(*a, **k):
        seq.append(name)
        return fn(*a, **k)
    return wrapped


def test_micro_tick_order_is_ordering_section6():
    """First micro-tick (every device divisor fires at k=0): devices
    sample truth, then each FCU in vehicle order, then Dryden, then the
    powertrain bus solve, then the single batched RK4."""
    eng = _engine(wind=8.0)
    assert eng.dryden is not None
    seq: list[str] = []
    eng.imu.sample = _label(seq, "imu", eng.imu.sample)
    eng.gps.tick = _label(seq, "gps", eng.gps.tick)
    eng.baro.sample = _label(seq, "baro", eng.baro.sample)
    eng.mag.sample = _label(seq, "mag", eng.mag.sample)
    eng.esc.sample = _label(seq, "esc", eng.esc.sample)
    for i, f in enumerate(eng.fcus):
        f.run_tick = _label(seq, f"fcu{i}", f.run_tick)
    eng.dryden.step = _label(seq, "dryden", eng.dryden.step)
    eng.pt.step = _label(seq, "pt", eng.pt.step)
    eng.plant.step = _label(seq, "plant", eng.plant.step)

    eng.run_macro_step(0.0, DT)
    first_tick = seq[:seq.index("plant") + 1]
    assert first_tick == ["imu", "gps", "baro", "mag", "esc",
                          "fcu0", "fcu1", "dryden", "pt", "plant"]


# ------------------------------------------------------- stand + flight

def test_disarmed_rows_frozen_devices_alive():
    """Stand convention: never-armed truth is frozen even under wind,
    while the devices keep sampling it with real noise (so alignment
    and the EKF run on honest data, the bench convention)."""
    eng = _engine(wind=8.0)
    state0 = eng.state.copy()
    _run(eng, 0.5)
    np.testing.assert_array_equal(eng.state, state0)
    # devices delivered frames meanwhile
    assert eng.hals[0].port("imu").read()[0] > 0
    assert eng.hals[0].port("gps").read()[0] > 0


def test_boot_arm_hover_calm():
    eng = _engine()
    t = _boot_and_arm(eng)
    _run(eng, 2.0, t)
    for i, (uid, start) in enumerate(STARTS):
        err = np.linalg.norm(eng.state[i, 0:3] - np.asarray(start))
        assert err < 2.5, f"{uid} drifted {err:.2f} m in calm hover"
    for f in eng.fcus:
        assert f.state == "ARMED" and f.failsafe == ""
        # P3-R F1 contract: no device frame may reach the EKF behind
        # its fusion horizon through real fleet timing.
        assert all(v == 0 for v in f.ekf.late_meas.values()), f.ekf.late_meas
        assert f.ekf.diverged is False


def test_hover_under_wind_and_dryden():
    """Wind enters as a plant force: 8 m/s mean + MIL-8785C Dryden
    gusts. The controllers must hold the fleet near the hold point
    through the force path alone."""
    eng = _engine(wind=8.0)
    assert eng.dryden is not None
    t = _boot_and_arm(eng)
    _run(eng, 2.0, t)
    mean = eng.weather.mean_wind_at(eng.state[:, 2])
    assert not np.allclose(eng._wind, mean)      # gusts actually present
    for i, (uid, start) in enumerate(STARTS):
        err = np.linalg.norm(eng.state[i, 0:3] - np.asarray(start))
        assert err < 3.0, f"{uid} drifted {err:.2f} m under wind"
    assert all(f.state == "ARMED" and f.failsafe == "" for f in eng.fcus)


def test_calm_weather_block_means_no_dryden():
    eng = SitlEngine(STARTS, RngRegistry(3), weather=_weather(3, 0.0),
                     world_dt=DT)
    assert eng.dryden is None


# -------------------------------------------------------------- determinism

def _trajectory(eng, t_fly=1.5):
    t = _boot_and_arm(eng)
    hist = []
    for _ in range(round(t_fly / DT)):
        eng.run_macro_step(t, DT)
        t += DT
        hist.append(eng.state.copy())
    navs = [tuple(f.nav.pos) + tuple(f.nav.vel) for f in eng.fcus]
    return np.stack(hist), navs


def test_run_twice_bit_identical():
    h1, n1 = _trajectory(_engine(wind=8.0))
    h2, n2 = _trajectory(_engine(wind=8.0))
    np.testing.assert_array_equal(h1, h2)
    assert n1 == n2


def test_fleet_size_invariance():
    """ORDERING §4: per-device-type parents spawn one child per vehicle,
    so adding u2 must leave u1's DRAW HISTORY identical (the §4 contract;
    bank-level bitwise pins live in test_hw_determinism).

    The gust history is truth-independent, so it is pinned bit-exact
    through the engine wiring. The flown trajectory is pinned to 1e-9:
    batched einsum/matmul kernels differ at the last ULP between n=1 and
    n=2 shapes (measured 1.6e-14 relative over this flight), while any
    stream-wiring fault diverges at device-noise scale — five-plus orders
    louder. Run-twice bitwise determinism (same shapes) is pinned above."""
    e1 = _engine(vehicles=STARTS[:1], wind=8.0)
    e2 = _engine(vehicles=STARTS, wind=8.0)
    gusts1, gusts2 = [], []
    for eng, log in ((e1, gusts1), (e2, gusts2)):
        step = eng.dryden.step
        eng.dryden.step = (lambda s=step, lg=log:
                           (lambda g: (lg.append(g[0].copy()), g)[1])(s()))
    h1, n1 = _trajectory(e1)
    h2, n2 = _trajectory(e2)
    np.testing.assert_array_equal(np.stack(gusts1), np.stack(gusts2))
    np.testing.assert_allclose(h1[:, 0, :], h2[:, 0, :], rtol=1e-9, atol=1e-9)
    np.testing.assert_allclose(n1[0], n2[0], rtol=1e-9, atol=1e-9)


# ------------------------------------------------------- exact-wrench IMU

def test_imu_accel_is_exact_wrench_over_mass():
    """The IMU samples the plant's force_world / m (gravity included) at
    the latched inputs — not a dv/dt finite difference (user decision
    2026-06-12: the P3 bench placeholder closes here)."""
    eng = _engine()
    t = _boot_and_arm(eng)
    # One step so the flying mask latches: devices sample the PRE-step
    # state, so the tick that processes the arm transition still reads
    # the stand (zero accel) — wrench-based accel starts next tick.
    t = _run(eng, DT, t)

    captured = {}
    wrench = eng.plant.wrench

    def spy_wrench(*a, **k):
        force, tau = wrench(*a, **k)
        captured["accel"] = force / eng.plant.mass[:, None]
        return force, tau
    eng.plant.wrench = spy_wrench

    sample = eng.imu.sample

    def spy_sample(quat, omega_body, accel_world):
        captured.setdefault("seen", []).append(
            (accel_world.copy(), captured["accel"].copy()))
        return sample(quat, omega_body, accel_world)
    eng.imu.sample = spy_sample

    eng.run_macro_step(t, DT)
    assert captured["seen"], "IMU never sampled"
    for got, expect in captured["seen"]:
        np.testing.assert_array_equal(got, expect)


def test_imu_accel_zero_on_stand():
    eng = _engine()
    seen = []
    sample = eng.imu.sample
    eng.imu.sample = lambda q, w, a: (seen.append(a.copy()),
                                      sample(q, w, a))[1]
    eng.run_macro_step(0.0, DT)
    assert seen and all(np.all(a == 0.0) for a in seen)


# ------------------------------------------------------ world.step seam

ENV = {
    "bounds": [-4000.0, -4000.0, 4000.0, 4000.0],
    "cell_size": 100.0,
    "default_zone": "SAFE",
    "zones": [],
    "assets": [{"name": "asset", "position": [0.0, 0.0, 0.0], "value": 1.0}],
}


def test_world_micro_seam_and_wind_skip():
    """Installed as World.micro the engine ticks in lockstep with the
    world clock, and windy weather displaces ONLY legacy friendlies:
    SITL vehicles take wind as a plant force (wind_displaced=False)."""
    from coopuavs.core.bus import MessageBus
    from coopuavs.interceptors.effectors import EFFECTOR_FACTORIES
    from coopuavs.interceptors.uav import InterceptorUav

    world = World(Environment.from_config(ENV), dt=DT, seed=5)
    world.weather = WeatherState(world.rng_registry.stream("weather"),
                                 wind_speed=8.0, wind_dir_deg=270.0)
    eng = SitlEngine(STARTS, world.rng_registry, weather=world.weather,
                     world_dt=world.dt)
    world.micro = eng
    fv = FriendlyVehicle(eng, "u1", home=(0.0, 0.0, 0.0))
    world.friendlies["u1"] = fv

    legacy = InterceptorUav("L1", MessageBus(), home=np.array([0.0, 0.0, 0.0]),
                            effector=EFFECTOR_FACTORIES["projectile"]())
    legacy.body.position = np.array([100.0, 0.0, 30.0])
    world.friendlies["L1"] = legacy

    p_sitl = fv.position.copy()
    p_legacy = legacy.position.copy()
    for _ in range(10):
        world.step()

    assert abs(eng.clock.now - world.t) < 1e-9
    np.testing.assert_array_equal(fv.position, p_sitl)     # frozen, NOT displaced
    assert np.linalg.norm(legacy.position - p_legacy) > 0.1  # legacy still is


def test_mean_wind_at_matches_wind_at_shear_law():
    """The vectorized gust-free helper the engine feeds the plant must
    track the scalar wind_at law exactly (gusts are the difference)."""
    w = _weather(seed=9, wind=8.0)
    w.step(DT)   # put nonzero OU gust state into wind_at
    for z in (0.0, 5.0, 50.0, 400.0):
        gustless = w.mean_wind_at(np.array([z]))[0]
        np.testing.assert_allclose(
            gustless + np.array([w._gust[0], w._gust[1], 0.0]),
            w.wind_at(z), rtol=0, atol=1e-12)
