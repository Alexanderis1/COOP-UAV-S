"""P2-2: hw/gps.py — GNSS receiver model.

Headline pins: 10 Hz output with EXACTLY 120 ms latency on the integer
device clock (96 ticks at 800 Hz — never a float-time comparison), white
noise + first-order Gauss-Markov correlated wander split horizontal /
vertical, and the fix-type field.
"""

import numpy as np
import pytest

from coopuavs.hw.gps import FIX_3D, Gps, GpsParams
from coopuavs.hw.params import load_devices


def _params(**over) -> GpsParams:
    base = dict(
        rate_hz=10.0, latency_s=0.120,
        sigma_pos_h=0.0, sigma_pos_v=0.0,
        gm_sigma_h=0.0, gm_sigma_v=0.0, gm_tau_s=60.0,
        sigma_vel=0.0,
    )
    base.update(over)
    return GpsParams(**base)


def _truth(t, n):
    pos = np.empty((n, 3))
    pos[:, 0] = 1.0 * t
    pos[:, 1] = -2.0 * t
    pos[:, 2] = 0.5 * t
    vel = np.broadcast_to([1.0, -2.0, 0.5], (n, 3)).copy()
    return pos, vel


def _run(gps, clock_hz, ticks, n):
    out = []
    for k in range(ticks):
        fix = gps.tick(*_truth(k / clock_hz, n))
        if fix is not None:
            out.append((k, fix))
    return out


# ------------------------------------------------------------ timing pins

def test_fix_arrives_exactly_96_ticks_after_each_sample_at_800_hz():
    clock_hz = 800
    gps = Gps(_params(), 2, np.random.default_rng(1), clock_hz)
    arrivals = _run(gps, clock_hz, 2000, 2)
    ticks = [k for k, _ in arrivals]
    assert len(ticks) == 24                # fails closed: fixes MUST arrive
    assert ticks == [96 + 80 * i for i in range(24)]   # sample lattice + latency
    for k, fix in arrivals:
        assert fix.stamp_ticks == k - 96                       # measured 120 ms ago, exact
        assert fix.stamp_s == fix.stamp_ticks / clock_hz


def test_latency_and_rate_must_be_exact_tick_counts():
    with pytest.raises(ValueError):
        Gps(_params(latency_s=0.1234), 1, np.random.default_rng(2), 800)
    with pytest.raises(ValueError):
        Gps(_params(rate_hz=7.0), 1, np.random.default_rng(2), 800)   # 800/7 not integer
    with pytest.raises(ValueError):
        Gps(_params(), 1, np.random.default_rng(2), 0)
    # 120 ms is exact at 100 Hz too (12 ticks)
    Gps(_params(), 1, np.random.default_rng(2), 100)


def test_quiet_gps_delivers_truth_of_120_ms_ago():
    clock_hz = 800
    n = 3
    gps = Gps(_params(), n, np.random.default_rng(3), clock_hz)
    arrivals = _run(gps, clock_hz, 4000, n)
    assert len(arrivals) == 49             # fails closed: fixes MUST arrive
    for k, fix in arrivals:
        pos_then, vel_then = _truth(fix.stamp_ticks / clock_hz, n)
        np.testing.assert_allclose(fix.pos, pos_then, atol=1e-12)
        np.testing.assert_array_equal(fix.vel, vel_then)


def test_fix_type_is_3d_for_every_vehicle():
    gps = Gps(_params(), 4, np.random.default_rng(4), 800)
    fixes = _run(gps, 800, 400, 4)
    assert FIX_3D == 3
    for _, fix in fixes:
        assert fix.fix_type.shape == (4,)
        assert fix.fix_type.dtype == np.uint8
        np.testing.assert_array_equal(fix.fix_type, 3)


# -------------------------------------------------------------- stochastic

def _ensemble_errors(params, n, n_fixes, seed, clock_hz=100):
    gps = Gps(params, n, np.random.default_rng(seed), clock_hz)
    errs = []
    for k, fix in _run(gps, clock_hz, n_fixes * clock_hz // 10 + 100, n):
        pos_then, vel_then = _truth(fix.stamp_ticks / clock_hz, n)
        errs.append((fix.pos - pos_then, fix.vel - vel_then))
    pos = np.stack([e[0] for e in errs])     # (fixes, n, 3)
    vel = np.stack([e[1] for e in errs])
    return pos, vel


def test_white_noise_std_split_h_v():
    p = _params(sigma_pos_h=0.4, sigma_pos_v=0.8, sigma_vel=0.1)
    pos, vel = _ensemble_errors(p, 2048, 40, seed=5)
    assert abs(pos[:, :, 0].std() - 0.4) / 0.4 < 0.05
    assert abs(pos[:, :, 1].std() - 0.4) / 0.4 < 0.05
    assert abs(pos[:, :, 2].std() - 0.8) / 0.8 < 0.05
    assert abs(vel.std() - 0.1) / 0.1 < 0.05


def test_gm_wander_stationary_variance_and_fix_to_fix_correlation():
    tau = 0.5                                 # fix dt 0.1 -> lag-1 rho e^-0.2
    p = _params(gm_sigma_h=1.2, gm_sigma_v=2.4, gm_tau_s=tau)
    pos, _ = _ensemble_errors(p, 4096, 60, seed=6)
    assert abs(pos[:, :, 0].var() - 1.2**2) / 1.2**2 < 0.10
    assert abs(pos[:, :, 2].var() - 2.4**2) / 2.4**2 < 0.10
    rho = np.mean(pos[:-1, :, 0] * pos[1:, :, 0]) / pos[:, :, 0].var()
    assert abs(rho - np.exp(-0.1 / tau)) < 0.05


# ------------------------------------------------------------ determinism

def test_run_twice_identical_and_seed_sensitivity():
    p = _params(sigma_pos_h=0.4, sigma_pos_v=0.8, gm_sigma_h=1.0,
                gm_sigma_v=1.0, gm_tau_s=30.0, sigma_vel=0.1)

    def run(seed):
        gps = Gps(p, 3, np.random.default_rng(seed), 800)
        return np.stack([f.pos for _, f in _run(gps, 800, 2000, 3)])

    np.testing.assert_array_equal(run(7), run(7))
    assert np.abs(run(7) - run(8)).max() > 0.0


def test_fleet_growth_leaves_existing_vehicles_fixes_identical():
    p = _params(sigma_pos_h=0.4, sigma_pos_v=0.8, gm_sigma_h=1.0,
                gm_sigma_v=1.0, gm_tau_s=30.0, sigma_vel=0.1)

    def run(n):
        gps = Gps(p, n, np.random.default_rng(9), 800)
        return np.stack([f.pos for _, f in _run(gps, 800, 2000, n)])

    np.testing.assert_array_equal(run(3), run(6)[:, :3, :])


# ------------------------------------------------------------- parameters

def test_params_load_from_yaml():
    cfg = load_devices("interceptor_devices")
    p = GpsParams.from_dict(cfg["gps"])
    assert p.rate_hz == 10.0 and p.latency_s == 0.120
    Gps(p, 2, np.random.default_rng(0), 800)


def test_params_validation_rejects_bad_values():
    with pytest.raises(ValueError):
        _params(rate_hz=0.0)
    with pytest.raises(ValueError):
        _params(latency_s=-0.1)
    with pytest.raises(ValueError):
        _params(sigma_pos_h=-1.0)
    with pytest.raises(ValueError):
        _params(gm_tau_s=0.0)
    with pytest.raises(ValueError):
        _params(sigma_vel=np.nan)
    with pytest.raises(ValueError):
        Gps(_params(), 0, np.random.default_rng(1), 800)
