"""P1-2: ISA atmosphere + MIL-F-8785C low-altitude Dryden turbulence.

Atmosphere pinned against the 1976 ISA table (sea level / 1 km / 11 km).
Dryden pinned three ways: (a) MIL-F-8785C parameter table (sigma/L vs h, W20),
(b) closed-form bilinear coefficients vs scipy.signal.bilinear, (c) measured
Welch PSD of the generated gusts vs the analytic Dryden spectrum, plus a
variance check. Stochastic tests use fixed seeds (deterministic).
"""

from __future__ import annotations

import numpy as np
from scipy import signal

from coopuavs.physics import atmosphere as atm
from coopuavs.physics import dryden

FT = 0.3048

# ------------------------------------------------------------------------- ISA


def test_isa_sea_level():
    assert abs(atm.temperature(0.0) - 288.15) < 1e-9
    assert abs(atm.pressure(0.0) - 101325.0) < 1e-6
    assert abs(atm.density(0.0) - 1.225) < 1e-3
    assert abs(atm.speed_of_sound(0.0) - 340.29) < 0.05


def test_isa_1000m_table_values():
    # US Standard Atmosphere 1976: h=1000 m -> T=281.65 K, p=89874.6 Pa, rho=1.1117 kg/m^3
    assert abs(atm.temperature(1000.0) - 281.65) < 1e-9
    assert abs(atm.pressure(1000.0) - 89874.6) / 89874.6 < 2e-4
    assert abs(atm.density(1000.0) - 1.1117) / 1.1117 < 2e-4


def test_isa_tropopause():
    # h=11 km: T=216.65 K, p=22632 Pa
    assert abs(atm.temperature(11000.0) - 216.65) < 1e-6
    assert abs(atm.pressure(11000.0) - 22632.0) / 22632.0 < 1e-3


def test_isa_vectorized_and_monotonic():
    h = np.linspace(0.0, 5000.0, 200)
    t, p, rho = atm.isa(h)
    assert t.shape == p.shape == rho.shape == h.shape
    assert np.all(np.diff(p) < 0) and np.all(np.diff(rho) < 0) and np.all(np.diff(t) < 0)
    # isa() consistent with individual functions
    np.testing.assert_allclose(rho, p / (atm.R_AIR * t), rtol=1e-12)


def test_isa_slightly_negative_altitude_continuous():
    t, p, rho = atm.isa(-10.0)
    assert t > 288.15 and p > 101325.0 and rho > 1.225


# ------------------------------------------------- MIL-F-8785C parameter table


def test_mil8785c_low_altitude_table():
    h_m, w20 = 50.0, 9.0  # 164.04 ft AGL, 9 m/s wind at 20 ft
    sigma, length = dryden.mil8785c_low_altitude(h_m, w20)
    h_ft = h_m / FT
    denom = 0.177 + 0.000823 * h_ft
    assert abs(sigma[2] - 0.1 * w20) < 1e-12                      # sigma_w = 0.1 W20
    assert abs(sigma[0] - 0.1 * w20 / denom**0.4) < 1e-12          # sigma_u = sigma_v
    assert abs(sigma[0] - sigma[1]) < 1e-15
    assert abs(length[2] - h_m) < 1e-9                             # L_w = h
    assert abs(length[0] - (h_ft / denom**1.2) * FT) < 1e-9        # L_u = L_v
    assert abs(length[0] - length[1]) < 1e-12


def test_mil8785c_altitude_clamped_to_spec_band():
    sig_low, len_low = dryden.mil8785c_low_altitude(1.0, 5.0)     # below 10 ft -> clamp
    sig_10ft, len_10ft = dryden.mil8785c_low_altitude(10.0 * FT, 5.0)
    np.testing.assert_allclose(sig_low, sig_10ft, rtol=1e-12)
    np.testing.assert_allclose(len_low, len_10ft, rtol=1e-12)


# ----------------------------------------------------- discretization coefficients


def test_bilinear_coefficients_match_scipy():
    v_air, h_m, w20, dt = 25.0, 60.0, 8.0, 1.0 / 200.0
    sigma, length = dryden.mil8785c_low_altitude(h_m, w20)
    model = dryden.DrydenGusts(1, dt, v_air, h_m, w20, np.random.default_rng(0))

    # u channel: H(s) = sigma_u sqrt(2 L_u / (pi V)) / (1 + (L_u/V) s)
    tau = length[0] / v_air
    k = sigma[0] * np.sqrt(2.0 * length[0] / (np.pi * v_air))
    bz, az = signal.bilinear([k], [tau, 1.0], fs=1.0 / dt)
    np.testing.assert_allclose([model.b[0, 0, 0], model.b[0, 0, 1]], bz, rtol=1e-10)
    np.testing.assert_allclose([model.a[0, 0, 0]], az[1:], rtol=1e-10)
    assert model.b[0, 0, 2] == 0.0 and model.a[0, 0, 1] == 0.0

    # w channel: H(s) = sigma_w sqrt(L_w/(pi V)) (1 + sqrt(3)(L_w/V) s) / (1 + (L_w/V) s)^2
    tau = length[2] / v_air
    k = sigma[2] * np.sqrt(length[2] / (np.pi * v_air))
    bz, az = signal.bilinear([k * np.sqrt(3.0) * tau, k], [tau * tau, 2.0 * tau, 1.0],
                             fs=1.0 / dt)
    np.testing.assert_allclose(model.b[0, 2], bz, rtol=1e-10)
    np.testing.assert_allclose(model.a[0, 2], az[1:], rtol=1e-10)


# ------------------------------------------------------------- spectrum + variance


def _generate(model, steps):
    out = np.empty((steps, model.n, 3))
    for k in range(steps):
        out[k] = model.step()
    return out


def test_psd_matches_analytic_spectrum():
    """Welch PSD of generated gusts matches the MIL-F-8785C Dryden spectrum."""
    n, dt, v_air, h_m, w20 = 8, 1.0 / 100.0, 30.0, 50.0, 9.0
    model = dryden.DrydenGusts(n, dt, v_air, h_m, w20, np.random.default_rng(8785))
    series = _generate(model, 60_000)  # 600 s

    freqs, pxx = signal.welch(series, fs=1.0 / dt, nperseg=4096, axis=0)
    pxx = pxx.mean(axis=1)             # average across the 8 vehicles -> (nfreq, 3)
    omega = 2.0 * np.pi * freqs
    band = (omega > 0.5) & (omega < 20.0)
    measured = pxx[band] / (2.0 * np.pi)   # one-sided per-Hz -> one-sided per-(rad/s)
    analytic = model.analytic_psd(omega[band])  # (3, nband)

    ratio = measured / analytic.T
    geo_mean = np.exp(np.mean(np.log(ratio), axis=0))
    assert np.all(geo_mean > 0.85) and np.all(geo_mean < 1.15), geo_mean
    assert ratio.min() > 0.5 and ratio.max() < 1.9


def test_gust_variance_matches_sigma():
    n, dt, v_air, h_m, w20 = 8, 1.0 / 100.0, 30.0, 50.0, 9.0
    model = dryden.DrydenGusts(n, dt, v_air, h_m, w20, np.random.default_rng(1797))
    series = _generate(model, 60_000)
    var = series.var(axis=0).mean(axis=0)          # (3,) averaged over vehicles
    sigma2 = model.sigma[0] ** 2
    # w channel has the shortest correlation time -> tightest estimate
    assert abs(var[2] - sigma2[2]) / sigma2[2] < 0.15
    assert abs(var[0] - sigma2[0]) / sigma2[0] < 0.30
    assert abs(var[1] - sigma2[1]) / sigma2[1] < 0.30
    assert abs(series.mean()) < 0.05 * np.sqrt(sigma2.max())


def test_determinism_and_stream_independence():
    args = (4, 0.01, 25.0, 40.0, 7.0)
    a = _generate(dryden.DrydenGusts(*args, np.random.default_rng(7)), 500)
    b = _generate(dryden.DrydenGusts(*args, np.random.default_rng(7)), 500)
    c = _generate(dryden.DrydenGusts(*args, np.random.default_rng(8)), 500)
    np.testing.assert_array_equal(a, b)
    assert np.abs(a - c).max() > 0.0


def test_zero_wind_zero_gusts():
    model = dryden.DrydenGusts(3, 0.01, 20.0, 50.0, 0.0, np.random.default_rng(1))
    series = _generate(model, 100)
    np.testing.assert_array_equal(series, 0.0)


def test_per_vehicle_altitude_broadcast():
    alts = np.array([20.0, 60.0, 200.0])
    model = dryden.DrydenGusts(3, 0.01, 25.0, alts, 6.0, np.random.default_rng(2))
    assert model.sigma.shape == (3, 3) and model.length.shape == (3, 3)
    # higher altitude -> longer scale lengths, sigma_u decreasing toward sigma_w
    assert model.length[2, 0] > model.length[0, 0]
    assert model.sigma[0, 0] > model.sigma[2, 0]
    out = model.step()
    assert out.shape == (3, 3)
