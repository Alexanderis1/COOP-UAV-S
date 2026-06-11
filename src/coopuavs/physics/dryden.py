"""MIL-F-8785C Dryden continuous-turbulence model, low-altitude form, batched.

Gust components [u_g, v_g, w_g] are along the vehicle's longitudinal / lateral
(left) / vertical (up) axes in m/s (zero mean, so the FLU sign convention does
not alter the statistics). Forming filters driven by white noise
[MIL-F-8785C section 3.7.2; Beard & McLain 2012 section 4.4]:

    H_u(s) = sigma_u sqrt(2 L_u / (pi V)) *  1 / (1 + (L_u/V) s)
    H_v(s) = sigma_v sqrt(L_v / (pi V)) * (1 + sqrt(3)(L_v/V) s) / (1 + (L_v/V) s)^2
    H_w(s) = sigma_w sqrt(L_w / (pi V)) * (1 + sqrt(3)(L_w/V) s) / (1 + (L_w/V) s)^2

yielding one-sided PSDs (rad/s) that integrate to sigma^2:

    Phi_u(w) = sigma_u^2 (2 L_u / (pi V)) *  1 / (1 + (L_u w / V)^2)
    Phi_vw(w) = sigma^2 (L / (pi V)) * (1 + 3 (L w / V)^2) / (1 + (L w / V)^2)^2

Low-altitude (h < 1000 ft) parameters, spec units feet (converted here):

    L_w = h,  L_u = L_v = h / (0.177 + 0.000823 h)^1.2
    sigma_w = 0.1 W20,  sigma_u = sigma_v = sigma_w / (0.177 + 0.000823 h)^0.4

Discretization: closed-form Tustin (bilinear) transform of each forming
filter at the supplied micro-step rate (validated against
scipy.signal.bilinear in tests), stepped as direct-form II transposed,
vectorized over vehicles and channels. The driving noise is i.i.d.
N(0, pi/dt) per (vehicle, channel, step), which has unit one-sided PSD in
rad/s up to Nyquist, so the discrete output PSD equals Phi(w) directly.
Filter parameters (V, h, W20) are frozen at construction (representative
flight condition); re-tuning in flight is a later, explicitly-tested change.
"""

from __future__ import annotations

import numpy as np

_FT = 0.3048
_H_MIN_FT = 10.0     # spec low-altitude band: 10..1000 ft AGL
_H_MAX_FT = 1000.0


def mil8785c_low_altitude(altitude_m, wind20_ms) -> tuple[np.ndarray, np.ndarray]:
    """MIL-F-8785C low-altitude turbulence intensities and scale lengths (SI).

    Returns (sigma, length) with channel order [u, v, w] on the last axis;
    altitude clamped to the spec's 10..1000 ft validity band.
    """
    h_ft = np.clip(np.asarray(altitude_m, dtype=float) / _FT, _H_MIN_FT, _H_MAX_FT)
    w20 = np.asarray(wind20_ms, dtype=float)
    denom = 0.177 + 0.000823 * h_ft
    sigma_w = 0.1 * w20
    sigma_u = sigma_w / denom**0.4
    l_w = h_ft * _FT
    l_u = (h_ft / denom**1.2) * _FT
    sigma = np.stack(np.broadcast_arrays(sigma_u, sigma_u, sigma_w), axis=-1)
    length = np.stack(np.broadcast_arrays(l_u, l_u, l_w), axis=-1)
    return sigma, length


class DrydenGusts:
    """Batched Dryden gust generator: ``step() -> (n, 3)`` m/s per micro-step.

    airspeed / altitude may be scalars or (n,) arrays; wind20_ms is the mean
    wind speed at 20 ft (the MIL-F-8785C severity knob); rng is a dedicated
    numpy Generator (named RNG stream, never shared).
    """

    def __init__(self, n: int, dt: float, airspeed_ms, altitude_m, wind20_ms,
                 rng: np.random.Generator):
        self.n = int(n)
        self.dt = float(dt)
        self.rng = rng
        v = np.broadcast_to(np.asarray(airspeed_ms, dtype=float), (self.n,))
        sigma, length = mil8785c_low_altitude(altitude_m, wind20_ms)
        self.sigma = np.broadcast_to(sigma, (self.n, 3)).copy()
        self.length = np.broadcast_to(length, (self.n, 3)).copy()
        self.airspeed = v.copy()

        c = 2.0 / self.dt                       # Tustin: s <- c (1 - z^-1)/(1 + z^-1)
        tau = self.length / v[:, None]          # (n, 3)
        b = np.zeros((self.n, 3, 3))            # numerator z^0, z^-1, z^-2
        a = np.zeros((self.n, 3, 2))            # denominator z^-1, z^-2 (a0 = 1)

        # u channel, first order: H(s) = k / (tau s + 1)
        k_u = self.sigma[:, 0] * np.sqrt(2.0 * self.length[:, 0] / (np.pi * v))
        tc = tau[:, 0] * c
        b[:, 0, 0] = k_u / (tc + 1.0)
        b[:, 0, 1] = b[:, 0, 0]
        a[:, 0, 0] = (1.0 - tc) / (1.0 + tc)

        # v, w channels, second order: H(s) = k (beta s + 1) / (tau s + 1)^2
        for ch in (1, 2):
            t_ = tau[:, ch]
            beta = np.sqrt(3.0) * t_
            k = self.sigma[:, ch] * np.sqrt(self.length[:, ch] / (np.pi * v))
            tc = t_ * c
            d0 = (tc + 1.0) ** 2
            b[:, ch, 0] = k * (beta * c + 1.0) / d0
            b[:, ch, 1] = 2.0 * k / d0
            b[:, ch, 2] = k * (1.0 - beta * c) / d0
            a[:, ch, 0] = 2.0 * (1.0 - tc * tc) / d0
            a[:, ch, 1] = (tc - 1.0) ** 2 / d0

        self.b = b
        self.a = a
        self._z1 = np.zeros((self.n, 3))
        self._z2 = np.zeros((self.n, 3))
        self._noise_std = np.sqrt(np.pi / self.dt)

    def step(self) -> np.ndarray:
        """Advance one micro-step; returns gust velocities (n, 3) [u, v, w] m/s."""
        x = self.rng.standard_normal((self.n, 3)) * self._noise_std
        y = self.b[:, :, 0] * x + self._z1
        self._z1 = self.b[:, :, 1] * x - self.a[:, :, 0] * y + self._z2
        self._z2 = self.b[:, :, 2] * x - self.a[:, :, 1] * y
        return y

    def analytic_psd(self, omega, vehicle: int = 0) -> np.ndarray:
        """One-sided Dryden PSD (rad/s) for the given vehicle, shape (3, len(omega))."""
        omega = np.asarray(omega, dtype=float)
        v = self.airspeed[vehicle]
        out = np.empty((3, omega.size))
        s_u, l_u = self.sigma[vehicle, 0], self.length[vehicle, 0]
        x = l_u * omega / v
        out[0] = s_u**2 * (2.0 * l_u / (np.pi * v)) / (1.0 + x * x)
        for ch in (1, 2):
            s, l_ = self.sigma[vehicle, ch], self.length[vehicle, ch]
            x = l_ * omega / v
            out[ch] = s**2 * (l_ / (np.pi * v)) * (1.0 + 3.0 * x * x) / (1.0 + x * x) ** 2
        return out
