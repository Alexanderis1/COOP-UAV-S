"""International Standard Atmosphere (ISA), troposphere layer (h <= 11 km).

Equations [US Standard Atmosphere 1976 / ICAO ISA; see docs/RESEARCH.md]:

    T(h)   = T0 - L h
    p(h)   = p0 (1 - L h / T0)^(g0 / (R L))
    rho(h) = p / (R T)
    a(h)   = sqrt(gamma R T)

Altitude is geometric metres above mean sea level (the <= 2 km sim envelope
makes the geometric/geopotential distinction negligible). Valid for
h <= 11 000 m; higher altitudes raise ValueError rather than silently
extrapolating into the (isothermal) stratosphere, and non-finite altitudes
(NaN/inf) raise too rather than silently propagating NaN.
"""

from __future__ import annotations

import numpy as np

ISA_T0 = 288.15          # K, sea-level standard temperature
ISA_P0 = 101325.0        # Pa, sea-level standard pressure
ISA_LAPSE = 0.0065       # K/m, tropospheric lapse rate
R_AIR = 287.05287        # J/(kg K), specific gas constant of dry air
GAMMA_AIR = 1.4          # ratio of specific heats
G0 = 9.80665             # m/s^2, standard gravity (ISA definition)

_TROPOPAUSE_M = 11_000.0
_P_EXP = G0 / (R_AIR * ISA_LAPSE)


def _check(alt_m: np.ndarray) -> np.ndarray:
    alt = np.asarray(alt_m, dtype=float)
    # NaN compares False to every bound, so a plain `any(alt > max)` would
    # let NaN slip through and silently propagate NaN T/p/rho downstream.
    if not np.all(np.isfinite(alt)) or np.any(alt > _TROPOPAUSE_M):
        raise ValueError(
            f"ISA troposphere model valid only up to {_TROPOPAUSE_M} m "
            "and requires finite altitude")
    return alt


def temperature(alt_m) -> np.ndarray:
    """Static air temperature (K)."""
    return ISA_T0 - ISA_LAPSE * _check(alt_m)


def pressure(alt_m) -> np.ndarray:
    """Static pressure (Pa)."""
    alt = _check(alt_m)
    return ISA_P0 * (1.0 - ISA_LAPSE * alt / ISA_T0) ** _P_EXP


def density(alt_m) -> np.ndarray:
    """Air density (kg/m^3) from the ideal gas law."""
    return pressure(alt_m) / (R_AIR * temperature(alt_m))


def speed_of_sound(alt_m) -> np.ndarray:
    """Speed of sound (m/s)."""
    return np.sqrt(GAMMA_AIR * R_AIR * temperature(alt_m))


def isa(alt_m) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(temperature K, pressure Pa, density kg/m^3) in one pass."""
    t = temperature(alt_m)
    p = ISA_P0 * (t / ISA_T0) ** _P_EXP
    return t, p, p / (R_AIR * t)
