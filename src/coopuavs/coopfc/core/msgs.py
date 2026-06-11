"""Typed intra-FCU messages (the TopicStore payloads).

Plain NamedTuples of floats/tuples — allocation-light, immutable,
numpy-free. Stamps are FCU-clock seconds (scheduler-derived, boot = 0).
Conventions: body FLU, world ENU, SI units throughout.
"""

from __future__ import annotations

from typing import NamedTuple

Vec3 = tuple[float, float, float]


class ImuSample(NamedTuple):
    stamp: float        # driver tick time, s
    gyro: Vec3          # rad/s, body FLU
    accel: Vec3         # m/s^2 specific force, body FLU


class GpsMsg(NamedTuple):
    stamp: float        # delivery time, s
    fix_stamp: float    # measurement time, s (delivery - latency; OOSM key)
    pos: Vec3           # m, world ENU
    vel: Vec3           # m/s, world ENU
    fix_type: int       # u-blox convention: 0 none / 2 2D / 3 3D


class BaroMsg(NamedTuple):
    stamp: float
    pressure_pa: float
    alt_m: float        # ISA pressure altitude (driver-converted)


class MagMsg(NamedTuple):
    stamp: float
    field_ut: Vec3      # uT, body FLU


class EscMsg(NamedTuple):
    stamp: float
    rpm: tuple          # mechanical shaft rpm per rotor (device units)
    omega: tuple        # rad/s per rotor (driver-converted)
    v_bus: float        # V
    i_bus: float        # A
