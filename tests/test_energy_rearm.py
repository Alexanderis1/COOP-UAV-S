"""Energy model and rearm cycle (SIM-PHX-002)."""

import numpy as np

from coopuavs.core.bus import MessageBus
from coopuavs.core.messages import UavMode
from coopuavs.interceptors.effectors import projectile_gun
from coopuavs.interceptors.uav import InterceptorUav


def make_uav(**kw) -> InterceptorUav:
    return InterceptorUav(
        uav_id="u1", bus=MessageBus(), home=np.array([0.0, 0.0, 100.0]),
        effector=projectile_gun(), max_speed=80.0, **kw,
    )


def test_drain_grows_quadratically_above_cruise():
    uav = make_uav()
    base = uav._drain_rate()                       # parked: baseline drain
    uav.body.velocity = np.array([uav.cruise_speed, 0.0, 0.0])
    at_cruise = uav._drain_rate()
    uav.body.velocity = np.array([80.0, 0.0, 0.0])
    at_dash = uav._drain_rate()
    assert base == at_cruise                       # no penalty up to cruise
    assert at_dash > 1.5 * at_cruise               # strong dash penalty


def test_low_battery_rtb_then_rearm_returns_to_available():
    uav = make_uav(turnaround_s=10.0)
    uav.battery = 0.10
    uav.effector.ammo = 1
    uav.body.position = np.array([600.0, 0.0, 100.0])

    t, dt = 0.0, 0.1
    modes = set()
    for _ in range(600):                            # 60 s of sim time
        uav.update(t, dt)
        modes.add(uav.mode)
        if uav.mode == UavMode.IDLE:
            break
        t += dt

    assert UavMode.RTB in modes                     # flew home on low battery
    assert UavMode.REARM in modes                   # turned around on the pad
    assert uav.mode == UavMode.IDLE                 # available again
    assert uav.battery > 0.95                       # recharged (minus idle hover)
    assert uav.effector.ammo == uav._ammo_capacity  # magazine restored


def test_rearm_takes_the_configured_turnaround():
    uav = make_uav(turnaround_s=20.0)
    uav.battery = 0.10                              # already on the pad
    t, dt = 0.0, 0.1
    rearm_started = None
    while t < 60.0:
        uav.update(t, dt)
        if uav.mode == UavMode.REARM and rearm_started is None:
            rearm_started = t
        if uav.mode == UavMode.IDLE and rearm_started is not None:
            break
        t += dt
    assert rearm_started is not None
    assert t - rearm_started >= 20.0 - dt
