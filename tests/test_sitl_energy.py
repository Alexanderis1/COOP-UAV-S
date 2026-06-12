"""P4-4 energy/telemetry rewire (user decisions 2026-06-12).

The MC's battery picture is the REAL ECM pack through FCU telemetry —
STATUS carries a voltage-proxy ``batt_frac`` (loaded v_cell mapped
crit→0 .. 4.20→1, conservative under sag) — replacing the legacy
synthetic drain model in sitl mode. The rearm cycle is physical:
RTB → LAND → touchdown+disarm on the pad → the engine's pad charger
refills the pack while docked → BATT_RESET (battery-swap semantics,
clears the FCU's upward-latched monitor, ground-only) → re-arm →
take off. Truth quarantine holds: the app reads telemetry, never the
engine pack.
"""

from __future__ import annotations

import numpy as np

from coopuavs.coopfc.battery_monitor import LOW, NORMAL, BatteryMonitor, BattParams
from coopuavs.coopfc.fcu import ARMED, OFFBOARD
from coopuavs.core.messages import UavMode
from coopuavs.core.rng import RngRegistry
from coopuavs.interceptors.effectors import projectile_gun
from coopuavs.mc.fcu_client import FcuClient
from coopuavs.mc.interceptor_app import InterceptorApp
from coopuavs.sil.fleet import SitlEngine
from coopuavs.sil.host import VirtualMCU

DT = 0.05


# ------------------------------------------------------------ voltage proxy

def test_battery_fraction_voltage_proxy():
    mon = BatteryMonitor(BattParams())
    assert mon.fraction() == 1.0                       # NaN before telemetry
    mon.update(0.0, 4.20 * 12)
    assert mon.fraction() == 1.0
    mon.update(0.1, 3.30 * 12)
    assert mon.fraction() == 0.0
    mon.update(0.2, 3.75 * 12)
    assert abs(mon.fraction() - 0.5) < 1e-12
    mon.update(0.3, 2.9 * 12)                          # below empty: clamped
    assert mon.fraction() == 0.0


def test_battery_reset_is_a_pack_swap():
    mon = BatteryMonitor(BattParams(debounce_s=0.5))
    mon.update(0.0, 3.40 * 12)
    mon.update(1.0, 3.40 * 12)                         # debounced below LOW
    assert mon.state == LOW
    mon.reset()
    assert mon.state == NORMAL and mon.fraction() == 1.0


def test_fcu_refuses_batt_reset_while_armed():
    """The in-flight sag latch must not be resettable from the air."""
    class _FakeArmed:
        pass
    from coopuavs.coopfc.fcu import Fcu
    from coopuavs.coopfc.hal import HalIO
    fcu = Fcu(HalIO())
    fcu.state = ARMED
    ok, why = fcu.cmd_batt_reset()
    assert not ok and "armed" in why
    fcu.state = "STANDBY"
    ok, _ = fcu.cmd_batt_reset()
    assert ok


# ------------------------------------------------------- full land-dock cycle

def _hosted(seed=21, turnaround_s=8.0):
    # Starts ON the pad (the scenario reality): arming home = pad datum,
    # first takeoff climbs to the MC loiter altitude.
    eng = SitlEngine([("u1", (0.0, 0.0, 0.0))], RngRegistry(seed),
                     world_dt=DT, heartbeat_hz=0.0,
                     fcu_overlay={"fcu.vel_max_h": 50.0,
                                  "fcu.vel_max_up": 20.0,
                                  "fcu.vel_max_down": 20.0})
    up, down = eng.attach_link("u1")
    client = FcuClient(up, down)

    def factory(clock, rng, ports):
        return InterceptorApp(clock, rng, ports, uav_id="u1",
                              home=np.array([0.0, 0.0, 0.0]),
                              effector=projectile_gun(), fcu_client=client,
                              max_speed=50.0, turnaround_s=turnaround_s)

    mcu = VirtualMCU("mc/u1", tick_hz=10, base_hz=800,
                     app_factory=factory, rng=None)
    eng.attach_mc("u1", mcu)
    eng.set_pad("u1", (0.0, 0.0, 0.0), recharge_s=turnaround_s)
    return eng, mcu, client


def _run(eng, t0, t_span):
    for m in range(round(t_span / DT)):
        eng.run_macro_step(t0 + m * DT, DT)
    return t0 + round(t_span / DT) * DT


def test_land_dock_recharge_rearm_cycle():
    """Drain the real pack mid-hover: the FCU battery failsafe leads the
    vehicle home, the app docks (LAND + hold_arm), the pad charger
    refills the ECM pack, BATT_RESET clears the latch, and the vehicle
    re-arms and lifts back to availability — telemetry-driven end to
    end."""
    eng, mcu, client = _hosted()
    app = mcu.app
    # boot + climb + settle at loiter hover: the voltage proxy reads the
    # LOADED pack, so it must be sampled at steady hover, not mid-climb
    t = _run(eng, 0.0, 14.0)
    fcu = eng.fcus[0]
    assert fcu.state == ARMED and fcu.mode == OFFBOARD
    assert app.battery > 0.5                           # healthy pack at hover

    # the mission ran long: pack down to sag-into-LOW territory
    eng.pt.battery.soc[0] = 0.03
    t = _run(eng, t, 3.0)
    assert fcu.failsafe in ("BATT_LOW", "BATT_CRIT")
    assert app.mode in (UavMode.RTB, UavMode.REARM)    # follows the failsafe home

    # the FCU lands it at the pad datum (LAND descends from the loiter
    # altitude at land_speed); docked = disarmed + frozen
    deadline = t + 15.0
    while t < deadline and not (fcu.touchdown and fcu.state == "STANDBY"):
        t = _run(eng, t, 0.5)
    assert fcu.touchdown and fcu.state == "STANDBY", \
        f"not docked by t={t:.0f}: {fcu.state}/{fcu.mode}"
    z_docked = eng.state[0, 2]
    assert abs(z_docked) < 2.0                         # ON the pad, not midair
    # ground recalibration: touchdown dropped the EKF for re-alignment
    # (the stand-stop is unobservable to the IMU; gates would lock out)
    t = _run(eng, t, 3.0)
    assert eng.pt.battery.soc[0] > 0.2                 # charger is filling it

    # turnaround completes: pack swapped (monitor reset), re-armed, full
    # magazine, climbing back out — telemetry-driven end to end. The
    # first climb-out may sag the (voltage-only, P5 scope) monitor into
    # CRIT once more — the FCU protects, tops up and retries, so the
    # pin is recovery within a bounded window, plus a bounded climb-out
    # (the P4 brake fix: slewed attitude setpoints + braking-aware
    # approach — pre-fix this ran away past 90 m).
    z_max = 0.0
    deadline = t + 22.0
    while t < deadline:
        t = _run(eng, t, 0.5)
        z_max = max(z_max, eng.state[0, 2])
        if (fcu.state == ARMED and fcu.mode == OFFBOARD
                and app.mode is not UavMode.REARM
                and eng.state[0, 2] > z_docked + 1.0):
            break
    else:
        raise AssertionError(
            f"not back in service by t={t:.0f}: {fcu.state}/{fcu.mode} "
            f"app={app.mode} z={eng.state[0, 2]:.1f}")
    assert fcu.batt.state == NORMAL
    # The turnaround timer starts at REARM entry (mid-air, the legacy
    # semantics), so landing time eats into the dock window: the vehicle
    # legitimately relaunches on a partial top-up. Recovered ≫ drained
    # (0.03) is the physical claim; the monitor gates flight-worthiness.
    assert eng.pt.battery.soc[0] > 0.5
    assert app.effector.ammo == app._ammo_capacity
    assert z_max < 30.0, f"climb-out ran to {z_max:.0f} m (loiter 15)"


def test_battery_reads_telemetry_not_truth():
    """Truth quarantine: the app's battery is exactly the wire value."""
    eng, mcu, client = _hosted()
    _run(eng, 0.0, 14.0)                # settle at loiter hover first
    assert mcu.app.battery == client.batt_frac
    assert client.status is not None
    # and the wire value is the FCU's (10 Hz STATUS-sampled) fraction,
    # not the engine SOC — hover ripple allows a small sampling skew
    assert abs(client.batt_frac - eng.fcus[0].batt.fraction()) < 0.3
    assert client.batt_frac != eng.pt.battery.soc[0]
