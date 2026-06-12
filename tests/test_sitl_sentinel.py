"""P4-5 sitl twin of test_sentinel: the patrol stack on a VirtualMCU.

The sentinel flies its orbit through the full stack (EKF estimates over
the coop-link, FCU envelope, real plant) and its mounted EO/RF payload
rides the FriendlyVehicle TRUTH adapter — the airborne look the ground
towers cannot make still builds the fused picture. Twin floors are its
own (plan P4-6 rule): the sitl annulus tolerance carries nav error +
transport lag + real turn dynamics on top of the legacy 160 m band.
"""

from __future__ import annotations

import copy

import numpy as np

from coopuavs.core.messages import UavMode
from coopuavs.core.rng import RngRegistry
from coopuavs.interceptors.sentinel import SitlShellSentinel
from coopuavs.mc.fcu_client import FcuClient
from coopuavs.mc.sentinel_app import SentinelApp
from coopuavs.sil.fleet import SitlEngine
from coopuavs.sil.host import VirtualMCU
from coopuavs.sil.vehicle import FriendlyVehicle
from coopuavs.sim import scenario as scenario_mod

DT = 0.05
FCU_OVERLAY = {"fcu.vel_max_h": 30.0, "fcu.vel_max_up": 10.0,
               "fcu.vel_max_down": 10.0}

SITL_SENTINEL_SCENARIO = {
    "name": "sitl-sentinel",
    "seed": 5,
    "dt": 0.05,
    "duration": 90.0,
    "fidelity": {"fleet": "sitl"},
    "sitl": {"fcu": FCU_OVERLAY},
    "environment": {
        "bounds": [-4000.0, -4000.0, 4000.0, 4000.0],
        "cell_size": 100.0,
        "default_zone": "SAFE",
        "assets": [{"name": "hq", "position": [0.0, 0.0, 0.0]}],
        "charging_stations": [
            {"id": "cs-1", "pos": [0.0, -400.0, 0.0], "rooftop": False},
        ],
    },
    # only a short-range ground EO sensor: the sentinel is the long eye
    "sensors": [
        {"type": "eo_ir", "name": "eo-base", "position": [0.0, -200.0, 20.0],
         "max_range": 600.0},
    ],
    "seekers": False,
    "interceptors": [],
    "sentinels": [
        {"id": "sent-1", "station": "cs-1",
         "orbit": {"center": [0.0, 600.0], "radius": 300.0, "alt": 120.0,
                   "speed": 20.0}},
    ],
    "threats": [
        # holds far north — invisible to the 600 m ground sensor
        {"time": 1.0, "class": "OWA_STRATEGIC",
         "spawn": [-3500.0, 2200.0, 800.0], "target": [3500.0, 2200.0, 0.0]},
    ],
}


def test_sitl_sentinel_orbit_and_truth_mounted_payload():
    sc = scenario_mod.build(copy.deepcopy(SITL_SENTINEL_SCENARIO))
    world = sc.world
    shell = sc.sentinels["sent-1"]
    assert isinstance(shell, SitlShellSentinel)
    fv = world.friendlies["sent-1"]
    assert isinstance(fv, FriendlyVehicle) and fv.tactical is shell
    # the payload mounts on TRUTH, not the estimate body
    eo = next(n for n in world.nodes if n.name == "eo-sent-1")
    assert eo._platform is fv
    # and the platform flies the endurance airframe class (P4-R(2))
    gi = world.micro._group_of[world.micro.index["sent-1"]]
    assert world.micro.groups[gi].airframe == "sentinel_quad"

    states, tracks = [], []
    world.bus.subscribe("uav/state", lambda m: states.append(m)
                        if m.uav_id == "sent-1" else None)
    world.bus.subscribe("tracks", lambda m: tracks.append(m))
    world.run(90.0, stop_when_clear=False)

    # reached the orbit annulus (TRUTH) and patrols it
    r = float(np.linalg.norm(fv.position[:2] - np.array([0.0, 600.0])))
    assert abs(r - 300.0) < 170.0, f"truth orbit radius error {r:.0f} m"
    assert abs(fv.position[2] - 120.0) < 60.0
    assert shell.mode == UavMode.PATROL
    # telemetry rides the wire: estimate positions, sentinel identity
    assert states and states[-1].kind == "sentinel" and states[-1].ammo == 0
    est = states[-1].position
    assert 0.001 < float(np.linalg.norm(est - fv.position)) < 15.0
    # only the airborne payload can have built the picture
    assert any(m.tracks for m in tracks), \
        "sentinel detections never fused into a track"
    assert not shell.mc_crashed


def test_sitl_sentinel_low_battery_breaks_off():
    """The shared land-dock cycle (P4-4) applies to sentinels: a drained
    pack sends the platform home off its orbit."""
    eng = SitlEngine([("s1", (0.0, 0.0, 0.0))], RngRegistry(8),
                     world_dt=DT, heartbeat_hz=0.0, fcu_overlay=FCU_OVERLAY)
    up, down = eng.attach_link("s1")
    client = FcuClient(up, down)

    def factory(clock, rng, ports):
        return SentinelApp(clock, rng, ports, uav_id="s1",
                           home=np.zeros(3),
                           orbit={"center": [0.0, 300.0], "radius": 200.0,
                                  "alt": 80.0, "speed": 15.0},
                           fcu_client=client, max_speed=30.0,
                           turnaround_s=10.0)

    mcu = VirtualMCU("mc/s1", tick_hz=10, base_hz=800,
                     app_factory=factory, rng=None)
    eng.attach_mc("s1", mcu)
    eng.set_pad("s1", (0.0, 0.0, 0.0), recharge_s=10.0)

    t = 0.0
    for _ in range(round(12.0 / DT)):                  # boot + transit out
        eng.run_macro_step(t, DT)
        t += DT
    assert mcu.app.mode in (UavMode.TRANSIT, UavMode.PATROL)

    eng.pt.battery.soc[0] = 0.03                       # pack collapses
    for _ in range(round(6.0 / DT)):
        eng.run_macro_step(t, DT)
        t += DT
    assert eng.fcus[0].failsafe in ("BATT_LOW", "BATT_CRIT")
    assert mcu.app.mode in (UavMode.RTB, UavMode.REARM)
    assert not mcu.crashed
