"""Comms simulation (SIM-COM-001/002/003): latency, range-dependent loss,
seeded determinism, jam events and the clearance interlock under loss."""

import copy

import numpy as np

from coopuavs.core.comms import CommsModel
from coopuavs.core.messages import (
    EngagementDecision,
    EngagementTask,
    FireClearance,
    Header,
    Track,
    TrackArray,
)
from coopuavs.interceptors.uav import CLEARANCE_TIMEOUT_S
from coopuavs.sim import scenario as scenario_mod
from coopuavs.sim.environment import Environment
from coopuavs.sim.world import World


class FakePlatform:
    def __init__(self, position):
        self.position = np.asarray(position, dtype=float)


def make_world(seed=2, **comms_cfg):
    env = Environment.from_config({
        "bounds": [-8000.0, -8000.0, 8000.0, 8000.0],
        "default_zone": "SAFE",
    })
    world = World(env, dt=0.05, seed=seed)
    comms = CommsModel(world, **comms_cfg)
    return world, comms


# -- latency ------------------------------------------------------------------


def test_latency_delays_delivery_until_drained():
    world, comms = make_world(latency_s=0.2)
    comms.register_endpoint("u1", FakePlatform([100.0, 0.0, 200.0]))
    got = []
    world.bus.subscribe("uav/command", got.append, endpoint="u1")

    world.bus.publish("uav/command", "rtb")          # ground -> u1 at t=0
    assert got == []                                  # not synchronous any more
    for _ in range(4):                                # drains at t=0 .. 0.15
        world.step()
    assert got == []                                  # still in flight
    world.step()                                      # drain at t=0.20
    assert got == ["rtb"]


def test_unrouted_topics_and_ground_links_stay_synchronous():
    world, comms = make_world(latency_s=5.0)
    got_ground, got_fire = [], []
    world.bus.subscribe("tracks", got_ground.append)             # ground endpoint
    world.bus.subscribe("engagement/fire", got_fire.append)      # not a radio topic
    world.bus.publish("tracks", "picture")
    world.bus.publish("engagement/fire", "release", sender="u1")
    assert got_ground == ["picture"] and got_fire == ["release"]


# -- loss vs range and determinism (SIM-COM-001, SIM-003) -------------------------


def run_loss_trial(seed, range_m):
    world, comms = make_world(seed=seed, latency_s=0.01, jitter_s=0.005,
                              base_loss=0.05, loss_per_km=0.1)
    comms.register_endpoint("u1", FakePlatform([range_m, 0.0, 300.0]))
    got = []
    world.bus.subscribe("engagement/tasks", got.append, endpoint="u1")
    arrivals = []
    for i in range(200):
        world.bus.publish("engagement/tasks", i)
        world.step()
        arrivals.append(list(got))
    return arrivals, len(got)


def test_loss_rises_with_range_from_base():
    _, near = run_loss_trial(seed=5, range_m=500.0)      # ~10% loss
    _, far = run_loss_trial(seed=5, range_m=7000.0)      # ~75% loss
    assert near > far
    assert near < 200                                     # some loss even near
    assert far > 0                                        # link degraded, not dead


def test_comms_are_deterministic_for_a_seed():
    a, _ = run_loss_trial(seed=9, range_m=3000.0)
    b, _ = run_loss_trial(seed=9, range_m=3000.0)
    assert a == b                                         # same drops, same timing
    c, _ = run_loss_trial(seed=10, range_m=3000.0)
    assert a != c                                         # the seed is the stream


# -- jam events (SIM-COM-002) ---------------------------------------------------------


def test_jam_event_kills_link_inside_area_and_window():
    world, comms = make_world(
        latency_s=0.01,
        jam=[{"t_start": 5.0, "t_end": 10.0, "area_center": [1000.0, 0.0],
              "area_radius": 800.0, "loss": 1.0}],
    )
    comms.register_endpoint("in", FakePlatform([1000.0, 200.0, 150.0]))
    comms.register_endpoint("out", FakePlatform([4000.0, 0.0, 150.0]))

    assert comms.link_loss("in", t=2.0) == 0.0            # before the window
    assert comms.link_loss("in", t=7.0) == 1.0            # jammed
    assert comms.link_loss("out", t=7.0) == 0.0           # outside the area
    assert comms.link_loss("in", t=11.0) == 0.0           # after the window
    assert comms.link_quality("in", t=7.0) == 0.0


JAM_SCENARIO = {
    "name": "jam",
    "seed": 11,
    "dt": 0.05,
    "duration": 12.0,
    "record_hz": 5.0,
    "environment": {
        "bounds": [-4000.0, -4000.0, 4000.0, 4000.0],
        "default_zone": "SAFE",
        "assets": [{"name": "substation", "position": [0.0, 0.0, 0.0]}],
    },
    "comms": {
        "base_pos": [0.0, -1000.0, 5.0],
        "jam": [{"t_start": 2.0, "t_end": 6.0, "area_center": [-200.0, -1000.0],
                 "area_radius": 2000.0, "loss": 1.0}],
    },
    "sensors": [],
    "interceptors": [
        {"id": "u1", "home": [-200.0, -1000.0, 0.0], "effector": "projectile"},
    ],
    "threats": [],
}


def test_jam_degrades_link_quality_in_uav_telemetry_and_frames():
    sc = scenario_mod.build(copy.deepcopy(JAM_SCENARIO))
    sc.world.run(12.0, stop_when_clear=False)
    uav = sc.uavs["u1"]
    assert uav.link_quality == 1.0                        # window over, recovered

    frames = sc.recorder.frames
    by_t = {f["t"]: f for f in frames if f["uavs"]}
    # During the jam, telemetry is lost: the last received state predates it.
    # Just after it ends, telemetry resumes carrying the degraded quality.
    after = [f for f in frames
             if 6.0 < f["t"] < 9.0 and f["uavs"] and f["uavs"][0]["link"] < 0.9]
    assert after, "post-jam frames must show degraded link quality"
    # By the end of the run the sliding window has recovered.
    assert frames[-1]["uavs"][0]["link"] > 0.9
    assert by_t                                            # sanity: states flowed


# -- clearance interlock under loss (SIM-COM-003) -----------------------------------


def make_engagement_world():
    """One armed UAV with a task and a fat slow target in envelope; no C2 —
    the test plays the ground segment by hand."""
    env = Environment.from_config({
        "bounds": [-3000.0, -3000.0, 3000.0, 3000.0],
        "default_zone": "SAFE",
    })
    world = World(env, dt=0.05, seed=4)
    comms = CommsModel(world, latency_s=0.01)
    from coopuavs.interceptors.effectors import projectile_gun
    from coopuavs.interceptors.uav import InterceptorUav

    uav = InterceptorUav("u1", world.bus, home=np.array([0.0, 0.0, 100.0]),
                         effector=projectile_gun())
    uav.body.position = np.array([0.0, 0.0, 100.0])
    comms.register_endpoint("u1", uav)
    world.friendlies["u1"] = uav
    world.add_node(uav)
    return world, uav


def drive(world, requests, target_pos, n_steps):
    track = Track(header=Header(stamp=world.t), track_id=1,
                  position=np.asarray(target_pos, dtype=float),
                  velocity=np.array([3.0, 0.0, 0.0]))
    task = EngagementTask(header=Header(stamp=world.t), task_id=1, track_id=1,
                          shooter_id="u1")
    for _ in range(n_steps):
        track.header = Header(stamp=world.t)
        world.bus.publish("tracks", TrackArray(header=Header(stamp=world.t),
                                               tracks=[track]))
        world.bus.publish("engagement/tasks", [task])
        world.step()


def test_lost_clearance_holds_fire_and_rerequests():
    world, uav = make_engagement_world()
    requests, fires = [], []
    world.bus.subscribe("engagement/fire_request", requests.append)
    world.bus.subscribe("engagement/fire", fires.append)

    # 8 s with the clearance channel silent (token "lost"): the interlock
    # must hold fire and re-request after CLEARANCE_TIMEOUT_S.
    drive(world, requests, [120.0, 0.0, 100.0], n_steps=160)
    assert len(requests) >= 2                              # re-asked, not deadlocked
    gap = requests[1].header.stamp - requests[0].header.stamp
    assert gap >= CLEARANCE_TIMEOUT_S - 0.2
    assert fires == []                                     # PHY-UAV-021: no token, no shot


def test_late_clearance_against_collapsed_geometry_aborts():
    world, uav = make_engagement_world()
    fires = []
    world.bus.subscribe("engagement/fire", fires.append)
    drive(world, [], [120.0, 0.0, 100.0], n_steps=40)      # request goes out

    # The token arrives very late; meanwhile the target left the envelope.
    uav._tracks[1].position = np.array([2500.0, 0.0, 400.0])
    world.bus.publish("engagement/clearance", FireClearance(
        header=Header(stamp=world.t), task_id=1, uav_id="u1", track_id=1,
        decision=EngagementDecision.AUTHORIZED,
    ))
    for _ in range(10):
        world.step()
    assert fires == []                                     # Pk re-check rejected it
    assert uav.effector.ammo == 8


def test_valid_clearance_still_releases():
    world, uav = make_engagement_world()
    fires, requests = [], []
    world.bus.subscribe("engagement/fire", fires.append)
    world.bus.subscribe("engagement/fire_request", requests.append)
    track_pos = [120.0, 0.0, 100.0]
    drive(world, requests, track_pos, n_steps=40)
    assert requests
    world.bus.publish("engagement/clearance", FireClearance(
        header=Header(stamp=world.t), task_id=1, uav_id="u1", track_id=1,
        decision=EngagementDecision.AUTHORIZED,
    ))
    drive(world, requests, track_pos, n_steps=20)
    assert len(fires) == 1                                 # token honoured
