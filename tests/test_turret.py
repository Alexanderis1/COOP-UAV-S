"""Ground turret: clearance interlock, burst adjudication, stray rounds."""

import numpy as np

from coopuavs.core.messages import (
    EngagementDecision,
    FireClearance,
    Header,
    ThreatClass,
    Track,
    TrackArray,
    ZoneClass,
)
from coopuavs.sim.adjudicator import EngagementAdjudicator
from coopuavs.sim.environment import Environment
from coopuavs.sim.turret import GroundTurret
from coopuavs.sim.world import World
from coopuavs.threats.enemy_drone import EnemyDrone


def make_battlefield():
    env = Environment.from_config({
        "bounds": [-3000.0, -3000.0, 3000.0, 3000.0],
        "default_zone": "SAFE",
        "zones": [{"rect": [400.0, -200.0, 1200.0, 200.0], "class": "DANGEROUS"}],
    })
    world = World(env, dt=0.05, seed=2)
    turret = GroundTurret("t1", world, np.array([0.0, 0.0, 5.0]),
                          dispersion_mrad=6.0, min_pk=0.0)
    world.turrets["t1"] = turret
    world.add_node(turret)
    world.add_node(EngagementAdjudicator(world, {}, {"t1": turret}))
    # A slow hostile hovering 500 m up-range at altitude.
    world.schedule_enemy(0.0, lambda: EnemyDrone(
        "owa-1", ThreatClass.OWA_STRATEGIC,
        np.array([500.0, 0.0, 300.0]), np.array([2900.0, 0.0, 0.0]),
        world.rng, world=world,
    ))
    world.step()                                   # spawn the enemy
    return world, turret


def publish_truth_track(world):
    enemy = world.enemies["owa-1"]
    world.bus.publish("tracks", TrackArray(
        header=Header(stamp=world.t),
        tracks=[Track(header=Header(stamp=world.t), track_id=1,
                      position=enemy.position.copy(),
                      velocity=enemy.velocity.copy(), p_decoy=0.0)],
    ))


def test_turret_holds_fire_without_clearance_then_fires_and_strays():
    world, turret = make_battlefield()
    requests, fires = [], []
    world.bus.subscribe("engagement/fire_request", requests.append)
    world.bus.subscribe("engagement/fire", fires.append)

    # Phase 1: tracks flowing, no clearance — the interlock must hold.
    for _ in range(100):                       # 5 s
        publish_truth_track(world)
        world.step()
    assert len(requests) >= 1                  # it asked for release authority
    assert requests[0].uav_id == "t1"
    assert fires == []                         # PHY-TUR-001: no token, no shot
    assert turret.state in ("tracking", "slewing")
    assert turret.magazine == 300

    # Phase 2: C2 grants release — bursts fly and rounds are accounted for.
    for _ in range(100):
        publish_truth_track(world)
        world.bus.publish("engagement/clearance", FireClearance(
            header=Header(stamp=world.t), task_id=0, uav_id="t1",
            decision=EngagementDecision.AUTHORIZED,
        ))
        world.step()
        if world.stray_impacts:
            break
    assert len(fires) >= 1
    assert turret.magazine < 300
    assert len(world.stray_impacts) >= 1       # SIM-EFF-003: misses land somewhere
    for stray in world.stray_impacts:
        assert isinstance(stray["zone"], ZoneClass)
        assert stray["shooter"] == "t1"
    # Stray rounds continue along the fire line BEYOND the target.
    enemy_x = world.enemies["owa-1"].position[0]
    assert all(s["pos"][0] > enemy_x * 0.5 for s in world.stray_impacts)


def test_denied_clearance_blacklists_the_track():
    world, turret = make_battlefield()
    fires = []
    world.bus.subscribe("engagement/fire", fires.append)
    requested = []
    world.bus.subscribe("engagement/fire_request", requested.append)

    for _ in range(60):
        publish_truth_track(world)
        if requested:
            world.bus.publish("engagement/clearance", FireClearance(
                header=Header(stamp=world.t), task_id=0, uav_id="t1",
                decision=EngagementDecision.DENIED,
            ))
        world.step()
    assert fires == []
    assert turret.target_track is None
    assert 1 in turret._denied
