"""Blocked-fire adjudication (SIM-EFF-006): a building in the sight line
means no Pk roll — the event is ``fire_blocked_los``, the target survives,
and a turret's rounds were still fired and still land somewhere."""

import numpy as np

from coopuavs.core.messages import (
    EffectorType,
    FireRequest,
    Header,
    ThreatClass,
)
from coopuavs.interceptors.effectors import projectile_gun
from coopuavs.interceptors.uav import InterceptorUav
from coopuavs.sim.adjudicator import EngagementAdjudicator
from coopuavs.sim.environment import Environment
from coopuavs.sim.turret import GroundTurret
from coopuavs.sim.world import World
from coopuavs.threats.enemy_drone import EnemyDrone

ENV_CFG = {
    "bounds": [-1000.0, -1000.0, 1000.0, 1000.0],
    "default_zone": "SAFE",
    "buildings": [{"rect": [-50.0, -50.0, 50.0, 50.0], "height": 30.0,
                   "kind": "residential_high"}],
}


def make_world() -> World:
    world = World(Environment.from_config(ENV_CFG), dt=0.05, seed=2)
    return world


def add_enemy(world: World, pos) -> EnemyDrone:
    enemy = EnemyDrone("owa-1", ThreatClass.OWA_STRATEGIC,
                       np.array(pos, dtype=float), np.zeros(3),
                       world.rng, world=world)
    world.enemies["owa-1"] = enemy
    return enemy


def fire(adj, uav_id, aim, rounds=0):
    adj._on_fire(FireRequest(
        header=Header(stamp=0.0), task_id=1, uav_id=uav_id, track_id=1,
        effector=EffectorType.PROJECTILE,
        predicted_intercept=np.asarray(aim, dtype=float),
        p_kill=0.5, rounds=rounds,
    ))


def test_uav_shot_through_building_is_blocked_no_pk_roll():
    world = make_world()
    enemy = add_enemy(world, [200.0, 0.0, 10.0])
    uav = InterceptorUav("u1", world.bus, home=np.zeros(3),
                         effector=projectile_gun(), max_speed=80.0)
    uav.body.position = np.array([-200.0, 0.0, 10.0])
    uav.body.velocity = np.array([60.0, 0.0, 0.0])
    adj = EngagementAdjudicator(world, {"u1": uav}, {})
    results = []
    world.bus.subscribe("engagement/result", results.append)

    fire(adj, "u1", enemy.position)
    kinds = [e["kind"] for e in world.events]
    assert kinds == ["fire_blocked_los"]       # no kill, no miss: no roll
    assert world.events[0]["effector"] == "projectile"
    assert enemy.alive
    assert results and not results[0].hit

    # Same lay over the roofline adjudicates normally.
    uav.body.position = np.array([-200.0, 0.0, 60.0])
    enemy.body.position = np.array([200.0, 0.0, 80.0])
    fire(adj, "u1", enemy.position)
    assert world.events[-1]["kind"] in ("kill", "miss")


def test_blocked_turret_burst_still_lands_stray_rounds():
    world = make_world()
    enemy = add_enemy(world, [200.0, 0.0, 10.0])
    turret = GroundTurret("t1", world, np.array([-200.0, 0.0, 5.0]))
    world.turrets["t1"] = turret
    adj = EngagementAdjudicator(world, {}, {"t1": turret})

    fire(adj, "t1", enemy.position, rounds=5)
    blocked = [e for e in world.events if e["kind"] == "fire_blocked_los"]
    assert blocked and blocked[0]["effector"] == "turret_gun"
    assert enemy.alive
    assert not any(e["kind"] in ("kill", "miss") for e in world.events)
    # SIM-EFF-006/SIM-EFF-003: the burst was fired — every round lands.
    assert len(world.stray_impacts) >= 1
    assert all(s["shooter"] == "t1" for s in world.stray_impacts)
