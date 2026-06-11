"""Engagement attribution (SIM-GT-004): events carry shooter/weapon/
target/pk and the metrics aggregate them per shooter and per weapon."""

from coopuavs.sim import scenario as scenario_mod

ATTRIBUTED = {"kill", "miss", "debris_neutralized"}


def test_events_and_engagement_summary():
    sc = scenario_mod.load("scenarios/residential_raid.yaml", seed=4)
    sc.run()
    events = sc.world.events

    shots = [e for e in events if e["kind"] in ATTRIBUTED]
    assert shots, "the reference raid produced no engagement events"
    for ev in shots:
        assert ev["uav_id"]
        assert ev["effector"] in ("net", "projectile", "turret_gun")
        assert "pk" in ev
        assert ev.get("enemy_id") or ev.get("debris_id")
        assert ev.get("target_kind") in ("track", "debris")
        assert isinstance(ev.get("pos"), list) and len(ev["pos"]) == 3

    m = sc.eval_tracker.metrics()
    eng = m["engagements"]
    assert set(eng) == {"by_shooter", "by_weapon"}
    # sums across shooters equal sums across weapons equal the event counts
    def total(table, key):
        return sum(r[key] for r in table.values())
    kills = sum(e["kind"] == "kill" for e in events)
    debris_kills = sum(e["kind"] == "debris_neutralized" for e in events)
    assert total(eng["by_shooter"], "kills") == kills
    assert total(eng["by_weapon"], "kills") == kills
    assert total(eng["by_shooter"], "debris_kills") == debris_kills
    assert total(eng["by_shooter"], "shots") == total(eng["by_weapon"], "shots")
    for row in eng["by_shooter"].values():
        assert row["weapon"] in ("net", "projectile", "turret_gun")
    # turret shots aggregate under the turret_gun weapon row
    turret_shots = sum(
        r["shots"] for sid, r in eng["by_shooter"].items() if sid.startswith("turret"))
    assert eng["by_weapon"].get("turret_gun", {}).get("shots", 0) == turret_shots


def test_turret_events_carry_turret_gun():
    """ICD: kill/miss events name the weapon net|projectile|turret_gun — a
    turret burst must log turret_gun even though its wire messages carry the
    physical PROJECTILE effector type."""
    import numpy as np
    from coopuavs.core.messages import (
        EffectorType, FireRequest, Header, ThreatClass,
    )
    from coopuavs.sim.adjudicator import EngagementAdjudicator
    from coopuavs.sim.environment import Environment
    from coopuavs.sim.turret import GroundTurret
    from coopuavs.sim.world import World
    from coopuavs.threats.enemy_drone import EnemyDrone

    env = Environment.from_config({
        "bounds": [-3000.0, -3000.0, 3000.0, 3000.0], "default_zone": "SAFE",
    })
    world = World(env, dt=0.05, seed=2)
    turret = GroundTurret("t1", world, np.array([0.0, 0.0, 5.0]))
    world.turrets["t1"] = turret
    enemy = EnemyDrone("owa-1", ThreatClass.OWA_STRATEGIC,
                       np.array([400.0, 0.0, 300.0]),
                       np.array([2900.0, 0.0, 0.0]), world.rng, world=world)
    world.enemies["owa-1"] = enemy
    adj = EngagementAdjudicator(world, {}, {"t1": turret})

    adj._on_fire(FireRequest(
        header=Header(stamp=0.0), task_id=0, uav_id="t1", track_id=1,
        effector=EffectorType.PROJECTILE,
        predicted_intercept=enemy.position.copy(), p_kill=0.5, rounds=5,
    ))
    shot = [e for e in world.events if e["kind"] in ("kill", "miss")]
    assert shot and shot[0]["effector"] == "turret_gun"

    # the no-target branch names the weapon too
    adj._on_fire(FireRequest(
        header=Header(stamp=0.0), task_id=0, uav_id="t1", track_id=2,
        effector=EffectorType.PROJECTILE,
        predicted_intercept=np.array([2000.0, 2000.0, 300.0]),
        p_kill=0.5, rounds=5,
    ))
    nt = [e for e in world.events if e["kind"] == "fire_no_target"]
    assert nt and nt[0]["effector"] == "turret_gun"


def test_debris_fire_no_target_carries_effector():
    """The debris no-target branch used to omit the weapon, corrupting the
    by_weapon attribution table with an 'unknown' row (SIM-GT-004)."""
    import numpy as np
    from coopuavs.core.messages import EffectorType, FireRequest, Header
    from coopuavs.interceptors.effectors import projectile_gun
    from coopuavs.interceptors.uav import InterceptorUav
    from coopuavs.sim.adjudicator import EngagementAdjudicator
    from coopuavs.sim.environment import Environment
    from coopuavs.sim.world import World

    env = Environment.from_config({
        "bounds": [-2000.0, -2000.0, 2000.0, 2000.0], "default_zone": "SAFE",
    })
    world = World(env, dt=0.05, seed=3)
    uav = InterceptorUav("hawk-1", world.bus, home=np.zeros(3),
                         effector=projectile_gun(), max_speed=80.0)
    adj = EngagementAdjudicator(world, {"hawk-1": uav}, {})
    adj._on_fire(FireRequest(
        header=Header(stamp=0.0), task_id=1, uav_id="hawk-1", track_id=-101,
        effector=EffectorType.PROJECTILE,
        predicted_intercept=np.array([0.0, 0.0, 300.0]), p_kill=0.5,
        target_kind="debris", debris_id="deb-gone",
    ))
    nt = [e for e in world.events if e["kind"] == "fire_no_target"]
    assert nt and nt[0]["effector"] == "projectile"
