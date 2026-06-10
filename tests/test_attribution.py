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
        assert ev["effector"] in ("net", "projectile")
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
        assert row["weapon"] in ("net", "projectile", "turret_gun", "unknown")
    # turret shots aggregate under the turret_gun weapon row
    turret_shots = sum(
        r["shots"] for sid, r in eng["by_shooter"].items() if sid.startswith("turret"))
    assert eng["by_weapon"].get("turret_gun", {}).get("shots", 0) == turret_shots
