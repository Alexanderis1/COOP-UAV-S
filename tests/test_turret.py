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
            header=Header(stamp=world.t), task_id=0, uav_id="t1", track_id=1,
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


def test_partial_last_burst_reports_actual_rounds():
    """A 3-round magazine fires a 3-round last burst: the fire message
    carries the true count so the adjudicator neither rolls Pk for nor
    lands strays from phantom rounds (SIM-EFF-003)."""
    world, turret = make_battlefield()
    turret.magazine = 3
    fires = []
    world.bus.subscribe("engagement/fire", fires.append)
    enemy = world.enemies["owa-1"]
    track = Track(header=Header(stamp=world.t), track_id=1,
                  position=enemy.position.copy(),
                  velocity=enemy.velocity.copy())
    turret._fire_burst(track, enemy.position.copy(), 500.0, world.t)
    assert fires[0].rounds == 3
    assert turret.magazine == 0


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
                header=Header(stamp=world.t), task_id=0, uav_id="t1", track_id=1,
                decision=EngagementDecision.DENIED,
            ))
        world.step()
    assert fires == []
    assert turret.target_track is None
    assert 1 in turret._denied


def test_denied_debris_lay_respects_denial_ttl():
    """A DENIED verdict on a debris pseudo-track (negative id) must abandon
    the lay and start the denial TTL exactly like a track denial — under
    weapons_hold the turret used to keep the lay and re-request the same
    falling object every 2 s for its whole fall."""
    from coopuavs.core.messages import DebrisArray, DebrisState

    world, turret = make_battlefield()
    requests = []
    world.bus.subscribe("engagement/fire_request", requests.append)

    def publish_debris():
        world.bus.publish("debris/state", DebrisArray(
            header=Header(stamp=world.t), debris=[DebrisState(
                header=Header(stamp=world.t), debris_id="deb-x", track_ref=-101,
                position=np.array([500.0, 0.0, 600.0]),
                velocity=np.array([0.0, 0.0, -10.0]),
                predicted_impact=np.array([500.0, 0.0, 0.0]),
                impact_zone=ZoneClass.DANGEROUS, t_impact=60.0,
            )]))

    for _ in range(200):                       # settle the lay, first request
        publish_debris()
        world.step()
        if requests:
            break
    assert requests and requests[0].track_id == -101

    world.bus.publish("engagement/clearance", FireClearance(
        header=Header(stamp=world.t), task_id=0, uav_id="t1", track_id=-101,
        decision=EngagementDecision.DENIED,
    ))
    assert turret.target_track is None
    assert -101 in turret._denied
    n0 = len(requests)
    for _ in range(int(6.0 / world.dt)):       # well inside DENIAL_TTL_S
        publish_debris()
        world.step()
    assert len(requests) == n0                 # no re-request spam


def test_turret_ignores_safe_bound_debris():
    """SAFE-bound wreckage never enters turret target selection
    (PHY-GCS-006): no lay, no clearance request."""
    from coopuavs.core.messages import DebrisArray, DebrisState

    world, turret = make_battlefield()
    requests = []
    world.bus.subscribe("engagement/fire_request", requests.append)
    for _ in range(100):
        world.bus.publish("debris/state", DebrisArray(
            header=Header(stamp=world.t), debris=[DebrisState(
                header=Header(stamp=world.t), debris_id="deb-safe",
                track_ref=-101, position=np.array([500.0, 0.0, 600.0]),
                velocity=np.array([0.0, 0.0, -10.0]),
                predicted_impact=np.array([2500.0, 2500.0, 0.0]),
                impact_zone=ZoneClass.SAFE, t_impact=60.0,
            )]))
        world.step()
    assert requests == []
    assert turret.state == "idle" and turret.target_track is None


def test_turret_debris_intercept_end_to_end():
    """The whole turret debris chain (PHY-GCS-006): pseudo-track selection,
    clearance interlock, burst adjudication against the live object, credit
    under debris_intercepted with the turret_gun weapon name."""
    from coopuavs.core.messages import DebrisArray, DebrisState, EffectorType
    from coopuavs.sim.debris_objects import FallingDebris

    env = Environment.from_config({
        "bounds": [-3000.0, -3000.0, 3000.0, 3000.0],
        "default_zone": "SAFE",
        "zones": [{"rect": [400.0, -200.0, 1200.0, 200.0], "class": "DANGEROUS"}],
    })
    world = World(env, dt=0.05, seed=2)
    turret = GroundTurret("t1", world, np.array([0.0, 0.0, 5.0]),
                          dispersion_mrad=3.0, min_pk=0.0)
    world.turrets["t1"] = turret
    world.add_node(turret)
    world.add_node(EngagementAdjudicator(world, {}, {"t1": turret}))
    deb = FallingDebris("deb-1", "owa-1", np.array([500.0, 0.0, 600.0]),
                        np.array([0.0, 0.0, 0.0]), EffectorType.PROJECTILE,
                        track_ref=-101)
    world.debris[deb.debris_id] = deb

    while world.debris and world.t < 16.0:
        live = world.debris.get("deb-1")
        if live is not None:
            impact = live.predicted_impact()
            world.bus.publish("debris/state", DebrisArray(
                header=Header(stamp=world.t), debris=[DebrisState(
                    header=Header(stamp=world.t), debris_id="deb-1",
                    track_ref=-101, position=live.position.copy(),
                    velocity=live.velocity.copy(),
                    predicted_impact=np.array([impact[0], impact[1], 0.0]),
                    impact_zone=world.env.risk_map.zone_at(impact[0], impact[1]),
                    t_impact=live.time_to_impact(),
                )]))
            world.bus.publish("engagement/clearance", FireClearance(
                header=Header(stamp=world.t), task_id=0, uav_id="t1",
                track_id=-101, decision=EngagementDecision.AUTHORIZED,
            ))
        world.step()

    assert world.debris_intercepted, "turret never neutralized the wreck"
    credit = world.debris_intercepted[0]
    assert credit["shooter"] == "t1" and credit["effector"] == "turret_gun"
    neut = [e for e in world.events if e["kind"] == "debris_neutralized"]
    assert neut and neut[0]["effector"] == "turret_gun"
    assert not world.wrecks                    # intercepted, never landed


def test_clearance_for_another_track_is_ignored():
    """The token is bound to the track the ROE costed (PHY-TUR-001): an
    AUTHORIZED token for track 9 must not release a burst at track 1, and
    a DENIED verdict for track 9 must not blacklist track 1."""
    world, turret = make_battlefield()
    fires = []
    world.bus.subscribe("engagement/fire", fires.append)

    for _ in range(100):
        publish_truth_track(world)
        world.bus.publish("engagement/clearance", FireClearance(
            header=Header(stamp=world.t), task_id=0, uav_id="t1", track_id=9,
            decision=EngagementDecision.AUTHORIZED,
        ))
        world.step()
    assert fires == []                         # wrong-track token: no shot
    assert turret.magazine == 300

    world.bus.publish("engagement/clearance", FireClearance(
        header=Header(stamp=world.t), task_id=0, uav_id="t1", track_id=9,
        decision=EngagementDecision.DENIED,
    ))
    assert 9 in turret._denied
    assert 1 not in turret._denied             # current lay survives
    assert turret.target_track == 1
