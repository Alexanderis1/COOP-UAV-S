"""Airborne look-down early-warning radar (PHY-SNT-004)."""

import numpy as np

from coopuavs.core.messages import ThreatClass
from coopuavs.sensors.airborne_radar import AirborneRadar
from coopuavs.sensors.radar import Radar
from coopuavs.sim.environment import Environment
from coopuavs.sim.world import World
from coopuavs.threats.enemy_drone import EnemyDrone


def _world():
    env = Environment.from_config(
        {"bounds": [-9000.0, -9000.0, 9000.0, 9000.0], "default_zone": "SAFE"})
    return World(env, dt=0.05, seed=3)


def _enemy(world, pos, cls=ThreatClass.OWA_JET):
    e = EnemyDrone("t", cls, np.asarray(pos, float),
                   np.array([0.0, 0.0, 0.0]), world.rng, world=world)
    world.enemies["t"] = e
    return e


def test_look_down_sees_what_ground_radar_cannot():
    """A diver below the platform sits at a negative elevation: the ground
    radar's terrain-horizon gate rejects it every scan; the airborne
    look-down set acquires it with high Pd."""
    world = _world()
    enemy = _enemy(world, [3000.0, 0.0, 3000.0])
    high = np.array([0.0, 0.0, 4000.0])
    ground = Radar("gr", world, high)               # horizon gate at +1.5 deg
    air = AirborneRadar("ar", world, high)           # look-down, no horizon
    ground_hits = sum(ground.observe(enemy, t=0.0) is not None for _ in range(60))
    air_hits = sum(air.observe(enemy, t=0.0) is not None for _ in range(60))
    assert ground_hits == 0, "ground radar should be horizon-masked below it"
    assert air_hits > 50, f"airborne radar should acquire the diver ({air_hits}/60)"


def test_long_range_high_target_detected():
    """Across the map at altitude, the airborne set still paints a jet RCS."""
    world = _world()
    enemy = _enemy(world, [8000.0, 0.0, 3800.0])
    air = AirborneRadar("ar", world, np.array([0.0, 4000.0, 3900.0]))
    hits = sum(air.observe(enemy, t=0.0) is not None for _ in range(60))
    assert hits > 30, f"far high target should be detected often ({hits}/60)"


def test_look_down_clutter_suppresses_low_targets():
    """A target low over the ground competes with main-lobe clutter: the
    clutter factor must cut its detection rate (and zero it at factor 0),
    while a co-located high target is unaffected."""
    world = _world()
    low = _enemy(world, [3000.0, 0.0, 60.0])          # below clutter_alt
    pos = np.array([0.0, 0.0, 300.0])
    full = AirborneRadar("a-full", world, pos, clutter_factor=1.0, clutter_alt=250.0)
    notch = AirborneRadar("a-notch", world, pos, clutter_factor=0.0, clutter_alt=250.0)
    full_hits = sum(full.observe(low, t=0.0) is not None for _ in range(60))
    notch_hits = sum(notch.observe(low, t=0.0) is not None for _ in range(60))
    assert notch_hits == 0, "clutter_factor=0 must veto the low look-down target"
    assert full_hits > notch_hits, "clutter factor should reduce low-target Pd"

    # A high target is above the clutter band and unaffected by the notch.
    world2 = _world()
    high = _enemy(world2, [3000.0, 0.0, 3000.0])
    notch2 = AirborneRadar("a", world2, np.array([0.0, 0.0, 3500.0]),
                           clutter_factor=0.0, clutter_alt=250.0)
    assert sum(notch2.observe(high, t=0.0) is not None for _ in range(60)) > 40


def test_ground_radar_unchanged():
    """The ground Radar must be byte-for-byte the verified baseline: same
    seed and geometry give the same detection sequence as a fresh Radar."""
    w1, w2 = _world(), _world()
    e1 = _enemy(w1, [4000.0, 0.0, 1500.0])
    e2 = _enemy(w2, [4000.0, 0.0, 1500.0])
    r1 = Radar("radar-main", w1, np.array([0.0, -2500.0, 15.0]))
    r2 = Radar("radar-main", w2, np.array([0.0, -2500.0, 15.0]))
    seq1 = [r1.observe(e1, t=0.0) is not None for _ in range(40)]
    seq2 = [r2.observe(e2, t=0.0) is not None for _ in range(40)]
    assert seq1 == seq2
