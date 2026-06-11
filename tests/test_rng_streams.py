"""P0-6: per-consumer RNG stream isolation (DESIGN_REVIEW 5.1).

One test per migrated consumer: the consumer must draw from its own
registry stream and leave the legacy shared `world.rng` untouched, so its
randomness no longer depends on call order relative to anyone else.
Capstone: the order-independence test — an extra no-op consumer leaves
every other draw identical.
"""

from __future__ import annotations

import numpy as np

from coopuavs.sim import scenario as scenario_mod

MINIMAL_ENV = {
    "bounds": [-2000.0, -2000.0, 2000.0, 2000.0],
    "cell_size": 100.0,
    "default_zone": "SAFE",
}


def _shared_stream_state(world):
    return world.rng.bit_generator.state


def _fresh_state(seed):
    return np.random.default_rng(seed).bit_generator.state


# -- weather -------------------------------------------------------------------

def test_windy_weather_does_not_consume_the_shared_stream():
    cfg = {
        "name": "wind-only",
        "seed": 13,
        "environment": dict(MINIMAL_ENV),
        "weather": {"wind_speed": 8.0, "wind_dir_deg": 270.0},
        "threats": [],
    }
    sc = scenario_mod.build(cfg)
    for _ in range(50):
        sc.world.step()
    assert np.linalg.norm(sc.world.weather._gust) > 0.0  # gusts really ran
    assert _shared_stream_state(sc.world) == _fresh_state(13)


def test_weather_stream_reproducible_for_seed():
    def gusts(seed):
        cfg = {
            "name": "wind-only",
            "seed": seed,
            "environment": dict(MINIMAL_ENV),
            "weather": {"wind_speed": 8.0},
            "threats": [],
        }
        sc = scenario_mod.build(cfg)
        out = []
        for _ in range(20):
            sc.world.step()
            out.append(tuple(sc.world.weather._gust))
        return out

    assert gusts(5) == gusts(5)
    assert gusts(5) != gusts(6)


# -- comms ---------------------------------------------------------------------

def test_lossy_comms_does_not_consume_the_shared_stream():
    cfg = {
        "name": "lossy-comms",
        "seed": 21,
        "environment": dict(MINIMAL_ENV),
        "comms": {"base_loss": 0.5, "latency_s": 0.02},
        "interceptors": [
            {"id": "u1", "home": [0.0, 0.0, 0.0], "effector": "projectile"},
        ],
        "seekers": False,  # keep the world free of other RNG consumers
        "threats": [],
    }
    sc = scenario_mod.build(cfg)
    for _ in range(50):
        sc.world.step()
    # the lossy link really rolled: delivery stats accumulated some failures
    stats = sc.world.comms._stats["u1"]
    assert any(not ok for _, ok in stats)
    assert _shared_stream_state(sc.world) == _fresh_state(21)


# -- sensors -------------------------------------------------------------------

def test_sensor_scans_do_not_consume_the_shared_stream():
    cfg = {
        "name": "radar-only",
        "seed": 31,
        "environment": dict(MINIMAL_ENV),
        "sensors": [
            {"type": "radar", "name": "radar-1",
             "position": [0.0, 0.0, 10.0], "max_range": 9000.0},
        ],
        "threats": [
            {"id": "t1", "time": 0.0, "class": "OWA_STRATEGIC",
             "spawn": [500.0, 500.0, 300.0], "target": [0.0, 0.0, 0.0]},
        ],
    }
    sc = scenario_mod.build(cfg)
    detections = []
    sc.world.bus.subscribe("detections", detections.append)

    sc.world.step()  # spawn tick: EnemyDrone.__init__ still draws shared (P0-6e)
    state_after_spawn = _shared_stream_state(sc.world)
    for _ in range(50):
        sc.world.step()

    assert detections  # the radar really scanned and detected
    assert _shared_stream_state(sc.world) == state_after_spawn


# -- adjudicator + debris model --------------------------------------------------

def test_full_battle_leaves_the_shared_stream_untouched():
    """Full SMALL_SCENARIO battle: weather, sensing, comms, threats, kill
    rolls, ROE debris footprints and wreck retention jitter all ride named
    streams — the legacy shared `world.rng` stays virgin end to end."""
    import copy

    from test_end_to_end import SMALL_SCENARIO

    sc = scenario_mod.build(copy.deepcopy(SMALL_SCENARIO))
    sc.run()
    assert any(e["kind"] == "kill" for e in sc.world.events)
    assert _shared_stream_state(sc.world) == _fresh_state(SMALL_SCENARIO["seed"])


# -- order independence (DESIGN_REVIEW 5.1 closure) ------------------------------

def test_extra_noop_consumer_leaves_every_other_draw_identical():
    """The 5.1 fix, demonstrated: register an extra consumer that drains
    10k draws from its own stream — every battle outcome stays identical."""
    import copy

    from test_end_to_end import SMALL_SCENARIO

    def run(extra_consumer: bool):
        sc = scenario_mod.build(copy.deepcopy(SMALL_SCENARIO))
        if extra_consumer:
            sc.world.rng_registry.stream("extra/noop").random(10_000)
        summary = sc.run()
        return sc.world.events, summary

    assert run(False) == run(True)
