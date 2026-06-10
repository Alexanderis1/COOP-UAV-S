"""Weather model: gust determinism, sensor degradation direction, wind drift."""

import numpy as np

from coopuavs.core.messages import ThreatClass
from coopuavs.sim.environment import Environment
from coopuavs.sim.weather import WeatherState
from coopuavs.sim.world import World
from coopuavs.threats.enemy_drone import EnemyDrone


def make_weather(seed=5, **kw) -> WeatherState:
    return WeatherState(np.random.default_rng(seed), **kw)


def gust_sequence(seed: int, n: int = 200) -> np.ndarray:
    wx = make_weather(seed, wind_speed=8.0, wind_dir_deg=270.0)
    out = []
    for _ in range(n):
        wx.step(0.05)
        out.append(wx.wind.copy())
    return np.array(out)


def test_gusts_deterministic_given_seed():
    assert np.array_equal(gust_sequence(5), gust_sequence(5))
    assert not np.array_equal(gust_sequence(5), gust_sequence(6))


def test_calm_default_draws_nothing_and_is_neutral():
    rng = np.random.default_rng(9)
    before = rng.bit_generator.state
    wx = WeatherState(rng)
    for _ in range(50):
        wx.step(0.05)
    assert rng.bit_generator.state == before          # SIM-003: stream untouched
    assert wx.eo_ir_range_factor() == 1.0
    assert wx.acoustic_range_factor() == 1.0
    assert wx.radar_range_factor() == 1.0
    assert np.allclose(wx.wind, 0.0)


def test_sensor_degradation_directions():
    clear_night = make_weather()
    fog = make_weather(fog=0.6)
    rain = make_weather(precip=0.8)
    windy = make_weather(wind_speed=15.0)
    dusk = make_weather(daylight=0.5)

    # EO/IR: fog and rain attenuate; night and full day are both fine (IR
    # carries the night), the crossover dip sits at dusk.
    assert fog.eo_ir_range_factor() < clear_night.eo_ir_range_factor()
    assert rain.eo_ir_range_factor() < clear_night.eo_ir_range_factor()
    assert dusk.eo_ir_range_factor() < clear_night.eo_ir_range_factor()
    assert make_weather(daylight=1.0).eo_ir_range_factor() == 1.0

    # Acoustic: wind and rain mask; radar: only rain, and mildly.
    assert windy.acoustic_range_factor() < clear_night.acoustic_range_factor()
    assert rain.acoustic_range_factor() < clear_night.acoustic_range_factor()
    assert rain.radar_range_factor() < clear_night.radar_range_factor()
    assert rain.radar_range_factor() > 0.8
    assert windy.radar_range_factor() == 1.0


def _drift_world(wind_speed: float) -> World:
    env = Environment.from_config({"bounds": [-2000.0, -2000.0, 2000.0, 2000.0]})
    world = World(env, dt=0.05, seed=3)
    world.weather = WeatherState(world.rng, wind_speed=wind_speed,
                                 wind_dir_deg=270.0, gust_std=0.0)
    world.schedule_enemy(0.0, lambda: EnemyDrone(
        "owa-1", ThreatClass.OWA_STRATEGIC,
        np.array([-1500.0, 0.0, 1000.0]), np.array([1500.0, 0.0, 0.0]),
        world.rng, world=world,
    ))
    return world


def test_wind_displaces_enemy_truth():
    calm, windy = _drift_world(0.0), _drift_world(12.0)
    for _ in range(200):
        calm.step()
        windy.step()
    calm_pos = calm.enemies["owa-1"].position
    windy_pos = windy.enemies["owa-1"].position
    # Wind FROM the west pushes the truth track further east.
    assert windy_pos[0] > calm_pos[0] + 50.0
