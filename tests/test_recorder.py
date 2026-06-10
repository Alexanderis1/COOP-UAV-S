"""Recorder terminal snapshot: a run that resolves between recorder ticks
must still end the replay with the final world state and tail events."""

import json

from coopuavs.sim.environment import Environment
from coopuavs.sim.world import World
from coopuavs.viz.recorder import Recorder

ENV_CFG = {
    "bounds": [-2000.0, -2000.0, 2000.0, 2000.0],
    "default_zone": "SAFE",
    "assets": [{"name": "substation", "position": [0.0, 0.0, 0.0]}],
}


def test_save_appends_a_terminal_snapshot(tmp_path):
    world = World(Environment.from_config(ENV_CFG))
    rec = Recorder(world, rate_hz=5.0)
    rec.update(0.0, 0.05)                     # last scheduled tick at t=0
    world.t = 3.7                             # the raid resolves mid-interval
    world.log_event("kill", uav_id="u1", enemy_id="owa-1")

    data = json.loads(rec.save(tmp_path / "r.json").read_text())
    last = data["frames"][-1]
    assert last["t"] == 3.7
    assert any(e["kind"] == "kill" for e in last["events"])


def test_save_is_idempotent(tmp_path):
    world = World(Environment.from_config(ENV_CFG))
    rec = Recorder(world, rate_hz=5.0)
    rec.update(0.0, 0.05)
    world.t = 3.7
    rec.save(tmp_path / "a.json")
    data = json.loads(rec.save(tmp_path / "b.json").read_text())
    assert len(data["frames"]) == 2           # no duplicate terminal frame
