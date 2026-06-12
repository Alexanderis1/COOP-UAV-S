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


def test_v03_frame_and_scene_wire_fields():
    """ICD-003 payload pin: the v0.3 additions (live debris, station
    occupancy, uav kind/effector, building kind/material/name, station
    rooftop flag) ship under exactly these names — a silent rename breaks
    every frontend consumer."""
    import numpy as np
    from coopuavs.core.messages import EffectorType, Header, UavMode, UavState
    from coopuavs.sim.debris_objects import FallingDebris

    env_cfg = dict(ENV_CFG)
    env_cfg["buildings"] = [{"rect": [-100.0, -100.0, 100.0, 100.0],
                             "height": 30.0, "kind": "residential_high",
                             "name": "block-a"}]
    env_cfg["charging_stations"] = [{"id": "cs-1", "pos": [500.0, 0.0, 30.0],
                                     "rooftop": True}]
    world = World(Environment.from_config(env_cfg))
    rec = Recorder(world, rate_hz=5.0)

    world.debris["deb-1"] = FallingDebris(
        "deb-1", "owa-1", np.array([0.0, 0.0, 300.0]),
        np.array([10.0, 0.0, 0.0]), EffectorType.PROJECTILE, track_ref=-101)
    world.bus.publish("uav/state", UavState(
        header=Header(stamp=0.0), uav_id="snt-1",
        position=np.array([500.0, 0.0, 30.0]), ammo=0, mode=UavMode.IDLE,
        kind="sentinel", effector=""))

    class Docked:                              # truth-side pad occupant
        position = np.array([500.0, 0.0, 30.0])
        home = np.array([500.0, 0.0, 30.0])
    world.friendlies["snt-1"] = Docked()

    frame = rec.snapshot()
    deb = frame["debris"][0]
    assert set(deb) == {"id", "pos", "vel", "impact", "zone", "t_impact"}
    assert deb["id"] == "deb-1" and deb["impact"][2] == 0.0
    assert frame["stations"] == [{"id": "cs-1", "occupied": 1}]
    u = frame["uavs"][0]
    assert u["kind"] == "sentinel" and u["effector"] is None and u["ammo"] == 0

    scene = rec.scene()
    b = scene["buildings"][0]
    assert b == {"rect": [-100.0, -100.0, 100.0, 100.0], "height": 30.0,
                 "kind": "residential_high", "material": "concrete",
                 "name": "block-a"}
    assert scene["stations"] == [{"id": "cs-1", "pos": [500.0, 0.0, 30.0],
                                  "rooftop": True}]


def test_save_is_idempotent(tmp_path):
    world = World(Environment.from_config(ENV_CFG))
    rec = Recorder(world, rate_hz=5.0)
    rec.update(0.0, 0.05)
    world.t = 3.7
    rec.save(tmp_path / "a.json")
    data = json.loads(rec.save(tmp_path / "b.json").read_text())
    assert len(data["frames"]) == 2           # no duplicate terminal frame
