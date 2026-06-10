"""RunController: ICD §5 seam — tick/pause/speed must not alter sim states."""

import copy

from coopuavs.sim import scenario as scenario_mod
from coopuavs.sim.runctl import RunController

CFG = {
    "name": "runctl",
    "seed": 11,
    "dt": 0.05,
    "duration": 30.0,
    "record_hz": 5.0,
    "environment": {
        "bounds": [-4000.0, -4000.0, 4000.0, 4000.0],
        "default_zone": "SAFE",
        "assets": [{"name": "substation", "position": [0.0, 0.0, 0.0]}],
    },
    "sensors": [
        {"type": "radar", "name": "radar-1", "position": [0.0, -1000.0, 10.0],
         "max_range": 9000.0},
    ],
    "interceptors": [
        {"id": "u1", "home": [-200.0, -1000.0, 0.0], "effector": "projectile",
         "max_speed": 80.0},
    ],
    "threats": [
        {"time": 2.0, "class": "OWA_STRATEGIC",
         "spawn": [-3800.0, 3800.0, 1200.0], "target": "substation"},
    ],
}


def normalise(frames):
    """Frame payloads minus the presentation-only run block, with track and
    task ids renumbered by first appearance: those counters are process-
    global (itertools.count at module scope), so two runs in one process
    legitimately produce identical states under shifted ids."""
    track_map, task_map = {}, {}

    def trk_id(i):
        return track_map.setdefault(i, len(track_map))

    out = []
    for f in frames:
        f = copy.deepcopy({k: v for k, v in f.items() if k != "run"})
        for trk in f["tracks"]:
            trk["id"] = trk_id(trk["id"])
        for u in f["uavs"]:
            if u["task_id"] is not None:
                u["task_id"] = task_map.setdefault(u["task_id"], len(task_map))
        for tur in f["turrets"]:
            if tur["target"] is not None:
                tur["target"] = trk_id(tur["target"])
        for ev in f["events"] + f["decisions"]:
            if ev.get("track_id") is not None:
                ev["track_id"] = trk_id(ev["track_id"])
            if ev.get("task_id") is not None:
                ev["task_id"] = task_map.setdefault(ev["task_id"], len(task_map))
        out.append(f)
    return out


def drive_uniform():
    ctl = RunController(scenario_mod.build(copy.deepcopy(CFG)))
    frames = []
    while ctl.status == "running":
        frames += ctl.tick(0.25)
    return ctl, frames


def drive_erratic():
    """Same run through pauses, speed changes and ragged wall ticks."""
    ctl = RunController(scenario_mod.build(copy.deepcopy(CFG)))
    frames = []
    frames += ctl.tick(0.013)
    ctl.pause()
    assert ctl.tick(5.0) == []                     # paused: wall time is ignored
    ctl.resume()
    ctl.set_speed(4.0)
    frames += ctl.tick(0.4)
    ctl.set_speed(0.5)
    n = 0
    while ctl.status == "running" and n < 100000:
        frames += ctl.tick(0.071)
        n += 1
    return ctl, frames


def test_tick_pattern_pause_and_speed_do_not_change_results():
    ctl_a, frames_a = drive_uniform()
    ctl_b, frames_b = drive_erratic()
    assert ctl_a.status == ctl_b.status == "done"
    assert normalise(frames_a) == normalise(frames_b)   # SIM-RT-002
    assert ctl_a.summary() == ctl_b.summary()


def test_speed_is_clamped():
    ctl = RunController(scenario_mod.build(copy.deepcopy(CFG)))
    ctl.set_speed(99.0)
    assert ctl.speed == 10.0
    ctl.set_speed(0.0)
    assert ctl.speed == 0.1


def test_payload_accessors_match_icd_shapes():
    ctl = RunController(scenario_mod.build(copy.deepcopy(CFG)))
    ctl.tick(2.0)

    scene = ctl.scene()
    for key in ("bounds", "grid", "assets", "sensors", "turrets", "homes", "run"):
        assert key in scene
    assert scene["run"]["seed"] == 11

    frame = ctl.frame()
    for key in ("t", "run", "tracks", "uavs", "turrets", "wrecks", "strays",
                "env", "events", "decisions"):
        assert key in frame

    truth = ctl.truth()
    assert set(truth) == {"t", "enemies", "metrics"}
    assert set(truth["metrics"]) == {"detection", "attrition", "economics",
                                     "collateral", "auth"}

    ctl.stop()
    assert ctl.status == "done"
    assert ctl.tick(1.0) == []
    assert "metrics" in ctl.summary()
