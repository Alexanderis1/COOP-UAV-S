"""EvalTracker: acquisition gating, detection latency and ICD §4 metrics."""

from coopuavs.sim import scenario as scenario_mod

CFG = {
    "name": "evaluation",
    "seed": 11,
    "dt": 0.05,
    "duration": 60.0,
    "environment": {
        "bounds": [-4000.0, -4000.0, 4000.0, 4000.0],
        "default_zone": "SAFE",
        "assets": [{"name": "substation", "position": [0.0, 0.0, 0.0]}],
    },
    "sensors": [
        {"type": "radar", "name": "radar-1", "position": [0.0, -1000.0, 10.0],
         "max_range": 9000.0},
    ],
    "interceptors": [],
    "threats": [
        # Spawn times off the 5 Hz sensor grid and positions far apart, so
        # latencies are strictly positive and neither threat can gate-match
        # the other's existing track.
        {"time": 5.1, "class": "OWA_STRATEGIC",
         "spawn": [-3500.0, 3500.0, 1200.0], "target": "substation"},
        {"time": 12.3, "class": "DECOY",
         "spawn": [3300.0, 3500.0, 1300.0], "target": "substation"},
    ],
}


def test_detection_latency_recorded_and_truth_payload_shape():
    sc = scenario_mod.build(CFG)
    sc.world.run(40.0, stop_when_clear=False)
    tracker = sc.eval_tracker

    metrics = tracker.metrics()
    det = metrics["detection"]
    assert det["total"] == 2
    assert det["acquired"] == 2                      # radar sees both at altitude
    for row in det["latencies"]:
        assert row["latency"] is not None
        assert 0.0 < row["latency"] < 30.0           # acquired after spawn, quickly
    assert det["mean_latency"] > 0.0

    # Truth enemies carry the acquisition bookkeeping for the ghost overlay.
    truth = tracker.truth_payload()
    for enemy in truth["enemies"]:
        assert enemy["acquired"] is True
        assert enemy["acquired_t"] is not None
        assert enemy["track_id"] is not None
        assert enemy["target"] == "substation"
    decoy = next(e for e in truth["enemies"] if e["cls"] == "decoy")
    assert decoy["warhead"] is False

    # Attrition table covers both classes; nobody was engaged (no shooters).
    assert metrics["attrition"]["owa_strategic"]["spawned"] == 1
    assert metrics["attrition"]["decoy"]["spawned"] == 1
    assert metrics["economics"]["shots"] == 0
    assert metrics["economics"]["ammo_per_kill"] is None

    # Auth counters tolerate the absence of an orchestrator.
    assert metrics["auth"] == {"requests": 0, "approved": 0, "denied": 0,
                               "expired": 0, "mean_latency": None}


def test_acquisition_events_logged_with_latency():
    sc = scenario_mod.build(CFG)
    sc.world.run(40.0, stop_when_clear=False)
    acquired = [e for e in sc.world.events if e["kind"] == "acquired"]
    assert len(acquired) == 2
    for ev in acquired:
        assert ev["latency"] > 0.0
        assert "track_id" in ev
