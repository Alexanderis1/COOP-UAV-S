"""Evaluate / A-B a trained cooperation policy against the classical baseline.

Runs whole battles (the unmodified pipeline, with whichever allocator the
scenario names) over a seed sweep and aggregates the outcome metrics that
matter for this project: armed leakers (defence failures), kills, ammo
economy, collateral (zone-weighted debris + strays), detection latency, and
the jet-OWA-specific kill/leak rate. The headline comparison is *learned +
CAP sentinels* vs *greedy + CAP sentinels* on the same scenario — does the
policy convert the sentinels' time margin into fewer leakers and less
collateral than the greedy allocator does?

numpy-only; importing a learned policy pulls in torch lazily via the
scenario's allocator spec.
"""

from __future__ import annotations

import copy

import numpy as np

from ..sim import scenario as scenario_mod


def run_battle(cfg: dict, seed: int, *, policy: str | None = None) -> dict:
    """Run one battle and return its outcome metrics. ``policy`` (a checkpoint
    path) selects the learned allocator; ``None`` keeps the scenario's
    configured allocator (classical by default)."""
    cfg = copy.deepcopy(cfg)
    if policy is not None:
        bs = cfg.setdefault("base_station", {})
        bs["allocator"] = "learned"
        bs["policy"] = policy
    sc = scenario_mod.build(cfg, seed=seed)
    summary = sc.run()
    m = sc.eval_tracker.metrics()
    jets = [e for e in sc.world.enemies.values()
            if e.threat_class.value == "owa_jet"]
    jet_lat = [e.acquired_t - e.spawn_t for e in jets if e.acquired_t is not None]
    return {
        "kills": summary["kills"],
        "armed_leakers": summary["armed_leakers"],
        "decoy_kills": summary["kills_decoy"],
        "ammo_per_kill": m["economics"]["ammo_per_kill"],
        "shots": m["economics"]["shots"],
        "debris_cost": m["collateral"]["debris_cost"],
        "strays": sum(m["collateral"]["strays_by_zone"].values()),
        "mean_det_latency": m["detection"]["mean_latency"],
        "jets": len(jets),
        "jet_kills": sum(e.killed for e in jets),
        "jet_leaks": sum(e.reached_target for e in jets),
        "jet_mean_acq": float(np.mean(jet_lat)) if jet_lat else None,
        "fallbacks": getattr(_find_bs(sc), "allocator_fallbacks", 0),
    }


def _find_bs(sc):
    from ..c2.base_station import BaseStation
    return next((n for n in sc.world.nodes if isinstance(n, BaseStation)), None)


def _agg(rows: list[dict]) -> dict:
    keys = ["kills", "armed_leakers", "decoy_kills", "shots", "debris_cost",
            "strays", "jet_kills", "jet_leaks"]
    out = {k: round(float(np.mean([r[k] for r in rows])), 3) for k in keys}
    apk = [r["ammo_per_kill"] for r in rows if r["ammo_per_kill"] is not None]
    out["ammo_per_kill"] = round(float(np.mean(apk)), 3) if apk else None
    acq = [r["jet_mean_acq"] for r in rows if r["jet_mean_acq"] is not None]
    out["jet_mean_acq"] = round(float(np.mean(acq)), 3) if acq else None
    out["total_fallbacks"] = int(sum(r["fallbacks"] for r in rows))
    return out


def ab_compare(scenario, seeds, *, policy: str) -> dict:
    """Aggregate greedy vs learned over ``seeds`` on the same scenario."""
    import yaml
    from pathlib import Path
    if isinstance(scenario, (str, bytes)) or hasattr(scenario, "read_text"):
        cfg = yaml.safe_load(Path(scenario).read_text())
    else:
        cfg = scenario
    seeds = list(seeds)
    greedy = [run_battle(cfg, s, policy=None) for s in seeds]
    learned = [run_battle(cfg, s, policy=policy) for s in seeds]
    return {"seeds": seeds,
            "greedy": _agg(greedy), "learned": _agg(learned),
            "greedy_rows": greedy, "learned_rows": learned}
