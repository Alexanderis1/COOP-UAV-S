#!/usr/bin/env python
"""Export an SFT dataset for the open-LLM supervisor from sim rollouts.

Runs a scenario across a seed range with the deterministic
:class:`~coopuavs.c2.supervisor.HeuristicSupervisor` as the teacher, wrapped
in a :class:`~coopuavs.c2.supervisor_dataset.RecordingSupervisor`, and writes
one chat-style JSONL example per supervisor tick — ready for supervised
fine-tuning of an open-weight model (Llama/Qwen-class).

    python scripts/export_supervisor_dataset.py scenarios/saturation_raid.yaml \
        --seeds 0-49 -o supervisor_sft.jsonl

Stage 2 (RL/preference fine-tuning to beat the teacher on leaked threat
value) re-scores these situations by replay outcome — see
docs/RESEARCH.md, "Hybrid orchestrator" appendix.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

from coopuavs.c2.supervisor import HeuristicSupervisor
from coopuavs.c2.supervisor_dataset import RecordingSupervisor
from coopuavs.sim import scenario as scenario_mod


def _parse_seeds(spec: str) -> list[int]:
    if "-" in spec:
        lo, hi = spec.split("-", 1)
        return list(range(int(lo), int(hi) + 1))
    return [int(x) for x in spec.split(",") if x]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("scenario", type=Path)
    ap.add_argument("--seeds", default="0-19", help="e.g. 0-49 or 1,2,3")
    ap.add_argument("-o", "--out", type=Path, default=Path("supervisor_sft.jsonl"))
    args = ap.parse_args()

    cfg = yaml.safe_load(args.scenario.read_text())
    cfg.setdefault("base_station", {})["supervisor"] = "heuristic"

    n_examples = 0
    with args.out.open("w") as fh:
        sink = lambda rec: fh.write(json.dumps(rec) + "\n")  # noqa: E731
        for seed in _parse_seeds(args.seeds):
            sc = scenario_mod.build(dict(cfg), seed=seed)
            base = next(n for n in sc.world.nodes if n.name == "base_station")
            recorder = RecordingSupervisor(HeuristicSupervisor(), sink)
            base.supervisor = recorder
            sc.run()
            n_examples += recorder.n

    print(f"wrote {n_examples} examples to {args.out}")


if __name__ == "__main__":
    main()
