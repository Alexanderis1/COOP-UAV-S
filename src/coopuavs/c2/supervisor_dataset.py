"""Fine-tuning data pipeline for the open-LLM supervisor.

The hybrid plan trains the open-weight LLM in two stages:

1. **Supervised fine-tuning (SFT)** to imitate the deterministic
   :class:`~coopuavs.c2.supervisor.HeuristicSupervisor` teacher — the model
   first learns to reproduce a known-good policy and the strict output
   schema. This module produces those ``(prompt, completion)`` pairs from
   simulation rollouts.
2. **Preference / RL fine-tuning** to *beat* the teacher on the real
   objective (minimise leaked threat value), scoring candidate directives by
   the leakage they produce in replay. The same records, re-scored by
   outcome, are the reward signal.

:class:`RecordingSupervisor` wraps any teacher policy, delegates the decision
unchanged (so a recorded run is identical to a normal run), and emits one
JSONL record per supervisor tick. Because the teacher is deterministic, the
dataset is fully reproducible from a seed list.
"""

from __future__ import annotations

import json
from typing import Callable

from .supervisor import (
    SYSTEM_PROMPT,
    SupervisorDirective,
    SupervisorPolicy,
    TacticalSituation,
)


def directive_to_completion(d: SupervisorDirective) -> str:
    """Serialise a directive as the exact JSON the model is trained to emit."""
    return json.dumps(
        {
            "target_weights": {str(k): round(v, 2) for k, v in d.target_weights.items()},
            "defer": sorted(d.defer),
            "confirm_first": sorted(d.confirm_first),
            "k_shooter": {str(k): v for k, v in d.k_shooter.items()},
            "rationale": d.rationale,
        },
        sort_keys=True,
    )


def build_record(situation: TacticalSituation, directive: SupervisorDirective,
                 system_prompt: str = SYSTEM_PROMPT) -> dict:
    """One SFT example in a chat-style schema portable to most open-LLM
    fine-tuning stacks (Llama/Qwen-class)."""
    return {
        "messages": [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": situation.to_json()},
            {"role": "assistant", "content": directive_to_completion(directive)},
        ]
    }


class RecordingSupervisor:
    """Teacher wrapper that records every decision as a training example.

    ``sink`` receives one dict per tick (e.g. a list's ``append`` or a
    function that writes a JSONL line). The wrapped ``teacher`` decides
    exactly as it would unwrapped — recording never perturbs the run.
    """

    def __init__(self, teacher: SupervisorPolicy, sink: Callable[[dict], None]):
        self.teacher = teacher
        self.sink = sink
        self.n = 0

    def decide(self, situation: TacticalSituation) -> SupervisorDirective:
        directive = self.teacher.decide(situation)
        # Skip empty-situation ticks — no tracks, nothing to learn.
        if situation.tracks:
            self.sink(build_record(situation, directive))
            self.n += 1
        return directive
