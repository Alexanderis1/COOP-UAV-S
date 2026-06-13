"""Slow-loop AI supervisor (the hybrid two-loop orchestration brain).

The leakage-critical *fast* loop — weapon-target assignment in
:mod:`coopuavs.c2.assignment` — stays a deterministic, millisecond,
auditable optimiser. This module is the *slow* loop that runs above it at a
few-second cadence and shapes how the fast loop spends the fleet:

* which ambiguous tracks to **confirm** with a sensor before committing a
  shooter (decoy economy),
* which lost-cause tracks to **defer** so force concentrates on the savable
  ones (the leakage lever under saturation),
* how to **re-weight** threats beyond the analytic score,
* where to put a **second shooter** (engagement depth) on a hard, savable,
  high-value target.

Two design invariants make it safe to put a learned model here:

1. **Advise-only.** A :class:`SupervisorDirective` can only *withhold*,
   *deprioritise*, *confirm-first*, or *add depth*. It never authorises a
   weapon release: the deterministic ROE + clearance interlock
   (:mod:`coopuavs.c2.roe`, :mod:`coopuavs.c2.orchestrator`) keeps sole
   release authority. Every field is clamped to that safe envelope on the
   way out of the policy.
2. **Deterministic fallback.** :class:`HeuristicSupervisor` is a pure,
   reproducible reference policy. It is both the offline fallback (used
   whenever the model is unavailable, slow, or returns garbage) and the
   imitation-learning *teacher* whose decisions seed the fine-tuning set for
   the open LLM (see :mod:`coopuavs.c2.supervisor_dataset`).

The fine-tuned open LLM plugs in through :class:`LLMSupervisor`, which is
handed a ``chat_fn`` (prompt -> completion) — e.g. an OpenAI-compatible
endpoint serving the fine-tuned Llama/Qwen-class model — formats the
tactical situation, parses the model's JSON directive, validates and clamps
it, and falls back to the heuristic on any error or timeout.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Callable, Protocol

# Hard ceilings the supervisor is clamped to (defence in depth around the
# learned model): a directive can never weight a threat to infinity nor pile
# the whole fleet onto one track.
MAX_TARGET_WEIGHT = 5.0
MIN_TARGET_WEIGHT = 0.0
MAX_K_SHOOTER = 3


@dataclass
class TrackSituation:
    """One track as the supervisor sees it — fused estimates only, never
    ground truth (mirrors what the C2 actually knows)."""

    track_id: int
    threat_class: str
    p_decoy: float
    speed: float
    threat_score: float
    time_to_impact: float
    savable: bool                 # a ready shooter can reach envelope before impact
    best_intercept_s: float | None
    impact_zone: str


@dataclass
class TacticalSituation:
    """The compact, model-friendly snapshot handed to the supervisor."""

    t: float
    tracks: list[TrackSituation]
    n_available_shooters: int
    inventory_rounds: int
    leakers_so_far: int
    decoy_shots_so_far: int
    roe_collateral_cap: float

    def to_json(self) -> str:
        return json.dumps(
            {
                "t": round(self.t, 1),
                "available_shooters": self.n_available_shooters,
                "inventory_rounds": self.inventory_rounds,
                "leakers_so_far": self.leakers_so_far,
                "decoy_shots_so_far": self.decoy_shots_so_far,
                "roe_collateral_cap": self.roe_collateral_cap,
                "tracks": [
                    {
                        "id": s.track_id,
                        "class": s.threat_class,
                        "p_decoy": round(s.p_decoy, 2),
                        "speed": round(s.speed, 1),
                        "threat": round(s.threat_score, 3),
                        "tti": round(s.time_to_impact, 1),
                        "savable": s.savable,
                        "intercept_s": (round(s.best_intercept_s, 1)
                                        if s.best_intercept_s is not None else None),
                        "impact_zone": s.impact_zone,
                    }
                    for s in self.tracks
                ],
            },
            sort_keys=True,
        )


@dataclass
class SupervisorDirective:
    """Advisory shaping of the fast-loop allocation. Every field is
    leakage-reducing or force-concentrating; none can release a weapon."""

    target_weights: dict[int, float] = field(default_factory=dict)
    defer: set[int] = field(default_factory=set)
    confirm_first: set[int] = field(default_factory=set)
    k_shooter: dict[int, int] = field(default_factory=dict)
    posture_hint: str | None = None
    rationale: str = ""

    def weight(self, track_id: int) -> float:
        return self.target_weights.get(track_id, 1.0)


def clamp_directive(d: SupervisorDirective, valid_ids: set[int]) -> SupervisorDirective:
    """Project any directive (heuristic or model-produced) onto the safe
    envelope: known track ids only, bounded weights, bounded depth. This is
    the guard that lets an unvetted model output be trusted at most to
    *shape* — never to escalate."""
    weights = {
        tid: max(MIN_TARGET_WEIGHT, min(MAX_TARGET_WEIGHT, float(w)))
        for tid, w in d.target_weights.items() if tid in valid_ids
    }
    k = {
        tid: max(1, min(MAX_K_SHOOTER, int(n)))
        for tid, n in d.k_shooter.items() if tid in valid_ids
    }
    return SupervisorDirective(
        target_weights=weights,
        defer={tid for tid in d.defer if tid in valid_ids},
        confirm_first={tid for tid in d.confirm_first if tid in valid_ids},
        k_shooter=k,
        posture_hint=d.posture_hint if d.posture_hint in (None, "human_confirm",
                                                          "pre_authorized",
                                                          "weapons_hold") else None,
        rationale=d.rationale[:500],
    )


class SupervisorPolicy(Protocol):
    def decide(self, situation: TacticalSituation) -> SupervisorDirective: ...


class HeuristicSupervisor:
    """Deterministic reference policy — the offline fallback and the
    imitation teacher.

    Triage rule (the leakage lever): when ready shooters are scarcer than the
    savable, credible threats, *defer the lost causes* (un-savable or
    low-threat) so the fleet concentrates on targets an engagement can still
    change. Ambiguous-decoy tracks are routed to confirm-first instead of
    burning a shooter, and a hard, savable, high-value target earns a second
    shooter when (and only when) spare capacity exists.
    """

    def __init__(
        self,
        confirm_lo: float = 0.45,
        defer_lo: float = 0.6,
        defer_hi: float = 0.85,
        defer_min_tti: float = 20.0,
        depth_threat: float = 0.6,
    ):
        # Bands: [confirm_lo, defer_lo) earns a sensor look; [defer_lo,
        # defer_hi) is leaning-decoy and is withheld; >= defer_hi is already
        # hard-ignored by the allocator (DECOY_IGNORE_THRESHOLD).
        self.confirm_lo = confirm_lo
        self.defer_lo = defer_lo
        self.defer_hi = defer_hi
        self.defer_min_tti = defer_min_tti
        self.depth_threat = depth_threat

    def decide(self, situation: TacticalSituation) -> SupervisorDirective:
        d = SupervisorDirective()
        # Spare-capacity gauge keyed on *credible* (p_decoy < defer_lo) savable
        # threats — depth is only spent when shooters outnumber them.
        savable_credible = [
            s for s in situation.tracks if s.p_decoy < self.defer_lo and s.savable
        ]
        scarce = situation.n_available_shooters <= len(savable_credible)

        for s in situation.tracks:
            # Leaning-decoy but not yet hard-ignored, and not urgent: withhold
            # a shooter and let EO/IR resolve it. A track only ever crosses
            # into this band once the sensors lean decoy — a real OWA sits
            # near p_decoy 0.5 until resolved, so this never delays a credible
            # threat (the failure mode that made the earlier tuning leak).
            if (self.defer_lo <= s.p_decoy < self.defer_hi
                    and s.time_to_impact > self.defer_min_tti):
                d.defer.add(s.track_id)
                continue

            # Mildly ambiguous: flag a confirming sensor look (no deprioritise,
            # no defer — confirming must not delay engaging a real threat).
            if self.confirm_lo <= s.p_decoy < self.defer_lo:
                d.confirm_first.add(s.track_id)

            # Concentrate force on savable, high-value threats; a second
            # shooter (depth) only on the hardest such target with spare
            # capacity.
            if s.savable and s.threat_score >= self.depth_threat:
                d.target_weights[s.track_id] = 1.4
                if (not scarce
                        and s.best_intercept_s is not None
                        and s.best_intercept_s > 0.5 * s.time_to_impact):
                    d.k_shooter[s.track_id] = 2

        d.rationale = (
            f"heuristic: {len(savable_credible)} credible savable, "
            f"shooters={situation.n_available_shooters} "
            f"({'scarce' if scarce else 'sufficient'}); "
            f"defer={len(d.defer)} confirm_first={len(d.confirm_first)} "
            f"depth={len(d.k_shooter)}"
        )
        return clamp_directive(d, {s.track_id for s in situation.tracks})


SYSTEM_PROMPT = (
    "You are the supervisory C2 agent for a cooperative counter-drone "
    "defence. You DO NOT authorise weapon release — a separate rules-of-"
    "engagement interlock does that. You only shape how the fast-loop "
    "weapon-target allocator spends a limited interceptor fleet, to MINIMISE "
    "the value of threats that leak through. Given the tactical situation as "
    "JSON, reply with ONLY a JSON object with optional keys: "
    "target_weights {id: 0..5}, defer [ids], confirm_first [ids], "
    "k_shooter {id: 1..3}, posture_hint, rationale. Prefer to: defer lost-"
    "cause or low-threat tracks when shooters are scarcer than savable "
    "credible threats; confirm_first ambiguous decoys (0.45<=p_decoy<=0.85) "
    "rather than spend a shooter; add a second shooter only to hard, savable, "
    "high-value threats when spare capacity exists."
)


class LLMSupervisor:
    """Fine-tuned open-LLM supervisor (advise-only).

    ``chat_fn`` maps a prompt string to a completion string — wire it to an
    OpenAI-compatible server hosting the fine-tuned open-weight model (or any
    callable). Any failure, timeout, malformed reply, or out-of-envelope
    directive degrades to the deterministic ``fallback`` so the defence is
    never blocked on the model.
    """

    def __init__(
        self,
        chat_fn: Callable[[str], str],
        fallback: SupervisorPolicy | None = None,
        system_prompt: str = SYSTEM_PROMPT,
    ):
        self.chat_fn = chat_fn
        self.fallback = fallback or HeuristicSupervisor()
        self.system_prompt = system_prompt

    def build_prompt(self, situation: TacticalSituation) -> str:
        return f"{self.system_prompt}\n\nSITUATION:\n{situation.to_json()}\n\nDIRECTIVE:"

    def decide(self, situation: TacticalSituation) -> SupervisorDirective:
        valid = {s.track_id for s in situation.tracks}
        try:
            raw = self.chat_fn(self.build_prompt(situation))
            directive = parse_directive(raw)
        except Exception:
            return self.fallback.decide(situation)
        if directive is None:
            return self.fallback.decide(situation)
        return clamp_directive(directive, valid)


def parse_directive(text: str) -> SupervisorDirective | None:
    """Extract a :class:`SupervisorDirective` from a model completion. Returns
    None when no usable JSON object is present (the caller falls back)."""
    start, end = text.find("{"), text.rfind("}")
    if start < 0 or end <= start:
        return None
    try:
        obj = json.loads(text[start:end + 1])
    except (json.JSONDecodeError, ValueError):
        return None
    if not isinstance(obj, dict):
        return None
    try:
        weights = {int(k): float(v) for k, v in obj.get("target_weights", {}).items()}
        k_shooter = {int(k): int(v) for k, v in obj.get("k_shooter", {}).items()}
        defer = {int(x) for x in obj.get("defer", [])}
        confirm = {int(x) for x in obj.get("confirm_first", [])}
    except (TypeError, ValueError):
        return None
    return SupervisorDirective(
        target_weights=weights,
        defer=defer,
        confirm_first=confirm,
        k_shooter=k_shooter,
        posture_hint=obj.get("posture_hint"),
        rationale=str(obj.get("rationale", ""))[:500],
    )
