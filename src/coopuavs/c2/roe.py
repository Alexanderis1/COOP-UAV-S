"""Rules of engagement: probabilistic, zone-aware fire authorisation.

Every shot must be cleared by the C2. The decision is made on the *predicted
debris footprint* of the kill, not on the target position: the debris model
is run at the proposed intercept point and the resulting Monte-Carlo ground
samples are costed against the risk map.

Decision logic
--------------
AUTHORIZED  expected collateral cost below threshold AND probability of any
            debris on a CRITICAL cell below its own hard cap.
AUTHORIZED  (now-or-never) cost is above the base threshold but *at its
            minimum over the target's predicted path*: the target is flying
            toward worse ground (into the city), so holding can only
            increase collateral. Take the least-bad shot while it exists.
HOLD        unsafe now, but the engagement geometry can still improve
            (target moving toward safer ground, or interceptors can herd
            it) — keep shaping, ask again.
AUTHORIZED  (last-resort) unsafe, but the target is about to hit and its
            threat score exceeds the last-resort bar: letting a warhead
            through is costlier than the debris. The relaxed cap still
            never clears a shot whose debris is *likely* to hit CRITICAL.
DENIED      decoy-grade target with unsafe geometry — not worth any risk.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..core.messages import (
    EffectorType,
    EngagementDecision,
    FireClearance,
    FireRequest,
    Header,
    ThreatAssessment,
)
from ..risk.debris import DebrisModel
from ..risk.zones import RiskMap


@dataclass
class RoeConfig:
    max_expected_collateral: float = 0.30   # zone-weighted cost per shot
    max_p_critical: float = 0.01            # hard cap, normal operations
    last_resort_time: float = 25.0          # s to impact
    last_resort_threat: float = 0.35        # minimum threat score
    last_resort_collateral: float = 2.0     # relaxed cost cap
    last_resort_p_critical: float = 0.05
    lookahead_times: tuple[float, ...] = (5.0, 10.0, 20.0)  # now-or-never horizon


class RulesOfEngagement:
    def __init__(self, risk_map: RiskMap, debris: DebrisModel, config: RoeConfig | None = None):
        self.risk_map = risk_map
        self.debris = debris
        self.config = config or RoeConfig()

    def evaluate(
        self,
        request: FireRequest,
        target_velocity: np.ndarray,
        effector: EffectorType,
        assessment: ThreatAssessment | None,
        t: float,
    ) -> FireClearance:
        cfg = self.config
        footprint = self.debris.footprint(request.predicted_intercept, target_velocity, effector)
        cost = self.risk_map.collateral_cost(footprint)
        p_crit = self.risk_map.critical_hit_probability(footprint)

        def clearance(decision: EngagementDecision, reason: str) -> FireClearance:
            return FireClearance(
                header=Header(stamp=t),
                task_id=request.task_id,
                uav_id=request.uav_id,
                decision=decision,
                expected_collateral=cost,
                reason=reason,
            )

        if cost <= cfg.max_expected_collateral and p_crit <= cfg.max_p_critical:
            return clearance(EngagementDecision.AUTHORIZED, "geometry_safe")

        threat = assessment.threat_score if assessment else 0.0
        # Now-or-never: cost the same kill at the target's predicted future
        # positions. If today's geometry is the best the path will offer and
        # is within the relaxed cap, holding only moves debris onto worse
        # ground.
        if (
            threat >= cfg.last_resort_threat
            and cost <= cfg.last_resort_collateral
            and p_crit <= cfg.last_resort_p_critical
        ):
            future_costs = []
            for dt in cfg.lookahead_times:
                future_pos = request.predicted_intercept + target_velocity * dt
                if future_pos[2] <= 0:
                    continue
                fp = self.debris.footprint(future_pos, target_velocity, effector)
                future_costs.append(self.risk_map.collateral_cost(fp))
            if future_costs and cost <= min(future_costs):
                return clearance(EngagementDecision.AUTHORIZED, "now_or_never")
        tti = assessment.time_to_impact if assessment else 1e9
        if (
            tti <= cfg.last_resort_time
            and threat >= cfg.last_resort_threat
            and cost <= cfg.last_resort_collateral
            and p_crit <= cfg.last_resort_p_critical
        ):
            return clearance(EngagementDecision.AUTHORIZED, "last_resort")

        if threat < cfg.last_resort_threat and tti <= cfg.last_resort_time:
            return clearance(EngagementDecision.DENIED, "low_threat_unsafe_geometry")
        return clearance(
            EngagementDecision.HOLD,
            f"collateral={cost:.2f} p_critical={p_crit:.3f} — shape geometry",
        )
