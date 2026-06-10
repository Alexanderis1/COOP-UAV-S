import numpy as np

from coopuavs.core.messages import (
    EffectorType,
    EngagementDecision,
    FireRequest,
    Header,
    ThreatAssessment,
    ZoneClass,
)
from coopuavs.risk.debris import DebrisModel
from coopuavs.risk.zones import RiskMap
from coopuavs.c2.roe import RoeConfig, RulesOfEngagement


def make_roe(default=ZoneClass.SAFE) -> RulesOfEngagement:
    rm = RiskMap((-5000, -5000, 5000, 5000), cell_size=100.0, default=default)
    rm.set_rect((-500, -500, 500, 500), ZoneClass.CRITICAL)
    return RulesOfEngagement(rm, DebrisModel(np.random.default_rng(3)), RoeConfig())


def request(pos, p_kill=0.5) -> FireRequest:
    return FireRequest(
        header=Header(stamp=0.0), task_id=1, uav_id="hawk-1", track_id=1,
        effector=EffectorType.NET, predicted_intercept=np.asarray(pos, float),
        p_kill=p_kill,
    )


def assessment(threat=0.8, tti=120.0) -> ThreatAssessment:
    return ThreatAssessment(
        header=Header(stamp=0.0), track_id=1, threat_score=threat,
        time_to_impact=tti, predicted_impact=np.zeros(3),
    )


def test_authorized_over_safe_ground():
    roe = make_roe()
    c = roe.evaluate(request([-3000.0, -3000.0, 400.0]), np.array([55.0, 0, 0]),
                     EffectorType.NET, assessment(), t=0.0)
    assert c.decision == EngagementDecision.AUTHORIZED


def test_hold_over_critical_zone():
    roe = make_roe()
    c = roe.evaluate(request([0.0, 0.0, 400.0]), np.array([55.0, 0, 0]),
                     EffectorType.NET, assessment(threat=0.8, tti=120.0), t=0.0)
    assert c.decision == EngagementDecision.HOLD


def test_last_resort_overrides_dangerous_ground():
    # All-DANGEROUS map: normally unsafe, but the target is about to hit.
    rm = RiskMap((-5000, -5000, 5000, 5000), cell_size=100.0,
                 default=ZoneClass.DANGEROUS)
    roe = RulesOfEngagement(rm, DebrisModel(np.random.default_rng(4)), RoeConfig())
    c = roe.evaluate(request([2000.0, 2000.0, 300.0]), np.array([55.0, 0, 0]),
                     EffectorType.NET, assessment(threat=0.9, tti=10.0), t=0.0)
    assert c.decision == EngagementDecision.AUTHORIZED
    # Uniform ground means "won't get better" (now_or_never) fires before
    # the time-based last_resort rule; both express the same authority.
    assert c.reason in ("now_or_never", "last_resort")


def test_denied_for_low_threat_unsafe_geometry():
    roe = make_roe(default=ZoneClass.DANGEROUS)
    # Probable decoy (low threat score), no time left, unsafe ground.
    c = roe.evaluate(request([2000.0, 2000.0, 300.0]), np.array([55.0, 0, 0]),
                     EffectorType.NET, assessment(threat=0.1, tti=10.0), t=0.0)
    assert c.decision == EngagementDecision.DENIED
