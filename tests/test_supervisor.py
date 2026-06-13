import numpy as np

from coopuavs.core.messages import (
    Header,
    ThreatAssessment,
    Track,
    UavState,
    ZoneClass,
)
from coopuavs.c2 import assignment
from coopuavs.c2.supervisor import (
    HeuristicSupervisor,
    LLMSupervisor,
    SupervisorDirective,
    TacticalSituation,
    TrackSituation,
    clamp_directive,
    parse_directive,
)
from coopuavs.risk.zones import RiskMap


def trk(tid, p_decoy=0.0, score=0.8, tti=40.0, savable=True, speed=55.0, best=10.0):
    return TrackSituation(
        track_id=tid, threat_class="owa_strategic", p_decoy=p_decoy, speed=speed,
        threat_score=score, time_to_impact=tti, savable=savable,
        best_intercept_s=best, impact_zone="DANGEROUS",
    )


def sit(tracks, shooters=4):
    return TacticalSituation(
        t=10.0, tracks=tracks, n_available_shooters=shooters, inventory_rounds=32,
        leakers_so_far=0, decoy_shots_so_far=0, roe_collateral_cap=0.3,
    )


# -- heuristic policy ---------------------------------------------------------

def test_heuristic_is_deterministic():
    s = sit([trk(1), trk(2, p_decoy=0.6), trk(3, savable=False, score=0.2)])
    a = HeuristicSupervisor().decide(s)
    b = HeuristicSupervisor().decide(s)
    assert a.defer == b.defer and a.confirm_first == b.confirm_first
    assert a.target_weights == b.target_weights and a.k_shooter == b.k_shooter


def test_heuristic_confirms_ambiguous_decoy():
    d = HeuristicSupervisor().decide(sit([trk(1, p_decoy=0.6)]))
    assert 1 in d.confirm_first
    assert d.target_weights.get(1, 1.0) < 1.0


def test_heuristic_defers_lost_cause_when_scarce():
    # One savable credible threat and one un-savable low-threat track, but
    # only one shooter: the lost cause is deferred to free the shooter.
    s = sit([trk(1, savable=True, score=0.8),
             trk(2, savable=False, score=0.2)], shooters=1)
    d = HeuristicSupervisor().decide(s)
    assert 2 in d.defer and 1 not in d.defer


def test_heuristic_keeps_lost_cause_when_capacity_spare():
    s = sit([trk(1, savable=True, score=0.8),
             trk(2, savable=False, score=0.2)], shooters=8)
    d = HeuristicSupervisor().decide(s)
    assert 2 not in d.defer


def test_heuristic_adds_depth_to_hard_savable_target():
    # Hard (late intercept relative to impact), savable, high value, spare
    # capacity -> a second shooter.
    s = sit([trk(1, score=0.9, tti=40.0, best=30.0)], shooters=6)
    d = HeuristicSupervisor().decide(s)
    assert d.k_shooter.get(1, 1) == 2


# -- parsing / clamping (the model-output guard) ------------------------------

def test_parse_directive_from_noisy_completion():
    raw = 'Sure! {"defer": [3], "k_shooter": {"1": 2}, "rationale": "x"} done'
    d = parse_directive(raw)
    assert d is not None and d.defer == {3} and d.k_shooter == {1: 2}


def test_parse_directive_rejects_garbage():
    assert parse_directive("no json here") is None
    assert parse_directive("{not valid json}") is None


def test_clamp_bounds_and_drops_unknown_ids():
    d = SupervisorDirective(
        target_weights={1: 99.0, 7: 1.0}, k_shooter={1: 50},
        defer={7}, posture_hint="launch_nukes",
    )
    c = clamp_directive(d, valid_ids={1})
    assert c.target_weights == {1: 5.0}      # weight capped, unknown id 7 dropped
    assert c.k_shooter == {1: 3}             # depth capped
    assert c.defer == set()                  # unknown id dropped
    assert c.posture_hint is None            # invalid posture rejected


# -- LLM adapter: fallback + safety ------------------------------------------

def test_llm_falls_back_on_bad_model():
    def broken(_prompt):
        raise RuntimeError("model down")
    d = LLMSupervisor(broken).decide(sit([trk(1, p_decoy=0.6)]))
    # Identical to the heuristic fallback.
    assert d.confirm_first == HeuristicSupervisor().decide(
        sit([trk(1, p_decoy=0.6)])).confirm_first


def test_llm_directive_is_clamped_not_trusted():
    # A model that tries to over-escalate one track is clamped to the safe
    # envelope; there is no field through which it can authorise a release.
    def adversarial(_prompt):
        return '{"target_weights": {"1": 1000}, "k_shooter": {"1": 99}}'
    d = LLMSupervisor(adversarial).decide(sit([trk(1)]))
    assert d.target_weights[1] <= 5.0 and d.k_shooter[1] <= 3
    assert not hasattr(d, "clearance") and not hasattr(d, "authorize")


# -- allocator honours the directive -----------------------------------------

def _atrack(tid, pos, vel, p_decoy=0.0):
    return Track(header=Header(stamp=0.0), track_id=tid,
                 position=np.asarray(pos, float), velocity=np.asarray(vel, float),
                 p_decoy=p_decoy)


def _uav(uid, pos, ammo=4):
    return UavState(header=Header(stamp=0.0), uav_id=uid,
                    position=np.asarray(pos, float), ammo=ammo)


def _assess(tid, score=0.8):
    return ThreatAssessment(header=Header(stamp=0.0), track_id=tid,
                            threat_score=score, time_to_impact=100.0,
                            predicted_impact=np.zeros(3))


def test_allocate_defers_track_in_directive():
    rm = RiskMap((-5000, -5000, 5000, 5000), default=ZoneClass.SAFE)
    speeds = {"u1": 60.0}
    tracks = {1: _atrack(1, [3000, 0, 1000], [-50, 0, 0])}
    directive = SupervisorDirective(defer={1})
    tasks = assignment.allocate([_assess(1)], tracks, [_uav("u1", [0, 0, 0])],
                                speeds, rm, t=0.0, directive=directive)
    assert tasks == []


def test_allocate_depth_adds_second_shooter():
    rm = RiskMap((-5000, -5000, 5000, 5000), default=ZoneClass.SAFE)
    speeds = {"u1": 60.0, "u2": 60.0}
    tracks = {1: _atrack(1, [2000, 0, 1000], [-50, 0, 0])}
    directive = SupervisorDirective(k_shooter={1: 2})
    tasks = assignment.allocate([_assess(1)], tracks,
                                [_uav("u1", [0, 0, 0]), _uav("u2", [200, 0, 0])],
                                speeds, rm, t=0.0, directive=directive)
    shooters = {t.shooter_id for t in tasks if t.track_id == 1}
    assert len(shooters) == 2          # primary + depth shooter on the same track
