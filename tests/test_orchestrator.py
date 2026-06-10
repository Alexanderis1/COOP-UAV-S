"""Orchestrator (SRS ORC-001..006): posture matrix, authorisation window,
sim-time expiry under pause, and operator command flow."""

import numpy as np
import pytest

from coopuavs.c2.orchestrator import Orchestrator
from coopuavs.core.bus import MessageBus
from coopuavs.core.messages import (
    EffectorType,
    EngagementDecision,
    FireClearance,
    FireRequest,
    Header,
    RoeEvaluation,
    UavMode,
)
from coopuavs.sim import scenario as scenario_mod
from coopuavs.sim.runctl import RunController

A = EngagementDecision.AUTHORIZED
H = EngagementDecision.HOLD
D = EngagementDecision.DENIED


def make_eval(decision, uav_id="hawk-1", track_id=3, t=1.0,
              reason="geometry_safe", pk=0.6, collateral=0.12) -> RoeEvaluation:
    request = FireRequest(header=Header(stamp=t), task_id=1, uav_id=uav_id,
                          track_id=track_id, effector=EffectorType.PROJECTILE,
                          p_kill=pk)
    clearance = FireClearance(header=Header(stamp=t), task_id=1, uav_id=uav_id,
                              decision=decision, expected_collateral=collateral,
                              reason=reason)
    return RoeEvaluation(header=Header(stamp=t), request=request, clearance=clearance)


class Harness:
    """Bare orchestrator on a bare bus: events, northbound and clearances taped."""

    def __init__(self, posture: str, console: bool = True):
        self.bus = MessageBus()
        self.events: list[dict] = []
        self.north: list[tuple[str, dict]] = []
        self.clearances: list[FireClearance] = []
        self.orch = Orchestrator(self.bus, posture=posture, log_event=self._log)
        if console:
            self.orch.set_northbound(lambda t, d: self.north.append((t, d)))
        self.bus.subscribe("engagement/clearance", self.clearances.append)
        self.orch.update(1.0, 0.1)

    def _log(self, kind, **data):
        self.events.append({"kind": kind, **data})

    def event_kinds(self):
        return [e["kind"] for e in self.events]


# -- posture matrix (HMI-AUT-003 x ROE outcomes) --------------------------------


@pytest.mark.parametrize("roe_decision", [A, H, D])
def test_weapons_hold_denies_everything(roe_decision):
    h = Harness("weapons_hold")
    h.bus.publish("c2/roe_evaluation", make_eval(roe_decision))
    assert len(h.clearances) == 1
    assert h.clearances[0].decision == D
    assert h.clearances[0].reason == "weapons hold"
    assert h.north == []                                     # nothing escalated
    decisions = [e for e in h.events if e["kind"] == "decision"]
    assert decisions and decisions[0]["by"] == "posture"


@pytest.mark.parametrize("roe_decision,reason", [
    (A, "geometry_safe"), (H, "shape geometry"), (D, "low_threat_unsafe_geometry"),
])
def test_pre_authorized_passes_roe_verdict_through(roe_decision, reason):
    h = Harness("pre_authorized")
    h.bus.publish("c2/roe_evaluation", make_eval(roe_decision, reason=reason))
    assert len(h.clearances) == 1
    assert h.clearances[0].decision == roe_decision
    assert h.clearances[0].reason == reason                  # ROE verdict intact
    assert h.north == []                                     # agent auto-clears (by=orc)
    assert "auth_request" not in h.event_kinds()


@pytest.mark.parametrize("roe_decision", [H, D])
def test_human_confirm_passes_roe_hold_and_denied_through(roe_decision):
    """ROE itself said no: don't bother the human (ORC-002)."""
    h = Harness("human_confirm")
    h.bus.publish("c2/roe_evaluation", make_eval(roe_decision))
    assert len(h.clearances) == 1
    assert h.clearances[0].decision == roe_decision
    assert h.north == []


def test_human_confirm_escalates_authorized_and_holds_clearance():
    h = Harness("human_confirm")
    h.bus.publish("c2/roe_evaluation", make_eval(A))
    assert h.clearances == []                                # withheld for the human
    assert [t for t, _ in h.north] == ["auth_request"]
    payload = h.north[0][1]
    assert payload["shooter"] == "hawk-1"
    assert payload["track_id"] == 3
    assert payload["roe"]["decision"] == "authorized"
    assert payload["expires_t"] == pytest.approx(payload["t"] + 12.0)
    assert "Pk 0.60" in payload["rationale"]                 # ORC-003 context
    assert "auth_request" in h.event_kinds()
    # Duplicate request for the same shooter/track is not re-escalated.
    h.bus.publish("c2/roe_evaluation", make_eval(A))
    assert len(h.north) == 1


def test_operator_approval_clears_and_logs_latency():
    h = Harness("human_confirm")
    h.bus.publish("c2/roe_evaluation", make_eval(A, reason="geometry_safe"))
    auth_id = h.north[0][1]["id"]
    h.orch.update(4.0, 0.1)
    assert h.orch.resolve(auth_id, approve=True) is True
    assert len(h.clearances) == 1
    assert h.clearances[0].decision == A
    assert h.clearances[0].reason == "geometry_safe"
    assert h.clearances[0].track_id == 3       # token bound to the costed track
    approved = next(e for e in h.events if e["kind"] == "auth_approved")
    assert approved["latency"] == pytest.approx(3.0)
    assert ("auth_resolved", {"id": auth_id, "approved": True, "by": "operator"}) \
        in h.north
    # Exactly-once: a second resolution is rejected.
    assert h.orch.resolve(auth_id, approve=True) is False


def test_operator_denial_publishes_denied():
    h = Harness("human_confirm")
    h.bus.publish("c2/roe_evaluation", make_eval(A))
    auth_id = h.north[0][1]["id"]
    h.orch.resolve(auth_id, approve=False)
    assert h.clearances[0].decision == D
    assert h.clearances[0].reason == "operator denied"
    assert "auth_denied" in h.event_kinds()


def test_expiry_in_sim_time_yields_hold_by_timeout():
    h = Harness("human_confirm")
    h.bus.publish("c2/roe_evaluation", make_eval(A))          # raised at t=1
    h.orch.update(12.9, 0.1)                                  # window is 12 s
    assert h.clearances == []
    h.orch.update(13.1, 0.1)
    assert len(h.clearances) == 1
    assert h.clearances[0].decision == H                      # window gone: hold
    assert "auth_expired" in h.event_kinds()
    assert ("auth_resolved", {"id": 1, "approved": False, "by": "timeout"}) in h.north


def test_unattended_human_confirm_falls_back_to_roe():
    """Headless/batch runs (no console attached): the ROE config is the
    human-pre-approved rule (SYS-004) — authorized shots pass through."""
    h = Harness("human_confirm", console=False)
    h.bus.publish("c2/roe_evaluation", make_eval(A))
    assert len(h.clearances) == 1 and h.clearances[0].decision == A


def test_posture_change_resolves_pending_requests():
    # -> weapons_hold: pending requests denied by the posture.
    h = Harness("human_confirm")
    h.bus.publish("c2/roe_evaluation", make_eval(A))
    h.orch.set_posture("weapons_hold")
    assert h.clearances[-1].decision == D
    assert h.clearances[-1].reason == "weapons hold"
    assert h.north[-1] == ("auth_resolved", {"id": 1, "approved": False,
                                             "by": "posture"})
    # -> pre_authorized: pending (all ROE-authorized) auto-cleared by the agent.
    h2 = Harness("human_confirm")
    h2.bus.publish("c2/roe_evaluation", make_eval(A))
    h2.orch.set_posture("pre_authorized")
    assert h2.clearances[-1].decision == A
    assert h2.north[-1] == ("auth_resolved", {"id": 1, "approved": True,
                                              "by": "orc"})


def test_invalid_posture_rejected():
    h = Harness("human_confirm")
    with pytest.raises(ValueError, match="free_fire"):
        h.orch.set_posture("free_fire")
    with pytest.raises(ValueError):
        Orchestrator(MessageBus(), posture="autonomous")


# -- expiry across pause/resume (sim time, not wall time) -------------------------


CFG = {
    "name": "orc-pause",
    "seed": 3,
    "dt": 0.05,
    "duration": 60.0,
    "environment": {
        "bounds": [-2000.0, -2000.0, 2000.0, 2000.0],
        "default_zone": "SAFE",
        "assets": [{"name": "substation", "position": [0.0, 0.0, 0.0]}],
    },
    "sensors": [],
    "interceptors": [
        {"id": "u1", "home": [0.0, -500.0, 0.0], "effector": "projectile"},
    ],
    "threats": [],
}


def test_pending_request_survives_pause_and_expires_in_sim_time():
    sc = scenario_mod.build(CFG)
    north = []
    sc.orchestrator.set_northbound(lambda t, d: north.append((t, d)))
    ctl = RunController(sc)
    ctl.tick(1.0)                                             # sim runs to ~1 s
    sc.world.bus.publish("c2/roe_evaluation", make_eval(A, t=sc.world.t))
    assert len(sc.orchestrator.pending_requests()) == 1

    ctl.pause()
    assert ctl.tick(3600.0) == []                             # an hour of wall time
    assert len(sc.orchestrator.pending_requests()) == 1       # ORC paused with world

    ctl.resume()
    while sc.world.t < 15.0:                                  # past the 12 s window
        ctl.tick(0.5)
    assert sc.orchestrator.pending_requests() == []
    assert any(e["kind"] == "auth_expired" for e in sc.world.events)
    assert north[-1][0] == "auth_resolved" and north[-1][1]["by"] == "timeout"


def test_runcontroller_set_posture_syncs_orchestrator_and_frames():
    sc = scenario_mod.build(CFG)
    ctl = RunController(sc)
    ctl.tick(0.5)
    ctl.set_posture("weapons_hold")
    assert sc.orchestrator.posture == "weapons_hold"
    assert ctl.frame()["run"]["posture"] == "weapons_hold"
    with pytest.raises(ValueError):
        ctl.set_posture("free_fire")


# -- operator RTB command (ICD §3 uav_command) -------------------------------------


def test_rtb_command_reaches_uav_and_is_logged():
    sc = scenario_mod.build(CFG)
    uav = sc.uavs["u1"]
    uav.body.position = np.array([1200.0, 800.0, 300.0])      # out on station
    sc.orchestrator.uav_command("u1", "rtb")
    for _ in range(6):                                        # command rides the link
        sc.world.step()
    assert uav.mode == UavMode.RTB
    decision = next(e for e in sc.world.events
                    if e["kind"] == "decision" and e.get("uav_id") == "u1")
    assert decision["actor"] == "operator"
    assert "rtb" in decision["text"]
    # The order completes into a rearm cycle once home.
    for _ in range(2000):
        sc.world.step()
        if uav.mode == UavMode.REARM:
            break
    assert uav.mode == UavMode.REARM
    assert uav._rtb_ordered is False
