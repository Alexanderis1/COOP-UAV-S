"""P5-5 FCU-side hard fire interlock (decision 3: release via FCU;
PHY-UAV-021/033).

The MC stages its FireRequest and commands WEAPON_RELEASE over the
coop-link; the FCU releases only ARMED, CBIT-clean, against the
mirrored clearance token for THAT track inside the freshness window —
then pulses its effector HAL port. The engine turns the pulse into a
release ack; the world-side shell publishes the staged request to the
bus only on the matching ack (one ack = one release) and restores the
round on NACK-by-timeout. Token freshness is compared in the MC clock
domain only (release.stamp - token.issued): the FCU clock is
boot-relative and never comparable.

Wire round-trips for CLEARANCE_TOKEN/WEAPON_RELEASE ride the registry
sweep in test_coopfc_link.py.
"""

from __future__ import annotations

import sys

import numpy as np

sys.path.insert(0, "tests")

from coopuavs.coopfc.fcu import ARMED, FCU_DEFAULTS
from coopuavs.core.bus import MessageBus
from coopuavs.core.messages import (
    EngagementDecision,
    Header,
    Track,
    TrackArray,
)
from coopuavs.interceptors.uav import RELEASE_TIMEOUT_S, SitlShellUav
from coopuavs.mc.fire_control import CLEARANCE_VALID_S
from test_coopfc_fcu import SynthHost
from test_sitl_stage2 import DT, _clr, _hosted_engine, _task

VALID_S = FCU_DEFAULTS["fcu.release_token_valid_s"]


# ------------------------------------------------ the one freshness window

def test_release_validity_window_matches_the_mc_interlock():
    # Two ends of ONE window: the MC discards tokens it would not
    # consume, the FCU refuses releases against tokens it considers
    # expired — a disagreement silently widens or deadlocks the chain.
    assert VALID_S == CLEARANCE_VALID_S


# ------------------------------------------------------ FCU unit: refusals

def _armed_host() -> SynthHost:
    h = SynthHost()
    h.boot_and_arm()
    assert h.fcu.state == ARMED
    return h


def test_release_refused_when_not_armed():
    h = SynthHost()
    h.run(2.6, hb_every=0.1)               # STANDBY, PBIT green, not armed
    ok, why = h.fcu.cmd_weapon_release(1.0, 1)
    assert (ok, why) == (False, "NOT_ARMED")
    assert h.fcu.effector_port.read() == (0, None)       # no pulse
    assert h.fcu.release_refusals == {"NOT_ARMED": 1}


def test_release_refused_without_token():
    h = _armed_host()
    ok, why = h.fcu.cmd_weapon_release(1.0, 1)
    assert (ok, why) == (False, "NO_TOKEN")
    assert h.fcu.effector_port.read() == (0, None)
    assert h.fcu.release_refusals == {"NO_TOKEN": 1}


def test_release_refused_on_track_mismatch_token_not_consumed():
    h = _armed_host()
    h.fcu.cmd_clearance_token(2, 10.0)
    ok, why = h.fcu.cmd_weapon_release(10.1, 1)
    assert (ok, why) == (False, "TOKEN_MISMATCH")
    # the mismatched attempt did not burn track 2's token
    ok, why = h.fcu.cmd_weapon_release(10.2, 2)
    assert ok, why
    assert h.fcu.release_refusals == {"TOKEN_MISMATCH": 1}


def test_release_refused_on_stale_token():
    h = _armed_host()
    h.fcu.cmd_clearance_token(1, 10.0)
    ok, why = h.fcu.cmd_weapon_release(10.0 + VALID_S + 0.01, 1)
    assert (ok, why) == (False, "TOKEN_STALE")
    assert h.fcu.effector_port.read() == (0, None)
    # at the window edge the same token still releases (stale refusal
    # does not consume)
    ok, why = h.fcu.cmd_weapon_release(10.0 + VALID_S, 1)
    assert ok, why


def test_release_refused_under_cbit_inhibit_even_with_valid_token():
    h = _armed_host()
    h.fcu.params._values["fcu.pos_kp"] = 999.0      # bit-rot -> PARAM_CRC
    h.run(2.0, hb_every=0.1)                        # 1 Hz slow monitor
    assert h.fcu.cbit.inhibit_fire
    h.fcu.cmd_clearance_token(1, h.now)
    ok, why = h.fcu.cmd_weapon_release(h.now + 0.1, 1)
    assert (ok, why) == (False, "CBIT_INHIBIT")
    assert h.fcu.effector_port.read() == (0, None)


def test_release_pulses_effector_and_consumes_token():
    h = _armed_host()
    h.fcu.cmd_clearance_token(7, 100.0)
    ok, why = h.fcu.cmd_weapon_release(100.5, 7)
    assert ok, why
    assert h.fcu.effector_port.read() == (1, (7, 100.5))
    assert h.fcu.releases == 1 and h.fcu.release_refusals == {}
    # one token = one release
    ok, why = h.fcu.cmd_weapon_release(100.6, 7)
    assert (ok, why) == (False, "NO_TOKEN")
    assert h.fcu.effector_port.read()[0] == 1            # still one pulse


# --------------------------------- hosted engine + shell + bus, end to end

def _shelled(seed=9):
    """test_sitl_stage2 hosted engine + the world-side shell on a bus."""
    eng, mcu = _hosted_engine(seed)
    bus = MessageBus()
    out = {"requests": [], "fires": []}
    bus.subscribe("engagement/fire_request", out["requests"].append)
    bus.subscribe("engagement/fire", out["fires"].append)
    shell = SitlShellUav("u1", bus, np.array([0.0, 0.0, 60.0]),
                         mcu.app.effector, mcu=mcu, max_speed=50.0)
    return eng, bus, shell, out


def _drive(eng, shell, t, t_span):
    for m in range(round(t_span / DT)):
        tm = t + m * DT
        eng.run_macro_step(tm, DT)
        shell.update(tm, DT)
    return t + round(t_span / DT) * DT


def _engage_until_request(eng, bus, shell, out):
    """Arm + chase a slow crosser kept near optimal range until the
    interlock asks for clearance (fresh track republished each chunk —
    the fusion picture the app would normally get)."""
    t = _drive(eng, shell, 0.0, 6.0)
    assert eng.fcus[0].state == ARMED
    bus.publish("engagement/tasks", [_task(track_id=1, task_id=1)])
    deadline = t + 30.0
    while not out["requests"] and t < deadline:
        own = eng.state[0, 0:3]
        bus.publish("tracks", TrackArray(header=Header(stamp=t), tracks=[
            Track(header=Header(stamp=t), track_id=1,
                  position=own + np.array([80.0, 0.0, 0.0]),
                  velocity=np.array([-5.0, 0.0, 0.0]))]))
        t = _drive(eng, shell, t, 0.5)
    assert out["requests"], "engagement never reached the request stage"
    return t


def test_release_via_fcu_happy_path():
    eng, bus, shell, out = _shelled()
    capacity = shell.effector.ammo
    t = _engage_until_request(eng, bus, shell, out)
    bus.publish("engagement/clearance", _clr(
        track_id=1, task_id=1, decision=EngagementDecision.AUTHORIZED, t=t))
    t = _drive(eng, shell, t, 2.0)
    # exactly one round left the rail, through the FCU, payload intact
    assert len(out["fires"]) == 1
    fire = out["fires"][0]
    assert fire.track_id == 1 and fire.uav_id == "u1"
    assert fire.p_kill >= 0.30                  # the release-floor gate
    assert shell.effector.ammo == capacity - 1
    assert shell.release_refused == 0
    fcu = eng.fcus[0]
    assert fcu.releases == 1 and fcu.release_refusals == {}


def test_lost_release_times_out_and_restores_the_round():
    eng, bus, shell, out = _shelled()
    capacity = shell.effector.ammo
    mcu_client = shell._mcu.app._client
    # the WEAPON_RELEASE command dies on the wire (SIM-COM-003 class)
    mcu_client.send_weapon_release = lambda track_id: None
    t = _engage_until_request(eng, bus, shell, out)
    bus.publish("engagement/clearance", _clr(
        track_id=1, task_id=1, decision=EngagementDecision.AUTHORIZED, t=t))
    t = _drive(eng, shell, t, RELEASE_TIMEOUT_S + 1.0)
    # no pulse, no fire on the bus; the round never left the rail
    assert out["fires"] == []
    assert shell.release_refused == 1
    assert shell.effector.ammo == capacity
    assert eng.fcus[0].releases == 0
