"""P5-1c: CBIT command authority — degraded modes, arming gate, HEALTH wire.

The failsafe priority chain, pinned (PLAN_PROBLEM1 P5 header):

    FAILSAFE_ATT (nav-loss) > BATT_CRIT > CBIT LAND > LINK_LOSS >
    BATT_LOW > CBIT RTL > OFFBOARD_TIMEOUT

The first-reason latch is the P3 contract: a later escalation switches
the MODE but never rewrites the latched ``failsafe`` reason. Mirror
rows (BATT/LINK) keep their legacy responses byte-identical — CBIT only
adds commands for faults nothing else handles.

FAILSAFE_ATT is rate-only flight: on a diverged estimator the position
and velocity loops are flying a fiction, so the FCU drops to gyro rate
damping at a fixed sub-hover thrust — a controlled, level-ish descent
that needs no nav solution at all (quad physics; what an attitude
failsafe can honestly promise).
"""

from __future__ import annotations

import numpy as np

from coopuavs.coopfc.cbit import ACT_NONE, FAULTS
from coopuavs.coopfc.fcu import (
    ARMED, FAILSAFE_ATT, LAND, OFFBOARD, POS_HOLD, RTL,
)
from coopuavs.coopfc.link.coop_link import (
    DEGRADED_CODES,
    DEGRADED_NAMES,
    FAILSAFE_CODES,
    MODE_CODES,
    Channel,
    FrameDecoder,
    decode_msg,
    encode_msg,
)
from coopuavs.mc.fcu_client import FcuClient
from coopuavs.sil.bench import Bench

from test_coopfc_cbit_monitors import CbitHost, armed_host


# ------------------------------------------------------------ wire tables

def test_wire_tables_cover_the_dictionary():
    # Every non-mirror fault that commands a response must be a legal
    # STATUS failsafe reason; the degraded-mode wire vocabulary matches
    # the dictionary's action vocabulary.
    for code, spec in FAULTS.items():
        if spec.degraded_mode != ACT_NONE and not spec.mirror:
            assert code in FAILSAFE_CODES, code
    assert set(DEGRADED_CODES) == {"", "RTL", "LAND", "FAILSAFE_ATT"}
    assert {v: k for k, v in DEGRADED_CODES.items()} == DEGRADED_NAMES
    assert FAILSAFE_ATT in MODE_CODES


def test_health_message_round_trip():
    frame = encode_msg("HEALTH", 2.5, (1 << 9) | (1 << 20), 0b11,
                       DEGRADED_CODES["FAILSAFE_ATT"])
    ((mid, payload),) = FrameDecoder().feed(frame)
    name, f = decode_msg(mid, payload)
    assert name == "HEALTH"
    assert f["faults"] == (1 << 9) | (1 << 20)
    assert f["flags"] == 0b11
    assert DEGRADED_NAMES[f["degraded"]] == "FAILSAFE_ATT"


def test_fcu_client_surfaces_health():
    up, down = Channel(0.0, 1e9), Channel(0.0, 1e9)
    client = FcuClient(up, down)
    assert client.fault_word == 0 and not client.cbit_inhibit_fire
    down.send(encode_msg("HEALTH", 1.0, 1 << 11, 0b10,
                         DEGRADED_CODES["LAND"]), 0.0)
    client.poll(1.0)
    assert client.fault_word == 1 << 11
    assert client.cbit_inhibit_fire and not client.cbit_inhibit_arming
    assert client.cbit_degraded == "LAND"


# ------------------------------------------------------- degraded actions

def test_ekf_diverged_enters_failsafe_att():
    h = armed_host()
    h.fcu.ekf.diverged = True
    h.run(0.2)
    assert h.fcu.mode == FAILSAFE_ATT
    assert h.fcu.failsafe == "EKF_DIVERGED"
    ok, why = h.fcu.cmd_set_mode(OFFBOARD)
    assert not ok and "FAILSAFE_ATT" in why
    # Motors alive: rate-only flight, not a cutoff.
    _, u = h.hal.port("actuators").read()
    assert sum(u) > 0.0


def test_failsafe_att_rate_damped_descent():
    b = Bench(seed=3)
    b.boot_and_arm()
    b.run(3.0)                                  # settle the hover
    b.fcu.ekf.diverged = True
    b.run(1.0)
    assert b.fcu.mode == FAILSAFE_ATT
    z0 = b.state[0, 2]
    b.run(2.0)
    assert b.state[0, 2] < z0 - 0.5             # descending
    assert b.state[0, 5] > -8.0                 # not ballistic
    assert float(np.max(np.abs(b.state[0, 10:13]))) < 1.0   # rate-damped


def test_motor_fault_lands_with_reason():
    h = armed_host()
    h.run(1.0)
    h.esc_rpm = (6000.0, 6000.0, 4500.0, 6000.0)
    h.run(1.5)
    assert h.fcu.cbit.raised("MOTOR_RESPONSE")
    assert h.fcu.mode == LAND
    assert h.fcu.failsafe == "MOTOR_RESPONSE"


def test_priority_nav_loss_beats_batt_crit_mode_but_not_reason():
    h = armed_host()
    h.v_cell = 3.20
    h.run(1.5)
    assert h.fcu.mode == LAND and h.fcu.failsafe == "BATT_CRIT"
    h.fcu.ekf.diverged = True
    h.run(0.2)
    assert h.fcu.mode == FAILSAFE_ATT           # nav-loss outranks LAND
    assert h.fcu.failsafe == "BATT_CRIT"        # first-reason latch (P3)


def test_priority_motor_land_escalates_batt_low_rtl():
    h = armed_host()
    h.v_cell = 3.45
    h.run(1.5)
    assert h.fcu.mode == RTL and h.fcu.failsafe == "BATT_LOW"
    h.esc_rpm = (6000.0, 6000.0, 4500.0, 6000.0)
    h.run(1.5)
    assert h.fcu.mode == LAND                   # get-down-now beats come-home
    assert h.fcu.failsafe == "BATT_LOW"         # first-reason latch


def test_dr_budget_commands_rtl():
    h = armed_host(overlay={"fcu.dr_sigma_budget_m": 1.5})
    h.gps_on = False
    h.run(25.0)
    assert h.fcu.cbit.raised("DR_BUDGET_LOW")
    assert h.fcu.mode in (RTL, LAND)            # LAND once over home
    assert h.fcu.failsafe == "DR_BUDGET_LOW"


def test_inhibit_arming_gates_cmd_arm():
    h = CbitHost()
    h.gyro_freeze = (0.0, 0.0, 0.0)             # stuck-at-zero from boot
    h.run(2.6)
    assert h.fcu.state != ARMED
    assert h.fcu.cbit.raised("GYRO_STUCK")
    ok, why = h.fcu.cmd_arm()
    assert not ok and "GYRO_STUCK" in why
    h.gyro_freeze = None
    h.run(0.5)
    h.fcu.cbit.clear("GYRO_STUCK")              # ground maintenance
    ok, why = h.fcu.cmd_arm()
    assert ok, why


def test_no_fault_chain_unchanged():
    h = armed_host()
    h.run(3.0)
    assert h.fcu.mode == POS_HOLD and h.fcu.failsafe == ""
    assert h.fcu.cbit.degraded() == ("", "")


# ------------------------------------------------------- engine HEALTH 1 Hz

def test_sitl_engine_streams_health():
    from coopuavs.core.rng import RngRegistry
    from coopuavs.sil.fleet import SitlEngine

    eng = SitlEngine([("u1", (0.0, 0.0, 50.0))], RngRegistry(7))
    up, down = eng.attach_link("u1")
    client = FcuClient(up, down)
    t = 0.0
    for _ in range(round(4.0 / 0.05)):
        client.tick(t, (0.0, 0.0, 0.0))
        eng.run_macro_step(t, 0.05)
        t += 0.05
    assert client.health is not None            # 1 Hz northbound
    assert client.fault_word == 0               # healthy boot


# ------------------------------------ P5-1d mag exclusion (user decision)

def test_mag_fault_excludes_mag_from_fusion():
    """Disable-mag fallback (2026-06-12 decision): a latched MAG_FAULT
    removes the corrupted yaw source; yaw rides the gyro + the existing
    GPS-maneuver observability pathway, and the yaw variance honestly
    grows instead of averaging a lie."""
    h = armed_host()
    h.mag_swapped = True
    h.run(2.0)
    assert h.fcu.cbit.raised("MAG_FAULT")
    assert h.fcu.ekf.mag_trusted is False
    rej0 = h.fcu.ekf.rejected["mag"]
    exc0 = h.fcu.ekf.mag_excluded
    p0 = h.fcu.ekf.P[8, 8]
    h.run(3.0)
    assert h.fcu.ekf.mag_excluded > exc0        # excluded at intake
    assert h.fcu.ekf.rejected["mag"] == rej0    # reject spam stopped
    assert h.fcu.ekf.P[8, 8] > p0               # sigma honesty: yaw grows
    h.mag_swapped = False
    h.run(1.0)
    assert h.fcu.ekf.mag_trusted is False       # latched per flight


def test_mag_exclusion_reapplied_to_rebuilt_ekf():
    # The touchdown recalibration path builds a NEW Ekf; the latched
    # MAG_FAULT must re-apply the exclusion to it.
    h = armed_host()
    h.mag_swapped = True
    h.run(2.0)
    assert h.fcu.cbit.raised("MAG_FAULT")
    h.fcu.cmd_disarm()
    h.fcu.ekf = None                            # the touchdown block
    h.fcu.align_result = None
    h.fcu._aligner = h.fcu._new_aligner()
    h.run(3.0)
    assert h.fcu.ekf is not None                # realigned
    assert h.fcu.ekf.mag_trusted is False       # exclusion re-applied


# ------------------------------------------- P5-1e inhibit_fire end-to-end

def test_inhibit_fire_suppresses_staged_release():
    """The staged-request scenario the plan names: the interlock is
    mid-engagement (request out, token pending) when the FCU's health
    word vetoes release — the AUTHORIZED token that then arrives must
    not fire, and release resumes only while the token is still fresh
    after the veto clears."""
    from coopuavs.core.messages import EngagementDecision
    from test_sitl_stage2 import _AppHost, _clr, _task, _tracks

    host = _AppHost()
    host.post("tasks", [_task(track_id=1, task_id=7)])
    host.post("tracks", _tracks(1))
    host.step(0.0)                              # in envelope: request staged
    assert host.app._fc.await_clearance
    assert len(host.requests) == 1

    host.app._client.cbit_inhibit_fire = True   # fault latches FCU-side
    host.post("clearance", _clr(track_id=1, task_id=7,
                                decision=EngagementDecision.AUTHORIZED))
    host.step(0.1)
    host.step(0.2)
    assert host.fires == []                     # token held, not consumed
    assert host.app._fc.clearance is not None

    host.app._client.cbit_inhibit_fire = False  # cleared, token still fresh
    host.step(0.3)
    assert len(host.fires) == 1                 # release resumes


def test_inhibit_fire_end_to_end_over_the_wire():
    """Full chain: a real FCU fault (param CRC) -> CBIT inhibit_fire ->
    HEALTH frame over the coop-link -> FcuClient -> app refuses to fire
    even holding an AUTHORIZED token."""
    from coopuavs.core.messages import EngagementDecision, Header, Track, TrackArray
    from test_sitl_stage2 import _clr, _hosted_engine, _run, _task

    eng, mcu = _hosted_engine()
    t = _run(eng, 0.0, 6.0)                     # boot + arm + OFFBOARD
    fcu = eng.fcus[0]
    assert fcu.state == ARMED

    fcu.params._values["fcu.pos_kp"] = 999.0    # simulated bit-rot
    t = _run(eng, t, 3.0)                       # CBIT raise + HEALTH + poll
    assert fcu.cbit.raised("PARAM_CRC")
    assert mcu.app._client.cbit_inhibit_fire

    # Task it against a target inside the envelope, hand it a valid token.
    own = eng.state[0, 0:3]
    mcu.ports.box("tasks").post([_task(track_id=1, task_id=1)])
    mcu.ports.box("tracks").post(TrackArray(header=Header(stamp=t), tracks=[
        Track(header=Header(stamp=t), track_id=1,
              position=own + np.array([60.0, 0.0, 0.0]),
              velocity=np.array([-5.0, 0.0, 0.0]))]))
    mcu.ports.box("clearance").post(_clr(track_id=1, task_id=1,
                                         decision=EngagementDecision.AUTHORIZED))
    _run(eng, t, 2.0)
    assert mcu.ports.box("fire").drain() == []
    assert mcu.ports.box("fire_request").drain() == []   # chain fully frozen
