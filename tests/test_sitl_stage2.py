"""P4-3 stage-2 MC split: the tactical stack on a VirtualMCU.

Clearance-interlock sitl twins (byte-equivalent): the same message
script drives the legacy ``InterceptorUav`` over the bus AND the
``mc/interceptor_app.py InterceptorApp`` over its mailboxes; every
emitted FireRequest (request and release) must compare byte-equal,
field by field, and the interlock state must match. Both hosts drive
the one ``mc/fire_control.FireControl`` machine — the twins guard the
ported update-flow around it (mode machine, task handling, publish
points) against drift.

Engine integration: the MCU ticks in the §6 step-3 slot on the micro
clock, arms its own FCU over the wire from inside the loop, and flies
mailbox-fed tasking. The crash fence is exercised end-to-end: a dying
MC app goes silent, the FCU starves of heartbeats/setpoints and flies
itself home (SIM-SIL-003 — a processor crash is a fault mode, not a
simulation error).
"""

from __future__ import annotations

import numpy as np

from coopuavs.coopfc.fcu import ARMED, OFFBOARD, RTL
from coopuavs.core.bus import MessageBus
from coopuavs.core.messages import (
    EngagementDecision,
    EngagementTask,
    FireClearance,
    Header,
    Track,
    TrackArray,
    UavMode,
)
from coopuavs.core.rng import RngRegistry
from coopuavs.interceptors.effectors import projectile_gun
from coopuavs.interceptors.uav import InterceptorUav
from coopuavs.mc.fcu_client import FcuClient
from coopuavs.mc.interceptor_app import InterceptorApp
from coopuavs.sil.fleet import SitlEngine
from coopuavs.sil.host import VirtualMCU

# ----------------------------------------------------------------- twins

POS = np.array([0.0, 0.0, 300.0])
VEL = np.array([50.0, 0.0, 0.0])


class _StubClient:
    """No wire: the twin pins tactical logic, not transport. Telemetry
    reads as a healthy pack so the energy branch matches the legacy
    host's full battery (and a clean CBIT/health word, P5-1e/P5-4)."""
    nav = None
    status = None
    state = "ARMED"
    batt_frac = 1.0
    failsafe = ""
    desired_mode = "OFFBOARD"
    hold_arm = False
    cbit_inhibit_fire = False
    cbit_inhibit_arming = False
    cbit_degraded = ""
    fault_word = 0

    def tick(self, now, v_cmd, yaw_sp=0.0):
        pass

    def request_batt_reset(self):
        pass


def _task(track_id, task_id):
    return EngagementTask(header=Header(stamp=0.0), task_id=task_id,
                          track_id=track_id, shooter_id="u1")


def _tracks(track_id, t=0.0):
    return TrackArray(header=Header(stamp=t), tracks=[Track(
        header=Header(stamp=t), track_id=track_id,
        position=np.array([100.0, 0.0, 300.0]),
        velocity=np.array([10.0, 0.0, 0.0]),
    )])


def _clr(track_id, task_id, decision, t=0.0):
    return FireClearance(header=Header(stamp=t), task_id=task_id,
                         uav_id="u1", track_id=track_id, decision=decision)


class _LegacyHost:
    def __init__(self):
        self.bus = MessageBus()
        self.requests, self.fires = [], []
        self.bus.subscribe("engagement/fire_request", self.requests.append)
        self.bus.subscribe("engagement/fire", self.fires.append)
        self.uav = InterceptorUav("u1", self.bus, home=np.zeros(3),
                                  effector=projectile_gun(), max_speed=80.0)
        self.uav.body.position = POS.copy()
        self.uav.body.velocity = VEL.copy()

    def post(self, kind, msg):
        topic = {"tasks": "engagement/tasks", "tracks": "tracks",
                 "clearance": "engagement/clearance"}[kind]
        self.bus.publish(topic, msg)

    def step(self, t):
        self.uav.update(t, 0.1)
        # scripted kinematics: both twins fly the same frozen state, so
        # every pk/geometry input is identical by construction
        self.uav.body.position = POS.copy()
        self.uav.body.velocity = VEL.copy()

    @property
    def clearance(self):
        return self.uav._clearance

    @property
    def mode(self):
        return self.uav.mode


class _AppHost:
    def __init__(self):
        from coopuavs.core.ports import Ports
        from coopuavs.sil.clock import VirtualClock
        self.ports = Ports()
        clock = VirtualClock(10)
        self.app = InterceptorApp(clock, None, self.ports, uav_id="u1",
                                  home=np.zeros(3), effector=projectile_gun(),
                                  fcu_client=_StubClient(), max_speed=80.0)
        self.app.body.position = POS.copy()
        self.app.body.velocity = VEL.copy()
        self._clock = clock

    def post(self, kind, msg):
        self.ports.box(kind).post(msg)

    def step(self, t):
        # drive the app at the scripted time (the twin compares logic,
        # not clock plumbing)
        self._clock.tick = round(t * 10)
        self.app.tick(t)
        # the stub body holds the scripted state (no NAV updates)
        self.app.body.position = POS.copy()
        self.app.body.velocity = VEL.copy()

    @property
    def requests(self):
        return self.ports.box("fire_request").drain()

    @property
    def fires(self):
        return self.ports.box("fire").drain()

    @property
    def clearance(self):
        return self.app._fc.clearance

    @property
    def mode(self):
        return self.app.mode


def _byte_equal(a, b):
    """Field-by-field FireRequest equality (np arrays compared exactly)."""
    assert type(a) is type(b)
    for f in ("task_id", "uav_id", "track_id", "effector", "p_kill",
              "target_kind", "debris_id"):
        assert getattr(a, f) == getattr(b, f), f
    assert a.header.stamp == b.header.stamp
    np.testing.assert_array_equal(a.predicted_intercept, b.predicted_intercept)


def _run_script(script):
    """Drive both hosts with the same script; return (legacy, app)."""
    legacy, app = _LegacyHost(), _AppHost()
    for entry in script:
        if entry[0] == "step":
            legacy.step(entry[1])
            app.step(entry[1])
        else:
            kind, msg = entry
            legacy.post(kind, msg)
            app.post(kind, msg)
    return legacy, app


def _assert_twin(legacy, app):
    app_requests, app_fires = app.requests, app.fires
    assert len(legacy.requests) == len(app_requests)
    assert len(legacy.fires) == len(app_fires)
    for a, b in zip(legacy.requests, app_requests):
        _byte_equal(a, b)
    for a, b in zip(legacy.fires, app_fires):
        _byte_equal(a, b)
    assert (legacy.clearance is None) == (app.clearance is None)
    assert legacy.mode == app.mode


def test_twin_token_for_another_track_never_releases():
    legacy, app = _run_script([
        ("tasks", [_task(track_id=1, task_id=7)]),
        ("tracks", _tracks(1)),
        ("clearance", _clr(track_id=2, task_id=3,
                           decision=EngagementDecision.AUTHORIZED)),
        ("step", 0.0),
        ("clearance", _clr(track_id=1, task_id=7,
                           decision=EngagementDecision.AUTHORIZED)),
        ("step", 0.1),
    ])
    assert len(legacy.fires) == 1          # matching token released
    _assert_twin(legacy, app)


def test_twin_stale_token_discarded():
    from coopuavs.mc.fire_control import CLEARANCE_VALID_S
    legacy, app = _run_script([
        ("tasks", [_task(track_id=1, task_id=7)]),
        ("tracks", _tracks(1)),
        ("clearance", _clr(track_id=1, task_id=7,
                           decision=EngagementDecision.AUTHORIZED)),
        ("step", CLEARANCE_VALID_S + 2.0),
    ])
    assert legacy.fires == []
    assert legacy.clearance is None
    _assert_twin(legacy, app)


def test_twin_retask_invalidates_state():
    legacy, app = _run_script([
        ("tasks", [_task(track_id=1, task_id=7)]),
        ("tracks", _tracks(1)),
        ("clearance", _clr(track_id=1, task_id=7,
                           decision=EngagementDecision.AUTHORIZED)),
        ("tasks", [_task(track_id=2, task_id=8)]),
        ("tracks", _tracks(2)),
        ("step", 0.1),
    ])
    assert legacy.fires == []
    assert legacy.clearance is None        # re-requests, does not fire
    _assert_twin(legacy, app)


def test_twin_denied_aborts_task():
    legacy, app = _run_script([
        ("tasks", [_task(track_id=1, task_id=7)]),
        ("tracks", _tracks(1)),
        ("clearance", _clr(track_id=1, task_id=7,
                           decision=EngagementDecision.DENIED)),
        ("step", 0.0),
    ])
    assert legacy.fires == []
    assert legacy.uav._task is None
    assert app.app._task is None
    _assert_twin(legacy, app)


# ------------------------------------------------------- engine integration

DT = 0.05
FCU_OVERLAY = {"fcu.vel_max_h": 80.0, "fcu.vel_max_up": 20.0,
               "fcu.vel_max_down": 20.0}


def _hosted_engine(seed=9):
    eng = SitlEngine([("u1", (0.0, 0.0, 60.0))], RngRegistry(seed),
                     world_dt=DT, heartbeat_hz=0.0, fcu_overlay=FCU_OVERLAY)
    up, down = eng.attach_link("u1")
    client = FcuClient(up, down)

    def factory(clock, rng, ports):
        return InterceptorApp(clock, rng, ports, uav_id="u1",
                              home=np.array([0.0, 0.0, 60.0]),
                              effector=projectile_gun(), fcu_client=client,
                              max_speed=50.0)

    mcu = VirtualMCU("mc/u1", tick_hz=10, base_hz=800,
                     app_factory=factory, rng=None)
    eng.attach_mc("u1", mcu)
    return eng, mcu


def _run(eng, t0, t_span):
    for m in range(round(t_span / DT)):
        eng.run_macro_step(t0 + m * DT, DT)
    return t0 + round(t_span / DT) * DT


def test_mc_app_arms_and_flies_from_inside_the_loop():
    """The hosted MC arms its FCU over the wire from the §6 step-3 slot
    and flies mailbox-fed tasking — no world, no bus."""
    eng, mcu = _hosted_engine()
    t = _run(eng, 0.0, 6.0)
    fcu = eng.fcus[0]
    assert fcu.state == ARMED and fcu.mode == OFFBOARD
    assert not mcu.crashed

    # task it against a slow crosser 400 m out
    mcu.ports.box("tasks").post([_task(track_id=1, task_id=1)])
    mcu.ports.box("tracks").post(TrackArray(header=Header(stamp=t), tracks=[
        Track(header=Header(stamp=t), track_id=1,
              position=np.array([400.0, 0.0, 120.0]),
              velocity=np.array([-15.0, 0.0, 0.0]))]))
    _run(eng, t, 3.0)
    assert mcu.app.mode in (UavMode.PURSUIT, UavMode.ENGAGE)
    # truth is actually chasing: closing speed toward the target
    v = eng.state[0, 3:6]
    assert v[0] > 5.0, v
    # telemetry flows out of the box for the world-side shell
    states = mcu.ports.box("uav_state").drain()
    assert states and states[-1].mode in (UavMode.PURSUIT, UavMode.ENGAGE)


def test_mc_crash_goes_silent_and_fcu_flies_home():
    """SIM-SIL-003: the app dies mid-flight; the MCU latches, the link
    starves, the FCU failsafes home. The sim never sees the exception."""
    eng, mcu = _hosted_engine()
    t = _run(eng, 0.0, 6.0)
    assert eng.fcus[0].state == ARMED

    def bomb(now):
        raise MemoryError("bitflip")
    mcu.app.tick = bomb
    _run(eng, t, 3.0)
    assert mcu.crashed and "MemoryError" in mcu.crash_reason
    fcu = eng.fcus[0]
    assert fcu.failsafe in ("OFFBOARD_TIMEOUT", "LINK_LOSS")
    # The autonomous chain after a dead MC: RTL home, LAND from the
    # loiter altitude, touchdown + disarm — wherever the window caught
    # it, it is on that chain and no longer accepting the silence.
    assert (fcu.mode in (RTL, "LAND")
            or (fcu.state == "STANDBY" and fcu.touchdown))
