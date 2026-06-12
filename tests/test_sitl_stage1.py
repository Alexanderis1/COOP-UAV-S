"""P4-2 stage-1 velocity passthrough: the tactical agent keeps its FSM and
flies a SITL vehicle over the modeled FCU<->MC coop-link.

``mc/fcu_client.py`` is the MC-side endpoint: ``SitlBody`` keeps the
PointMass duck (``command_velocity`` / ``step`` / ``.position``) so
``InterceptorUav`` is untouched, but commands ride VEL_SP frames to the
FCU in OFFBOARD and the reported state is the NAV **estimate** — the
agent never reads truth (SIM-GT-001). The engine hosts the FCU side of
the wire in the §6 pipeline slot (drain + dispatch at 50 Hz, NAV 25 Hz /
STATUS 10 Hz down), using the P3-R F10 wire enum tables.

Pins: the autonomous arm->OFFBOARD flow over the wire; velocity
passthrough closes on the setpoint; link silence trips the real
LINK_LOSS failsafe (heartbeats only ride the wire now); the
``test_guidance.test_pursuit_converges`` sitl twin (own floor); scenario
sitl build wiring (engine installed, truth adapters in world.friendlies/
adjudicator/seekers/comms, SitlBody on the agent); and a full-pipeline
1-interceptor kill in SITL_SMALL_SCENARIO with the truth quarantine
visibly held (telemetry != truth).
"""

from __future__ import annotations

import numpy as np

from coopuavs.coopfc.fcu import ARMED, OFFBOARD, RTL
from coopuavs.core.rng import RngRegistry
from coopuavs.interceptors.guidance import pursuit_velocity
from coopuavs.mc.fcu_client import FcuClient, SitlBody
from coopuavs.sil.fleet import SitlEngine
from coopuavs.sil.vehicle import FriendlyVehicle
from coopuavs.sim import scenario as scenario_mod
from coopuavs.sim.adjudicator import EngagementAdjudicator

DT = 0.05
# Racer-class quad (T/W 3.6): the conservative 5 m/s default climb limit
# would turn every intercept into a slow elevator ride; the sitl twins
# fly with a representative climb/descent authority via the scenario
# `sitl.fcu` overlay seam (params are data, SIM-RT-004).
FCU_OVERLAY = {"fcu.vel_max_up": 20.0, "fcu.vel_max_down": 20.0}


class _Host:
    """World-shaped harness: micro window [t, t+dt] then the MC node at
    t, 10 Hz cadence — exactly the World.step ordering (§1 items 6-7)."""

    def __init__(self, seed=4, start=(0.0, 0.0, 50.0), max_speed=50.0):
        self.eng = SitlEngine([("u1", (float(start[0]), float(start[1]),
                                       float(start[2])))],
                              RngRegistry(seed), world_dt=DT,
                              heartbeat_hz=0.0, fcu_overlay=FCU_OVERLAY)
        up, down = self.eng.attach_link("u1")
        self.client = FcuClient(up, down)
        self.body = SitlBody(self.client, home=start, max_speed=max_speed,
                             clock=lambda: self.t)
        self.fcu = self.eng.fcus[0]
        self.t = 0.0
        self._steps = 0

    def step(self, v_cmd=(0.0, 0.0, 0.0)):
        self.eng.run_macro_step(self.t, DT)
        if self._steps % 2 == 0:                      # 10 Hz MC node
            self.body.command_velocity(np.asarray(v_cmd, dtype=float))
            self.body.step(0.1)
        self.t += DT
        self._steps += 1

    def run(self, t_span, v_cmd=(0.0, 0.0, 0.0)):
        for _ in range(round(t_span / DT)):
            self.step(v_cmd)

    def arm_offboard(self, t_max=8.0):
        while self.t < t_max:
            self.step()
            if self.fcu.state == ARMED and self.fcu.mode == OFFBOARD:
                return
        raise AssertionError(
            f"no OFFBOARD by {t_max} s: state={self.fcu.state} "
            f"mode={self.fcu.mode} pbit={self.fcu.pbit_reasons}")

    @property
    def truth(self):
        return self.eng.state[0]


def test_client_arms_and_enters_offboard_over_the_wire():
    h = _Host()
    h.arm_offboard()
    # MC's picture caught up through STATUS telemetry
    h.run(0.3)
    assert h.client.state == "ARMED" and h.client.mode == "OFFBOARD"
    # the estimate the agent flies tracks truth to nav-error class
    err = np.linalg.norm(h.body.position - h.truth[0:3])
    assert err < 3.0, f"estimate {err:.2f} m from truth"


def test_velocity_passthrough_closes_on_setpoint():
    h = _Host()
    h.arm_offboard()
    h.run(3.0, v_cmd=(5.0, -3.0, 2.0))
    v = h.truth[3:6]
    assert np.linalg.norm(v - (5.0, -3.0, 2.0)) < 1.0, v
    assert h.fcu.failsafe == ""


def test_link_silence_brings_the_vehicle_home():
    """Heartbeats and setpoints ride the wire now: an MC that stops
    ticking starves both. The setpoint timeout fires first (0.5 s,
    latched as the FIRST failsafe reason — the P3 priority contract),
    then link loss escalates the mode to RTL at 2 s."""
    h = _Host()
    h.arm_offboard()
    h.run(3.0, v_cmd=(10.0, 0.0, 0.0))               # ~30 m from home
    for _ in range(round(2.5 / DT)):                 # wire goes silent
        h.eng.run_macro_step(h.t, DT)
        h.t += DT
        h._steps += 1
    assert h.fcu.failsafe == "OFFBOARD_TIMEOUT"      # first reason latched
    assert h.fcu.mode == RTL                         # link-loss escalation
    # and it is actually flying back toward the arming point
    assert h.truth[3] < 0.0, h.truth[3:6]


def test_pursuit_converges_sitl_twin():
    """test_guidance.test_pursuit_converges through the full stack:
    pursuit commands computed from the NAV estimate, flown by the FCU
    over the link, scored against TRUTH closest approach. Own floor
    (plan P4-6 rule): legacy point-mass pins < 10 m; the sitl twin
    carries nav error + transport lag + real dynamics."""
    h = _Host(seed=7)
    h.arm_offboard()
    target_pos = np.array([600.0, 200.0, 250.0])
    target_vel = np.array([-25.0, 0.0, 0.0])
    closest = 1e9
    for _ in range(round(20.0 / DT)):
        v_cmd = pursuit_velocity(h.body.position, target_pos, target_vel, 50.0)
        h.step(v_cmd)
        target_pos = target_pos + target_vel * DT
        closest = min(closest, float(
            np.linalg.norm(target_pos - h.truth[0:3])))
    assert closest < 10.0, f"closest truth approach {closest:.1f} m"


# ------------------------------------------------------------ scenario wiring

SITL_SMALL_SCENARIO = {
    "name": "sitl-smoke",
    "seed": 11,
    "dt": 0.05,
    "duration": 120.0,
    "fidelity": {"fleet": "sitl"},
    "sitl": {"base_hz": 800,
             "link": {"latency_s": 0.02, "bandwidth_bps": 57600.0},
             "fcu": FCU_OVERLAY},
    "environment": {
        "bounds": [-4000.0, -4000.0, 4000.0, 4000.0],
        "cell_size": 100.0,
        "default_zone": "SAFE",
        "zones": [
            {"rect": [-800, -800, 800, 800], "class": "DANGEROUS"},
            {"rect": [-300, -300, 300, 300], "class": "CRITICAL"},
        ],
        "assets": [
            {"name": "substation", "position": [0.0, 0.0, 0.0], "value": 1.0}
        ],
    },
    "base_station": {"rate_hz": 1.0},
    "sensors": [
        {"type": "radar", "name": "radar-1", "position": [0.0, -1000.0, 10.0],
         "max_range": 9000.0},
        {"type": "eo_ir", "name": "eo-1", "position": [0.0, 0.0, 20.0]},
    ],
    # Homes sit on the FPV approach axis: the intercept happens early
    # and far from the CRITICAL box, so ROE collateral never blocks the
    # re-attack; two shooters give the smoke the legacy scenario's
    # miss-roll margin (the pipeline is the claim, not a hard chase).
    "interceptors": [
        {"id": "u1", "home": [-600.0, 900.0, 0.0], "effector": "projectile",
         "max_speed": 80.0},
        {"id": "u2", "home": [-900.0, 1300.0, 0.0], "effector": "projectile",
         "max_speed": 80.0},
    ],
    # One low-and-slow FPV: inside the fleet's climb envelope, slow
    # enough for a clean stern intercept on the smoke geometry.
    "threats": [
        {"time": 5.0, "class": "FPV",
         "spawn": [-2000.0, 2500.0, 80.0], "target": "substation"},
    ],
}


def test_sitl_scenario_build_wiring():
    from coopuavs.interceptors.uav import SitlShellUav

    sc = scenario_mod.build(SITL_SMALL_SCENARIO)
    world = sc.world
    assert isinstance(world.micro, SitlEngine)
    fv = world.friendlies["u1"]
    assert isinstance(fv, FriendlyVehicle)
    assert fv.tactical is sc.uavs["u1"]
    # stage 2: the agent is a thin shell over a hosted MC (P4-3); its
    # body view is the app's link-backed estimate body
    shell = sc.uavs["u1"]
    assert isinstance(shell, SitlShellUav)
    mcu = world.micro.mcs[world.micro.index["u1"]]
    assert mcu is not None and shell.body is mcu.app.body
    assert isinstance(shell.body, SitlBody)
    assert shell.effector is mcu.app.effector
    # the referee resolves against truth adapters, not the agent
    adj = next(n for n in world.nodes if isinstance(n, EngagementAdjudicator))
    assert adj.uavs["u1"] is fv
    # the radio rides the airframe truth too
    assert world.comms._endpoints["u1"] is fv
    # seekers mount on truth
    from coopuavs.sensors.seeker import OnboardSeeker
    seeker = next(n for n in world.nodes if isinstance(n, OnboardSeeker))
    assert seeker.uav is fv


def test_sitl_small_scenario_kill():
    sc = scenario_mod.build(SITL_SMALL_SCENARIO)
    truth_vs_est = []

    def probe(world):
        fv = world.friendlies["u1"]
        est = sc.uavs["u1"].body.position
        truth_vs_est.append(float(np.linalg.norm(fv.position - est)))

    summary = sc.run(on_step=probe)
    assert summary["kills"] >= 1, summary
    assert summary["wrecks_by_zone"].get("CRITICAL", 0) == 0
    kinds = {e["kind"] for e in sc.world.events}
    assert "kill" in kinds
    # Truth quarantine is visible: the agent flies an estimate that is
    # genuinely distinct from truth (GM wander class), yet bounded.
    diffs = np.asarray(truth_vs_est[100:])           # past boot
    assert diffs.max() > 1e-3, "estimate suspiciously equals truth"
    assert diffs.max() < 10.0, f"nav error {diffs.max():.1f} m unbounded"
