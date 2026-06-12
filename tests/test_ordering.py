"""P0-9: pins for the ordering contract in docs/ORDERING.md
(DESIGN_REVIEW 5.2 doc-side closure).

These freeze the execution-order guarantees the rest of the suite quietly
relies on; a failure here means the contract document is stale and
seed-reproducibility claims need re-verification.
"""

from __future__ import annotations

import copy

from coopuavs.core.bus import MessageBus
from coopuavs.core.node import Node
from coopuavs.sil.clock import MicroScheduler
from coopuavs.sim import scenario as scenario_mod
from coopuavs.sim.environment import Environment
from coopuavs.sim.world import World
from test_end_to_end import SMALL_SCENARIO


def test_bus_delivery_is_synchronous_in_subscription_order():
    bus = MessageBus()
    got = []
    bus.subscribe("topic", lambda m: got.append(("first", m)))
    bus.subscribe("topic", lambda m: got.append(("second", m)))
    bus.publish("topic", "x")
    assert got == [("first", "x"), ("second", "x")]


def test_node_first_tick_at_t0_and_missed_ticks_rebase():
    class Probe(Node):
        def __init__(self, bus):
            super().__init__("probe", bus, rate_hz=1.0)
            self.fired = []

        def update(self, t, dt):
            self.fired.append(t)

    n = Probe(MessageBus())
    n.maybe_update(0.0, 0.05)
    assert n.fired == [0.0]            # first update at t=0
    n.maybe_update(0.5, 0.05)
    assert n.fired == [0.0]            # not due yet
    n.maybe_update(5.0, 0.05)
    assert n.fired == [0.0, 5.0]       # fires once, no catch-up burst
    n.maybe_update(5.5, 0.05)
    assert n.fired == [0.0, 5.0]       # rebased to t=6.0, not 2.0
    n.maybe_update(6.0, 0.05)
    assert n.fired == [0.0, 5.0, 6.0]


def test_scenario_pipeline_order_is_sense_fuse_decide_act_adjudicate_record():
    sc = scenario_mod.build(copy.deepcopy(SMALL_SCENARIO))
    assert [type(n).__name__ for n in sc.world.nodes] == [
        "Radar", "EoIrSensor",                 # sense (ground sensors, YAML order)
        "OnboardSeeker", "OnboardSeeker",      # sense (per interceptor)
        "FusionNode",                          # fuse
        "DebrisReporter",                      # debris picture before C2 plans
        "BaseStation", "Orchestrator",         # decide
        "InterceptorUav", "InterceptorUav",    # act
        "EngagementAdjudicator",               # adjudicate
        "EvalTracker", "Recorder",             # evaluate, record
    ]


def test_micro_seam_runs_before_the_nodes_each_step():
    env = Environment.from_config({
        "bounds": [-100.0, -100.0, 100.0, 100.0],
        "cell_size": 50.0,
        "default_zone": "SAFE",
    })
    world = World(env)
    order = []

    micro = MicroScheduler(world_dt=world.dt, base_hz=20)
    micro.add("probe", 20, lambda now: order.append("micro"))
    world.micro = micro

    class ProbeNode(Node):
        def __init__(self):
            super().__init__("probe-node", world.bus, rate_hz=20.0)

        def update(self, t, dt):
            order.append("node")

    world.add_node(ProbeNode())
    for _ in range(3):
        world.step()
    assert order == ["micro", "node"] * 3
