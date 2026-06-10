"""Reactive threat evasion (SIM-THR-003): agile classes dodge interceptors."""

import numpy as np

from coopuavs.core.messages import ThreatClass
from coopuavs.sim.environment import Environment
from coopuavs.sim.world import World
from coopuavs.threats.enemy_drone import EnemyDrone


class FakeInterceptor:
    """Truth-side stand-in: evasion only reads ``.position``."""

    def __init__(self, position):
        self.position = np.asarray(position, dtype=float)


def fly(threat_class: ThreatClass, interceptor_pos=None, steps: int = 100):
    env = Environment.from_config({"bounds": [-3000.0, -3000.0, 3000.0, 3000.0]})
    world = World(env, dt=0.05, seed=4)
    if interceptor_pos is not None:
        world.friendlies["u1"] = FakeInterceptor(interceptor_pos)
    drone = EnemyDrone(
        "e1", threat_class,
        np.array([0.0, 0.0, 80.0]), np.array([2500.0, 0.0, 0.0]),
        world.rng, world=world,
    )
    trajectory = []
    for k in range(steps):
        drone.step(0.05, k * 0.05)
        trajectory.append(drone.position.copy())
    return np.array(trajectory)


def test_fpv_dodges_nearby_interceptor():
    clean = fly(ThreatClass.FPV)
    # Interceptor parked ahead-right of the FPV's path, well inside 275 m.
    dodged = fly(ThreatClass.FPV, interceptor_pos=[120.0, 30.0, 80.0])
    # The trajectory visibly diverges laterally and the FPV sheds altitude.
    lateral_gap = np.max(np.abs(dodged[:, 1] - clean[:, 1]))
    assert lateral_gap > 20.0
    assert dodged[-1, 2] < clean[-1, 2] - 5.0


def test_loitering_also_evades_but_owa_does_not():
    assert np.max(np.abs(
        fly(ThreatClass.LOITERING, interceptor_pos=[120.0, 30.0, 80.0])[:, 1]
        - fly(ThreatClass.LOITERING)[:, 1]
    )) > 10.0
    # Strategic OWA flies its programmed route regardless.
    assert np.array_equal(
        fly(ThreatClass.OWA_STRATEGIC, interceptor_pos=[120.0, 30.0, 80.0]),
        fly(ThreatClass.OWA_STRATEGIC),
    )


def test_far_interceptor_changes_nothing():
    assert np.array_equal(
        fly(ThreatClass.FPV, interceptor_pos=[2000.0, 2000.0, 200.0]),
        fly(ThreatClass.FPV),
    )
