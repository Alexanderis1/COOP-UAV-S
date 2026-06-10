"""Threat endgame bookkeeping: a leaker is a drone that reaches its target;
a ground impact anywhere else is a crash and must not score as one."""

import numpy as np

from coopuavs.core.messages import ThreatClass
from coopuavs.threats.enemy_drone import EnemyDrone


def make_drone(pos, target, cls=ThreatClass.FPV):
    rng = np.random.default_rng(0)
    return EnemyDrone(
        "e1", cls, np.asarray(pos, dtype=float), np.asarray(target, dtype=float), rng
    )


def test_ground_crash_far_from_target_is_not_a_leaker():
    drone = make_drone([0.0, 0.0, 0.5], [5000.0, 0.0, 0.0])
    drone.body.velocity = np.array([0.0, 0.0, -60.0])
    drone.step(0.05, 0.0)
    assert drone.position[2] <= 0.0
    assert not drone.alive
    assert not drone.reached_target
    assert not drone.killed


def test_reaching_target_scores_leaker():
    drone = make_drone([40.0, 0.0, 20.0], [0.0, 0.0, 0.0])
    for k in range(40):
        drone.step(0.05, k * 0.05)
        if not drone.alive:
            break
    assert not drone.alive
    assert drone.reached_target
