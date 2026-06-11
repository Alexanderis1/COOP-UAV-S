"""P1-7 @oracle: our multirotor plant vs committed RotorPy traces.

Replays the motor-speed command schedule recorded in each CSV (single
source of truth) through our plant at 800 Hz with the same first-order
motor lag (exact exponential, thrust uses the within-step mean speed) and
compares trajectories over 10 s: position RMSE < 0.5 m, attitude geodesic
RMSE < 3 deg. Both sides drag-free per the matched-parameter scoping
documented in scripts/oracle/export_rotorpy.py (drag models differ by
design and are pinned by our own unit tests). Run with `pytest -m oracle`.
"""

from __future__ import annotations

import dataclasses
from pathlib import Path

import numpy as np
import pytest

from coopuavs.physics import rigid_body as rb
from coopuavs.physics.multirotor import MultirotorParams, MultirotorPlant
from coopuavs.physics.params import load_airframe

ORACLE_DIR = Path(__file__).parent / "fixtures" / "oracle"
FLIGHTS = ["hover_hold", "climb_step", "tilt_dash", "yaw_spin", "pitch_pulse"]
TAU_M = 0.05          # must match the exporter
DT_MICRO = 1.0 / 800.0
SUBSTEPS = 8          # 100 Hz command rows -> 8 micro-steps each


def dragfree_params() -> MultirotorParams:
    base = MultirotorParams.from_dict(load_airframe("interceptor_quad"))
    return dataclasses.replace(base, drag_linear_diag=np.zeros(3), cda_iso=0.0)


def quat_angle_deg(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    dot = np.clip(np.abs(np.sum(q1 * q2, axis=-1)), 0.0, 1.0)
    return np.rad2deg(2.0 * np.arccos(dot))


def replay(rows: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Integrate our plant under the CSV command schedule; returns (pos, quat)
    sampled at the CSV row times."""
    plant = MultirotorPlant(dragfree_params(), 1)
    state = np.zeros((1, rb.STATE_DIM))
    state[0, rb.POS] = rows[0, 1:4]
    state[0, rb.VEL] = rows[0, 4:7]
    state[0, rb.QUAT] = rows[0, 7:11]
    state[0, rb.OMEGA] = rows[0, 11:14]
    rotor = rows[0, 14:18].copy()

    decay = np.exp(-DT_MICRO / TAU_M)
    mean_w = (TAU_M / DT_MICRO) * (1.0 - decay)

    pos = np.empty((len(rows), 3))
    quat = np.empty((len(rows), 4))
    pos[0], quat[0] = state[0, rb.POS], state[0, rb.QUAT]
    for k in range(len(rows) - 1):
        cmd = rows[k, 18:22]
        for _ in range(SUBSTEPS):
            # exact ZOH of the motor lag; thrust sees the within-step mean speed
            w_eff = cmd + (rotor - cmd) * mean_w
            rotor = cmd + (rotor - cmd) * decay
            state = plant.step(state, DT_MICRO, w_eff[None, :],
                               np.zeros((1, 3)), 1.225)
        pos[k + 1], quat[k + 1] = state[0, rb.POS], state[0, rb.QUAT]
    return pos, quat


@pytest.mark.oracle
@pytest.mark.parametrize("flight", FLIGHTS)
def test_rotorpy_trace_match(flight):
    path = ORACLE_DIR / f"rotorpy_{flight}.csv"
    rows = np.loadtxt(path, delimiter=",")
    assert rows.shape[1] == 22 and len(rows) == 1001

    pos, quat = replay(rows)
    pos_err = np.linalg.norm(pos - rows[:, 1:4], axis=1)
    pos_rmse = float(np.sqrt(np.mean(pos_err**2)))
    att_err = quat_angle_deg(quat, rows[:, 7:11])
    att_rmse = float(np.sqrt(np.mean(att_err**2)))

    assert pos_rmse < 0.5, f"{flight}: pos RMSE {pos_rmse:.3f} m (max {pos_err.max():.3f})"
    assert att_rmse < 3.0, f"{flight}: att RMSE {att_rmse:.2f} deg (max {att_err.max():.2f})"
