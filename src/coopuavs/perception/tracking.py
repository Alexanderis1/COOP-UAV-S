"""Single-track state estimation: constant-velocity Kalman filter.

State is [x, y, z, vx, vy, vz] in the map frame; measurements are 3D
positions with the full per-detection covariance supplied by the sensor
(which is how bearing-only RF/acoustic geometry enters the filter
correctly). An IMM (CV + coordinated-turn + dive) is the planned upgrade for
terminal-phase manoeuvres — see docs/RESEARCH.md.
"""

from __future__ import annotations

import itertools

import numpy as np

from ..core.messages import Detection, ThreatClass

_track_ids = itertools.count(1)


def reset_track_ids() -> None:
    """Restart track numbering. Called by each new ``World`` (with
    ``reset_message_seq``) so runs are reproducible run-to-run even when
    several share one Python process — batch, serve, tests (SIM-003)."""
    global _track_ids
    _track_ids = itertools.count(1)


_H = np.hstack([np.eye(3), np.zeros((3, 3))])   # position measurement model


class KalmanTrack:
    def __init__(self, det: Detection, process_noise: float = 4.0):
        self.track_id = next(_track_ids)
        self.q = process_noise
        self.x = np.zeros(6)
        self.x[:3] = det.position
        self.P = np.eye(6) * 100.0
        self.P[:3, :3] = det.cov + np.eye(3) * 25.0
        self.P[3:, 3:] = np.eye(3) * 400.0       # unknown velocity

        self.t_last = det.header.stamp        # filter time (advanced by predict)
        self.t_last_meas = det.header.stamp   # last measurement absorption
        self.n_hits = 1
        self.t_created = det.header.stamp
        self.class_belief: dict[ThreatClass, float] = {}
        self.rf_signature: str | None = det.rf_signature

    # -- KF core ---------------------------------------------------------------

    def predict(self, t: float) -> None:
        dt = max(0.0, t - self.t_last)
        if dt == 0.0:
            return
        F = np.eye(6)
        F[:3, 3:] = np.eye(3) * dt
        # White-noise acceleration process model.
        q = self.q
        G = np.vstack([np.eye(3) * dt**2 / 2, np.eye(3) * dt])
        Q = G @ G.T * q**2
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.t_last = t

    def innovation(self, det: Detection) -> tuple[np.ndarray, np.ndarray]:
        y = det.position - _H @ self.x
        S = _H @ self.P @ _H.T + det.cov
        return y, S

    def mahalanobis2(self, det: Detection) -> float:
        y, S = self.innovation(det)
        return float(y @ np.linalg.solve(S, y))

    def update(self, det: Detection) -> None:
        y, S = self.innovation(det)
        K = self.P @ _H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ _H) @ self.P
        self.n_hits += 1
        self.t_last_meas = det.header.stamp
        if det.rf_signature:
            self.rf_signature = det.rf_signature

    # -- convenience ---------------------------------------------------------------

    @property
    def position(self) -> np.ndarray:
        return self.x[:3]

    @property
    def velocity(self) -> np.ndarray:
        return self.x[3:]

    def time_since_update(self, t: float) -> float:
        return t - self.t_last_meas
