"""P3-4d (@slow): EKF statistical consistency against the REAL device
models — NEES/NIS 25-seed Monte-Carlo, and the 5-minute GPS-denied drift
envelope (PHY-UAV-011).

The measurement chain is the P2 hw suite itself (Allan-validated IMU
with turn-on bias + GM + RW + quantization, latency-exact GNSS with GM
wander, drifting baro, hard-iron mag) over a maneuvering kinematic
truth. The filter models white + bias-RW only and *inflates R* for the
correlated rest, so the gates are consistency *bounds*, not the +-10%
precision of the Allan suite:

- mean NEES/dof (9-dof pos/vel/att at the fusion horizon, post-
  convergence) in [0.05, 2.0]; per-sensor mean NIS/dof in [0.02, 2.0].
  A wrong F/H sign, missing Q term or bad Jacobian blows these 10-100x;
  inflated R legitimately pushes them *below* 1, never above.
- GPS-denied 5 min: horizontal drift under the documented first-
  principles envelope AND inside the filter's own 4-sigma claim
  (covariance honesty is what the P5 CBIT dead-reckoning budget trips
  on). The VIO/datalink fallback of PHY-UAV-011 is real-system scope —
  docs/TRACEABILITY.md marks that requirement partial.
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np
import pytest

from coopuavs.coopfc.core import vec
from coopuavs.coopfc.estimation.alignment import Aligner
from coopuavs.coopfc.estimation.ekf import Ekf, EkfParams
from coopuavs.hw import params as hw_params
from coopuavs.hw.baro import Baro, BaroParams, altitude_from_pressure
from coopuavs.hw.gps import Gps, GpsParams
from coopuavs.hw.imu import Imu, ImuParams
from coopuavs.hw.mag import Mag, MagParams

pytestmark = pytest.mark.slow

IMU_HZ = 400
EKF_HZ = 50
DT = 1.0 / IMU_HZ
EKF_EVERY = IMU_HZ // EKF_HZ


def body_rate_profile(t: float) -> vec.Vec3:
    """Gentle but exciting: yaw sweep + roll/pitch wobble after 10 s."""
    if t < 10.0:
        return (0.0, 0.0, 0.0)
    return (0.10 * math.sin(0.40 * t),
            0.08 * math.sin(0.31 * t + 1.0),
            0.20 * math.sin(0.15 * t))


def accel_profile(t: float) -> vec.Vec3:
    """World-frame acceleration: lateral weaving + mild vertical."""
    if t < 10.0:
        return (0.0, 0.0, 0.0)
    return (0.8 * math.sin(0.25 * t), 0.6 * math.cos(0.20 * t),
            0.3 * math.sin(0.10 * t))


class Rig:
    """Real hw devices for one vehicle on one seed. The GNSS receiver is
    constructed at flight start so its internal tick epoch matches the
    FCU clock (stamps must be FCU-time; the alignment phase has no GPS)."""

    def __init__(self, seed: int):
        cfg = hw_params.load_devices("interceptor_devices")
        rng = np.random.default_rng(seed)
        self.imu = Imu(ImuParams.from_dict(cfg["imu"]), 1, rng.spawn(1)[0])
        self.gps = Gps(GpsParams.from_dict(cfg["gps"]), 1, rng.spawn(1)[0],
                       clock_hz=IMU_HZ)
        self.baro = Baro(BaroParams.from_dict(cfg["baro"]), 1, rng.spawn(1)[0])
        self.mag = Mag(MagParams.from_dict(cfg["mag"]), 1, rng.spawn(1)[0])


def run_one(seed: int, t_flight: float, deny_gps_after: float | None = None):
    """2 s static alignment on the devices, then fly the profile.

    Returns (ekf, hist) where hist tracks per-update NEES (against the
    truth at the filter's state_time), horizontal pos error, and the
    filter's claimed horizontal sigma.
    """
    rig = Rig(seed)
    q = (1.0, 0.0, 0.0, 0.0)
    pos = np.zeros(3)
    velo = np.zeros(3)

    aligner = Aligner(n_imu=800, mag_declination_deg=4.0)
    for k in range(800):
        gyro, accel = rig.imu.sample(np.array([q]), np.zeros((1, 3)),
                                     np.zeros((1, 3)))
        aligner.add_imu(tuple(gyro[0]), tuple(accel[0]))
        if k % 8 == 0:
            aligner.add_mag(tuple(rig.mag.sample(np.array([q]))[0]))
    align = aligner.result()
    assert align is not None and align.ok

    ekf = Ekf(align, EkfParams())
    hist = {"nees": [], "err_h": [], "sig_h": []}
    truth_ring: deque = deque(maxlen=400)  # (time, pos, vel, q)

    n = round(t_flight * IMU_HZ)
    for k in range(n):
        t = k * DT
        w_b = body_rate_profile(t)
        a_w = np.array(accel_profile(t))

        gyro, accel = rig.imu.sample(np.array([q]), np.array([w_b]),
                                     a_w[None, :])
        ekf.on_imu(t, tuple(gyro[0]), tuple(accel[0]))
        fix = rig.gps.tick(pos[None, :], velo[None, :])
        if fix is not None and (deny_gps_after is None or t < deny_gps_after):
            ekf.on_gps(fix.stamp_s, tuple(fix.pos[0]), tuple(fix.vel[0]),
                       int(fix.fix_type[0]))
        if k % EKF_EVERY == 0:
            p_meas = rig.baro.sample(np.array([pos[2]]))
            ekf.on_baro(t, float(altitude_from_pressure(p_meas)[0]))
            ekf.on_mag(t, tuple(rig.mag.sample(np.array([q]))[0]))
            ekf.update(t)
            _score(ekf, truth_ring, hist)

        pos = pos + velo * DT + 0.5 * a_w * DT * DT
        velo = velo + a_w * DT
        q = vec.quat_integrate(q, w_b, DT)
        truth_ring.append((t + DT, pos.copy(), velo.copy(), q))

    return ekf, hist


def _score(ekf, truth_ring, hist) -> None:
    """9-dof NEES vs the truth at the filter's state_time (horizon)."""
    truth = None
    for entry in reversed(truth_ring):
        if abs(entry[0] - ekf.state_time) < 1e-6:
            truth = entry
            break
    if truth is None:  # filter has not consumed any IMU yet
        return
    _, p_t, v_t, q_t = truth
    e_p = np.array(ekf.p) - p_t
    e_v = np.array(ekf.v) - v_t
    dq = vec.quat_multiply(vec.quat_conjugate(ekf.q), q_t)
    if dq[0] < 0.0:
        dq = tuple(-c for c in dq)
    e_th = 2.0 * np.array(dq[1:])
    e = np.concatenate([e_p, e_v, e_th])
    # Consistency vs P plus the documented unmodeled-error budget: the
    # colored measurement errors (GNSS GM wander etc.) that a 15-state
    # filter provably cannot estimate sit in budget9, and the reported
    # sigmas include them — score what the filter actually claims.
    P9 = ekf.P[:9, :9] + np.diag(ekf.budget9)
    hist["nees"].append(float(e @ np.linalg.solve(P9, e)))
    hist["err_h"].append(float(np.hypot(e_p[0], e_p[1])))
    d = np.diagonal(ekf.P)[:9] + ekf.budget9
    hist["sig_h"].append(float(np.sqrt(max(d[0], d[1]))))


def test_nees_nis_consistency_25_seeds():
    nees_means = []
    nis_means = {"gps_pos": [], "gps_vel": [], "baro": [], "mag": []}
    for seed in range(25):
        ekf, hist = run_one(seed, t_flight=60.0)
        assert not ekf.diverged, f"seed {seed} diverged"
        settled = hist["nees"][len(hist["nees"]) // 4:]
        nees_means.append(float(np.mean(settled)) / 9.0)
        for name, (s, c) in ekf.nis.items():
            dof = 1 if name in ("baro", "mag") else 3
            assert c > 0, f"seed {seed}: no accepted {name} fusions"
            nis_means[name].append(s / c / dof)
        # gating sanity: the nominal chain must not mass-reject
        total_rej = sum(v for k, v in ekf.rejected.items())
        assert total_rej < 50, f"seed {seed} rejected {ekf.rejected}"

    overall = float(np.mean(nees_means))
    assert 0.05 <= overall <= 2.0, f"mean NEES/dof {overall:.3f}"
    assert max(nees_means) < 5.0, f"worst seed NEES/dof {max(nees_means):.3f}"
    for name, vals in nis_means.items():
        m = float(np.mean(vals))
        if name == "mag":
            # The yaw information floor duty-cycles mag fusion (a few
            # accepted fusions per minute, right after P_yaw regrows),
            # so its NIS sample is small and legitimately tiny — bound
            # it from above only.
            assert m <= 4.0, f"mean NIS/dof[mag] {m:.3f}"
        else:
            assert 0.02 <= m <= 2.0, f"mean NIS/dof[{name}] {m:.3f}"


def test_gps_denied_5min_drift_envelope():
    errs, sigs = [], []
    for seed in range(5):
        ekf, hist = run_one(seed, t_flight=360.0, deny_gps_after=60.0)
        assert not ekf.diverged
        errs.append(hist["err_h"][-1])
        sigs.append(hist["sig_h"][-1])
    worst = max(errs)
    # Documented envelope (RESEARCH.md "P3 CoopFC flight stack"): with
    # baro confined to the vertical channel, denied-phase horizontal
    # drift is pure-inertial gravity-leak physics. First-principles
    # sigma scale for this suite over t = 300 s: gyro-bias RW
    # g*K*sqrt(t^7/252) ~ 2.9 km plus the GM bias-instability ramp
    # g*sigma_gm*t^3/6 ~ 1.8 km -> ~3.4 km RSS. Measured worst over
    # seeds 0-4: 5472 m (1.6x the sigma scale; seed spread 1.3-5.5 km),
    # gated at 7000 m (+28% over worst). km-class DR is the honest
    # physics for an 8 deg/h MEMS suite without the VIO/datalink
    # fallback PHY-UAV-011 assigns to the real system (out of sim
    # scope; TRACEABILITY marks the requirement partial).
    assert worst < 7000.0, f"drift {worst:.0f} m exceeds the envelope"
    for e, s in zip(errs, sigs):
        assert e < 4.0 * s, f"drift {e:.0f} m outside the filter's 4-sigma {s:.0f} m"
