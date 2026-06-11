"""P1-7: export RotorPy oracle traces for the multirotor physics core.

Offline tool (RotorPy is an oracle, never a runtime dependency):

    .venv\\Scripts\\python.exe scripts/oracle/export_rotorpy.py

writes tests/fixtures/oracle/rotorpy_<flight>.csv consumed by the @oracle
tests. Re-running overwrites committed traces — do that only as a
sanctioned re-baseline (same policy as golden pins).

Matched-parameter scoping: both simulators run DRAG-FREE (RotorPy
aero=False; the test zeroes our Faessler/parasitic terms). The two
packages model drag with intentionally different forms (Faessler lumped-D
vs per-rotor H-force), so drag is pinned by our unit tests
(terminal-speed, dissipation) while the oracle pins what the models share:
quaternion 6DOF rigid body, rotor thrust/moment allocation, first-order
motor lag, gravity. Mapping checks (verified numerically): RotorPy body
frame == our FLU, rotor_directions == -spin, k_eta == kf, k_m == km.

Flights (10 s each, 100 Hz piecewise-constant motor-speed commands, the
command columns in the CSV are the single source of truth for replay):
  hover_hold   exact hover speeds                  (statics + conventions)
  climb_step   +2% collective for 2 s              (motor lag + thrust scale)
  tilt_dash    15 deg initial tilt, hover speeds   (quat/thrust direction)
  yaw_spin     CCW +1% / CW -1%                    (k_m sign + magnitude)
  pitch_pulse  front/back +-0.6% doublet, 2 x 0.25 s (moment arm + inertia;
               a doublet steps attitude ~3.5 deg then holds, keeping the
               vehicle airborne -- a one-sided pulse tumbles it through the
               ground plane where our ground-effect model would diverge
               from RotorPy, which has none)

CSV columns: t, x, y, z, vx, vy, vz, qw, qx, qy, qz (OUR scalar-first
order), wx, wy, wz, r1..r4 (true rotor speeds), c1..c4 (command held over
[t, t+0.01)). RotorPy integrates with DOP853 at rtol=1e-10 (oracle-grade).
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src"))

from rotorpy.vehicles.multirotor import Multirotor  # noqa: E402

from coopuavs.physics.params import load_airframe  # noqa: E402

OUT_DIR = REPO / "tests" / "fixtures" / "oracle"
DT_CTRL = 0.01
T_END = 10.0
Z0 = 50.0
TAU_M = 0.05          # s, motor lag used on BOTH sides of the comparison
SPEED_MAX = 1500.0    # rad/s, above the airframe ceiling -> never clips


def quad_params_from(cfg: dict) -> dict:
    rotors = cfg["rotors"]
    pos = np.asarray(rotors["positions"], dtype=float)
    spin = np.asarray(rotors["spin"], dtype=float)
    inertia = cfg["inertia_diag"]
    return {
        "mass": float(cfg["mass"]),
        "Ixx": float(inertia[0]), "Iyy": float(inertia[1]), "Izz": float(inertia[2]),
        "Ixy": 0.0, "Ixz": 0.0, "Iyz": 0.0,
        "num_rotors": int(rotors["count"]),
        "rotor_pos": {f"r{i + 1}": pos[i] for i in range(len(pos))},
        # RotorPy: M_yaw = +dir * k_m * w^2; ours: tau_z = -spin * km * w^2
        "rotor_directions": -spin,
        "c_Dx": 0.0, "c_Dy": 0.0, "c_Dz": 0.0,
        "k_eta": float(rotors["kf"]),
        "k_m": float(rotors["km"]),
        "k_d": 0.0, "k_z": 0.0, "k_h": 0.0, "k_flap": 0.0,
        "tau_m": TAU_M,
        "rotor_speed_min": 0.0,
        "rotor_speed_max": SPEED_MAX,
        "motor_noise_std": 0.0,
    }


def flight_set(w_h: float) -> dict:
    """name -> (cmd_fn(t) -> (4,), initial tilt angle about +y, rad)."""
    ones = np.ones(4)
    front = np.array([1.006, 0.994, 1.006, 0.994])   # r1 FR, r2 BL, r3 FL, r4 BR
    back = np.array([0.994, 1.006, 0.994, 1.006])
    ccw = np.array([1.01, 1.01, 0.99, 0.99])         # our spin=+1 rotors are r1, r2

    def pitch_doublet(t):
        if 1.0 <= t < 1.25:
            return w_h * front
        if 1.25 <= t < 1.5:
            return w_h * back
        return w_h * ones

    return {
        "hover_hold": (lambda t: w_h * ones, 0.0),
        "climb_step": (lambda t: w_h * (1.02 if t < 2.0 else 1.0) * ones, 0.0),
        "tilt_dash": (lambda t: w_h * ones, np.deg2rad(15.0)),
        "yaw_spin": (lambda t: w_h * ccw, 0.0),
        "pitch_pulse": (pitch_doublet, 0.0),
    }


def run_flight(quad_params: dict, cmd_fn, tilt_y: float) -> np.ndarray:
    vehicle = Multirotor(
        quad_params, aero=False, enable_ground=False,
        integrator_kwargs={"method": "DOP853", "rtol": 1e-10, "atol": 1e-12})
    # our wxyz tilt about +y -> rotorpy xyzw
    q0_wxyz = np.array([np.cos(tilt_y / 2.0), 0.0, np.sin(tilt_y / 2.0), 0.0])
    state = {
        "x": np.array([0.0, 0.0, Z0]),
        "v": np.zeros(3),
        "q": np.roll(q0_wxyz, -1),
        "w": np.zeros(3),
        "wind": np.zeros(3),
        # start every flight at its t=0 commanded steady state (gate review:
        # inferring hover from cmd_fn(T_END)[0] was wrong for yaw_spin)
        "rotor_speeds": np.asarray(cmd_fn(0.0), dtype=float).copy(),
    }
    steps = round(T_END / DT_CTRL)
    rows = np.empty((steps + 1, 22))
    for k in range(steps + 1):
        t = k * DT_CTRL
        cmd = np.asarray(cmd_fn(t), dtype=float)
        q_wxyz = np.roll(state["q"], 1)
        if q_wxyz[0] < 0:
            q_wxyz = -q_wxyz
        rows[k] = [t, *state["x"], *state["v"], *q_wxyz, *state["w"],
                   *state["rotor_speeds"], *cmd]
        if k < steps:
            state = vehicle.step(state, {"cmd_motor_speeds": cmd}, DT_CTRL)
    return rows


def main() -> None:
    cfg = load_airframe("interceptor_quad")
    qp = quad_params_from(cfg)
    w_h = float(np.sqrt(qp["mass"] * 9.81 / (4.0 * qp["k_eta"])))
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    header = ("RotorPy oracle trace (see scripts/oracle/export_rotorpy.py)\n"
              f"airframe=interceptor_quad drag-free tau_m={TAU_M} dt_ctrl={DT_CTRL} "
              f"z0={Z0} hover_omega={w_h:.6f}\n"
              "t,x,y,z,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz,r1,r2,r3,r4,c1,c2,c3,c4")
    for name, (cmd_fn, tilt) in flight_set(w_h).items():
        rows = run_flight(qp, cmd_fn, tilt)
        path = OUT_DIR / f"rotorpy_{name}.csv"
        np.savetxt(path, rows, delimiter=",", header=header, fmt="%.12g")
        print(f"{name}: {rows.shape[0]} rows -> {path}")


if __name__ == "__main__":
    main()
