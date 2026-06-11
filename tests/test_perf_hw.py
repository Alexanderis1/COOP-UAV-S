"""P2 perf gate: full per-vehicle sensor stack CPU budget.

Gate (plan P2): the 20-vehicle device stack — IMU 400 Hz (+ 25 Hz FIFO
drain), GPS clocked at 800 Hz (10 Hz fixes, 120 ms latency), baro 50 Hz,
mag 50 Hz, ESC telemetry 10 Hz, seeker gimbal servo/FOV 10 Hz — within
0.1 s CPU per simulated second. The 30-vehicle design-envelope run is
gated at the budget-table sensors figure (0.15 s/sim-s at N=30, plan
"Performance budget"). Each measured rep spans 4 sim-s so the reading
resolves well above the Windows process_time quantum (15.625 ms — a
1 sim-s rep here reads as exactly one quantum, gate-review finding).
Run with `pytest -m perf`.
"""

from __future__ import annotations

import time

import numpy as np
import pytest

from coopuavs.hw.baro import Baro, BaroParams
from coopuavs.hw.esc_telem import EscTelem, EscTelemParams
from coopuavs.hw.gps import Gps, GpsParams
from coopuavs.hw.imu import Imu, ImuParams
from coopuavs.hw.mag import Mag, MagParams
from coopuavs.hw.params import load_devices
from coopuavs.hw.seeker_gimbal import SeekerGimbal, SeekerGimbalParams

BASE_HZ = 800


def _build(n: int):
    cfg = load_devices("interceptor_devices")
    rng = np.random.default_rng(7)
    return {
        "imu": Imu(ImuParams.from_dict(cfg["imu"]), n, rng.spawn(1)[0]),
        "gps": Gps(GpsParams.from_dict(cfg["gps"]), n, rng.spawn(1)[0], BASE_HZ),
        "baro": Baro(BaroParams.from_dict(cfg["baro"]), n, rng.spawn(1)[0]),
        "mag": Mag(MagParams.from_dict(cfg["mag"]), n, rng.spawn(1)[0]),
        "esc": EscTelem(EscTelemParams.from_dict(cfg["esc_telem"]), n, 4,
                        rng.spawn(1)[0]),
        "gimbal": SeekerGimbal(
            SeekerGimbalParams.from_dict(cfg["seeker_gimbal"]), n),
    }


def _tick_range(devs, n, truth, start: int, ticks: int) -> None:
    quat, omega, accel, pos, vel, alt, rotor, v_bus, i_bus, los = truth
    for k in range(start, start + ticks):
        devs["gps"].tick(pos, vel)
        if k % 2 == 0:
            devs["imu"].sample(quat, omega, accel)
        if k % 16 == 0:
            devs["baro"].sample(alt)
            devs["mag"].sample(quat)
        if k % 32 == 0:
            devs["imu"].fifo_read()                         # 25 Hz drain
        if k % 80 == 0:
            devs["esc"].sample(rotor, v_bus, i_bus)
            devs["gimbal"].point_at(los)
            devs["gimbal"].step(0.1)
            devs["gimbal"].in_fov(los)


def _cpu_per_sim_s(n: int, sim_seconds: float = 4.0) -> float:
    devs = _build(n)
    rng = np.random.default_rng(11)
    quat = np.zeros((n, 4))
    quat[:, 0] = 1.0
    truth = (quat, rng.normal(size=(n, 3)), rng.normal(size=(n, 3)),
             rng.uniform(-50, 50, (n, 3)), rng.normal(size=(n, 3)),
             rng.uniform(40, 120, n), np.full((n, 4), 900.0),
             np.full(n, 44.4), np.full(n, 120.0),
             rng.normal(size=(n, 3)) + [[200.0, 0.0, 0.0]])
    steps = round(sim_seconds * BASE_HZ)
    _tick_range(devs, n, truth, 0, 160)                     # warmup
    best = np.inf
    tick = 160
    for _ in range(3):
        t0 = time.process_time()
        _tick_range(devs, n, truth, tick, steps)
        best = min(best, (time.process_time() - t0) / sim_seconds)
        tick += steps
    return best


@pytest.mark.perf
def test_perf_sensor_stack_under_budget():
    cpu = _cpu_per_sim_s(20)
    cpu30 = _cpu_per_sim_s(30)
    print(f"\nsensor stack CPU/sim-s: N=20 {cpu:.4f} s (gate 0.1), "
          f"N=30 {cpu30:.4f} s (gate 0.15)")
    assert cpu <= 0.1, (
        f"20-vehicle sensor stack used {cpu:.4f} s CPU/sim-s (> 0.1)")
    assert cpu30 <= 0.15, (
        f"30-vehicle sensor stack used {cpu30:.4f} s CPU/sim-s (> 0.15)")
