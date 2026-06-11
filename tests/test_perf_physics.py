"""P1-6 perf gate: batched plant RK4 CPU budget (plan: Performance budget).

Gate (plan P1-6): 20-vehicle multirotor RK4 at 800 Hz <= 0.25 s CPU per
simulated second. The 30-vehicle design-envelope timing is also measured
and reported (budget table: batched plant ~0.2 s/sim-s at N=30) but the
hard gate is the 20-vehicle number. Run with `pytest -m perf`.
"""

from __future__ import annotations

import time

import numpy as np
import pytest

from coopuavs.physics import rigid_body as rb
from coopuavs.physics.multirotor import MultirotorParams, MultirotorPlant
from coopuavs.physics.params import load_airframe


def _cpu_per_sim_s(n_vehicles: int, sim_seconds: float = 1.0, hz: int = 800) -> float:
    params = MultirotorParams.from_dict(load_airframe("interceptor_quad"))
    plant = MultirotorPlant(params, n_vehicles)
    rng = np.random.default_rng(7)
    state = np.zeros((n_vehicles, rb.STATE_DIM))
    state[:, 0:2] = rng.uniform(-50, 50, size=(n_vehicles, 2))
    state[:, 2] = rng.uniform(40, 120, size=n_vehicles)
    state[:, rb.QUAT] = [1.0, 0.0, 0.0, 0.0]
    w_h = np.sqrt(params.mass * 9.81 / (params.n_rotors * params.kf))
    rotor = rng.uniform(0.9, 1.1, size=(n_vehicles, params.n_rotors)) * w_h
    wind = rng.normal(scale=3.0, size=(n_vehicles, 3))
    dt = 1.0 / hz
    steps = round(sim_seconds * hz)

    for _ in range(40):  # warmup (allocator, caches)
        state = plant.step(state, dt, rotor, wind, 1.225)
    best = np.inf
    for _ in range(3):
        t0 = time.process_time()
        s = state
        for _ in range(steps):
            s = plant.step(s, dt, rotor, wind, 1.225)
        best = min(best, (time.process_time() - t0) / sim_seconds)
    assert np.isfinite(s).all()
    return best


@pytest.mark.perf
def test_perf_20_vehicle_rk4_800hz_under_0p25s():
    cpu = _cpu_per_sim_s(20)
    cpu30 = _cpu_per_sim_s(30)
    print(f"\nplant RK4 CPU/sim-s: N=20 {cpu:.3f} s (gate 0.25), N=30 {cpu30:.3f} s "
          f"(budget ~0.2 informational)")
    assert cpu <= 0.25, f"20-vehicle RK4 at 800 Hz used {cpu:.3f} s CPU/sim-s (> 0.25)"
