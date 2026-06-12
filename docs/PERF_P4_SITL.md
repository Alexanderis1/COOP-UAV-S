# P4-8 committed profile — residential_raid, fidelity.fleet=sitl

Measured 2026-06-12 (Windows 11, `time.process_time`, settled machine,
solo process). Scenario: `scenarios/residential_raid.yaml` + sitl
fidelity (8 interceptors = 8 CoopFC FCUs at 800 Hz + 8 MC VirtualMCUs at
10 Hz over coop-links, full sensor/fusion/C2/turret pipeline, 2 turrets,
mixed 10-threat raid).

**RTF 0.81× headless (1.24 s CPU/sim-s) over a 20 sim-s boot+raid
slice — gate ≥ 0.5× (plan perf budget), 60% headroom.** Gate lives in
`tests/test_sitl_perf.py` (`@perf`).

cProfile, 10 sim-s slice (top cumulative, trimmed):

```
ncalls    cumtime  function
200       18.53    sil/fleet.py run_macro_step          (the micro seam)
8000      18.50    sil/fleet.py _tick                   (800 Hz × 10 s)
64000      8.78    coopfc/fcu.py run_tick               (8 FCUs — 46%)
4000       5.89    coopfc fcu _est_update → ekf.update  (EKF dominates FCU)
8000       5.50    physics/multirotor.py step           (ONE batched RK4 — 29%)
256000     3.02    ekf.py _strapdown_step               (mainline + output replay)
4000       2.89    ekf.py _output                       (full replay, fidelity decision P3-R2)
185000     1.98    rigid_body.py _cross3
8000       1.72    sil/fleet.py _devices                (vectorized banks + HAL writes)
32000      1.97    coopfc fcu _rate_mix_task            (400 Hz rate loop)
257600     1.63    coopfc/core/vec.py quat_integrate
```

Reading: per-FCU cost ≈ 0.055 s CPU/sim-s (8 × ≈ 0.44), batched plant ≈
0.28 (N-shape-independent), devices+link+MC ≈ 0.1, macro pipeline
(sensors/fusion/C2/recorder) ≈ 0.4. Consistent with the P3-8 projection
(C20 model; the EKF `_fuse_sel`/rank-m work is why the FCU share is
affordable). First fallback lever on a future miss: scenario rate
profiles (CI 200/100/25 Hz), then mixed-fidelity fleets — numba/C only
with a committed profile and explicit user approval (plan policy).

Caveat (P2/P3 lesson): `@perf` figures here are only comparable when
re-measured solo on a settled machine; SMT/cache contention and
post-suite thermals read low.
