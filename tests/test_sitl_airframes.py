"""P4 gate-review resolution 2 (user delegated, fidelity/determinism):
per-class airframe banks in the fleet engine + the sentinel endurance
airframe.

With the P4-4 energy rewire, a sentinel flying the racer pack would hit
BATT_LOW mid-raid — a surveillance gap that is an ARTIFACT of the wrong
airframe, actively distorting the reference scenarios. ``sentinel_quad``
is the interceptor airframe with a 24 Ah endurance pack (identical
flight dynamics by design — the binding claim is the endurance class);
the engine batches one plant/powertrain per airframe class and keeps
one fleet-wide device suite.

Determinism contract unchanged: stable vehicle order ⇒ per-vehicle
device children unchanged; one RK4 per class is bitwise reproducible
run-to-run; adding a sentinel class does not move an interceptor's rows.
"""

from __future__ import annotations

import numpy as np

from coopuavs.core.rng import RngRegistry
from coopuavs.physics.params import load_airframe
from coopuavs.sil.fleet import SitlEngine

DT = 0.05
MIXED = [("u1", (0.0, 0.0, 50.0)),
         ("s1", (40.0, 0.0, 50.0), "sentinel_quad")]


def _engine(vehicles=MIXED, seed=3):
    return SitlEngine(vehicles, RngRegistry(seed), world_dt=DT)


def _boot_arm_run(eng, t_fly=2.0, t_max=8.0):
    t = 0.0
    while t < t_max and not all(f.pbit_ok for f in eng.fcus):
        eng.run_macro_step(t, DT)
        t += DT
    assert all(f.pbit_ok for f in eng.fcus)
    for f in eng.fcus:
        ok, why = f.cmd_arm()
        assert ok, why
    hist = []
    for _ in range(round(t_fly / DT)):
        eng.run_macro_step(t, DT)
        t += DT
        hist.append(eng.state.copy())
    return t, np.stack(hist)


def test_sentinel_airframe_is_the_endurance_variant():
    racer = load_airframe("interceptor_quad")
    endur = load_airframe("sentinel_quad")
    assert endur["battery"]["capacity_ah"] == 24.0
    assert racer["battery"]["capacity_ah"] == 16.0
    # identical flight dynamics by design (idealization, pinned):
    for key in ("mass", "inertia_diag", "rotors", "motor", "drag"):
        assert endur[key] == racer[key], key


def test_mixed_fleet_groups_and_hover():
    eng = _engine()
    assert [g.airframe for g in eng.groups] == ["interceptor_quad",
                                                "sentinel_quad"]
    assert [list(g.rows) for g in eng.groups] == [[0], [1]]
    _, hist = _boot_arm_run(eng)
    for i, (uid, start, *_) in enumerate(MIXED):
        err = np.linalg.norm(eng.state[i, 0:3] - np.asarray(start))
        assert err < 2.5, f"{uid} drifted {err:.2f} m in mixed hover"
    assert all(f.state == "ARMED" and f.failsafe == "" for f in eng.fcus)


def test_mixed_fleet_run_twice_bit_identical():
    _, h1 = _boot_arm_run(_engine())
    _, h2 = _boot_arm_run(_engine())
    np.testing.assert_array_equal(h1, h2)


def test_class_grouping_leaves_solo_vehicle_draws_intact():
    """Adding a sentinel CLASS must not perturb the interceptor: same
    draw-history contract as fleet-size invariance (trajectory at 1e-9 —
    batched-kernel ULP noise; wiring faults are noise-scale louder)."""
    _, h_solo = _boot_arm_run(_engine(vehicles=MIXED[:1]))
    _, h_mixed = _boot_arm_run(_engine())
    np.testing.assert_allclose(h_solo[:, 0, :], h_mixed[:, 0, :],
                               rtol=1e-9, atol=1e-9)


def test_endurance_pack_drains_proportionally_slower():
    """Same hover draw, 1.5x capacity: the sentinel pack's SOC drop over
    the same flight must be ~1/1.5 of the racer's (the PHY-SNT endurance
    claim at the physics level)."""
    eng = _engine()
    t, _ = _boot_arm_run(eng, t_fly=2.0)
    g_racer, g_endur = eng.groups
    soc0_r = float(g_racer.pt.battery.soc[0])
    soc0_e = float(g_endur.pt.battery.soc[0])
    for _ in range(round(10.0 / DT)):
        eng.run_macro_step(t, DT)
        t += DT
    drop_r = soc0_r - float(g_racer.pt.battery.soc[0])
    drop_e = soc0_e - float(g_endur.pt.battery.soc[0])
    assert drop_r > 0.0 and drop_e > 0.0
    ratio = drop_r / drop_e
    assert 1.25 < ratio < 1.75, f"capacity ratio not honored: {ratio:.2f}"
