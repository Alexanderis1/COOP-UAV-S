"""P2-5: cross-device determinism and RNG stream-uniqueness suite.

The hw package's randomness contract (docs/ORDERING.md section 4 extended
to devices): every device type takes its own named registry stream as
parent and spawns one child per vehicle. Pins here: run-twice determinism
of the whole 5-device stack, registry order-independence (an extra
consumer shifts nothing), cross-device independence (removing one device
leaves the others' draws identical), fleet-growth prefix invariance, and
the sharing hazard (two devices on ONE parent stream are NOT independent
copies — names must be unique).
"""

import numpy as np

from coopuavs.core.rng import RngRegistry
from coopuavs.hw.baro import Baro, BaroParams
from coopuavs.hw.esc_telem import EscTelem, EscTelemParams
from coopuavs.hw.gps import Gps, GpsParams
from coopuavs.hw.imu import Imu, ImuParams
from coopuavs.hw.mag import Mag, MagParams
from coopuavs.hw.params import load_devices

BASE_HZ = 800


def _build(reg: RngRegistry, n: int, skip: set | None = None) -> dict:
    skip = skip or set()
    cfg = load_devices("interceptor_devices")
    out = {}
    if "imu" not in skip:
        out["imu"] = Imu(ImuParams.from_dict(cfg["imu"]), n,
                         reg.stream("sensor/imu"))
    if "gps" not in skip:
        out["gps"] = Gps(GpsParams.from_dict(cfg["gps"]), n,
                         reg.stream("sensor/gps"), BASE_HZ)
    if "baro" not in skip:
        out["baro"] = Baro(BaroParams.from_dict(cfg["baro"]), n,
                           reg.stream("sensor/baro"))
    if "mag" not in skip:
        out["mag"] = Mag(MagParams.from_dict(cfg["mag"]), n,
                         reg.stream("sensor/mag"))
    if "esc" not in skip:
        out["esc"] = EscTelem(EscTelemParams.from_dict(cfg["esc_telem"]), n,
                              4, reg.stream("sensor/esc_telem"))
    return out


def _run(devices: dict, n: int, ticks: int = 400) -> dict:
    """Scripted half-second of truth through whichever devices exist."""
    quat = np.zeros((n, 4))
    quat[:, 0] = 1.0
    omega = np.tile([0.1, -0.2, 0.05], (n, 1))
    accel = np.tile([0.0, 0.0, -9.81], (n, 1))
    rotor = np.full((n, 4), 900.0)
    v_bus = np.full(n, 44.4)
    i_bus = np.full(n, 120.0)
    out = {k: [] for k in devices}
    for k in range(ticks):
        t = k / BASE_HZ
        pos = np.tile([10.0 * t, -5.0 * t, 80.0], (n, 1))
        vel = np.tile([10.0, -5.0, 0.0], (n, 1))
        if "gps" in devices:
            fix = devices["gps"].tick(pos, vel)
            if fix is not None:
                out["gps"].append(fix.pos)
        if "imu" in devices and k % 2 == 0:                  # 400 Hz
            out["imu"].append(np.concatenate(
                devices["imu"].sample(quat, omega, accel), axis=1))
        if k % 16 == 0:                                      # 50 Hz
            if "baro" in devices:
                out["baro"].append(devices["baro"].sample(pos[:, 2]))
            if "mag" in devices:
                out["mag"].append(devices["mag"].sample(quat))
        if "esc" in devices and k % 80 == 0:                 # 10 Hz
            out["esc"].append(devices["esc"].sample(rotor, v_bus, i_bus).rpm)
    return {k: np.stack(v) for k, v in out.items() if v}


def test_full_stack_run_twice_is_identical():
    a = _run(_build(RngRegistry(13), 4), 4)
    b = _run(_build(RngRegistry(13), 4), 4)
    assert set(a) == {"imu", "gps", "baro", "mag", "esc"}
    for k in a:
        np.testing.assert_array_equal(a[k], b[k])
    c = _run(_build(RngRegistry(14), 4), 4)
    for k in a:
        assert np.abs(a[k] - c[k]).max() > 0.0


def test_extra_registry_consumer_shifts_no_device():
    a = _run(_build(RngRegistry(13), 4), 4)
    reg = RngRegistry(13)
    reg.stream("extra/noop").random(10_000)      # noisy unrelated consumer
    b = _run(_build(reg, 4), 4)
    for k in a:
        np.testing.assert_array_equal(a[k], b[k])


def test_removing_one_device_leaves_the_others_identical():
    full = _run(_build(RngRegistry(13), 4), 4)
    partial = _run(_build(RngRegistry(13), 4, skip={"gps"}), 4)
    assert "gps" not in partial
    for k in partial:
        np.testing.assert_array_equal(full[k], partial[k])


def test_fleet_growth_leaves_existing_vehicles_identical_across_the_stack():
    small = _run(_build(RngRegistry(13), 4), 4)
    big = _run(_build(RngRegistry(13), 7), 7)
    for k in small:
        np.testing.assert_array_equal(small[k], big[k][:, :4, ...])


def test_sharing_one_parent_stream_is_not_an_independent_copy():
    # Spawning twice from one parent advances its spawn counter: the second
    # device gets DIFFERENT children. Unique stream names per device are a
    # correctness contract, not a style rule (ORDERING.md section 4).
    cfg = ImuParams.from_dict(load_devices("interceptor_devices")["imu"])
    reg = RngRegistry(13)
    imu_a = Imu(cfg, 2, reg.stream("sensor/imu"))
    imu_b = Imu(cfg, 2, reg.stream("sensor/imu"))    # same parent object
    assert np.abs(imu_a.generate(50) - imu_b.generate(50)).max() > 0.0
