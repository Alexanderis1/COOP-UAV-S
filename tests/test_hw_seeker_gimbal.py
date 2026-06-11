"""P2-4: hw/seeker_gimbal.py (FOV / slew / servo, PHY-UAV-012) and the
GimbaledSeeker adapter in sensors/seeker.py.

The gimbal closes the documented PHY-UAV-012 deviation "no gimbal FOV
constraint": detections only happen inside the boresight cone, and the
boresight obeys a rate-limited first-order servo with travel limits. The
adapter changes nothing about the legacy OnboardSeeker (pinned: identical
detections when the target is inside the FOV, plus the untouched legacy
suite/golden files).
"""

import numpy as np
import pytest

from coopuavs.core.messages import ThreatClass
from coopuavs.hw.params import load_devices
from coopuavs.hw.seeker_gimbal import SeekerGimbal, SeekerGimbalParams
from coopuavs.interceptors.effectors import projectile_gun
from coopuavs.interceptors.uav import InterceptorUav
from coopuavs.sensors.seeker import GimbaledSeeker, OnboardSeeker
from coopuavs.sim.environment import Environment
from coopuavs.sim.world import World
from coopuavs.threats.enemy_drone import EnemyDrone

ENV_CFG = {"bounds": [-1000.0, -1000.0, 1000.0, 1000.0],
           "default_zone": "SAFE", "buildings": []}


def _params(**over) -> SeekerGimbalParams:
    base = dict(fov_half_deg=17.5, slew_max_dps=200.0, tau_s=0.05,
                az_max_deg=170.0, el_min_deg=-90.0, el_max_deg=30.0)
    base.update(over)
    return SeekerGimbalParams(**base)


# ------------------------------------------------------------------ servo

def test_large_error_slews_at_exactly_the_rate_limit():
    g = SeekerGimbal(_params(), 1)
    g.command(np.array([[np.radians(170.0), 0.0]]))
    dt, slew = 0.1, np.radians(200.0)
    trace = []
    for _ in range(10):
        g.step(dt)
        trace.append(g.az[0])
    expect = np.minimum(slew * dt * np.arange(1, 11), np.radians(170.0))
    np.testing.assert_allclose(trace, expect, atol=1e-12)


def test_small_error_settles_first_order():
    tau, dt = 0.05, 0.01
    g = SeekerGimbal(_params(tau_s=tau, slew_max_dps=1e6), 1)
    err0 = np.radians(5.0)
    g.command(np.array([[err0, 0.0]]))
    errs = []
    for _ in range(15):
        g.step(dt)
        errs.append(err0 - g.az[0])
    expect = err0 * (1.0 - dt / tau) ** np.arange(1, 16)
    np.testing.assert_allclose(errs, expect, atol=1e-12)
    assert errs[-1] < 0.05 * err0           # 95% settled within 3 tau


def test_deadbeat_when_dt_exceeds_tau_never_overshoots():
    g = SeekerGimbal(_params(tau_s=0.05, slew_max_dps=1e6), 1)
    cmd = np.radians(3.0)
    g.command(np.array([[cmd, -cmd]]))
    g.step(0.1)                              # dt > tau: jump to command
    np.testing.assert_allclose([g.az[0], g.el[0]], [cmd, -cmd], atol=1e-15)
    g.step(0.1)
    np.testing.assert_allclose([g.az[0], g.el[0]], [cmd, -cmd], atol=1e-15)


def test_travel_limits_clamp_command():
    g = SeekerGimbal(_params(), 1)
    g.command(np.array([[np.pi, np.radians(80.0)]]))
    for _ in range(100):
        g.step(0.05)
    np.testing.assert_allclose(g.az[0], np.radians(170.0), atol=1e-12)
    np.testing.assert_allclose(g.el[0], np.radians(30.0), atol=1e-12)


def test_point_at_converges_boresight_to_direction():
    rng = np.random.default_rng(5)
    n = 6
    az = rng.uniform(-2.0, 2.0, n)
    el = rng.uniform(-1.0, 0.4, n)
    d = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az),
                  np.sin(el)], axis=1) * 250.0      # arbitrary magnitude
    g = SeekerGimbal(_params(), n)
    g.point_at(d)
    for _ in range(200):
        g.step(0.02)
    np.testing.assert_allclose(
        g.boresight_body() * np.linalg.norm(d, axis=1, keepdims=True), d,
        atol=1e-9)


def test_fov_edge_is_inclusive_and_just_beyond_is_out():
    half = np.radians(17.5)
    g = SeekerGimbal(_params(), 1)                  # boresight forward
    on_edge = np.array([[np.cos(half), np.sin(half), 0.0]])
    beyond = np.array([[np.cos(half + 1e-3), np.sin(half + 1e-3), 0.0]])
    ahead = np.array([[5.0, 0.0, 0.0]])
    assert g.in_fov(on_edge)[0]
    assert not g.in_fov(beyond)[0]
    assert g.in_fov(ahead)[0]
    assert not g.in_fov(np.array([[0.0, 0.0, 0.0]]))[0]   # degenerate LOS


def test_batch_matches_scalar():
    n = 4
    rng = np.random.default_rng(6)
    cmds = rng.uniform(-1.5, 1.5, (n, 2)) * [1.0, 0.3]
    batch = SeekerGimbal(_params(), n)
    batch.command(cmds)
    singles = [SeekerGimbal(_params(), 1) for _ in range(n)]
    for i, s in enumerate(singles):
        s.command(cmds[i:i + 1])
    for _ in range(37):
        batch.step(0.03)
        for s in singles:
            s.step(0.03)
    for i, s in enumerate(singles):
        assert batch.az[i] == s.az[0] and batch.el[i] == s.el[0]


def test_params_validation_and_yaml():
    with pytest.raises(ValueError):
        _params(fov_half_deg=0.0)
    with pytest.raises(ValueError):
        _params(fov_half_deg=95.0)
    with pytest.raises(ValueError):
        _params(slew_max_dps=0.0)
    with pytest.raises(ValueError):
        _params(tau_s=0.0)
    with pytest.raises(ValueError):
        _params(az_max_deg=190.0)
    with pytest.raises(ValueError):
        _params(el_min_deg=10.0, el_max_deg=-10.0)
    with pytest.raises(ValueError):
        _params(el_min_deg=-100.0)
    with pytest.raises(ValueError):
        SeekerGimbal(_params(), 0)
    p = SeekerGimbalParams.from_dict(load_devices("interceptor_devices")["seeker_gimbal"])
    assert 0.0 < p.fov_half_deg <= 90.0
    SeekerGimbal(p, 2)


# ---------------------------------------------------------------- adapter

def _world_with_enemy(pos, seed=2):
    world = World(Environment.from_config(ENV_CFG), dt=0.05, seed=seed)
    enemy = EnemyDrone("owa-1", ThreatClass.OWA_STRATEGIC,
                       np.array(pos, dtype=float), np.zeros(3),
                       world.rng, world=world)
    world.enemies["owa-1"] = enemy
    return world, enemy


def _uav(world):
    return InterceptorUav("u1", world.bus, home=np.zeros(3),
                          effector=projectile_gun(), max_speed=80.0)


def test_target_behind_is_blind_until_the_gimbal_slews_onto_it():
    world, _ = _world_with_enemy([-300.0, 0.0, 0.0])     # dead astern
    uav = _uav(world)
    seeker = GimbaledSeeker("skr", world, uav, gimbal=SeekerGimbal(_params(), 1))
    dets = []
    world.bus.subscribe("detections", dets.append)
    first_at = None
    for k in range(12):
        seeker.update(k * 0.1, 0.05)
        if dets and first_at is None:
            first_at = k
    # 200 deg/s from forward to the 170 deg az stop: inside the 17.5 deg
    # cone (az >= 162.5 deg) first on the 9th scan (k = 8).
    assert first_at == 8
    np.testing.assert_allclose(seeker.gimbal.az[0], np.radians(170.0),
                               atol=1e-9)


def test_in_fov_detections_match_plain_onboard_seeker_exactly():
    def run(cls, **extra):
        world, _ = _world_with_enemy([300.0, 0.0, 0.0], seed=4)
        uav = _uav(world)
        skr = cls("skr", world, uav, **extra)
        dets = []
        world.bus.subscribe("detections", dets.append)
        for k in range(5):
            skr.update(k * 0.1, 0.05)
        return dets

    plain = run(OnboardSeeker)
    gimbaled = run(GimbaledSeeker, gimbal=SeekerGimbal(_params(), 1))
    assert len(plain) == len(gimbaled) == 5
    for a, b in zip(plain, gimbaled):
        np.testing.assert_array_equal(a.position, b.position)
        assert a.class_likelihoods == b.class_likelihoods


def test_auto_cue_picks_the_nearest_alive_threat():
    world, near = _world_with_enemy([100.0, 50.0, 0.0])
    far = EnemyDrone("owa-2", ThreatClass.OWA_STRATEGIC,
                     np.array([400.0, -200.0, 0.0]), np.zeros(3),
                     world.rng, world=world)
    world.enemies["owa-2"] = far
    uav = _uav(world)
    seeker = GimbaledSeeker("skr", world, uav, gimbal=SeekerGimbal(_params(), 1))
    for k in range(40):
        seeker.update(k * 0.1, 0.05)
    los = near.position - seeker.position
    expect_az = np.arctan2(los[1], los[0])
    np.testing.assert_allclose(seeker.gimbal.az[0], expect_az, atol=1e-6)
    # nearest dies -> cue swings to the survivor
    near.alive = False
    for k in range(40, 80):
        seeker.update(k * 0.1, 0.05)
    los = far.position - seeker.position
    np.testing.assert_allclose(seeker.gimbal.az[0],
                               np.arctan2(los[1], los[0]), atol=1e-6)
