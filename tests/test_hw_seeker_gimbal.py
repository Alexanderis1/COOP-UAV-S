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

from coopuavs.core.messages import EngagementTask, Header, ThreatClass, Track
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


def test_initial_pose_is_clipped_into_the_travel_band():
    # An el band excluding 0 must not leave the boresight parked outside
    # the mechanical limits (gate-review finding).
    g = SeekerGimbal(_params(el_min_deg=10.0, el_max_deg=20.0), 2)
    np.testing.assert_allclose(g.el, np.radians(10.0), atol=1e-15)
    np.testing.assert_array_equal(g.az, 0.0)
    for _ in range(50):                          # idle stepping holds pose
        g.step(0.05)
    np.testing.assert_allclose(g.el, np.radians(10.0), atol=1e-15)


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


def _assign_track(uav, track_id, pos):
    """Steer the UAV's tactical picture directly (unit-test idiom, like
    test_kill_bookkeeping steering adj._rng): a fused track estimate plus
    a shooter task on it — the inputs seeker_cue() is defined over."""
    uav._tracks[track_id] = Track(header=Header(stamp=0.0), track_id=track_id,
                                  position=np.asarray(pos, dtype=float))
    uav._task = EngagementTask(header=Header(stamp=0.0), task_id=1,
                               track_id=track_id, shooter_id=uav.uav_id)


def test_target_behind_is_blind_until_the_gimbal_slews_onto_it():
    world, _ = _world_with_enemy([-300.0, 0.0, 0.0])     # dead astern
    uav = _uav(world)
    seeker = GimbaledSeeker("skr", world, uav, gimbal=SeekerGimbal(_params(), 1))
    _assign_track(uav, 7, [-300.0, 0.0, 0.0])            # MC cue on the target
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


def test_fov_skipped_enemy_shifts_later_draws_in_the_scan():
    """Scopes the equivalence claim (gate-review finding): an in-range
    enemy OUTSIDE the cone is skipped before its noise draw, so a later
    in-FOV enemy sees different draws than under the plain seeker — the
    same skip-shifts-draws behavior the base class's range and occlusion
    gates already have. Pinned here so the behavior is contractual."""
    def run(cls, **extra):
        world, _ = _world_with_enemy([-400.0, 0.0, 0.0], seed=4)   # astern
        ahead = EnemyDrone("owa-2", ThreatClass.OWA_STRATEGIC,
                           np.array([300.0, 0.0, 0.0]), np.zeros(3),
                           world.rng, world=world)
        world.enemies["owa-2"] = ahead
        uav = _uav(world)
        skr = cls("skr", world, uav, **extra)
        dets = []
        world.bus.subscribe("detections", dets.append)
        skr.update(0.0, 0.05)                    # single scan
        return dets

    # Untasked: the gimbal stays caged forward — ahead enemy in the cone,
    # astern enemy FOV-gated out (slew rate irrelevant; pinned slow anyway).
    plain = run(OnboardSeeker)
    gimbaled = run(GimbaledSeeker,
                   gimbal=SeekerGimbal(_params(slew_max_dps=1.0), 1))
    assert len(plain) == 2                       # both enemies detected
    assert len(gimbaled) == 1                    # astern one FOV-gated out
    # the in-FOV enemy's detection used DIFFERENT draws (astern enemy's
    # skipped draw shifted the stream) — deterministically pinned
    in_fov_plain = [d for d in plain if np.linalg.norm(
        d.position - [300.0, 0.0, 0.0]) < 50.0][0]
    assert not np.array_equal(in_fov_plain.position, gimbaled[0].position)
    again = run(GimbaledSeeker,
                gimbal=SeekerGimbal(_params(slew_max_dps=1.0), 1))
    np.testing.assert_array_equal(gimbaled[0].position, again[0].position)


def test_cue_follows_assigned_track_estimate_and_holds_when_untasked():
    """The gimbal is cued by the engaged target's fused TRACK — pure
    estimate: no enemy exists in this test at all, so the cue path
    provably reads no ground truth (SIM-GT-001). Retasking swings the
    boresight to the new estimate; untasked, the gimbal holds its pose
    (caged) instead of falling back to any truth-derived target."""
    world = World(Environment.from_config(ENV_CFG), dt=0.05, seed=2)
    uav = _uav(world)
    seeker = GimbaledSeeker("skr", world, uav, gimbal=SeekerGimbal(_params(), 1))
    _assign_track(uav, 7, [100.0, 50.0, 0.0])
    for k in range(40):
        seeker.update(k * 0.1, 0.05)
    np.testing.assert_allclose(seeker.gimbal.az[0],
                               np.arctan2(50.0, 100.0), atol=1e-6)
    _assign_track(uav, 8, [400.0, -200.0, 0.0])          # retasked
    for k in range(40, 80):
        seeker.update(k * 0.1, 0.05)
    np.testing.assert_allclose(seeker.gimbal.az[0],
                               np.arctan2(-200.0, 400.0), atol=1e-6)
    uav._task = None                                     # untasked: hold
    held = seeker.gimbal.az[0]
    for k in range(80, 120):
        seeker.update(k * 0.1, 0.05)
    assert seeker.gimbal.az[0] == held
