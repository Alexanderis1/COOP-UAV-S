"""P4-1: the world.friendlies duck-type, pinned as a protocol-conformance test.

Sim-side consumers reach into friendly-UAV objects without isinstance
checks; that implicit contract is Problem-1 top risk #5 (hidden truth
coupling). This file states the contract once and holds every
implementation to it: the legacy ``InterceptorUav``/``SentinelUav`` and
the P4 ``FriendlyVehicle`` truth adapter over a ``SitlEngine`` row.

The contract, by consumer (file:line at pin time):

- ``world.step`` friendly wind: ``.position[2]``, ``.mode != REARM``,
  ``.body.position +=`` displacement — legacy only: a SITL vehicle takes
  wind as a plant FORCE, so it opts out via ``wind_displaced = False``;
- adjudicator: ``.position``, ``.velocity``, ``.effector.p_kill/.type``
  — must be TRUTH (the referee resolves against ground truth);
- threat evasion (``enemy_drone._evade``): ``.position`` — truth;
- comms (``register_endpoint``): reads ``.position`` (radio physics),
  writes ``.link_quality`` every step;
- recorder: ``.position`` (pad occupancy), ``.home`` (scene);
- seekers + ``mounted()`` payloads: ``.body.position`` (truth mount),
  ``.seeker_cue()`` (estimate-only track picture, SIM-GT-001);
- scenario build: ``.uav_id``, ``.max_speed``, ``.effector.type.value``;
- C2/tests: ``.mode``, ``.battery`` (UavMode / [0,1] float).
"""

from __future__ import annotations

import numpy as np
import pytest

from coopuavs.core.bus import MessageBus
from coopuavs.core.messages import Track, UavMode
from coopuavs.core.rng import RngRegistry
from coopuavs.interceptors.effectors import EFFECTOR_FACTORIES, EffectorType
from coopuavs.interceptors.sentinel import SentinelUav
from coopuavs.interceptors.uav import InterceptorUav
from coopuavs.sil.fleet import SitlEngine
from coopuavs.sil.vehicle import FriendlyVehicle


def _assert_base_contract(uav, uav_id: str):
    """Every world.friendlies entry, interceptor or sentinel."""
    assert uav.uav_id == uav_id
    assert uav.comms_endpoint == uav_id

    pos = uav.position
    assert isinstance(pos, np.ndarray) and pos.shape == (3,)
    assert pos.dtype.kind == "f"
    vel = uav.velocity
    assert isinstance(vel, np.ndarray) and vel.shape == (3,)

    home = np.asarray(uav.home, dtype=float)
    assert home.shape == (3,)

    assert isinstance(uav.mode, UavMode)
    assert 0.0 <= float(uav.battery) <= 1.0
    assert float(uav.max_speed) >= 0.0

    # comms writes this back every step.
    uav.link_quality = 0.5
    assert uav.link_quality == 0.5
    uav.link_quality = 1.0

    # seeker / mounted payloads ride .body.position (truth mount point).
    bpos = uav.body.position
    assert isinstance(bpos, np.ndarray) and bpos.shape == (3,)
    np.testing.assert_array_equal(bpos, uav.position)

    # wind contract: absent attribute means legacy displacement applies.
    assert isinstance(getattr(uav, "wind_displaced", True), bool)


def _assert_interceptor_contract(uav):
    """Armed-platform extension: adjudicator + gimballed-seeker cueing."""
    eff = uav.effector
    assert isinstance(eff.type, EffectorType)
    assert int(eff.ammo) >= 0
    assert float(eff.max_range) > 0.0
    assert float(eff.reload_time) >= 0.0
    pk = eff.p_kill(np.array([50.0, 0.0, 0.0]), np.zeros(3), np.zeros(3))
    assert 0.0 <= pk <= 1.0
    assert callable(eff.quality_window)
    cue = uav.seeker_cue()
    assert cue is None or isinstance(cue, Track)


def test_interceptor_conforms():
    uav = InterceptorUav(
        "u1", MessageBus(), home=np.array([0.0, 0.0, 0.0]),
        effector=EFFECTOR_FACTORIES["projectile"]())
    _assert_base_contract(uav, "u1")
    _assert_interceptor_contract(uav)
    # legacy bodies are wind-displaced by the world
    assert getattr(uav, "wind_displaced", True) is True


def test_sentinel_conforms():
    sent = SentinelUav(
        "s1", MessageBus(), home=np.array([10.0, 0.0, 0.0]),
        orbit={"center": [0.0, 0.0], "radius": 500.0, "alt": 300.0})
    _assert_base_contract(sent, "s1")
    assert getattr(sent, "wind_displaced", True) is True


class _StubTactical:
    """Minimal MC-side delegate (the InterceptorUav node from P4-2 on)."""

    def __init__(self):
        self.mode = UavMode.PURSUIT
        self.battery = 0.7
        self.max_speed = 60.0
        self.effector = EFFECTOR_FACTORIES["projectile"]()
        self._cue = None

    def seeker_cue(self):
        return self._cue


def _engine_one(seed=3):
    return SitlEngine([("u1", (40.0, -20.0, 50.0))], RngRegistry(seed))


def test_friendly_vehicle_conforms():
    eng = _engine_one()
    fv = FriendlyVehicle(eng, "u1", home=(40.0, -20.0, 0.0),
                         tactical=_StubTactical())
    _assert_base_contract(fv, "u1")
    _assert_interceptor_contract(fv)
    # SITL wind is a plant force: the world must NOT displace this body.
    assert fv.wind_displaced is False
    # tactical fields forward to the MC-side delegate
    assert fv.mode is UavMode.PURSUIT
    assert fv.battery == 0.7
    assert fv.max_speed == 60.0


def test_friendly_vehicle_without_delegate_has_safe_defaults():
    eng = _engine_one()
    fv = FriendlyVehicle(eng, "u1", home=(40.0, -20.0, 0.0))
    assert fv.mode is UavMode.IDLE
    assert fv.battery == 1.0
    assert fv.max_speed == 0.0
    assert fv.effector is None
    assert fv.seeker_cue() is None


def test_friendly_vehicle_is_engine_truth():
    """position/velocity/body.position read the live engine state row —
    the plant replaces its state array every tick, so the adapter must
    follow the engine, not a captured view."""
    eng = _engine_one()
    fv = FriendlyVehicle(eng, "u1", home=(40.0, -20.0, 0.0))
    np.testing.assert_array_equal(fv.position, eng.state[0, 0:3])

    eng.state = eng.state.copy()          # what plant.step does each tick
    eng.state[0, 0:3] = (1.0, 2.0, 3.0)
    eng.state[0, 3:6] = (4.0, 5.0, 6.0)
    np.testing.assert_array_equal(fv.position, [1.0, 2.0, 3.0])
    np.testing.assert_array_equal(fv.velocity, [4.0, 5.0, 6.0])
    np.testing.assert_array_equal(fv.body.position, [1.0, 2.0, 3.0])


def test_friendly_vehicle_unknown_id_rejected():
    eng = _engine_one()
    with pytest.raises(KeyError):
        FriendlyVehicle(eng, "nope", home=(0.0, 0.0, 0.0))
