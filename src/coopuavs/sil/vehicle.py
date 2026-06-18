"""FriendlyVehicle — the world-side truth adapter for one SitlEngine row.

In sitl fidelity the object registered in ``world.friendlies`` (and handed
to the adjudicator, the comms model, the seekers and the recorder) is no
longer the tactical agent: truth lives in the fleet engine's batched plant
state, and the agent flies on EKF estimates. This adapter keeps the
``world.friendlies`` duck-type intact (pinned by
``tests/test_sil_vehicle.py``) so every sim-side consumer stays unchanged:

- ``.position`` / ``.velocity`` / ``.body.position`` are the engine's
  TRUTH row — the adjudicator's Pk geometry, threat evasion, the radio
  channel physics and the seeker mount all read ground truth, exactly as
  they did off the legacy point-mass body;
- ``.mode`` / ``.battery`` / ``.max_speed`` / ``.effector`` /
  ``.seeker_cue()`` forward to the MC-side tactical delegate (the
  ``InterceptorUav`` node from P4-2 on; estimate-only by construction,
  SIM-GT-001) — safe defaults before one is attached;
- ``.link_quality`` is written back by the comms model every step, read
  by the tactical node's telemetry;
- ``wind_displaced = False`` opts out of the world's truth-side wind
  displacement: a SITL vehicle feels wind as a plant FORCE through the
  engine micro-loop, and displacing it again would double-apply weather.

The engine replaces ``engine.state`` every plant step, so all truth
accessors read through the engine attribute — never a captured view.
"""

from __future__ import annotations

import numpy as np

from ..core.messages import UavMode


class _TruthBody:
    """Mount-point shim: ``.position``/``.velocity`` of the truth row
    (seekers and mounted payloads ride ``uav.body.position``)."""

    __slots__ = ("_engine", "_i")

    def __init__(self, engine, i: int):
        self._engine = engine
        self._i = i

    @property
    def position(self) -> np.ndarray:
        return self._engine.state[self._i, 0:3]

    @property
    def velocity(self) -> np.ndarray:
        return self._engine.state[self._i, 3:6]


class FriendlyVehicle:
    """One ``world.friendlies`` entry backed by SitlEngine row ``uav_id``."""

    # Wind is a plant force for SITL vehicles (PLAN_PROBLEM1 P4-1):
    # World.step must not displace this body.
    wind_displaced = False

    def __init__(self, engine, uav_id: str, home, tactical=None):
        self.engine = engine
        self.uav_id = uav_id
        self.i = engine.index[uav_id]   # KeyError on unknown id, by design
        self.home = np.asarray(home, dtype=float)
        self.tactical = tactical
        self.comms_endpoint = uav_id
        self._link_quality = 1.0
        self.body = _TruthBody(engine, self.i)

    @property
    def link_quality(self) -> float:
        return self._link_quality

    @link_quality.setter
    def link_quality(self, q: float) -> None:
        # The comms model writes the radio's telemetry onto the platform
        # it registered (this adapter — the radio rides the truth
        # airframe); the tactical node reads its own copy for UavState.
        self._link_quality = q
        if self.tactical is not None:
            self.tactical.link_quality = q

    # -- truth (sim-side consumers) -----------------------------------------

    @property
    def position(self) -> np.ndarray:
        return self.engine.state[self.i, 0:3]

    @property
    def velocity(self) -> np.ndarray:
        return self.engine.state[self.i, 3:6]

    # -- tactical forwarding (MC-side delegate, P4-2) -------------------------

    @property
    def mode(self) -> UavMode:
        return self.tactical.mode if self.tactical is not None else UavMode.IDLE

    @property
    def battery(self) -> float:
        return self.tactical.battery if self.tactical is not None else 1.0

    @property
    def max_speed(self) -> float:
        return self.tactical.max_speed if self.tactical is not None else 0.0

    @property
    def effector(self):
        return self.tactical.effector if self.tactical is not None else None

    def seeker_cue(self):
        """Estimate-only gimbal cue (SIM-GT-001), from the tactical node."""
        return self.tactical.seeker_cue() if self.tactical is not None else None
