"""Message definitions for the COOP-UAV-S middleware.

Every message is a plain, typed dataclass deliberately shaped like a ROS 2
message (``std_msgs/Header``, ``geometry_msgs/PoseStamped``-style fields).
When the project migrates to ROS 2 these classes map 1:1 onto ``.msg`` files
and the publish/subscribe call sites stay unchanged.

Conventions
-----------
* Frame: local ENU, metres. ``frame_id="map"`` is the world frame anchored at
  the base station; per-vehicle body frames use the vehicle id.
* Time: simulation seconds (float) carried in :class:`Header.stamp`.
* All vectors are ``numpy.ndarray`` of shape (3,) unless stated otherwise.
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass, field
from enum import Enum, IntEnum

import numpy as np

_msg_seq = itertools.count()


def _vec3() -> np.ndarray:
    return np.zeros(3)


@dataclass
class Header:
    """Equivalent of ``std_msgs/Header``."""

    stamp: float = 0.0
    frame_id: str = "map"
    seq: int = field(default_factory=lambda: next(_msg_seq))


# ---------------------------------------------------------------------------
# Enumerations
# ---------------------------------------------------------------------------


class ThreatClass(str, Enum):
    """Threat taxonomy mirrored from the operational analysis in the README."""

    OWA_STRATEGIC = "owa_strategic"   # Shahed-136 / Geran-2 type
    OWA_JET = "owa_jet"               # Geran-3 type
    FPV = "fpv"                       # tactical kamikaze quadcopter
    LOITERING = "loitering"           # Lancet-3 type
    DECOY = "decoy"                   # Gerbera-type false signature
    UNKNOWN = "unknown"


class ZoneClass(IntEnum):
    """Ground risk classes for the populated area beneath the engagement."""

    SAFE = 0        # open field, water, rubble — debris acceptable
    DANGEROUS = 1   # roads, light residential — debris strongly discouraged
    CRITICAL = 2    # schools, hospitals, shelters, dense housing — never


class EffectorType(str, Enum):
    NET = "net"
    PROJECTILE = "projectile"


class EngagementDecision(str, Enum):
    AUTHORIZED = "authorized"
    HOLD = "hold"                 # geometry unsafe now, keep shaping
    DENIED = "denied"             # no acceptable geometry reachable


class UavMode(str, Enum):
    IDLE = "idle"
    TRANSIT = "transit"
    PURSUIT = "pursuit"
    HERDING = "herding"           # cooperative role: push target, don't shoot
    BLOCKING = "blocking"         # cooperative role: deny an escape direction
    ENGAGE = "engage"
    RTB = "rtb"


# ---------------------------------------------------------------------------
# Sensing / perception messages
# ---------------------------------------------------------------------------


@dataclass
class Detection:
    """A single sensor measurement of (possibly) one airborne object.

    Maps onto a future ``coopuavs_msgs/Detection.msg``. Position fields are
    only meaningful for the dimensions the sensor actually measures; the
    measurement covariance ``cov`` (3x3, map frame) encodes the rest, with
    very large variance on unobserved axes (e.g. range for an RF bearing).
    """

    header: Header
    sensor_id: str
    position: np.ndarray = field(default_factory=_vec3)
    cov: np.ndarray = field(default_factory=lambda: np.eye(3) * 1e6)
    radial_velocity: float | None = None        # radar Doppler, m/s
    rf_signature: str | None = None             # RF fingerprint hash, if any
    class_likelihoods: dict[ThreatClass, float] = field(default_factory=dict)
    snr: float = 0.0


@dataclass
class Track:
    """Fused, filtered estimate of one hostile object (system track).

    ``class_belief`` is the running Bayesian belief over
    :class:`ThreatClass`; ``p_decoy`` is the marginal probability the object
    is a non-explosive decoy and is the key input to engagement priority.
    """

    header: Header
    track_id: int
    position: np.ndarray = field(default_factory=_vec3)
    velocity: np.ndarray = field(default_factory=_vec3)
    cov: np.ndarray = field(default_factory=lambda: np.eye(6))
    class_belief: dict[ThreatClass, float] = field(default_factory=dict)
    p_decoy: float = 0.0
    n_hits: int = 0
    age: float = 0.0
    time_since_update: float = 0.0

    @property
    def speed(self) -> float:
        return float(np.linalg.norm(self.velocity))


@dataclass
class TrackArray:
    header: Header
    tracks: list[Track] = field(default_factory=list)


# ---------------------------------------------------------------------------
# C2 messages
# ---------------------------------------------------------------------------


@dataclass
class ThreatAssessment:
    """Output of threat evaluation for one track (TEWA stage 1)."""

    header: Header
    track_id: int
    threat_score: float                 # 0..1, drives engagement ordering
    time_to_impact: float               # s until predicted ground impact
    predicted_impact: np.ndarray = field(default_factory=_vec3)
    impact_zone: ZoneClass = ZoneClass.SAFE


@dataclass
class EngagementTask:
    """Assignment of one or more UAVs to a track (TEWA stage 2 output).

    ``shooter_id`` carries the designated effector platform; ``support_ids``
    are cooperative wingmen flying herding/blocking roles to shape the
    target's trajectory toward ``desired_kill_box``.
    """

    header: Header
    task_id: int
    track_id: int
    shooter_id: str
    support_ids: list[str] = field(default_factory=list)
    desired_kill_box: np.ndarray = field(default_factory=_vec3)  # centre, map
    priority: float = 0.0


@dataclass
class FireRequest:
    """Shooter asks the C2 for release authority (man-on-the-loop ready)."""

    header: Header
    task_id: int
    uav_id: str
    track_id: int
    effector: EffectorType = EffectorType.NET
    predicted_intercept: np.ndarray = field(default_factory=_vec3)
    p_kill: float = 0.0


@dataclass
class FireClearance:
    header: Header
    task_id: int
    uav_id: str
    decision: EngagementDecision = EngagementDecision.HOLD
    expected_collateral: float = 0.0
    reason: str = ""


@dataclass
class EngagementResult:
    header: Header
    task_id: int
    track_id: int
    uav_id: str
    hit: bool = False
    debris_impact: np.ndarray | None = None
    debris_zone: ZoneClass | None = None


# ---------------------------------------------------------------------------
# Platform state messages
# ---------------------------------------------------------------------------


@dataclass
class UavState:
    """Telemetry of a friendly interceptor (maps to ``nav_msgs/Odometry`` +
    mission fields)."""

    header: Header
    uav_id: str = ""
    position: np.ndarray = field(default_factory=_vec3)
    velocity: np.ndarray = field(default_factory=_vec3)
    mode: UavMode = UavMode.IDLE
    battery: float = 1.0                # 0..1 remaining
    ammo: int = 0
    task_id: int | None = None
