"""Threat evaluation (the TE of TEWA).

Scores every confirmed track for engagement ordering. The score blends:

* expected lethality — probability the object carries a warhead
  (``1 - p_decoy``); decoys are deprioritised but never zeroed, because a
  5% chance of a 200 kg warhead still beats most certainties;
* value of the asset it is predicted to hit and the ground class under the
  predicted impact point;
* urgency — inverse time-to-impact.
"""

from __future__ import annotations

import numpy as np

from ..core.messages import Header, ThreatAssessment, Track
from ..sim.environment import Environment


def predicted_impact(track: Track, env: Environment, horizon: float = 300.0) -> tuple[np.ndarray, float]:
    """Predicted ground impact point and time.

    Geometry is evaluated in the horizontal plane: a cruising OWA at 1500 m
    AGL is still *heading at* its ground asset, so altitude must not mask
    the threat. Time-to-impact for a matched asset is the horizontal
    closest-approach time (the terminal dive is not modelled yet);
    otherwise straight-line extrapolation to the horizon.

    Only *closing* geometry matches an asset: a track that just overflew
    an asset outbound is within the miss radius but receding — pinning its
    TTI to zero would hand an egressing drone maximum urgency and outrank
    every inbound threat.
    """
    p, v = track.position, track.velocity
    speed_xy = float(np.linalg.norm(v[:2]))
    if speed_xy < 1.0:
        return p.copy(), horizon

    best_t, best_point = horizon, p + v * horizon
    for asset in env.assets:
        rel_xy = asset.position[:2] - p[:2]
        closing = float(rel_xy @ v[:2])
        if closing <= 0.0:
            continue
        t_closest = float(np.clip(closing / (speed_xy**2), 0.0, horizon))
        miss = float(np.linalg.norm(p[:2] + v[:2] * t_closest - asset.position[:2]))
        if miss < 400.0 and t_closest < best_t:
            best_t, best_point = t_closest, asset.position.copy()
    return best_point, best_t


def assess(track: Track, env: Environment, t: float) -> ThreatAssessment:
    impact, tti = predicted_impact(track, env)
    zone = env.risk_map.zone_at(impact[0], impact[1])

    asset_value = 0.2
    for asset in env.assets:
        if np.linalg.norm(impact[:2] - asset.position[:2]) < 300.0:
            asset_value = max(asset_value, asset.value)

    lethality = 1.0 - track.p_decoy
    urgency = 1.0 / (1.0 + tti / 60.0)
    zone_factor = 1.0 + 0.5 * int(zone)
    score = float(np.clip(lethality * urgency * asset_value * zone_factor, 0.0, 1.0))

    return ThreatAssessment(
        header=Header(stamp=t),
        track_id=track.track_id,
        threat_score=score,
        time_to_impact=tti,
        predicted_impact=impact,
        impact_zone=zone,
    )
