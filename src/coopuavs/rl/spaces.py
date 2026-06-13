"""Observation/action encoding for the learned weapon-target policy.

Pure-numpy, dependency-free (no torch/gymnasium): the feature layout is the
contract shared by the training env (:mod:`coopuavs.rl.env`), the MAPPO
networks (:mod:`coopuavs.rl.mappo`) and the deployment allocator
(:mod:`coopuavs.c2.learned_allocator`), so all three encode the world the
same way. Keeping it here and importable without the optional ML stack lets
the encoding be unit-tested in the base install.

Design (validated against the classical allocator, docs/MARL.md):

* **Agents** are the interceptors; one parameter-shared actor is applied per
  agent on an **ego-centric** observation (relative positions/velocities),
  so the same weights generalise across platforms and fleet sizes.
* **Action** is a single masked categorical over ``1 + 2*K`` choices: idle,
  *shoot* one of the top-``K`` threat tracks, or *block* (cooperative
  support post) one of them. Blocking-vs-herding is not an agent choice — it
  is derived downstream from target-vs-shooter speed in the interceptor FSM,
  exactly as in the classical path — so the policy only commits role intent.
* **Top-K by threat score**: the action index ``k`` selects the k-th track
  of an ordered table the env rebuilds every cycle (sorted by threat score,
  the same order ``assignment.allocate`` serves tracks in). The integer
  action therefore means the same *kind* of decision every step even though
  the physical track set is dynamic; the env maps ``k`` back to the live
  ``track_id`` for reconciliation.
* Per-track flags mirror the caller-owned state the classical allocator is
  gated on (denied TTL, incumbent shooter, debris pseudo-track, catchability)
  so the policy is not blind to the constraints the reconciliation enforces.
"""

from __future__ import annotations

import zlib

import numpy as np

# --- sizes -----------------------------------------------------------------
K_TRACKS = 6          # threat tracks the policy attends to / can act on
M_MATES = 4           # teammates summarised in the observation
NUM_ACTIONS = 1 + 2 * K_TRACKS    # idle | shoot k (1..K) | block k (K+1..2K)

# --- normalisation scales (fixed so the obs space is stationary) -----------
POS_SCALE = 6000.0    # ~map half-extent, m
ALT_SCALE = 5000.0    # operational altitude band, m
VEL_SCALE = 120.0     # just above the jet OWA cruise speed, m/s
TTI_SCALE = 120.0     # time-to-impact horizon, s
AMMO_SCALE = 8.0      # projectile magazine, rounds

# Per-block feature widths (asserted against the encoders below).
OWN_FEAT = 13
TRACK_FEAT = 20
MATE_FEAT = 10
GLOBAL_FEAT = 3
OBS_DIM = OWN_FEAT + K_TRACKS * TRACK_FEAT + M_MATES * MATE_FEAT + GLOBAL_FEAT


def _extrapolate(pos, vel, dt: float):
    """Constant-velocity extrapolation to the decision instant — the same
    correction guidance and fire control apply (uav.py), so the policy sees
    the world the executors act on, not a tick-stale snapshot."""
    return np.asarray(pos, dtype=float) + np.asarray(vel, dtype=float) * max(0.0, dt)


def id_token(uav_id: str) -> float:
    """Stable per-platform self-context scalar in [0, 1) derived from the id
    (crc32 is unsalted, so it is identical in training and deployment and
    independent of which platforms happen to be available this cycle — a
    positional slot would shift as airframes recover/rearm)."""
    return (zlib.crc32(uav_id.encode()) % 1000) / 1000.0


def own_features(pos, vel, *, battery: float, ammo: int, is_projectile: bool,
                 is_tasked: bool, self_token: float) -> np.ndarray:
    speed = float(np.linalg.norm(vel))
    f = np.array([
        pos[0] / POS_SCALE, pos[1] / POS_SCALE, pos[2] / ALT_SCALE,
        vel[0] / VEL_SCALE, vel[1] / VEL_SCALE, vel[2] / VEL_SCALE,
        speed / VEL_SCALE,
        float(np.clip(battery, 0.0, 1.0)),
        float(np.clip(ammo / AMMO_SCALE, 0.0, 1.0)),
        1.0 if is_projectile else 0.0,        # effector one-hot: projectile
        0.0 if is_projectile else 1.0,        # effector one-hot: net
        1.0 if is_tasked else 0.0,
        # self-context token for the parameter-shared actor (M4).
        float(self_token),
    ], dtype=np.float32)
    assert f.size == OWN_FEAT
    return f


def track_features(track, threat_score: float, tti: float, impact_zone: int,
                   ego_pos, t: float, *, catchable: bool, incumbent: bool,
                   denied: bool, debris: bool, predicted_impact=None) -> np.ndarray:
    """One threat-track slot, ego-relative and extrapolated to ``t``."""
    dt = t - float(track.header.stamp)
    pos = _extrapolate(track.position, track.velocity, dt)
    vel = np.asarray(track.velocity, dtype=float)
    rel = pos - np.asarray(ego_pos, dtype=float)
    zone = int(impact_zone)
    zone_oh = [1.0 if zone == z else 0.0 for z in (0, 1, 2)]
    if predicted_impact is None:
        predicted_impact = pos
    imp_rel = np.asarray(predicted_impact, dtype=float)[:2] - np.asarray(ego_pos, dtype=float)[:2]
    f = np.array([
        1.0,                                   # valid (real, not padding)
        rel[0] / POS_SCALE, rel[1] / POS_SCALE, rel[2] / ALT_SCALE,
        vel[0] / VEL_SCALE, vel[1] / VEL_SCALE, vel[2] / VEL_SCALE,
        float(np.linalg.norm(vel)) / VEL_SCALE,
        float(np.clip(threat_score, 0.0, 1.0)),
        float(np.clip(tti / TTI_SCALE, 0.0, 1.0)),
        float(np.clip(getattr(track, "p_decoy", 0.0), 0.0, 1.0)),
        zone_oh[0], zone_oh[1], zone_oh[2],
        1.0 if catchable else 0.0,
        1.0 if incumbent else 0.0,
        1.0 if denied else 0.0,
        1.0 if debris else 0.0,
        imp_rel[0] / POS_SCALE, imp_rel[1] / POS_SCALE,
    ], dtype=np.float32)
    assert f.size == TRACK_FEAT
    return f


def mate_features(uav, ego_pos, t: float) -> np.ndarray:
    """One teammate slot: ego-relative position, role, ammo, capability."""
    dt = t - float(uav.header.stamp)
    pos = _extrapolate(uav.position, uav.velocity, dt)
    rel = pos - np.asarray(ego_pos, dtype=float)
    mode = getattr(uav.mode, "value", str(uav.mode))
    is_shooter = mode in ("pursuit", "engage")
    is_support = mode in ("blocking", "herding")
    f = np.array([
        1.0,                                   # valid
        rel[0] / POS_SCALE, rel[1] / POS_SCALE, rel[2] / ALT_SCALE,
        1.0 if is_shooter else 0.0,
        1.0 if is_support else 0.0,
        1.0 if not (is_shooter or is_support) else 0.0,   # idle/other
        float(np.clip(uav.ammo / AMMO_SCALE, 0.0, 1.0)),
        float(uav.max_speed) / VEL_SCALE,
        1.0 if uav.effector == "projectile" else 0.0,
    ], dtype=np.float32)
    assert f.size == MATE_FEAT
    return f


def global_features(t: float, horizon: float, n_tracks: int,
                    fleet_ammo_frac: float) -> np.ndarray:
    f = np.array([
        float(np.clip(t / max(horizon, 1.0), 0.0, 1.0)),
        float(np.clip(n_tracks / K_TRACKS, 0.0, 2.0)),
        float(np.clip(fleet_ammo_frac, 0.0, 1.0)),
    ], dtype=np.float32)
    assert f.size == GLOBAL_FEAT
    return f


def assemble(own: np.ndarray, tracks: list[np.ndarray],
             mates: list[np.ndarray], glob: np.ndarray) -> np.ndarray:
    """Concatenate the blocks, zero-padding the track/teammate slots to a
    fixed length. Padding slots carry ``valid=0`` (first element), so the
    network can tell a pad from a genuine all-zero feature."""
    track_block = np.zeros((K_TRACKS, TRACK_FEAT), dtype=np.float32)
    for i, tf in enumerate(tracks[:K_TRACKS]):
        track_block[i] = tf
    mate_block = np.zeros((M_MATES, MATE_FEAT), dtype=np.float32)
    for i, mf in enumerate(mates[:M_MATES]):
        mate_block[i] = mf
    obs = np.concatenate([own, track_block.ravel(), mate_block.ravel(), glob])
    assert obs.size == OBS_DIM
    return obs.astype(np.float32)


def action_mask(n_tracks: int, denied: list[bool], debris: list[bool],
                ego_is_projectile: bool) -> np.ndarray:
    """Legal-action mask of length :data:`NUM_ACTIONS`.

    idle is always legal. ``shoot k`` / ``block k`` require a real track in
    slot ``k``; a denied track takes neither; debris pseudo-tracks accept a
    *shoot* only from a projectile carrier (nets cannot destroy a falling
    airframe) and never a block (ballistics ignore herding) — mirroring the
    classical allocator's gates so a sampled action always reconciles to a
    valid task."""
    mask = np.zeros(NUM_ACTIONS, dtype=bool)
    mask[0] = True
    for k in range(min(n_tracks, K_TRACKS)):
        is_denied = denied[k] if k < len(denied) else False
        is_debris = debris[k] if k < len(debris) else False
        if is_denied:
            continue
        if is_debris:
            if ego_is_projectile:
                mask[1 + k] = True            # shoot only, projectile only
            continue
        mask[1 + k] = True                    # shoot k
        mask[1 + K_TRACKS + k] = True         # block k
    return mask


def encode_observations(agent_states, mate_states, track_table, assess_by_id,
                        tracks, t, *, denied, incumbents, debris_info, horizon,
                        fleet_ammo_frac):
    """Encode the per-agent observation + action mask for a set of agents.

    The single encoder shared by training (:mod:`coopuavs.rl.env`) and
    deployment (:mod:`coopuavs.c2.learned_allocator`), so the policy sees an
    identically-built observation in both — no train/deploy drift.

    ``agent_states`` is ``[(uav_id, UavState), ...]`` for the agents to
    encode; ``mate_states`` is the pool of teammate ``UavState`` for the
    nearest-``M`` summary. Per-track ego flags (catchable/incumbent/denied/
    debris) mirror the classical allocator's gates.
    """
    from ..mc.guidance import intercept_time

    glob = global_features(t, horizon, len(track_table), fleet_ammo_frac)
    denied_flags = [tid in denied for tid in track_table]
    debris_flags = [tid in debris_info for tid in track_table]
    obs, masks = {}, {}
    for uid, ego in agent_states:
        ego_pos = np.asarray(ego.position, dtype=float)
        ego_speed = ego.max_speed or 1.0
        is_proj = (ego.effector == "projectile")
        track_feats = []
        for tid in track_table:
            trk, a = tracks[tid], assess_by_id[tid]
            catch = intercept_time(np.asarray(trk.position) - ego_pos,
                                   np.asarray(trk.velocity), ego_speed) is not None
            track_feats.append(track_features(
                trk, a.threat_score, a.time_to_impact, int(a.impact_zone),
                ego_pos, t, catchable=catch,
                incumbent=(incumbents.get(tid) == uid),
                denied=(tid in denied), debris=(tid in debris_info),
                predicted_impact=a.predicted_impact))
        mates = sorted(
            ((float(np.linalg.norm(np.asarray(s.position) - ego_pos)), s)
             for s in mate_states if s.uav_id != uid), key=lambda d: d[0])
        mate_feats = [mate_features(s, ego_pos, t) for _, s in mates[:M_MATES]]
        own = own_features(
            ego_pos, np.asarray(ego.velocity, dtype=float),
            battery=ego.battery, ammo=ego.ammo, is_projectile=is_proj,
            is_tasked=(ego.task_id is not None), self_token=id_token(uid))
        obs[uid] = assemble(own, track_feats, mate_feats, glob)
        masks[uid] = action_mask(len(track_table), denied_flags, debris_flags, is_proj)
    return obs, masks


def make_spaces():
    """gymnasium ``(observation_space, action_space)`` if gymnasium is
    installed, else ``None`` — the trainer reads dims from the constants
    above and does not require the formal spaces."""
    try:
        from gymnasium import spaces
    except Exception:       # pragma: no cover - optional dependency
        return None
    obs = spaces.Box(low=-np.inf, high=np.inf, shape=(OBS_DIM,), dtype=np.float32)
    act = spaces.Discrete(NUM_ACTIONS)
    return obs, act
