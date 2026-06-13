"""Reconcile per-agent policy actions into the C2 task set.

This is the safety-critical seam between the learned cooperation policy and
the unchanged engagement stack. The policy only expresses *commitment
intent* — which threat each interceptor should engage, and whether as a
shooter or a cooperative blocker. Everything that follows is the classical,
trusted machinery:

* the *actual* trigger platform for a track is chosen by
  :func:`coopuavs.c2.assignment._best_shooter` verbatim, so the Pk-aware,
  closing-speed-eligible, incumbent-discounted pick the ROE geometry assumes
  is still authoritative even if the policy mis-commits;
* blocking-vs-herding is derived downstream from target-vs-shooter speed in
  the interceptor FSM (``mc/cooperation``), not chosen here;
* falling debris is handled by the classical allocator over the platforms
  the policy left spare, so wreckage interception (PHY-GCS-006) is never
  dropped by swapping in the policy;
* denied tracks, debris-effector eligibility and platform availability are
  re-checked here — a sampled action can never produce an unsafe task.

The policy therefore learns the genuinely hard, cooperative decision —
*which* targets to commit *how many* platforms to, in *what role*, given the
whole picture — while the deterministic core keeps the engagement safe and
the task-id/clearance correlation intact.
"""

from __future__ import annotations

import numpy as np

from ..c2 import assignment
from ..core.messages import EngagementTask, Header
from .spaces import K_TRACKS


def build_track_table(assessments: dict, debris_info: dict | None,
                      k: int = K_TRACKS) -> list[int]:
    """Ordered top-``k`` *real* (non-debris) track ids by threat score — the
    table action index ``1..k`` (shoot) / ``k+1..2k`` (block) selects into.
    Same ordering the classical allocator serves tracks in
    (``assignment.allocate`` sorts by ``threat_score`` descending)."""
    debris_info = debris_info or {}
    real = [a for a in assessments.values() if a.track_id not in debris_info]
    real.sort(key=lambda a: a.threat_score, reverse=True)
    return [a.track_id for a in real[:k]]


def decode_action(action: int) -> tuple[int, str]:
    """Map a flat action to ``(slot, role)``: slot is the 0-based index into
    the track table; role is 'idle' | 'shoot' | 'block'."""
    a = int(action)
    if a <= 0:
        return -1, "idle"
    if a <= K_TRACKS:
        return a - 1, "shoot"
    if a <= 2 * K_TRACKS:
        return a - 1 - K_TRACKS, "block"
    return -1, "idle"


def actions_to_tasks(
    actions: dict[str, int],
    track_table: list[int],
    *,
    assessments: dict,
    tracks: dict,
    available: dict,
    uav_speeds: dict,
    risk_map,
    t: float,
    incumbents: dict | None = None,
    task_ids: dict | None = None,
    debris_info: dict | None = None,
    uav_effectors: dict | None = None,
    denied_tracks=frozenset(),
) -> list[EngagementTask]:
    """Turn ``{uav_id: action}`` into a list of :class:`EngagementTask`.

    ``available`` is the C2's currently-usable shooter set (ammo/battery/mode/
    telemetry filtered) keyed by id; actions from platforms not in it are
    ignored (they are recovering or off the net)."""
    incumbents = incumbents or {}
    task_ids = task_ids if task_ids is not None else {}
    debris_info = debris_info or {}
    uav_effectors = uav_effectors or {}

    # Group commitments by track id, honouring shoot-vs-block intent.
    shoot_choosers: dict[int, list[str]] = {}
    block_choosers: dict[int, list[str]] = {}
    for uid, action in actions.items():
        if uid not in available:
            continue
        slot, role = decode_action(action)
        if role == "idle" or slot < 0 or slot >= len(track_table):
            continue
        track_id = track_table[slot]
        if track_id not in tracks or track_id not in assessments:
            continue
        if track_id in denied_tracks:
            # ROE found no acceptable geometry: never commit, whatever the
            # policy chose (defensive — the obs mask already discourages it).
            continue
        (shoot_choosers if role == "shoot" else block_choosers).setdefault(
            track_id, []).append(uid)

    pool = dict(available)          # consumed as platforms are assigned
    tasks: list[EngagementTask] = []

    # Engage in threat order so the highest-priority track claims the best
    # platform first under contention (same discipline as the classical path).
    committed_tracks = set(shoot_choosers) | set(block_choosers)
    for track_id in sorted(committed_tracks,
                           key=lambda tid: assessments[tid].threat_score,
                           reverse=True):
        if not pool:
            break
        trk = tracks[track_id]
        a = assessments[track_id]
        is_debris = track_id in debris_info
        # Honour shoot intent for the trigger role; fall back to any committer
        # (promotion) so a block-only commitment is not orphaned.
        shooters = [u for u in shoot_choosers.get(track_id, []) if u in pool]
        others = [u for u in block_choosers.get(track_id, []) if u in pool]
        shooter_pool = shooters or others
        if is_debris:
            # Debris is destroyed only by projectile carriers (PHY-GCS-006);
            # net intent on debris is dropped.
            shooter_pool = [u for u in shooter_pool
                            if uav_effectors.get(u, "projectile") == "projectile"]
        if not shooter_pool:
            continue
        cand = {u: pool[u] for u in shooter_pool}
        shooter_id, _t_int = assignment._best_shooter(
            trk, a, cand, uav_speeds, incumbents, uav_effectors)
        if shooter_id is None:
            continue
        del pool[shooter_id]

        support_ids: list[str] = []
        if not is_debris:       # ballistics ignore herding/blocking
            extra = [u for u in (shooters + others)
                     if u != shooter_id and u in pool]
            for u in extra[:assignment.MAX_SUPPORT_PER_TASK]:
                support_ids.append(u)
                del pool[u]

        pairing = (track_id, shooter_id)
        if pairing not in task_ids:
            task_ids[pairing] = max(task_ids.values(), default=0) + 1
        kill_box_xy = risk_map.nearest_safe_cell(trk.position[0], trk.position[1])
        tasks.append(EngagementTask(
            header=Header(stamp=t),
            task_id=task_ids[pairing],
            track_id=track_id,
            shooter_id=shooter_id,
            support_ids=support_ids,
            desired_kill_box=np.array([kill_box_xy[0], kill_box_xy[1], 0.0]),
            priority=a.threat_score,
            target_kind="debris" if is_debris else "track",
            debris_id=debris_info.get(track_id, "") if is_debris else "",
        ))

    # Falling debris the policy did not (or cannot) task is handled by the
    # classical allocator over the platforms left spare — wreckage
    # interception must never be lost by swapping in the policy.
    untasked_debris = {tid: a for tid, a in assessments.items()
                       if tid in debris_info
                       and not any(task.track_id == tid for task in tasks)}
    if untasked_debris and pool:
        debris_tracks = {tid: tracks[tid] for tid in untasked_debris if tid in tracks}
        debris_tasks = assignment.allocate(
            list(untasked_debris.values()), debris_tracks, list(pool.values()),
            uav_speeds, risk_map, t,
            incumbents=incumbents, task_ids=task_ids,
            debris_info={tid: debris_info[tid] for tid in untasked_debris},
            uav_effectors=uav_effectors,
        )
        tasks.extend(debris_tasks)

    return tasks
