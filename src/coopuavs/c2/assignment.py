"""Weapon-target assignment (the WA of TEWA).

Priority-greedy allocation: tracks are served in threat order, and each
track gets the *package* it actually needs — a shooter when it is
kinematically catchable, a shooter **plus reserved blockers** when it
outruns every interceptor. This is the crucial difference from a plain
optimal-matching (Hungarian) assignment: under saturation, matching spreads
every UAV thin as a lone shooter and the cooperative relay never forms, so
fast targets leak with probability one. Letting low-priority tracks queue
is the correct trade.

Two stabilisers:

* incumbent discount — re-assignment must clearly beat the current pairing,
  or converged pursuit geometry is thrown away on estimate jitter;
* lead-point selection — shooters/blockers are scored against the target's
  *future* corridor, not its current position; a UAV behind a fast target
  is worthless however close it is.

The decentralised upgrade path (consensus-based bundle auctions, CBBA)
keeps this exact interface: ``allocate(...) -> list[EngagementTask]`` — see
docs/RESEARCH.md §2.
"""

from __future__ import annotations

import numpy as np

from ..core.messages import EngagementTask, Header, ThreatAssessment, Track, UavState
from ..interceptors.guidance import intercept_time
from ..risk.zones import RiskMap

DECOY_IGNORE_THRESHOLD = 0.85   # p_decoy above which a track gets no shooter
MAX_SUPPORT_PER_TASK = 2
INCUMBENT_DISCOUNT = 0.7        # hysteresis: re-assignment must beat this margin
CORRIDOR_LEAD_S = 30.0          # blockers are scored against t+30 s corridor


def allocate(
    assessments: list[ThreatAssessment],
    tracks: dict[int, Track],
    uavs: list[UavState],
    uav_speeds: dict[str, float],
    risk_map: RiskMap,
    t: float,
    denied_tracks: set[int] = frozenset(),
    incumbents: dict[int, str] | None = None,
    task_ids: dict[tuple[int, str], int] | None = None,
    directive: "object | None" = None,
) -> list[EngagementTask]:
    """``task_ids`` is the caller-owned registry mapping a (track, shooter)
    pairing to its task id. A pairing keeps its id across planning cycles —
    a :class:`~coopuavs.core.messages.FireClearance` must stay correlatable
    to the engagement it was requested for, which a fresh id per cycle
    would silently break."""
    incumbents = incumbents or {}
    task_ids = task_ids if task_ids is not None else {}
    # The optional slow-loop supervisor directive (advise-only): it may
    # re-weight, defer, confirm-first, or request engagement depth, but it
    # reaches allocation only as ordering and tasking — never as clearance.
    deferred = set(getattr(directive, "defer", ()) or ())
    confirm_first = set(getattr(directive, "confirm_first", ()) or ())
    weights = getattr(directive, "target_weights", {}) or {}
    k_shooter = getattr(directive, "k_shooter", {}) or {}

    def eff_weight(tid: int) -> float:
        w = float(weights.get(tid, 1.0))
        if tid in confirm_first and tid not in weights:
            w *= 0.5            # earn a sensor ID before committing a shooter
        return w

    engage = [
        a for a in assessments
        if a.track_id in tracks
        and a.track_id not in denied_tracks
        and a.track_id not in deferred
        and tracks[a.track_id].p_decoy < DECOY_IGNORE_THRESHOLD
    ]
    engage.sort(key=lambda a: a.threat_score * eff_weight(a.track_id), reverse=True)
    available = {u.uav_id: u for u in uavs}
    tasks: list[EngagementTask] = []
    catchable_shooters: dict[int, str] = {}   # track -> primary shooter (for depth)

    for idx, a in enumerate(engage):
        if not available:
            break
        trk = tracks[a.track_id]

        shooter_id, t_int = _best_shooter(trk, a, available, uav_speeds, incumbents)
        shooter_speed = uav_speeds.get(shooter_id, 30.0)
        del available[shooter_id]
        if t_int is not None:
            catchable_shooters[a.track_id] = shooter_id

        kill_box_xy = risk_map.nearest_safe_cell(trk.position[0], trk.position[1])
        pairing = (a.track_id, shooter_id)
        if pairing not in task_ids:
            task_ids[pairing] = max(task_ids.values(), default=0) + 1
        task = EngagementTask(
            header=Header(stamp=t),
            task_id=task_ids[pairing],
            track_id=a.track_id,
            shooter_id=shooter_id,
            desired_kill_box=np.array([kill_box_xy[0], kill_box_xy[1], 0.0]),
            priority=a.threat_score,
        )

        # Reserve blockers when the shooter cannot win alone: target faster
        # than the shooter, or no tail-chase solution at all. The blockers
        # take cutoff posts down-corridor; as the target reaches a post the
        # blocker becomes the catchable best shooter and roles rotate.
        # Budget rule: a queued track's shooter is never spent as a blocker
        # — saturating raids would otherwise starve on support reservation.
        needs_support = t_int is None or trk.speed > 0.95 * shooter_speed
        remaining_tracks = len(engage) - idx - 1
        support_budget = max(0, len(available) - remaining_tracks)
        if needs_support and support_budget > 0:
            n_sup = min(MAX_SUPPORT_PER_TASK, support_budget)
            lead_point = trk.position + trk.velocity * CORRIDOR_LEAD_S
            by_lead = sorted(
                available.values(),
                key=lambda u: float(np.linalg.norm(lead_point - u.position)),
            )
            for u in by_lead[:n_sup]:
                task.support_ids.append(u.uav_id)
                del available[u.uav_id]

        tasks.append(task)

    # Depth pass (shoot-look-shoot): a hard, savable, high-value target the
    # supervisor flagged for k>=2 gets a second independent shooter from
    # whatever capacity is left after every other track has its primary. Each
    # extra shooter is a normal task and still passes the ROE per shot.
    for tid, k in sorted(k_shooter.items(), key=lambda kv: -tracks[kv[0]].speed
                         if kv[0] in tracks else 0.0):
        if not available or k < 2 or tid not in catchable_shooters or tid not in tracks:
            continue
        trk = tracks[tid]
        for _ in range(k - 1):
            if not available:
                break
            a = next((x for x in engage if x.track_id == tid), None)
            extra_id, t_int = _best_shooter(trk, a, available, uav_speeds, incumbents)
            if extra_id is None or t_int is None:
                break              # no remaining UAV can actually catch it
            del available[extra_id]
            kill_box_xy = risk_map.nearest_safe_cell(trk.position[0], trk.position[1])
            pairing = (tid, extra_id)
            if pairing not in task_ids:
                task_ids[pairing] = max(task_ids.values(), default=0) + 1
            tasks.append(EngagementTask(
                header=Header(stamp=t),
                task_id=task_ids[pairing],
                track_id=tid,
                shooter_id=extra_id,
                desired_kill_box=np.array([kill_box_xy[0], kill_box_xy[1], 0.0]),
                priority=a.threat_score if a else trk.speed,
            ))

    return tasks


def _best_shooter(
    trk: Track,
    a: ThreatAssessment,
    available: dict[str, UavState],
    uav_speeds: dict[str, float],
    incumbents: dict[int, str],
) -> tuple[str, float | None]:
    """Lowest effective intercept time among available UAVs.

    Uncatchable pairings cost the corridor flight time instead, so when
    nobody can catch the target the UAV best placed on the future corridor
    still takes the (blocking) shot."""
    lead_point = trk.position + trk.velocity * CORRIDOR_LEAD_S
    best_id, best_cost, best_t_int = None, np.inf, None
    for uav_id, u in available.items():
        speed = uav_speeds.get(uav_id, 30.0)
        t_int = intercept_time(trk.position - u.position, trk.velocity, speed)
        if t_int is not None:
            cost = t_int
        else:
            cost = 300.0 + float(np.linalg.norm(lead_point - u.position)) / max(speed, 1.0)
        if incumbents.get(trk.track_id) == uav_id:
            cost *= INCUMBENT_DISCOUNT
        if cost < best_cost:
            best_id, best_cost, best_t_int = uav_id, cost, t_int
    return best_id, best_t_int
