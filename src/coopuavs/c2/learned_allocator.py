"""Deployment-side learned weapon-target allocator.

Wraps a trained MAPPO actor (:mod:`coopuavs.rl`) behind the exact
``assignment.allocate`` signature, so it drops straight into the
:class:`~coopuavs.c2.base_station.BaseStation` allocator seam. At each C2
planning cycle it encodes the same observation the policy was trained on
(via the shared :func:`coopuavs.rl.spaces.encode_observations`), runs the
shared actor once per available platform (deterministically by default), and
hands the chosen commitments to :func:`coopuavs.rl.reconcile.actions_to_tasks`
— which routes the actual shooter pick through ``assignment._best_shooter``
and preserves the debris/denied/effector safety gates. The learned policy
therefore decides *cooperation* (which targets, how many platforms, what
role); the trusted classical core keeps the engagement safe.

PyTorch is an optional dependency: :func:`get_allocator` raises a clear error
if a learned policy is requested without torch or a checkpoint. Transient
inference faults at run time are caught by the BaseStation fence, which falls
back to the classical allocator for that cycle (deployment robustness).
"""

from __future__ import annotations

import numpy as np

from . import assignment


class LearnedAllocator:
    """Callable with the ``assignment.allocate`` signature backed by a policy."""

    def __init__(self, checkpoint_path: str, *, horizon: float = 600.0,
                 deterministic: bool = True, map_location: str = "cpu"):
        import torch  # noqa: F401 — fail loudly here if the extra is missing
        from ..rl.models import ActorCritic

        self.model = ActorCritic.load(checkpoint_path, map_location=map_location)
        self.actor = self.model.actor
        self.actor.eval()
        self.horizon = float(horizon)
        self.deterministic = bool(deterministic)
        self.checkpoint_path = checkpoint_path

    def __call__(self, assessments, tracks, uavs, uav_speeds, risk_map, t,
                 *, denied_tracks=frozenset(), incumbents=None, task_ids=None,
                 debris_info=None, uav_effectors=None):
        import torch

        from ..rl import reconcile, spaces

        incumbents = incumbents or {}
        debris_info = debris_info or {}
        uav_effectors = uav_effectors or {}
        assess_by_id = {a.track_id: a for a in assessments}
        available = {u.uav_id: u for u in uavs}
        table = reconcile.build_track_table(assess_by_id, debris_info)

        actions = {uid: 0 for uid in available}
        if table and available:
            agent_states = [(u.uav_id, u) for u in uavs]
            fleet_ammo = float(np.clip(
                np.mean([u.ammo / spaces.AMMO_SCALE for u in uavs]), 0.0, 1.0))
            obs, masks = spaces.encode_observations(
                agent_states, list(uavs), table, assess_by_id, tracks, t,
                denied=set(denied_tracks), incumbents=incumbents,
                debris_info=debris_info, horizon=self.horizon,
                fleet_ammo_frac=fleet_ammo)
            ids = [u.uav_id for u in uavs]
            obs_b = torch.from_numpy(np.stack([obs[i] for i in ids]).astype(np.float32))
            mask_b = torch.from_numpy(np.stack([masks[i] for i in ids]))
            with torch.no_grad():
                act, _ = self.actor.act(obs_b, mask_b, deterministic=self.deterministic)
            actions = {ids[k]: int(act[k]) for k in range(len(ids))}

        return reconcile.actions_to_tasks(
            actions, table, assessments=assess_by_id, tracks=tracks,
            available=available, uav_speeds=uav_speeds, risk_map=risk_map, t=t,
            incumbents=incumbents, task_ids=task_ids, debris_info=debris_info,
            uav_effectors=uav_effectors, denied_tracks=set(denied_tracks))


def get_allocator(spec=None, checkpoint_path: str | None = None, **kwargs):
    """Resolve a base-station allocator from a config spec.

    ``spec`` of ``None`` / ``"greedy"`` / ``"classical"`` returns the classical
    :func:`assignment.allocate`. ``"learned"`` (with ``checkpoint_path``) or a
    checkpoint path string returns a :class:`LearnedAllocator`. Raises if a
    learned policy is requested without torch or a checkpoint — a config error
    must surface, not silently degrade to greedy.
    """
    if spec in (None, "greedy", "classical", "default", False):
        return assignment.allocate
    path = checkpoint_path
    if spec not in ("learned", True) and path is None:
        path = spec                     # spec given directly as the path
    if not path:
        raise ValueError(
            "a learned allocator requires a policy checkpoint path "
            "(base_station.policy or the spec string)")
    try:
        import torch  # noqa: F401
    except ImportError as exc:
        raise ImportError(
            "the learned allocator requires PyTorch — install the training "
            "extras: pip install -e '.[train]'") from exc
    return LearnedAllocator(path, **kwargs)
