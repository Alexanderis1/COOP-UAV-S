"""Multi-agent environment wrapping the battle sim for learned WTA.

A PettingZoo-``ParallelEnv``-shaped cooperative environment in which the
agents are the interceptors and the action is the weapon-target *commitment*
the base-station C2 would otherwise compute classically. The env installs an
allocator hook on the :class:`~coopuavs.c2.base_station.BaseStation` (the B2
seam) so the policy drives exactly the decision ``assignment.allocate``
drives, and **nothing downstream changes** — sensing, fusion, ROE,
fire-control and the orchestrator are the unmodified, trusted stack.

Cadence (one env step = one C2 planning cycle, ~1 s of sim):

    reset(seed)         build a fresh World (B4 — never pickle a live one),
                        advance to the first BaseStation tick, return obs.
    step(actions)       stage the per-agent actions, advance ~20 dt=0.05
                        sub-steps to the next BaseStation tick where the
                        allocator hook reconciles them into tasks, then read
                        obs/reward/done.

The policy's plan at decision *t* is committed at the next C2 cycle (a one-
tick, ~1 s planning latency — realistic, and harmless when the trained net
is later run synchronously inside the deployment allocator).

Training hygiene: macro-step World only (no SITL micro-loop), the Recorder
is detached (pure overhead), and the raid is domain-randomised per seed so
the parameter-shared policy generalises across raid sizes and axes (M7).

Reward (shared team outcome + per-agent waste, the design's factored credit):
big penalty for armed leakers, reward for armed kills, zone-weighted penalty
for debris/stray collateral, per-agent penalty for that agent's ammo and
decoy shots, a task-churn penalty re-creating the classical incumbent
hysteresis, and a potential-based safety-shaping term. Weights are tunable.
"""

from __future__ import annotations

import copy

import numpy as np

from ..c2.base_station import BaseStation
from ..core.messages import Header, ThreatClass, UavState
from ..risk.zones import ZONE_WEIGHTS
from ..sim import scenario as scenario_mod
from ..viz.recorder import Recorder
from . import reconcile, spaces

try:    # be a genuine PettingZoo ParallelEnv when the extra is present
    from pettingzoo import ParallelEnv as _ParallelEnvBase
except Exception:    # pragma: no cover - optional dependency
    _ParallelEnvBase = object

# Event kinds that spend a munition (mirrors evaluation.SHOT_EVENT_KINDS).
_SHOT_KINDS = {"kill", "miss", "fire_no_target", "debris_neutralized",
               "fire_blocked_los"}

DEFAULT_REWARD_WEIGHTS = {
    "armed_kill": 1.0,       # +per armed threat destroyed
    "armed_leak": 3.0,       # -per armed threat that reaches its asset (worst)
    "decoy_kill": 0.2,       # -per decoy destroyed (wasted munition)
    "debris": 0.1,           # -per unit zone-weighted wreck/stray cost
    "ammo": 0.03,            # -per own munition released
    "decoy_shot": 0.3,       # -per own shot at a (true) decoy
    "churn": 0.02,           # -per track whose shooter changed vs last cycle
    "shape": 0.5,            # potential-based safety shaping coefficient
    "time": 0.003,           # -per step, nudge toward resolving quickly
}
_PHI_SCALE = 8000.0          # proximity falloff for the safety potential, m


def _initial_state(uav) -> UavState:
    """Synthetic telemetry for a platform that has not published yet (the
    first BaseStation tick runs before the interceptor nodes in step 0)."""
    return UavState(
        header=Header(stamp=0.0), uav_id=uav.uav_id,
        position=uav.body.position.copy(), velocity=uav.body.velocity.copy(),
        mode=uav.mode, battery=uav.battery, ammo=uav.effector.ammo,
        max_speed=uav.max_speed, kind="interceptor",
        effector=uav.effector.type.value,
    )


class CoopWtaParallelEnv(_ParallelEnvBase):
    """Parallel multi-agent env over the COOP-UAV-S battle simulation.

    Implements the PettingZoo ParallelEnv surface (``reset``/``step``/
    ``observation_space``/``action_space``/``agents``) without importing
    pettingzoo, so it is usable in the base install; a conformance test runs
    only when pettingzoo is present.
    """

    metadata = {"name": "coop_wta_v0", "is_parallelizable": True}
    render_mode = None

    def render(self):       # the dashboard/recorder is the real visualiser
        return None

    def __init__(self, scenario_cfg, *, horizon: int = 300, gamma: float = 0.99,
                 randomize: bool = True, reward_weights: dict | None = None,
                 seed: int | None = None):
        if isinstance(scenario_cfg, (str, bytes)) or hasattr(scenario_cfg, "read_text"):
            import yaml
            from pathlib import Path
            scenario_cfg = yaml.safe_load(Path(scenario_cfg).read_text())
        self._cfg = copy.deepcopy(scenario_cfg)
        # Training scenarios run on the fast macro-step world; the SITL
        # micro-loop adds cost and MC-staleness with no training value.
        self._cfg.pop("fidelity", None)
        self._cfg.pop("sitl", None)
        self.horizon = int(horizon)
        self.gamma = float(gamma)
        self.randomize = bool(randomize)
        self.w = dict(DEFAULT_REWARD_WEIGHTS)
        if reward_weights:
            self.w.update(reward_weights)
        self._base_seed = seed

        # Discover the (fixed) agent set once from the config.
        ids = [u["id"] for u in self._cfg.get("interceptors", [])]
        self.possible_agents = sorted(ids)
        self.agents: list[str] = list(self.possible_agents)

        self._spaces = spaces.make_spaces()
        self._sc = None
        self._world = None
        self._bs: BaseStation | None = None
        self._uav_states: dict[str, UavState] = {}
        self._pending: dict[str, int] = {}
        self._tick_fired = False
        self._obs: dict[str, np.ndarray] = {}
        self._masks: dict[str, np.ndarray] = {}
        self._track_table: list[int] = []
        self._prev_shooters: dict[int, str] = {}
        self._ev_cursor = 0
        self._wreck_cursor = 0
        self._stray_cursor = 0
        self._phi = 0.0
        self._steps = 0

    # -- spaces -----------------------------------------------------------------

    def observation_space(self, agent=None):
        return self._spaces[0] if self._spaces else None

    def action_space(self, agent=None):
        return self._spaces[1] if self._spaces else None

    # -- episode lifecycle ------------------------------------------------------

    def reset(self, seed: int | None = None, options=None):
        if seed is None:
            seed = self._base_seed if self._base_seed is not None else 0
        cfg = self._build_episode_cfg(seed)
        self._sc = scenario_mod.build(cfg, seed=seed)
        self._world = self._sc.world
        # Detach the Recorder: frame storage is pure overhead in training.
        self._world.nodes = [n for n in self._world.nodes
                             if not isinstance(n, Recorder)]
        self._bs = next(n for n in self._world.nodes if isinstance(n, BaseStation))
        self._bs.allocator = self._allocator
        self._bs.allocator_strict = True       # surface policy bugs in training
        # Track every interceptor's latest telemetry off the bus, seeded with
        # the spawn state so the first tick has a full picture.
        self._uav_states = {uid: _initial_state(u)
                            for uid, u in self._sc.uavs.items()}
        self._world.bus.subscribe("uav/state", self._on_uav_state)

        self.agents = list(self.possible_agents)
        self._pending = {}
        self._prev_shooters = {}
        self._steps = 0
        self._tick_fired = False
        self._advance_to_tick()           # first plan (no actions -> all idle)
        # Start the reward cursors after the pre-game advance so events from
        # the first (idle) plan never leak into the first step's reward.
        self._ev_cursor = len(self._world.events)
        self._wreck_cursor = len(self._world.wrecks)
        self._stray_cursor = len(self._world.stray_impacts)
        self._phi = self._potential()
        infos = {a: {"action_mask": self._masks.get(a)} for a in self.agents}
        return dict(self._obs), infos

    def step(self, actions: dict[str, int]):
        self._pending = {a: int(actions.get(a, 0)) for a in self.agents}
        self._tick_fired = False
        done = self._advance_to_tick()
        rewards = self._compute_reward()
        terminated = self._raid_resolved()
        truncated = self._steps >= self.horizon
        term = {a: bool(terminated) for a in self.agents}
        trunc = {a: bool(truncated and not terminated) for a in self.agents}
        infos = {a: {"action_mask": self._masks.get(a)} for a in self.agents}
        if terminated or truncated or done:
            self.agents = []
        return dict(self._obs), rewards, term, trunc, infos

    def close(self):
        self._sc = self._world = self._bs = None

    # -- the allocator hook (B2/B3): obs + reconciliation at the C2 tick --------

    def _allocator(self, assessments, tracks, uavs, uav_speeds, risk_map, t,
                   *, denied_tracks=frozenset(), incumbents=None, task_ids=None,
                   debris_info=None, uav_effectors=None):
        incumbents = incumbents or {}
        debris_info = debris_info or {}
        uav_effectors = uav_effectors or {}
        assess_by_id = {a.track_id: a for a in assessments}
        available = {u.uav_id: u for u in uavs}
        table = reconcile.build_track_table(assess_by_id, debris_info)
        self._track_table = table
        self._build_obs(table, assess_by_id, tracks, t, denied_tracks,
                        incumbents, debris_info, uav_effectors)
        tasks = reconcile.actions_to_tasks(
            self._pending, table,
            assessments=assess_by_id, tracks=tracks, available=available,
            uav_speeds=uav_speeds, risk_map=risk_map, t=t,
            incumbents=incumbents, task_ids=task_ids,
            debris_info=debris_info, uav_effectors=uav_effectors,
            denied_tracks=set(denied_tracks))
        self._tick_fired = True
        return tasks

    def _build_obs(self, table, assess_by_id, tracks, t, denied, incumbents,
                   debris_info, uav_effectors):
        agent_states = [(uid, self._uav_states[uid])
                        for uid in self.possible_agents
                        if uid in self._uav_states]
        mate_states = list(self._uav_states.values())
        self._obs, self._masks = spaces.encode_observations(
            agent_states, mate_states, table, assess_by_id, tracks, t,
            denied=denied, incumbents=incumbents, debris_info=debris_info,
            horizon=self.horizon, fleet_ammo_frac=self._fleet_ammo_frac())

    # -- world stepping ---------------------------------------------------------

    def _advance_to_tick(self) -> bool:
        """Step the macro world until the next BaseStation tick fires (the
        hook ran) or the episode ends. Returns True if the world ran dry."""
        max_substeps = int(round(2.0 / self._world.dt)) + 5   # ~2 C2 cycles guard
        for _ in range(max_substeps):
            self._world.step()
            self._steps_partial()
            if self._tick_fired:
                self._steps += 1
                return False
            if self._raid_resolved():
                self._steps += 1
                return True
        # No tick within the guard (e.g. raid already clear): count a step.
        self._steps += 1
        return self._raid_resolved()

    def _steps_partial(self):
        pass   # hook point; reward is computed once per env step from cursors

    def _raid_resolved(self) -> bool:
        w = self._world
        return (not w._spawn_queue and bool(w.enemies)
                and not any(e.alive for e in w.enemies.values())
                and not w.debris)

    # -- reward -----------------------------------------------------------------

    def _compute_reward(self) -> dict[str, float]:
        w = self.w
        enemies = self._world.enemies
        decoy_ids = {e.id for e in enemies.values()
                     if e.threat_class == ThreatClass.DECOY}
        armed_kills = decoy_kills = armed_leaks = 0
        own_shots: dict[str, int] = {a: 0 for a in self.possible_agents}
        own_decoy_shots: dict[str, int] = {a: 0 for a in self.possible_agents}
        for ev in self._world.events[self._ev_cursor:]:
            kind = ev["kind"]
            if kind == "kill":
                e = enemies.get(ev.get("enemy_id"))
                if e is not None and e.profile.warhead:
                    armed_kills += 1
                elif e is not None:
                    decoy_kills += 1
            elif kind == "leaker" and ev.get("warhead"):
                armed_leaks += 1
            if kind in _SHOT_KINDS and ev.get("uav_id") in own_shots:
                own_shots[ev["uav_id"]] += 1
                if kind in ("kill", "miss") and ev.get("enemy_id") in decoy_ids:
                    own_decoy_shots[ev["uav_id"]] += 1
        self._ev_cursor = len(self._world.events)

        debris_cost = sum(ZONE_WEIGHTS[wk["zone"]]
                          for wk in self._world.wrecks[self._wreck_cursor:])
        debris_cost += sum(ZONE_WEIGHTS[s["zone"]]
                           for s in self._world.stray_impacts[self._stray_cursor:])
        self._wreck_cursor = len(self._world.wrecks)
        self._stray_cursor = len(self._world.stray_impacts)

        # Task churn vs the previous cycle (re-creates incumbent hysteresis).
        cur = dict(self._bs._shooters)
        churn = sum(1 for tid, sid in cur.items()
                    if self._prev_shooters.get(tid, sid) != sid)
        self._prev_shooters = cur

        # Potential-based safety shaping: F = gamma*phi' - phi (policy-invariant).
        phi_next = self._potential()
        shape = self.gamma * phi_next - self._phi
        self._phi = phi_next

        team = (w["armed_kill"] * armed_kills
                - w["armed_leak"] * armed_leaks
                - w["decoy_kill"] * decoy_kills
                - w["debris"] * debris_cost
                - w["churn"] * churn
                + w["shape"] * shape
                - w["time"])
        return {a: float(team
                         - w["ammo"] * own_shots.get(a, 0)
                         - w["decoy_shot"] * own_decoy_shots.get(a, 0))
                for a in self.agents}

    def _potential(self) -> float:
        """Safety potential phi(s) = -sum over live armed threats of their
        proximity to their target asset; higher (less negative) is safer, so
        a threat advancing lowers phi and the shaping term penalises it."""
        total = 0.0
        for e in self._world.enemies.values():
            if not e.alive or not e.profile.warhead:
                continue
            d = float(np.linalg.norm(np.asarray(e.position) - np.asarray(e.target)))
            total += float(np.clip(1.0 - d / _PHI_SCALE, 0.0, 1.0))
        return -total

    # -- helpers ----------------------------------------------------------------

    def _on_uav_state(self, msg: UavState) -> None:
        if msg.uav_id in self._uav_states or msg.uav_id in self.possible_agents:
            self._uav_states[msg.uav_id] = msg

    def _fleet_ammo_frac(self) -> float:
        states = [self._uav_states[a] for a in self.possible_agents
                  if a in self._uav_states]
        if not states:
            return 0.0
        return float(np.clip(np.mean([s.ammo / spaces.AMMO_SCALE for s in states]),
                             0.0, 1.0))

    # -- domain-randomised raid (M7) --------------------------------------------

    def _build_episode_cfg(self, seed: int) -> dict:
        cfg = copy.deepcopy(self._cfg)
        if not self.randomize:
            return cfg
        rng = np.random.default_rng(seed)
        assets = [a["name"] for a in cfg["environment"]["assets"]]
        bounds = cfg["environment"]["bounds"]
        cx = 0.5 * (bounds[0] + bounds[2])
        cy = 0.5 * (bounds[1] + bounds[3])
        radius = 0.5 * float(np.hypot(bounds[2] - bounds[0],
                                      bounds[3] - bounds[1])) + 150.0
        from ..threats.enemy_drone import THREAT_PROFILES
        # A mixed raid with a meaningful share of high divers, randomised in
        # count, bearing, altitude jitter and timing so the policy sees a
        # distribution of raids rather than one script.
        plan = [
            ("OWA_STRATEGIC", rng.integers(2, 5)),
            ("OWA_JET", rng.integers(1, 4)),
            ("DECOY", rng.integers(1, 3)),
            ("LOITERING", rng.integers(0, 2)),
            ("FPV", rng.integers(0, 3)),
        ]
        threats, t0 = [], 6.0
        for cls_name, n in plan:
            tc = ThreatClass[cls_name]
            alt0 = THREAT_PROFILES[tc].cruise_alt
            for _ in range(int(n)):
                bearing = np.deg2rad(rng.uniform(300.0, 420.0) % 360.0)  # N-ish arc
                spawn = np.array([cx + radius * np.sin(bearing),
                                  cy + radius * np.cos(bearing)])
                alt = float(np.clip(alt0 * rng.uniform(0.85, 1.25), 60.0, 4800.0))
                threats.append({
                    "time": round(float(t0), 1),
                    "class": cls_name,
                    "spawn": [float(spawn[0]), float(spawn[1]), alt],
                    "target": str(rng.choice(assets)),
                })
                t0 += float(rng.uniform(2.0, 7.0))
        rng.shuffle(threats)
        threats.sort(key=lambda th: th["time"])
        cfg["threats"] = threats
        cfg["duration"] = float(self.horizon + 30)
        return cfg
