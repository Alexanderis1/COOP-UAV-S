# Learned cooperative weapon-target assignment (MARL)

A trained multi-agent policy that replaces the classical priority-greedy
weapon-target allocator (`c2/assignment.py`) with a learned **cooperation
policy** — the second innovation pillar's "MARL benchmark" (ROADMAP Phase 2),
now implemented. The policy decides *which* threats to commit *which*
interceptors to and in *what role*; everything else — guidance, the
debris-footprint ROE, fire-control, the orchestration/authorisation chain —
is the unchanged, trusted stack.

It exists because forward sensing alone is not enough: the high-altitude CAP
sentinels (PHY-SNT-004) detect the diving jet OWA ~36× earlier (mean
acquisition latency 2.15 s → 0.06 s in `scenarios/high_diver_raid.yaml`), but
the *greedy* allocator does not convert that time margin into kills under
saturation — it commits shooters the instant every threat appears and can let
a third diver leak that staggered detection would have caught. Converting
early warning into defended airspace is a **cooperation** problem, which is
what this policy learns.

## The seam (why it's safe to swap in a network)

`BaseStation` calls its allocator through one hook:

```python
tasks = (self.allocator or assignment.allocate)(assessments, tracks, available,
            uav_speeds, risk_map, t, denied_tracks=…, incumbents=…,
            task_ids=…, debris_info=…, uav_effectors=…)
```

wrapped in a fence: any allocator exception falls back to the classical
allocator for that cycle (logged, `allocator_fallbacks`), so a misbehaving
policy can never freeze tasking. In training the env sets `allocator_strict`
so bugs surface instead of hiding.

The learned allocator (`c2/learned_allocator.LearnedAllocator`) and the
training env (`rl/env.CoopWtaParallelEnv`) drive *exactly* this hook. Critically,
the reconciliation `rl/reconcile.actions_to_tasks` routes the final shooter
pick through `assignment._best_shooter` **verbatim** and re-applies the
denied-track / debris-effector / availability gates, so the Pk-aware,
closing-speed-eligible, incumbent-discounted pick the ROE geometry assumes
stays authoritative even if the policy mis-commits. Falling debris the policy
does not task is handled by the classical allocator over the spare platforms,
so wreckage interception (PHY-GCS-006) is never dropped.

## Problem formulation

- **Agents** — the interceptors. One **parameter-shared actor** runs per
  agent on an ego-centric observation (relative positions/velocities + a
  stable per-platform self-token), so the policy generalises across platforms
  and fleet sizes and deploys decentrally. A **centralised critic** sees the
  joint observation in training only (CTDE / MAPPO). Sentinels are sensors,
  not agents.
- **Observation** (`rl/spaces.py`, dim 176, all extrapolated to the decision
  instant so the policy sees the world the 10 Hz executors act on):
  own state; the top-`K=6` threat tracks by threat score (relative kinematics,
  threat score, time-to-impact, p_decoy, impact-zone, and the caller-owned
  flags `catchable-by-ego`, `incumbent-by-ego`, `denied`, `debris`, `valid`);
  the nearest `M=4` teammates (relative position, role, ammo, capability);
  and a small global block (time, live-track count, fleet ammo).
- **Action** — one masked categorical over `1 + 2·K = 13` choices: *idle*,
  *shoot* one of the top-K tracks, or *block* (cooperative support) one of
  them. Blocking-vs-herding is **not** an agent choice — it is derived
  downstream from target-vs-shooter speed in the interceptor FSM, exactly as
  in the classical path. The mask forbids acting on empty slots, denied
  tracks, and net-on-debris, so a sampled action always reconciles to a valid
  task.
- **Reward** (shared team outcome + per-agent waste): `+`armed kills,
  `−`armed leakers (heaviest — the defence failure), `−`zone-weighted debris
  and stray collateral, `−`decoy kills, per-agent `−`own ammo and `−`own decoy
  shots, `−`task churn (re-creates the classical incumbent hysteresis), and a
  potential-based safety-shaping term `γΦ(s′)−Φ(s)` where `Φ` is the negative
  sum of live armed threats' proximity to their assets (policy-invariant
  shaping). Weights in `rl/env.DEFAULT_REWARD_WEIGHTS`.
- **Episode** — one env step = one 1 Hz C2 planning cycle (the env advances
  ~20 `dt=0.05` sub-steps per step). The policy's plan at decision *t* is
  committed at the next cycle (a realistic ~1 s planning latency; harmless
  when the trained net is later run synchronously in deployment). Fixed
  horizon with early termination when the raid is resolved; time-limit
  truncations bootstrap from `V(final state)`.
- **Domain randomisation** — each episode draws a fresh raid (counts per
  class, bearings on the northern arc, altitude jitter, timing), so the policy
  sees a *distribution* of raids, not one script (the env attends to the
  6 highest-threat tracks per cycle, the same order the classical allocator
  serves them in).

## Training (CPU, on the many-core box)

```bash
pip install -e ".[train]"          # torch (CPU wheel is fine) + gym/pettingzoo

# Saturate the cores. Pin BLAS so N worker processes don't oversubscribe.
OMP_NUM_THREADS=1 coopuavs train scenarios/high_diver_raid.yaml \
    --n-envs 30 --steps 8000000 --rollout 64 --horizon 220 --out runs/marl
#   or:  python scripts/train_marl.py scenarios/high_diver_raid.yaml --n-envs 30 ...
```

- The simulation is pure-Python and CPU-bound, so throughput scales with
  **processes** (`SubprocVecEnv`), not threads — set `--n-envs ≈ cores − 2`.
- For a small WTA policy the env step, not the network, is the bottleneck;
  no GPU is needed (matches the project's CPU-bound design and the €100
  CPU-only Scaleway plan). On Google Colab a GPU adds little here.
- Logs to `runs/marl/train_log.csv` (and TensorBoard under `runs/marl/tb` if
  installed); checkpoints `runs/marl/policy.pt`.
- Reproducible: the World is seeded per worker/episode; policy sampling uses a
  seeded torch generator.

## Deploying & A/B evaluating the policy

Point any scenario's base station at the checkpoint:

```yaml
base_station:
  allocator: learned
  policy: runs/marl/policy.pt
```

Compare it to the classical baseline over a seed sweep (same scenario, CAP
sentinels included — does the policy convert the time margin into fewer
leakers and less collateral?):

```bash
coopuavs eval scenarios/high_diver_raid.yaml --policy runs/marl/policy.pt -n 20
#   or:  python scripts/eval_policy.py … --policy runs/marl/policy.pt -n 20
```

## Module map

| Module | Role | Deps |
|---|---|---|
| `rl/spaces.py` | obs/action encoding, masks, shared encoder | numpy |
| `rl/reconcile.py` | actions → `EngagementTask`s via `_best_shooter` | numpy |
| `rl/env.py` | `CoopWtaParallelEnv` (PettingZoo-shaped) + reward | numpy |
| `rl/models.py` | shared actor + centralised critic | torch |
| `rl/mappo.py` | MAPPO trainer (GAE, clipped PPO, checkpoints) | torch |
| `rl/vec_env.py` | `SubprocVecEnv` / `SyncVectorEnv` | torch-free |
| `rl/evaluate.py` | greedy-vs-learned A/B | numpy |
| `c2/learned_allocator.py` | deployment allocator + `get_allocator` | torch (lazy) |

The numpy-only modules are importable and unit-tested in the base install;
torch enters only the trainer and the deployment inference path, so the core
simulation stays pure-Python.
