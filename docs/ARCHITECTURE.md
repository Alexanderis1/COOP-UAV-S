# COOP-UAV-S — System Architecture

> Version 0.1 — hackathon baseline. Companion documents:
> [SRS.md](SRS.md) (System Requirements Specification v0.2 — the binding
> three-element definition: physical segment / simulation environment /
> command interface + orchestration agent),
> [RESEARCH.md](RESEARCH.md) (state of the art, citations, study path) and
> [ROADMAP.md](ROADMAP.md) (planned evolution).

## 1. Design philosophy

Three decisions shape everything else:

1. **Custom Python simulation, ROS 2-shaped seams.** Iteration speed wins a
   hackathon; architectural discipline wins the months after it. Every
   software component is a `Node` talking over a pub/sub `MessageBus` with
   typed dataclass messages — a 1:1 image of ROS 2 nodes / topics / `.msg`
   files. Migration to ROS 2 + Gazebo/PX4 replaces two small classes
   (`bus.py`, `node.py`) and the sim-side plugins; tactical code does not
   change. See RESEARCH.md §7 for the migration rules.

2. **Probabilistic engagement, not ballistics.** Effectors expose an
   engagement *envelope* (range, off-axis angle, closing speed → kill
   probability) and kills produce *sampled debris footprints*. This is the
   right fidelity to study the questions the project is about — cooperation
   geometry and collateral-risk-aware authorisation — and it keeps every
   model swappable for higher-fidelity versions later.

3. **Ground truth is quarantined.** Only explicitly *sim-side* components
   (sensors, the engagement adjudicator) may read the world's true state,
   exactly like Gazebo plugins. Perception, C2 and the interceptor agents
   see nothing but messages. Decoys are therefore genuinely
   indistinguishable until a sensor earns the discrimination.

## 2. Component graph

```
                                 SIM SIDE (owns ground truth)
  ┌──────────────────────────────────────────────────────────────────────┐
  │  World (clock, RNG, enemies, events)      Environment (RiskMap,      │
  │  EnemyDrone behaviours (cruise/dive/weave)  assets, buildings)       │
  │                                                                      │
  │  Radar   RF-DF   EO/IR   Acoustic   OnboardSeeker (rides each UAV)  │
  │    └───────┴───────┴────────┴────────────┘                           │
  │            'detections' ▼                EngagementAdjudicator       │
  └────────────────────────│────────────────────▲──────────│─────────────┘
                           │                    │'engagement/fire'
                           ▼                    │          ▼'engagement/result'
  ┌─────────────────┐  'tracks'   ┌─────────────┴───────────────────────┐
  │   FusionNode    ├────────────▶│            BaseStation (C2)         │
  │ KF + GNN/scan   │             │  ThreatEvaluation → Assignment      │
  │ class belief,   │             │  (priority-greedy + blockers)       │
  │ p_decoy         │             │  RulesOfEngagement (debris ROE)     │
  └─────────────────┘             └──┬───────────────▲──────────────────┘
          ▲                          │'engagement/   │'engagement/
          │ 'detections'             │  tasks'       │  fire_request'
          │ (seeker)                 ▼               │ + 'clearance'
  ┌───────┴──────────────────────────────────────────┴──────────────────┐
  │            InterceptorUav × N  (mode FSM, PN-style guidance,         │
  │            cooperative cutoff/herding posts, effector)               │
  │            publishes 'uav/state'                                     │
  └──────────────────────────────────────────────────────────────────────┘
                           │ topic subscriptions + world truth (eval side)
                           ▼
                Recorder → recording JSON / live websocket → Three.js C2 dashboard
```

### Topic contract (the future `.msg` files)

| Topic | Message | Producer → Consumer |
|---|---|---|
| `detections` | `Detection` | all sensors → FusionNode |
| `tracks` | `TrackArray` | FusionNode → C2, UAVs, Recorder |
| `uav/state` | `UavState` | each UAV → C2, peer UAVs, Recorder |
| `engagement/tasks` | `list[EngagementTask]` | C2 → UAVs |
| `engagement/fire_request` | `FireRequest` | shooter UAV → C2 |
| `engagement/clearance` | `FireClearance` | C2 → shooter UAV |
| `engagement/fire` | `FireRequest` | shooter UAV → Adjudicator |
| `engagement/result` | `EngagementResult` | Adjudicator → C2 |

## 3. Layer-by-layer

### 3.1 Sensing (`coopuavs/sensors`)

Layered, deliberately imperfect, mutually compensating:

| Sensor | Strength | Weakness modelled |
|---|---|---|
| `Radar` | long range, Doppler | R⁴ Pd falloff, radar horizon hides low FPVs |
| `RfSensor` | very long range, signature hash | bearing-only; decoys share the OWA signature by design |
| `EoIrSensor` | the decoy discriminator | short range; ID quality ramps with proximity, uninformative at range |
| `AcousticSensor` | hears below the radar horizon, engine-type cue | short range, coarse bearing |
| `OnboardSeeker` | terminal accuracy + close-range ID | only where an interceptor already is |

All produce the same `Detection` with a full 3×3 covariance: bearing-only
geometry is encoded as an anisotropic covariance rather than a special
case, so the tracker needs no per-sensor logic.

### 3.2 Perception (`coopuavs/perception`)

- `tracking.KalmanTrack` — 6-state constant-velocity KF per object;
  separate filter-time vs measurement-time bookkeeping (coast pruning).
- `fusion.FusionNode` — per-*scan* global-nearest-neighbour association
  (Hungarian over Mahalanobis-gated costs), sequential fusion across
  sensors, precision-ordered so radar seeds tracks rather than RF blobs.
- `classification` — Bayesian class belief per track from sensor likelihoods
  + RF signature evidence (deliberately joint over {OWA, decoy});
  kinematic consistency blended *idempotently* at readout (never
  accumulated — double-counting saturates the posterior). Output that
  matters: `p_decoy`.

Planned upgrades (RESEARCH.md §3): IMM for dive manoeuvres, JPDA/LMB for
dense raids, track-before-detect for low-RCS FPVs.

### 3.3 Command & Control (`coopuavs/c2`)

TEWA loop at 1 Hz in `BaseStation`:

1. **Threat evaluation** — horizontal-plane impact prediction against
   asset list; score = lethality (1 − p_decoy) × urgency × asset value ×
   ground-zone factor.
2. **Assignment** (`assignment.allocate`) — priority-greedy: each track in
   threat order receives the package it needs — a shooter if catchable, a
   shooter *plus reserved blockers* if it outruns the fleet. Budget rule
   prevents support reservation from starving queued tracks under
   saturation. Incumbent discount (0.7×) stops jitter-driven shooter swaps.
   Decoys above `p_decoy = 0.85` get nothing: wasting interceptors on
   Gerberas is the enemy's actual objective.
3. **Fire authorisation** (`roe.RulesOfEngagement`) — every release costs a
   Monte-Carlo debris footprint against the risk map:
   - `AUTHORIZED (geometry_safe)` — under base thresholds;
   - `AUTHORIZED (now_or_never)` — above base threshold but the footprint
     cost is *minimal over the target's predicted path* (it is flying into
     the city: holding only moves debris onto worse ground);
   - `AUTHORIZED (last_resort)` — impact imminent on a high-value target;
   - `HOLD` — geometry can still improve; `DENIED` — decoy-grade target,
     unsafe geometry, disengage.

   Fire requests are answered out-of-band (immediately), not at the
   planning rate: an envelope window against a 55 m/s target lasts seconds.

### 3.4 Risk model (`coopuavs/risk`)

- `zones.RiskMap` — rasterised SAFE / DANGEROUS / CRITICAL grid (default
  DANGEROUS: unknown ground in a residential area is populated), zone
  weights 0.02 / 1.0 / 25.0, plus `nearest_safe_cell` for kill-box
  placement. SORA/JARUS-inspired (RESEARCH.md §6).
- `debris.DebrisModel` — sampled ballistic envelope: mechanism-dependent
  horizontal velocity retention (net 0.15 vs projectile 0.65 — the ROE
  *feels* the difference between effectors), terminal-velocity fall time,
  altitude-growing dispersion. Used twice: predictively inside ROE, and
  generatively when a kill actually happens.

### 3.5 Interceptors (`coopuavs/interceptors`)

- `guidance` — intercept-triangle solution (`intercept_time` — the
  quadratic whose *absence of roots* defines "uncatchable" and triggers
  cooperation), lead-pursuit velocity commands.
- `cooperation` — the innovation pillar: `cutoff_points` posts blockers at
  corridor points they can reach *before* the target (Apollonius logic —
  slower interceptors beat faster targets on geometry, in relay);
  `herding_post` flanks opposite the designated kill box.
- `effectors` — net gun (short envelope, debris-friendly) vs projectile
  gun (longer envelope, throws the wreck forward); Pk surface over range /
  off-axis / closing speed.
- `uav.InterceptorUav` — mode FSM (IDLE / PURSUIT / ENGAGE / BLOCKING /
  HERDING / RTB), fire-control on tracks extrapolated to now, shot
  discipline (request ≥ 0.25 Pk, abort release < 0.15), hard rule: **no
  release without clearance** — the safety chain lives in the message flow,
  which is where a human-on-the-loop console will plug in.

### 3.6 Simulation core (`coopuavs/core`, `coopuavs/sim`)

Deterministic fixed-step world (default 20 Hz) with per-node rates; single
seeded RNG (every run reproducible — `test_deterministic_given_seed`
enforces it); YAML scenarios are the experiment definition (map, zones,
laydown, fleet, raid, ROE thresholds — experiments are data, not code);
`EngagementAdjudicator` referees fire events against ground truth.

### 3.7 Visualisation (`coopuavs/viz`)

`Recorder` samples truth + track picture + telemetry at 5 Hz. One Three.js
page (`viz/web/index.html`, CDN-loaded) serves both modes: JSON replay with
timeline/speed controls, or live websocket streaming
(`coopuavs run --live`). Zone-coloured ground, hostiles/decoys, wireframe
track ghosts that fade with `p_decoy`, interceptor status board, event log.

## 4. Verified baseline behaviour

10-seed Monte-Carlo of `scenarios/residential_raid.yaml` (9 threats incl.
2 decoys vs 6 gun + 2 net interceptors):

- **0 critical-zone wrecks** across all seeds — the ROE invariant holds;
- **0 shots at identified decoys** — ammunition economics preserved;
- ~50 % overall armed-threat attrition under deliberate saturation, with
  class structure that matches reality: strategic OWAs and FPVs engaged
  effectively; the 100 m/s jet OWA documented as beyond a propeller
  interceptor tier (roadmap: fast-interceptor tier).

## 5. Repository layout

```
src/coopuavs/
  core/         bus, node, messages          (the ROS 2 seam)
  sim/          world, physics, environment, scenario, adjudicator
  threats/      enemy drone profiles + behaviours
  sensors/      radar, rf, eo_ir, acoustic, seeker
  perception/   tracking, fusion, classification
  c2/           threat_evaluation, assignment, roe, base_station
  risk/         zones (risk map), debris (footprint model)
  interceptors/ uav agent, guidance, cooperation, effectors
  viz/          recorder, server, web/index.html
scenarios/      YAML battle definitions
tests/          unit + end-to-end (deterministic) suite
docs/           RESEARCH.md / ARCHITECTURE.md / ROADMAP.md
```
