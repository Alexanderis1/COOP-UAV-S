# COOP-UAV-S

**Cooperative autonomous UAV system for counter-drone defence of populated areas — a simulation framework.**

> Design, simulate and validate an AI-assisted, cooperative counter-UAS (C-UAS) system: a team of interceptor UAVs + a sensor-fused base station defeating non-cooperating hostile drone swarms over a residential area, with **probabilistic, collateral-damage-aware engagement** as a first-class constraint.

Built against a real-world operational baseline from the Ukrainian theatre (2022–2026), where raids mix strategic one-way-attack (OWA) platforms with same-signature decoys, attack densities exhaust interceptor stocks, and every kill drops ~200 kg of wreckage on someone's neighbourhood.

`Python ≥3.10` · pure-Python core (numpy/scipy) · deterministic, seeded runs · 750+ tests · GPL-3.0

---

## Quickstart

Everything is driven by one command, `coopuavs`, and pure-YAML scenario files. No GPU, no ROS, no Gazebo required.

### 1 — Install

```bash
git clone https://github.com/Alexanderis1/COOP-UAV-S.git
cd COOP-UAV-S
pip install -e ".[viz,dev]"     # core + 3D dashboard + test deps
```

### 2 — Run a battle, headless

```bash
coopuavs run scenarios/residential_raid.yaml --headless
```

Runs the reference raid and prints the engagement summary — who was killed, who leaked through, and where the wreckage fell:

```json
{
  "enemies_total": 9,
  "kills": 6,
  "leakers": 3,           "armed_leakers": 1,
  "wrecks_by_zone": { "SAFE": 1, "DANGEROUS": 2 },   // never CRITICAL
  "debris_intercepted": 3
}
```

`armed_leakers` are warhead-carrying hostiles that reached a protected asset (the real defence failures); `wrecks_by_zone` is where killed-drone debris fell on the SAFE / DANGEROUS / CRITICAL ground rasterisation. The run takes about a minute (it simulates ~200 s of battle) and writes a recording to `runs/<scenario>.json` (relative to your working directory) for replay.

### 3 — Watch it in the 3D dashboard

```bash
coopuavs run scenarios/residential_raid.yaml          # runs, records, then serves the replay
# open http://localhost:8000/
```

A zone-coloured city, hostiles vs. the fused track picture (decoy-suspect tracks fade out), interceptor status and the decision log. Or serve the console with this scenario auto-started and watch it unfold in real time (no replay file is written in this mode):

```bash
coopuavs run scenarios/residential_raid.yaml --live --speed 4
```

### 4 — Monte-Carlo over seeds

```bash
coopuavs batch scenarios/residential_raid.yaml -n 20   # defence-effectiveness statistics (~1 min/run)
```

### 5 — The command console (operator in the loop)

```bash
coopuavs serve            # open http://localhost:8000/
```

The console starts idle — each battle is launched from the *New Execution* form (no prior run needed). Authorise engagements, and watch ghost threats turn solid as the sensor network acquires them.

### Run the test suite

```bash
pytest                    # full deterministic suite incl. an end-to-end raid
```

> **Scenarios are data, not code.** Each YAML file fully describes a battle — map, risk zones, sensor laydown, fleet, raid composition, ROE thresholds. Copy one and edit it; no Python required.

| Scenario | What it exercises |
|---|---|
| `scenarios/residential_raid.yaml` | The reference raid: 12×12 km area, two protected assets, a 9-drone three-wave mix of OWAs, decoys, FPVs and one jet OWA. |
| `scenarios/high_diver_raid.yaml` | The high-altitude diving-jet problem + **forward CAP (combat air patrol) sentinels** with a look-down airborne early-warning radar. |
| `scenarios/urban_raid.yaml` | A large procedurally-generated city — regenerate with `coopuavs citygen` (writes this file, deterministic by `--seed`): 20 interceptors, 10 sentinels, occlusion, live debris. |

---

## The two ideas this project is about

**1. Cooperation beats speed.** A lone interceptor slower than its target loses a tail chase — provably (the intercept triangle has no solution). But a hostile drone is mission-bound to a predictable corridor, so a *team* posts blockers at corridor points they can reach first and lets the target fly into the engagement. The C2 reserves blocking packages for exactly the targets that outrun the fleet, in relay — the **geometry**, not the airspeed, wins. Blocker posts are now placed with exact **Apollonius-circle** rendezvous geometry (`mc/apollonius.py`).

**2. Where the wreck falls is part of the fire decision.** The defended area is rasterised into SAFE / DANGEROUS / CRITICAL ground. Before any munition release, the C2 runs a Monte-Carlo debris footprint of the *predicted kill* against that map and clears, holds, or denies the shot — including a "now-or-never" rule that authorises an imperfect shot when the target's own trajectory guarantees every later shot is worse. Net kills drop wrecks nearly straight down; gun kills throw them forward — and the ROE feels the difference.

In the 10-seed Monte-Carlo of the reference raid: **zero wrecks on critical ground and zero rounds spent on identified decoys**, under deliberate saturation.

---

## What's implemented

A complete, deterministic, message-driven battle simulation:

- **Threats** — Shahed-type strategic OWA, jet OWA, FPV kamikaze, Lancet-type loitering munition, and Gerbera-type decoys that share the OWA's RF signature and flight profile (perception genuinely cannot tell until a sensor earns it). Agile classes react to interceptors.
- **Layered sensing** — radar (R⁴ Pd, radar horizon), passive RF (bearing-only, signature hashes), EO/IR towers (range-ramped identification), acoustic pickets (hear below the radar horizon), terminal onboard seekers, and **forward high-altitude CAP sentinels** carrying a look-down airborne early-warning radar that paints high-altitude divers seconds-to-tens-of-seconds before the ground set.
- **Perception** — multi-sensor Kalman tracking with per-scan GNN association, Bayesian threat-class belief, decoy probability fused from RF/EO/acoustic/kinematic evidence.
- **C2 (TEWA — Threat Evaluation & Weapon Assignment)** — threat evaluation against protected assets, weapon-target assignment with reserved cooperative blockers and incumbent hysteresis, **Apollonius-circle** cutoff geometry, and debris-footprint rules of engagement with clearance / hold / deny semantics. No release without clearance — the human-on-the-loop seam is already in the message flow.
- **Cooperation AI** — the allocator is a clean seam: the classical priority-greedy planner ships by default, and a **learned MAPPO cooperation policy** drops in behind the same interface (see below).
- **Interceptors** — mode-FSM agents (pursuit / engage / blocking / herding / RTB), intercept-triangle guidance, net-gun and projectile effectors with probabilistic kill envelopes; ground anti-air turrets under the same clearance interlock.
- **3D C2 dashboard** — Three.js, replay or live-streamed over websocket.
- **Higher-fidelity flight stack (SITL)** — an optional software-in-the-loop mode runs the tactical stack on a virtual flight controller (EKF, mixers, CBIT fault monitors) over a modelled datalink, for the scenarios that need it.

### The cooperation AI: classical baseline → learned policy

The weapon-target allocator (`c2/assignment.py`) is the seam. A trained multi-agent policy (MAPPO) can replace it behind the *same* interface — it decides *which* threats to commit *which* interceptors to, and in *what role*, while the trusted classical core still picks the trigger platform and enforces every ROE / clearance gate. PyTorch is an optional dependency; the core simulation stays pure-Python.

```bash
pip install -e ".[train]"                                   # adds torch + gym/pettingzoo
coopuavs train scenarios/high_diver_raid.yaml --minutes 55  # CPU-parallel; wall-clock budget
coopuavs eval  scenarios/high_diver_raid.yaml --policy runs/marl/policy.pt -n 20
```

Then deploy it in any scenario with `base_station: {allocator: learned, policy: runs/marl/policy.pt}`. Full design, the reward, and a one-hour **Google Colab notebook** are in [docs/MARL.md](docs/MARL.md) and [`notebooks/train_colab.ipynb`](notebooks/train_colab.ipynb).

### Architecture at a glance

Every component is a node on a pub/sub bus shaped exactly like ROS 2 topics — migration replaces two small classes, not the tactical code. Ground truth is quarantined behind sim-side sensors and the engagement adjudicator, Gazebo-plugin style.

```
sensors (radar, RF, EO/IR, acoustic, airborne EW, onboard seekers)
   │ 'detections'
   ▼
fusion (KF + GNN, class belief, p_decoy)
   │ 'tracks'
   ▼
base station C2:  threat evaluation → assignment (shooters + Apollonius blockers) → ROE
   │ 'engagement/tasks'                      ▲ 'fire_request' / ▼ 'clearance'
   ▼
interceptor UAVs (pursuit, cutoff, herding, effectors)   +   anti-air turrets
   │ 'engagement/fire'
   ▼
adjudicator (truth Pk roll → debris sample onto risk map) → 'engagement/result'
```

---

## Operational context & threat taxonomy

The system is designed around operationally-validated drone threat classes (all in `coopuavs/threats`):

| Class | Example | Mass | Speed | Altitude (AGL) | Behaviour |
|---|---|---|---|---|---|
| A — Strategic OWA | Shahed-136 / Geran-2 | ~200 kg | 50–65 m/s | 50 m – 5 km | Swarm saturation, decoy mixing, terminal dive |
| A+ — Jet OWA | Geran-3 / Shahed-238 | ~200 kg | ~100 m/s | 2–5 km | High-speed dive, very low intercept window |
| B — Tactical FPV | Quadcopter kamikaze | 1–5 kg | 30–40 m/s | 0–200 m | Agile, fibre-optic guided (jam-resistant) |
| C — Loitering munition | Lancet-3 | 12 kg | ~80 m/s | 50–500 m | AI terminal seeker, precision strike |
| D — Decoy | Gerbera | ~18 kg | as class A | as class A | Identical RF/radar signature, no warhead — exists to exhaust interceptor stocks |

**Tactical patterns addressed:** altitude switching (low terrain-masking vs. high dive) · saturation attacks (answered with low-cost-per-shot kinetic effectors, never missile economics) · decoy integration (multi-modal Bayesian discrimination) · jam-immune fibre-optic FPV (answered kinetically). Operating envelope: nocturnal, all-weather, −25 °C to +45 °C, winds to 20 m/s, 50 m – 5 km AGL.

---

## Documentation

| Document | Contents |
|---|---|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Full system design: component graph, topic contract, layer-by-layer rationale. **Start here for the code map.** |
| [docs/MARL.md](docs/MARL.md) | The learned cooperation policy: env, observation/action/reward, training recipe (incl. Colab), deployment & A/B evaluation. |
| [docs/RESEARCH.md](docs/RESEARCH.md) | Literature survey (pursuit-evasion / Apollonius, TEWA, tracking, decoy discrimination, SORA ground risk, Ukraine data) with citations and an ordered study path. |
| [docs/SRS.md](docs/SRS.md) | System Requirements Specification — the three-element definition (physical segment, simulation environment, command interface). |
| [docs/ICD_RUNTIME.md](docs/ICD_RUNTIME.md) | Wire contract between the simulator backend and the command console (/ops and /eval websockets). |
| [docs/TRACEABILITY.md](docs/TRACEABILITY.md) | PHY→SIM traceability: which simulation mechanism reproduces each physical-segment requirement, at what fidelity. |
| [docs/ROADMAP.md](docs/ROADMAP.md) | Phased evolution and an honest list of current limitations. |

The smallest complete battle to read first is `tests/test_end_to_end.py`.

## Project status

Hackathon-born, built for the long run: deterministic seeded runs, a full test suite, YAML-driven experiments, and ROS 2-shaped seams. Recently landed: forward high-altitude CAP sentinels with a look-down airborne EW radar, Apollonius-circle cooperative interception, and a learned MAPPO weapon-target-assignment policy behind the classical allocator seam ([docs/ROADMAP.md](docs/ROADMAP.md) tracks what's next — IMM tracking, decentralised CBBA allocation, intercept-point optimisation, ROS 2 / Gazebo migration).

> **Honest note on the learned policy:** the training/inference pipeline is complete and verified, but a policy that *beats* the well-tuned classical baseline needs a real training run (hours on a many-core CPU box, or a partial run on Colab). The shipped behaviour is the classical planner; the learned policy is opt-in.

License: **GPL-3.0** (see [LICENSE](LICENSE)).
