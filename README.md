# COOP-UAV-S

**Cooperative autonomous UAV system for counter-drone defence of populated areas — simulation framework.**

> A research and engineering framework for the design, simulation, and validation of an AI-assisted, cooperative counter-UAS (C-UAS) system: interceptor UAVs + a sensor-fused base station defeating non-cooperating hostile drone swarms over a residential area, with **probabilistic, collateral-damage-aware engagement** as a first-class constraint.

Built against a real-world operational baseline derived from the Ukrainian theatre (2022–2026), where drone raids mix strategic OWA platforms with same-signature decoys, attack densities exhaust interceptor stocks, and every kill drops 200 kg of wreckage on someone's neighbourhood.

---

## The two ideas this project is about

1. **Cooperation beats speed.** A lone interceptor slower than its target loses a tail chase — provably (the intercept triangle has no solution). But a hostile drone is mission-bound to a predictable corridor, so a *team* posts blockers at corridor points they can reach first and lets the target fly into the engagement. The C2 reserves blocking packages for exactly the targets that outrun the fleet, in relay, so the geometry — not the airspeed — wins.

2. **Where the wreck falls is part of the fire decision.** The defended area is rasterised into SAFE / DANGEROUS / CRITICAL ground. Before any munition release, the C2 runs a Monte-Carlo debris footprint of the *predicted kill* against that map and clears, holds, or denies the shot — including a "now-or-never" rule that authorises an imperfect shot when the target's own trajectory guarantees every later shot is worse (it is flying into the city). Net kills drop wrecks nearly straight down; gun kills throw them forward — the ROE feels the difference.

In the 10-seed Monte-Carlo of the reference raid: **zero wrecks on critical ground and zero rounds spent on identified decoys**, under deliberate saturation.

## What's implemented (v0.1)

A complete, deterministic, message-driven battle simulation:

- **Threats** — Shahed-type strategic OWA, jet OWA, FPV kamikaze, Lancet-type loitering munition, and Gerbera-type decoys that share the OWA's RF signature and flight profile (perception genuinely cannot tell until a sensor earns it).
- **Layered sensing** — radar (R⁴ Pd, radar horizon), passive RF (bearing-only, signature hashes), EO/IR towers (range-ramped identification), acoustic pickets (hear below the radar horizon), and terminal onboard seekers riding each interceptor.
- **Perception** — multi-sensor Kalman tracking with per-scan GNN association, Bayesian threat-class belief, decoy probability from fused RF/EO/acoustic/kinematic evidence.
- **C2 (TEWA)** — threat evaluation against protected assets, priority-greedy weapon-target assignment with reserved cooperative blockers and incumbent hysteresis, debris-footprint rules of engagement with clearance/hold/deny semantics. No release without clearance — the human-on-the-loop seam is already in the message flow.
- **Interceptors** — mode-FSM agents (pursuit / engage / blocking / herding / RTB), intercept-triangle guidance, net-gun and projectile effectors with probabilistic kill envelopes.
- **3D C2 dashboard** — Three.js, replay or live-streamed over websocket: zone-coloured city, hostiles vs the track picture (decoy-suspect tracks fade out), interceptor status, event log.

```
pip install -e ".[viz,dev]"

# headless run + engagement summary
coopuavs run scenarios/residential_raid.yaml --headless

# run, record, then open the 3D replay dashboard at http://localhost:8000/
coopuavs run scenarios/residential_raid.yaml

# stream the battle live to the dashboard at 4x real time
coopuavs run scenarios/residential_raid.yaml --live --speed 4

# Monte-Carlo over seeds
coopuavs batch scenarios/residential_raid.yaml -n 20

pytest        # 21 tests incl. deterministic end-to-end raid
```

Scenarios are pure YAML — map, risk zones, sensor laydown, fleet, raid composition, ROE thresholds. Experiments are data, not code: see `scenarios/residential_raid.yaml` (a 12×12 km residential area, two protected assets, a 9-drone three-wave raid).

## Documentation

| Document | Contents |
|---|---|
| [docs/SRS.md](docs/SRS.md) | **System Requirements Specification (v0.2)** — the three-element definition: E1 physical segment (interceptor UAVs, GCS, anti-air turrets, sensor network — specification only, the fidelity reference), E2 high-fidelity simulation environment, E3 command interface + main orchestration agent; full ICD, evaluation ghost-threat overlay, scenario-launch requirements |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Full system design: component graph, topic contract, layer-by-layer rationale, verified baseline behaviour |
| [docs/RESEARCH.md](docs/RESEARCH.md) | Deep literature survey (pursuit-evasion games, TEWA, multi-target tracking, decoy discrimination, guidance, SORA ground risk, Ukraine operational data) with verified citations, recommended algorithms, and an ordered study path |
| [docs/ROADMAP.md](docs/ROADMAP.md) | Phased evolution: IMM tracking, Apollonius containment, CBBA, MARL benchmark, intercept-point optimisation, ROS 2/Gazebo migration — plus an honest list of v0.1 limitations |

### Architecture at a glance

Every component is a node on a pub/sub bus shaped exactly like ROS 2 topics — migration replaces two small classes, not the tactical code. Ground truth is quarantined behind sim-side sensors and the engagement adjudicator, Gazebo-plugin style.

```
sensors (radar, RF, EO/IR, acoustic, onboard seekers)
   │ 'detections'
   ▼
fusion (KF + GNN, class belief, p_decoy)
   │ 'tracks'
   ▼
base station C2: threat evaluation → assignment (shooters + blockers) → ROE
   │ 'engagement/tasks'              ▲ 'fire_request' / ▼ 'clearance'
   ▼
interceptor UAVs (pursuit, cutoff, herding, effectors)
   │ 'engagement/fire'
   ▼
adjudicator (truth Pk roll → debris sample onto risk map) → 'engagement/result'
```

---

## Operational Context & Threat Taxonomy

### Threat Classes

The system is designed around operationally validated drone threat classes (all implemented in `coopuavs/threats`):

| Class | Example | Mass | Speed | Altitude (AGL) | Behaviour |
|-------|---------|------|-------|----------------|-----------|
| A — Strategic OWA | Shahed-136 / Geran-2 | ~200 kg | 50–65 m/s | 50 m – 5 km (adaptive) | Swarm saturation, decoy mixing, terminal dive |
| A+ — Jet OWA | Geran-3 / Shahed-238 | ~200 kg | ~103 m/s | 2–5 km | High-speed approach, low intercept window |
| B — Tactical FPV | Quadcopter kamikaze | 1–5 kg | 30–40 m/s | 0–200 m | Agile, fiber-optic guided (jam-resistant), human-in-loop or autonomous |
| C — Loitering Munition | Lancet-3 | 12 kg | ~80 m/s | 50–500 m | AI-guided terminal seeker, precision strike |
| D — Decoy | Gerbera | ~18 kg | as class A | as class A | Identical RF/radar signature, no warhead; exists to exhaust interceptor stocks |

### Key Environmental Constraints

```
Temperature:     -25°C to +45°C (operational); battery degradation below -10°C
Wind:            Up to 20 m/s operational ceiling for small drones
Precipitation:   Rain, snow, dense fog — all-weather operation required
Illumination:    Primarily nocturnal operations; thermal imaging dominant
Engagement zone: Urban / peri-urban, mixed civilian/military structures
Altitude band:   50 m – 5,000 m AGL (multi-layer coverage required)
Attack density:  Up to 400+ drones/night over a metropolitan area
```

### Tactical Patterns Addressed

- **Altitude switching:** drones adapt flight profile (low-level terrain masking vs. high-altitude dive) based on detected defences
- **Saturation attacks:** 24-hour attack cycles designed to exhaust missile reserves — addressed with low-cost-per-shot kinetic effectors (nets, projectiles), never interceptor-missile economics
- **Decoy integration:** unarmed replicas with identical radar/visual signatures mixed into strike packages — addressed with multi-modal Bayesian discrimination and engagement deprioritisation
- **Fiber-optic FPV:** radio-jamming-immune, human-piloted short-range munitions — addressed kinetically; jamming is deliberately *not* in the effector mix
- **Two-phase strikes:** secondary hit timed to target first responders (roadmap)

## Status & contributing

Hackathon-born, built for the long run: deterministic seeded runs, a full test suite, YAML-driven experiments, and ROS 2-shaped seams. Start with `docs/RESEARCH.md` for the theory, `docs/ARCHITECTURE.md` for the code map, and `tests/test_end_to_end.py` for the smallest complete battle.

License: GPL-3.0 (see [LICENSE](LICENSE)).
