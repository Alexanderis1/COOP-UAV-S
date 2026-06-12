# COOP-UAV-S — System Requirements Specification (SRS)

> **Version 0.3 — urban environment, sentinels, interceptable debris.**
> Adds: building-typed urban world model with civilian-presence zone
> derivation (SIM-ENV-004/005/006), material-dependent sensor and fire
> line-of-sight occlusion (SIM-SEN-005, SIM-EFF-006), live interceptable
> debris (§4.11), sentinel surveillance UAVs (§3.9), charging stations
> (§3.10), debris-intercept tasking (PHY-GCS-006/007) and engagement
> attribution (SIM-GT-004).
> **Version 0.2 — three-segment definition.**
> Supersedes the implicit requirements scattered across
> [README](../README.md) and [ARCHITECTURE.md](ARCHITECTURE.md) v0.1.
> Companion documents: [ARCHITECTURE.md](ARCHITECTURE.md) (v0.1 baseline
> design), [RESEARCH.md](RESEARCH.md) (literature), [ROADMAP.md](ROADMAP.md)
> (phasing).

---

## 1. Introduction

### 1.1 Purpose

This SRS defines the COOP-UAV-S system as **three distinct elements**:

| Element | Name | Status in current programme stage |
|---|---|---|
| **E1** | **Physical System Segment** — interceptor UAVs, ground control stations, fixed sensors and anti-air turrets | **Specification only.** Not developed at this stage. Its requirements exist to be the *fidelity reference* that Element 2 must reproduce. |
| **E2** | **Simulation Environment** — high-fidelity digital twin of E1, its software execution, its communications, the threats, and the physical world | Developed and continuously extended. |
| **E3** | **Command Interface** — the human-facing 3D real-time C2 console plus the main orchestration agent attached to it | Developed and continuously extended. |

The governing principle of this stage:

> **E1 is not built; it is simulated.** Every requirement on the physical
> segment (PHY-\*) shall have a corresponding simulation requirement
> (SIM-\*) that recreates it at high fidelity, so that E3 — and the humans
> and the orchestration agent behind it — cannot functionally distinguish
> the simulator from the real system, except through the explicitly
> evaluation-only channels defined in §5.4.

### 1.2 Scope of this stage

- **In scope:** complete requirement definition for E1, E2, E3; design and
  implementation of E2 and E3; evaluation, demonstration and tuning of
  system performance against simulated raids.
- **Out of scope:** procurement, manufacturing, flight testing or
  integration of any E1 hardware; live-fire activity of any kind.

### 1.3 Requirement conventions

- Identifiers: `PHY-…` (Element 1), `SIM-…` (Element 2), `HMI-…`
  (Element 3 interface), `ORC-…` (orchestration agent), `ICD-…`
  (interfaces between elements), `SYS-…` (system-wide).
- *shall* = binding requirement; *should* = design goal; *may* = option.
- Threat classes A / A+ / B / C / D are defined in §2.3.

### 1.4 Definitions and acronyms

| Term | Meaning |
|---|---|
| C-UAS | Counter-unmanned-aircraft system |
| OWA | One-way attack (kamikaze) drone |
| GCS | Ground Control Station |
| C2 | Command and Control |
| TEWA | Threat Evaluation and Weapon Assignment |
| ROE | Rules of Engagement |
| SIL / HIL | Software- / Hardware-in-the-loop |
| ICD | Interface Control Document/Definition |
| Pk | Probability of kill |
| Ghost threat | A threat that exists in simulation ground truth but has not yet been acquired by any sensor (evaluation display concept, §5.4) |
| Acquired threat | A threat with at least one fused track held by the perception layer |

---

## 2. Overall description

### 2.1 System context — production configuration (target, not this stage)

```
                       ┌────────────────────────────┐
   human operators ───▶│  E3  Command Interface     │◀─── ORC orchestration agent
                       │  (3D real-time C2 console) │
                       └─────────────┬──────────────┘
                                     │  ICD protocols (§6)
        ┌──────────────┬─────────────┼───────────────┬─────────────────┐
        ▼              ▼             ▼               ▼                 ▼
  Interceptor      Radar / RF /   Acoustic       Anti-air         GCS / relay
  UAV fleet (E1)   EO-IR towers   pickets        gun turrets      infrastructure
```

### 2.2 System context — evaluation configuration (this stage)

```
                       ┌────────────────────────────┐
   human operators ───▶│  E3  Command Interface     │◀─── ORC orchestration agent
                       │  + evaluation overlays      │
                       └─────────────┬──────────────┘
                                     │  the SAME ICD protocols (§6)
                                     ▼
                       ┌────────────────────────────┐
                       │  E2  Simulation Environment │
                       │  – simulates every E1 asset │
                       │  – runs E1 software (SIL)   │
                       │  – simulates threats, world,│
                       │    physics, comms, weather  │
                       │  – plus evaluation-only     │
                       │    ground-truth channel     │
                       └────────────────────────────┘
```

- **SYS-001** — The system shall be partitioned into the three elements E1,
  E2, E3 with the interfaces of §6 as the only coupling between them.
- **SYS-002** — E3 shall connect to E2 through the same protocol surface it
  would use toward real E1 assets, with the single addition of the
  evaluation-only ground-truth channel (ICD-EVAL, §6).
- **SYS-003** — Replacing E2 with real E1 assets (or HIL hybrids) shall
  require no change to E3 other than disabling the evaluation channel.
- **SYS-004** — All engagement authority shall follow the human-on-the-loop
  chain of §5.5: no simulated or real weapon release without an explicit
  clearance message traceable to a human action or to a human-pre-approved
  ROE rule.

### 2.3 Operational baseline (unchanged from v0.1)

Threat taxonomy the system is designed against:

| Class | Example | Mass | Speed | Altitude AGL | Behaviour |
|---|---|---|---|---|---|
| A — Strategic OWA | Shahed-136 / Geran-2 | ~200 kg | 50–65 m/s | 50 m – 5 km | Swarm saturation, decoy mixing, terminal dive |
| A+ — Jet OWA | Geran-3 / Shahed-238 | ~200 kg | ~103 m/s | 2–5 km | High-speed approach, short intercept window |
| B — Tactical FPV | Quadcopter kamikaze | 1–5 kg | 30–40 m/s | 0–200 m | Agile, fibre-optic guided, terrain masking |
| C — Loitering munition | Lancet-type | ~12 kg | ~80 m/s | 50–500 m | AI terminal seeker, precision strike |
| D — Decoy | Gerbera-type | ~18 kg | as class A | as class A | Class-A RF/radar signature, no warhead |

Environmental envelope: −25 °C to +45 °C; wind to 20 m/s; rain / snow /
fog; primarily nocturnal; urban and peri-urban ground with SAFE /
DANGEROUS / CRITICAL zoning; raid density up to 400+ vehicles per night
over a metropolitan area.

---

## 3. Element 1 — Physical System Segment (PHY)

> **Status: specification baseline only.** Nothing in this section is to be
> built at this stage. Every PHY requirement is normative *for Element 2*:
> §4 must simulate what §3 specifies (traceability rule SIM-001).

### 3.1 Interceptor UAV — structure and airframe

- **PHY-UAV-001** — The fleet shall comprise two interceptor tiers:
  - **Tier-P (propeller):** VTOL-capable multirotor or hybrid airframe,
    MTOW ≤ 25 kg, dash speed ≥ 80 m/s in fixed-wing/hybrid mode, endurance
    ≥ 30 min at patrol speed.
  - **Tier-F (fast):** fixed-wing or ducted-fan airframe, dash speed
    ≥ 150 m/s, dedicated to class A+ jet OWA threats.
- **PHY-UAV-002** — Airframes shall tolerate the §2.3 environmental
  envelope, including icing-aware operation and battery thermal management
  below −10 °C.
- **PHY-UAV-003** — Airframes shall survive the recoil/release loads of
  their installed effector (§3.3) without loss of controlled flight.
- **PHY-UAV-004** — Launch shall be possible from unprepared ground or
  vehicle-mounted catapult/box within 60 s of scramble order; recovery
  shall be autonomous (vertical landing or net/arrested recovery).

### 3.2 Interceptor UAV — onboard hardware and compute

- **PHY-UAV-010** — Each UAV shall carry a dual-computer avionics stack:
  1. **Flight Control Unit (FCU):** real-time flight controller (PX4-class
     autopilot on an ARM Cortex-M/R class MCU) running attitude/rate
     control, navigation and failsafes under a hard-real-time scheduler;
  2. **Mission Computer (MC):** Linux-class companion computer with an AI
     accelerator (NVIDIA Jetson Orin-class or equivalent NPU, ≥ 40 TOPS)
     running perception, guidance, cooperation and the effector fire
     control application.
- **PHY-UAV-011** — Navigation sensors: IMU (≥ 400 Hz), barometer,
  magnetometer, GNSS with anti-jam/anti-spoof posture, and a
  GNSS-denied fallback (visual-inertial odometry and/or datalink-based
  positioning) sufficient for engagement-grade navigation for ≥ 5 min.
- **PHY-UAV-012** — Seeker suite: gimballed EO camera (≥ 1080p, global
  shutter, ≥ 60 fps) co-boresighted with a LWIR thermal imager
  (≥ 640×512) for nocturnal operation, plus a short-range ranging sensor
  (laser rangefinder or radar altimeter-class) for terminal fire control.
- **PHY-UAV-013** — Health monitoring: per-cell battery telemetry, motor
  ESC telemetry, effector ammunition/charge state, link quality — all
  published on the C2 datalink at ≥ 1 Hz.

### 3.3 Interceptor UAV — effectors

- **PHY-UAV-020** — Tier-P UAVs shall mount exactly one of:
  - **Net gun:** short-envelope capture effector (engagement range
    ~10–40 m), low debris horizontal-velocity retention (wreck falls
    nearly straight down);
  - **Projectile gun:** longer-envelope kinetic effector (~30–120 m),
    higher debris forward throw; magazine ≥ 10 effective engagements.
- **PHY-UAV-021** — Effector fire control shall enforce, in onboard
  software, the hard interlock: **no release without a valid, unexpired
  clearance token** received over the C2 datalink (see ICD-C2, §6).
- **PHY-UAV-022** — Effectors shall expose a calibrated engagement
  envelope (range, off-axis angle, closing speed → Pk) to the fire-control
  software; the same envelope definition is the simulator's Pk model
  (SIM-EFF-002).

### 3.4 Interceptor UAV — onboard software

- **PHY-UAV-030** — The MC software shall be composed of message-passing
  nodes over a ROS 2-compatible middleware, matching the topic contract of
  §6 exactly (the v0.1 Python `core/bus.py` seam is the reference).
- **PHY-UAV-031** — Onboard software functions (each a separable node):
  guidance (intercept-triangle / PN lead pursuit), cooperation client
  (cutoff/blocking/herding post execution), mode FSM (IDLE / PURSUIT /
  ENGAGE / BLOCKING / HERDING / RTB), seeker perception, effector fire
  control, health/telemetry, datalink manager.
- **PHY-UAV-032** — Onboard AI models:
  - seeker detector/tracker (small-object EO/IR detection network,
    ByteTrack/BoT-SORT-class association) running ≥ 30 Hz on the MC
    accelerator;
  - target classifier contributing class belief and decoy evidence
    (`p_decoy`) at terminal range.
- **PHY-UAV-033** — Onboard autonomy shall degrade gracefully under C2
  link loss: continue current task within pre-authorised constraints,
  never self-authorise weapon release, RTB on timeout.
- **PHY-UAV-034** — All onboard software shall be buildable and runnable
  on x86/ARM Linux without the vehicle (this is what makes SIL execution
  in E2 possible — SIM-SIL-001).

### 3.5 Interceptor UAV — communications

- **PHY-UAV-040** — Primary C2 datalink: encrypted, authenticated,
  low-latency IP datalink to the GCS (target ≤ 50 ms one-way latency,
  ≥ 99 % availability inside the defended volume), with frequency-agile /
  mesh-capable radios.
- **PHY-UAV-041** — UAV-to-UAV mesh: peer state sharing (`uav/state`) for
  cooperative geometry at ≥ 2 Hz even when GCS-relayed.
- **PHY-UAV-042** — Link security: mutual authentication, replay
  protection, and signed clearance tokens (PHY-UAV-021).
- **PHY-UAV-043** — Comms degradation (jamming, terrain masking, range)
  shall be detected and reported as link-quality telemetry; behaviour
  under loss follows PHY-UAV-033.

### 3.6 Ground Control Station(s)

- **PHY-GCS-001** — At least one transportable GCS shall host: the C2/TEWA
  software stack (threat evaluation, weapon-target assignment with
  cooperative blocker reservation, debris-aware ROE engine), the sensor
  fusion stack (multi-sensor Kalman tracking, GNN association, Bayesian
  classification / decoy discrimination), and the datalink head-ends.
- **PHY-GCS-002** — GCS compute: redundant rugged servers with GPU
  inference capacity for the fusion/classification models; total TEWA
  cycle latency ≤ 1 s for 400 concurrent tracks.
- **PHY-GCS-003** — The GCS shall own the sensor network interfaces:
  surveillance radar(s), passive RF direction finders, EO/IR towers,
  acoustic picket arrays — each delivering `Detection` messages per §6.
- **PHY-GCS-004** — The GCS shall expose the full ICD of §6 northbound to
  Element 3; the GCS is the single authority that turns a human/ORC
  authorisation into a signed clearance token.
- **PHY-GCS-005** — GCS power, shelter and comms shall support 24 h
  continuous operation and displacement (pack-up/set-up) in ≤ 30 min.
- **PHY-GCS-006** — The C2/TEWA stack shall generate **debris-intercept
  tasks** for falling wreckage whose predicted impact point lies in
  DANGEROUS or CRITICAL ground: CRITICAL impacts shall be prioritised over
  DANGEROUS impacts, debris predicted to land in SAFE ground shall not be
  engaged, and debris tasks shall be assignable **only to kinetic
  effectors capable of physical destruction** (projectile-gun UAVs and
  anti-air turrets — never net or electronic-warfare effectors). The
  optimisation objective is the minimisation of expected collateral damage
  and loss of life as expressed by the ground-risk zone weights.
- **PHY-GCS-007** — Weapon-target assignment shall be Pk-aware: a shooter
  whose effector engagement envelope (range, off-axis, closing speed)
  cannot achieve a viable kill probability against the track's estimated
  kinematics shall not be assigned to that track.

### 3.7 Fixed effectors — anti-air gun turrets

- **PHY-TUR-001** — The system shall integrate remote-controlled anti-air
  gun turrets as ground-based effectors of last resort, slaved to the GCS
  fire-control loop and subject to the same clearance interlock as UAV
  effectors (PHY-UAV-021).
- **PHY-TUR-002** — Turret performance reference: slew rate ≥ 90°/s
  (azimuth) / 60°/s (elevation), effective engagement ceiling ≥ 1 500 m
  slant range against class A/B/C targets, dispersion and time-of-flight
  characterised well enough to compute Pk and debris footprints.
- **PHY-TUR-003** — Turret fire solutions shall come from the fused track
  picture (not an independent picture), so ROE debris evaluation applies
  identically.

### 3.8 Fixed sensor network

- **PHY-SEN-001** — Surveillance radar: R⁴-law detection performance,
  radar-horizon limited, Doppler measurement; medium-range coverage of the
  defended volume.
- **PHY-SEN-002** — Passive RF sensors: very long range, bearing-only,
  emitter signature hashing (by design unable to separate class A from
  class D).
- **PHY-SEN-003** — EO/IR towers: short-range identification-quality
  imagery; the principal decoy discriminator; performance degrades with
  range, weather and illumination.
- **PHY-SEN-004** — Acoustic pickets: detection below the radar horizon,
  engine-type cue, coarse bearing, short range.
- **PHY-SEN-005** — All sensors (fixed and airborne) are line-of-sight
  instruments: buildings and terrain shall mask or attenuate their
  channels according to the obstructing material (see SIM-SEN-005 for the
  simulation counterpart).

### 3.9 Sentinel surveillance UAVs

- **PHY-SNT-001** — The fleet shall include **unarmed sentinel UAVs**
  carrying a gimballed EO/IR payload and a passive RF receiver, providing
  persistent airborne overwatch of the defended urban area, in particular
  of volumes masked from the fixed sensor network by buildings.
- **PHY-SNT-002** — Sentinels shall fly pre-planned patrol orbits
  (station, radius, altitude, speed) and feed their detections into the
  common fusion picture through the same `detections` channel as the
  fixed sensor network; they carry no effector and never receive
  engagement tasks.
- **PHY-SNT-003** — Sentinel endurance, energy management and
  recharge/turnaround cycles follow the interceptor rules (PHY-UAV-013,
  SIM-PHX-002), with patrol resumed automatically after turnaround.

### 3.10 Charging and rearm stations

- **PHY-CHG-001** — UAV bases shall be explicit **charging stations**
  sited realistically on building rooftops or on ground pads adjacent to
  buildings; each interceptor and sentinel is homed to exactly one
  station, where it recharges and (for interceptors) rearms during the
  turnaround cycle.

---

## 4. Element 2 — Simulation Environment (SIM)

### 4.1 Purpose and fidelity rule

- **SIM-001 (traceability rule)** — For every PHY-\* requirement in §3
  there shall exist at least one SIM model or SIL mechanism that
  reproduces the specified behaviour, with its fidelity level and known
  deviations documented in a living PHY→SIM traceability table kept next
  to this SRS.
- **SIM-002** — The simulator shall present, northbound to E3, **exactly
  the ICD of §6** — the same messages the real GCS and assets would send —
  plus the evaluation-only ICD-EVAL channel. E3 shall not be able to
  distinguish E2 from E1 through the operational channels.
- **SIM-003** — Simulation shall be deterministic for a given scenario and
  RNG seed (regression: the existing `test_deterministic_given_seed`
  discipline extends to all new models).

### 4.2 World, time and execution

- **SIM-RT-001** — Fixed-step world integration (reference 20 Hz physics
  step, configurable) with per-node update rates, exactly as in the v0.1
  `sim/world.py` design.
- **SIM-RT-002** — Execution modes: (a) **real-time** wall-clock-locked for
  interactive demonstration through E3; (b) **scaled** real-time (0.1×–10×)
  for demonstration and tuning; (c) **headless as-fast-as-possible** for
  Monte-Carlo batches. Mode switching shall not alter results for a given
  seed.
- **SIM-RT-003** — The simulator shall support pause / resume / single-step
  and mid-run snapshot for inspection from E3.
- **SIM-RT-004** — A run shall be fully described by a scenario definition
  (YAML or the E3 scenario-start request of HMI-SCN-\*): map, risk zones,
  assets, sensor laydown, fleet, turrets, raid composition, weather,
  ROE thresholds, seed. Experiments are data, not code.

### 4.3 Flight physics

- **SIM-PHX-001** — Interceptor flight dynamics shall model, per tier:
  thrust/drag envelope, acceleration and turn-rate limits, climb/descent
  performance, VTOL transition (Tier-P), and the speed differences that
  make class A+ uncatchable by Tier-P (the physics that motivates
  cooperation must be real in the model).
- **SIM-PHX-002** — Energy model: battery/fuel drain as a function of
  airspeed, manoeuvre load and temperature; low-energy RTB triggering;
  recovery and rearm/recharge turnaround cycles at base stations.
- **SIM-PHX-003** — Wind field (mean + gusts, altitude-dependent) shall
  displace all airborne objects and degrade hover/track accuracy;
  precipitation and icing shall degrade performance per PHY-UAV-002.
- **SIM-PHX-004** — Threat flight dynamics per class (§2.3): cruise,
  terrain-masking low-level flight, altitude switching, weave, terminal
  dive profiles, jet-speed dash (A+), FPV agility (B).
- **SIM-PHX-005** — Fidelity upgrades (Gazebo/PX4 SITL for a small vehicle
  slice) shall be possible behind the same world interface without
  touching tactical software (ROADMAP Phase 3).

### 4.4 Threat behaviour

- **SIM-THR-001** — Threat generators for all classes A / A+ / B / C / D
  with per-class kinematics, signatures (radar RCS, RF emission hash,
  thermal/visual, acoustic) and warhead/decoy truth.
- **SIM-THR-002** — Raid composition shall be parametric: number of
  vehicles per class, per-vehicle objective (target asset or waypoint
  corridor), wave timing, approach axes, decoy mixing ratio. This is the
  contract behind the E3 "new execution" request (HMI-SCN-002).
- **SIM-THR-003** — Threat behaviours shall include the documented tactical
  patterns: saturation waves, altitude switching, decoy integration,
  two-phase strikes; class B/C threats shall support reactive evasion
  policies (dodge nearest interceptor, altitude drop) so herding and
  containment have a real adversary.
- **SIM-THR-004** — Decoys (class D) shall be indistinguishable from class
  A in every simulated sensor channel except those PHY defines as
  discriminating (close-range EO/IR, acoustic engine cue, kinematic
  anomalies) — ground truth stays quarantined (SIM-GT-001).

### 4.5 Sensor simulation

- **SIM-SEN-001** — Sensor models for every PHY-SEN-\* and PHY-UAV-012
  sensor: radar (R⁴ Pd falloff, radar horizon, Doppler), passive RF
  (bearing-only, signature hash), EO/IR towers (range-ramped ID quality),
  acoustic pickets (below-horizon hearing, coarse bearing), onboard
  seekers (terminal accuracy, close-range ID).
- **SIM-SEN-002** — All sensor outputs shall be `Detection` messages with
  full 3×3 covariance, noise, false alarms, missed detections and
  latency — never ground truth.
- **SIM-SEN-003** — Sensor performance shall be coupled to the environment
  model: weather (rain/fog attenuation of EO/IR and acoustic), lighting
  (day/night/thermal crossover for EO vs IR), terrain occlusion and radar
  horizon vs threat altitude.
- **SIM-SEN-004** — Turret fire-control sensing (slaved optics/track input)
  shall be modelled with its own latency and error budget.
- **SIM-SEN-005** — Sensor-to-target visibility shall be computed by a
  2.5D ray-versus-building query against the SIM-ENV-004 building set,
  with **material- and modality-dependent transmittance** per crossed
  building: EO/IR and seekers are fully blocked by any opaque structure;
  radar is blocked by concrete/brick and partially transmitted by
  glass/steel and light-metal structures (two-way attenuation); passive RF
  is partially attenuated per material; acoustic sensing diffracts around
  structures with mild attenuation. Parks and water bodies do not
  obstruct. The query shall be a pure function of static geometry
  (deterministic, SIM-003) and cheap enough for every sensor-target pair
  at sensor update rates.

### 4.6 Onboard software execution (software-in-the-loop)

- **SIM-SIL-001** — The simulator shall execute the *actual* E1 tactical
  software — guidance, cooperation, mode FSM, fire control, fusion,
  TEWA/ROE — as SIL nodes on the message bus, not behavioural
  re-implementations (enabled by PHY-UAV-030/034). The v0.1 Python nodes
  are the current SIL stand-ins; the seam to rclpy/ROS 2 shall be
  preserved.
- **SIM-SIL-002** — Per-board execution shall be schedulable at the rates
  the real boards would achieve; the simulator shall support injecting
  compute latency budgets (e.g. seeker inference at 30 Hz, TEWA at 1 Hz)
  so timing-dependent behaviour is representative.
- **SIM-SIL-003** — The simulator shall support fault injection: node
  crash/restart, sensor dropout, degraded GNSS, stale tracks — to exercise
  PHY-UAV-033 autonomy degradation.

### 4.7 Communications simulation

- **SIM-COM-001** — All inter-element and inter-vehicle traffic shall pass
  through a simulated network layer modelling: latency distributions,
  jitter, packet loss vs range/terrain, link capacity, and message
  serialisation per the ICD.
- **SIM-COM-002** — Jamming and link-denial events shall be scriptable per
  scenario (area, band, duration), driving the link-quality telemetry and
  autonomy fallback of PHY-UAV-033/043.
- **SIM-COM-003** — The clearance-token interlock shall traverse the same
  simulated network as everything else: lost or late clearances must
  produce the same hold/abort behaviour the real interlock would.

### 4.8 Effector, projectile and turret physics

- **SIM-EFF-001** — Engagement adjudication: kills are resolved by the
  sim-side adjudicator against ground truth (truth Pk roll), never by the
  tactical software (existing `sim/adjudicator.py` pattern).
- **SIM-EFF-002** — Effector Pk surfaces (range / off-axis / closing speed)
  per PHY-UAV-022 for net gun and projectile gun; configurable per
  scenario for tuning studies.
- **SIM-EFF-003** — Projectile physics for gun effectors and turrets:
  muzzle velocity, ballistic drop and drag, dispersion, time of flight
  against the target's predicted motion, and **miss-trajectory terminal
  impact points** scored against the ground risk map (stray rounds are
  collateral too, not only wrecks).
- **SIM-EFF-004** — Gun turret physics: slew/elevation rate and
  acceleration limits, settle time, firing arcs and masking, rate of fire,
  magazine and thermal limits per PHY-TUR-002.
- **SIM-EFF-005** — Debris model: mechanism-dependent wreck ballistics
  (net ≈ 0.15 horizontal velocity retention vs projectile ≈ 0.65), fall
  time, altitude-growing dispersion — used predictively in ROE and, since
  v0.3, generatively on kill as a live falling object (§4.11).
- **SIM-EFF-006** — Munition release and adjudication shall require
  shooter-to-target **line of sight**: shots whose sight line crosses a
  solid building are inhibited by fire control where the tactical picture
  allows, and adjudicated as blocked (`fire_blocked_los`, no Pk roll) by
  the sim-side adjudicator otherwise; blocked turret bursts still produce
  stray rounds (SIM-EFF-003).

### 4.9 Environment, weather and lighting

- **SIM-ENV-001** — World model: terrain and building geometry of the
  defended area, protected asset list, SAFE / DANGEROUS / CRITICAL risk
  raster (default DANGEROUS), kill-box derivation (`nearest_safe_cell`).
- **SIM-ENV-004** — Every building shall carry a **kind**
  (`residential_high`, `residential_low`, `school`, `hospital`,
  `commercial`, `industrial`, `park`, `water`) and a **material**
  (`concrete`, `brick`, `glass_steel`, `light_metal`, `wood`, `none`),
  with kind-appropriate material defaults; kind drives civilian-presence
  zoning (SIM-ENV-005) and material drives occlusion (SIM-SEN-005).
- **SIM-ENV-005** — The risk raster shall be **derivable from building
  kinds** (`zone_source: buildings`) as a civilian-presence map:
  **CRITICAL (red) = civilians certainly present** — hospital and school
  footprints plus a 100 m buffer, dense residential blocks plus a 50 m
  buffer; **DANGEROUS (yellow) = civilians possibly present** — low
  residential and commercial footprints plus kind-specific buffers and
  the street fabric between them; **SAFE (green) = civilian-free ground**
  — parks, water and restricted industrial ground. Hand-painted rect
  zones (`zone_source: rects`) remain supported for legacy scenarios and
  as manual overrides. The existing zone weights, collateral-cost and ROE
  machinery consume the derived raster unchanged.
- **SIM-ENV-006** — A deterministic, seeded **city generator** shall emit
  a complete scenario definition (street-grid building layout with all
  kinds of SIM-ENV-004, derived zones, charging stations, sensor and
  fleet laydown, threat raid) so realistic urban scenarios are data, not
  code (SIM-RT-004).
- **SIM-ENV-002** — Weather state machine per scenario: wind profile,
  precipitation type/intensity, fog density, temperature — all coupled
  into flight physics (SIM-PHX-003) and sensors (SIM-SEN-003).
- **SIM-ENV-003** — Lighting model: sun/moon ephemeris or scripted
  day/night state, driving EO vs IR sensor performance and the E3 visual
  presentation (HMI-MAP-006).

### 4.10 Ground truth management and evaluation outputs

- **SIM-GT-001** — Ground truth shall remain quarantined: only sim-side
  components (sensor models, adjudicator, evaluation recorder) read true
  state; tactical SIL software sees only ICD messages (v0.1 principle,
  unchanged).
- **SIM-GT-002** — The simulator shall publish ICD-EVAL (§6): the full
  ground-truth object list (including unacquired threats, decoy truth,
  true kinematics) on a channel separate from the operational ICD,
  consumed only by E3 evaluation overlays and the metrics recorder.
- **SIM-GT-003** — Per-run evaluation record: time-stamped truth + track
  picture + telemetry + every C2 decision and authorisation, sufficient to
  replay the run in E3 and to compute the §7 metrics (detection latency
  per threat, classification/decoy timelines, intercept geometry, debris
  cost, ammunition economics).
- **SIM-GT-004 (engagement attribution)** — Every fire, kill, miss and
  blocked-shot event shall carry the **shooter identity, weapon
  (effector) type, target identity, target kind (track or debris),
  outcome and the engaged-time Pk**; the per-run metrics shall include
  per-shooter and per-weapon engagement summaries (shots, hits, kills,
  debris kills, mean Pk) so "who shot what with which weapon" is
  answerable from the record and from the live display (HMI-MAP-002).

### 4.11 Live debris and debris interception

- **SIM-DEB-001** — A kill shall spawn one **live falling-debris object**
  integrated per world tick: mechanism-dependent horizontal velocity
  retention at spawn (SIM-EFF-005), gravity-accelerated fall capped at
  terminal velocity, horizontal velocity preserved — numerically
  consistent with the predictive footprint model used by ROE.
- **SIM-DEB-002** — Each live debris object shall be published on a
  dedicated `debris/state` topic with position, velocity, analytic
  predicted ground-impact point, the risk-zone class under that point and
  time-to-impact (this channel stands in for a debris-tracking radar;
  fidelity class *representative*).
- **SIM-DEB-003** — Debris whose predicted impact zone is DANGEROUS or
  CRITICAL shall be interceptable per PHY-GCS-006 (kinetic effectors
  only, CRITICAL before DANGEROUS, SAFE-bound debris never engaged); a
  successful intercept removes the object and replaces it with
  negligible-collateral fragments, and the run metrics shall credit the
  averted zone cost (`debris_saved_cost`).
- **SIM-DEB-004** — Fragments produced by intercepting debris shall be
  modelled as negligible: debris interception shall not spawn further
  interceptable objects (no recursive hazard).
- **SIM-DEB-005** — Debris that reaches the ground impacts as a wreck,
  scored against the risk map exactly as v0.1 instantaneous wrecks were.

---

## 5. Element 3 — Command Interface (HMI) and Orchestration Agent (ORC)

### 5.1 Role and connectivity

- **HMI-001** — E3 is the single human interaction point of the system:
  state control, authorisations, inspection of UAV operations, threat
  picture, sensor and effector status — all on a 3D real-time map.
- **HMI-002** — E3 shall communicate exclusively over the §6 ICD. In the
  production configuration the peers are real UAVs, radar and sensor
  network, anti-air turrets and GCS; **at this stage the single peer is
  the simulator (E2), which emulates all of them** (SYS-002/003).
- **HMI-003** — E3 shall function identically whether E2 runs live or a
  recorded run is replayed (record/replay parity, building on the v0.1
  recorder/dashboard).

### 5.2 3D real-time map

- **HMI-MAP-001** — Central view: 3D real-time map of the defended area —
  terrain, buildings, risk-zone colouring (SAFE / DANGEROUS / CRITICAL),
  protected assets, sensor positions and coverage envelopes, turret
  positions and firing arcs, base/recovery stations.
- **HMI-MAP-002** — Live entities rendered at display rate ≥ 30 fps with
  data updates at the ICD rates: interceptor UAVs (position, attitude,
  mode, task, energy, ammunition, link quality), acquired threat tracks
  (class belief, `p_decoy`, predicted trajectory and impact point),
  engagement events (fire requests, clearances, releases, results, debris
  footprints).
- **HMI-MAP-003** — Time controls in replay/scaled modes: timeline scrub,
  speed factor, pause/step (driving SIM-RT-002/003 when connected live).
- **HMI-MAP-004** — Selection and inspection: clicking any entity opens
  its detail panel — full telemetry history for UAVs, fused-track evidence
  trail for threats (which sensors contributed, belief evolution), decision
  log entries for C2 actions.
- **HMI-MAP-005** — Operational layers shall be toggleable (sensor
  coverage, predicted impacts, debris previews, comms links, kill boxes).
- **HMI-MAP-006** — The scene shall reflect simulated lighting and weather
  (night, fog, precipitation) so demonstrations communicate the sensing
  conditions (fed by SIM-ENV-002/003 over ICD-EVAL or scenario metadata).
- **HMI-MAP-007** — Buildings shall be rendered by kind and material
  (recognisable residential blocks, visually distinct hospitals and
  schools, parks and water), charging stations shall be shown at their
  rooftop/ground positions with occupancy state, and the red/yellow/green
  civilian-presence zoning shall remain clearly readable even where
  buildings stand on coloured ground (e.g. zone-tinted roofs and zone
  border outlines), with an on-screen legend. UAV and threat models shall
  be true-scale 3D airframes with zoom-aware magnification so they stay
  visible at map scale and become 1:1 when viewed close (operator-
  adjustable magnification).
- **HMI-MAP-008** — Live debris shall be rendered in flight with its
  predicted impact point coloured by the zone class beneath it;
  engagements shall be visually attributable — weapon-coloured tracer
  from shooter to target on every release, kill markers naming the
  shooter, and debris-intercept effects (HMI counterpart of SIM-GT-004 /
  SIM-DEB-002).

### 5.3 Threat display — acquired threats (production behaviour)

- **HMI-THR-001** — In production mode, E3 shall display **only acquired
  threats**: tracks delivered by the fusion layer over the operational
  ICD. The interface cannot and shall not present knowledge the sensor
  network does not have.
- **HMI-THR-002** — Acquired-threat presentation shall encode class belief
  and decoy probability visually (e.g. solidity/colour by class, fade with
  `p_decoy`), with predicted impact point and time-to-impact for
  threat-scored tracks.
- **HMI-THR-003** — Track lifecycle shall be visible: new acquisition,
  coasting (sensor dropout), re-acquisition, drop — with the contributing
  sensor(s) inspectable per HMI-MAP-004.

### 5.4 Threat display — evaluation overlay (this stage)

- **HMI-EVAL-001** — Because this stage uses the system for **evaluation,
  demonstration and performance tuning**, E3 shall additionally render
  **not-yet-acquired threats as grey wireframe "ghost" entities** at their
  ground-truth positions, sourced exclusively from ICD-EVAL (SIM-GT-002).
- **HMI-EVAL-002** — On first acquisition by the perception layer, the
  entity's representation shall transition (ghost grey wireframe → solid
  tracked entity per HMI-THR-002), visibly time-stamping the
  detection event; the transition shall be recorded for metrics
  (detection-latency measurement per threat).
- **HMI-EVAL-003** — Ghost rendering shall also expose, on inspection,
  truth-vs-track deltas for acquired threats (true vs estimated position,
  true class vs belief, decoy truth) for tuning work.
- **HMI-EVAL-004** — The evaluation overlay shall be a clearly marked,
  globally toggleable mode, visually unambiguous (grey wireframe + an
  "EVALUATION" state indicator), and shall be entirely absent — not merely
  hidden — when E3 is built/configured for production (SYS-003): if
  ICD-EVAL is not connected, no ghost code path is reachable.
- **HMI-EVAL-005** — Evaluation dashboards: live and post-run §7 metrics
  (detection latency distribution, classification timelines, intercept
  outcomes, debris cost, ammo per kill), per run and per Monte-Carlo
  batch.
- **HMI-EVAL-006** — The interface shall present a live **engagement
  summary** (per shooter: weapon, shots, hits, kills; per weapon type:
  the same aggregates) sourced from the SIM-GT-004 attribution fields,
  and the event log shall render engagements human-readably
  (`shooter → target [weapon] OUTCOME (pk)`).

### 5.5 Authorisations and human-on-the-loop control

- **HMI-AUT-001** — E3 shall present every authorisation request —
  fire-clearance requests with their ROE evaluation (geometry_safe /
  now_or_never / last_resort / hold / denied rationale and debris-footprint
  preview on the map), engagement-task confirmations, ROE threshold
  changes, turret release — as actionable items with a single-action
  approve/deny and full context.
- **HMI-AUT-002** — Authorisation latency budget: a fire-clearance request
  shall be presentable to the operator within 200 ms of receipt and
  answerable in one interaction; expiry of the engagement window shall be
  shown live.
- **HMI-AUT-003** — E3 shall allow operators to set the autonomy posture
  per ROE category: *human-confirm each release*, *pre-authorised within
  ROE bounds* (C2/ORC may auto-clear geometry_safe shots), or *weapons
  hold* — globally and per threat class.
- **HMI-AUT-004** — Every authorisation, denial, posture change and
  override shall be logged immutably with operator identity, timestamp and
  full decision context (the audit trail of SYS-004).
- **HMI-AUT-005** — Manual interventions shall be available at all times:
  global weapons hold, per-UAV abort/RTB, task reassignment veto.

### 5.6 Scenario control

- **HMI-SCN-001** — E3 shall allow starting a **new environment
  execution** on the connected simulator, and stopping/resetting the
  current one (SIM-RT-004).
- **HMI-SCN-002** — The new-execution form shall accept at minimum:
  - **number of threats per class** (A, A+, B, C, D);
  - **per-threat (or per-group) objectives**: target asset, approach axis
    or corridor, wave timing;
  - scenario selection (map/laydown/fleet preset), weather and lighting,
    speed factor, RNG seed (blank = random, displayed for
    reproducibility).
- **HMI-SCN-003** — The request shall be transmitted over ICD-SCN (§6);
  E2 validates it (capacity, asset references) and answers with the
  resolved scenario or a structured rejection shown to the user.
- **HMI-SCN-004** — Batch mode: the same parametrisation shall support
  launching N-seed Monte-Carlo batches headlessly, with results landing in
  the HMI-EVAL-005 dashboards.

### 5.7 Main orchestration agent (ORC)

- **ORC-001** — A main orchestration agent shall be connected to E3 and,
  through it, to the same operational ICD. It supervises and directs UAV
  operations and engagement decisions: monitoring the fused picture,
  steering TEWA priorities, tasking/retasking interceptors and turrets,
  and managing fleet posture (CAP stations, reserves, RTB/rearm cycles).
- **ORC-002** — The agent shall operate strictly within the SYS-004
  authority chain: whenever a decision requires human authorisation under
  the current autonomy posture (HMI-AUT-003) — weapon release above
  pre-authorised ROE, ROE escalation (now_or_never / last_resort),
  engagement of an ambiguous (`p_decoy`-borderline) track, sacrificial or
  high-risk manoeuvres — it shall raise the request **on the interface**
  and proceed only on explicit approval.
- **ORC-003** — Every agent recommendation and action shall carry a
  human-readable rationale (threat scores, geometry, ROE evaluation,
  alternatives considered), displayed in the HMI-AUT-001 panels and logged
  per HMI-AUT-004.
- **ORC-004** — Agent latency: routine tasking decisions within one TEWA
  cycle (1 s); escalation requests forwarded to the operator immediately
  upon detection of the triggering condition.
- **ORC-005** — The operator shall be able to bound or suspend the agent
  at any time (full manual fallback, HMI-AUT-005); agent unavailability
  shall never block manual operation of the system.
- **ORC-006** — The agent shall not consume ICD-EVAL ground truth.
  It operates on the same information a production deployment would have;
  only humans and the metrics pipeline see ghosts (preserves evaluation
  validity).

---

## 6. Interface control summary (ICD)

The pub/sub topic contract is the single seam between elements. It extends
the v0.1 contract (ARCHITECTURE.md §2) and is the surface E2 must emulate
bit-for-bit (SIM-002).

### 6.1 Operational channels (production-identical)

| Channel | Content | Producer → Consumer |
|---|---|---|
| `detections` | `Detection` (3×3 covariance, sensor id) | sensors → fusion |
| `tracks` | `TrackArray` (state, class belief, p_decoy) | fusion → C2, UAVs, E3, ORC |
| `uav/state` | `UavState` (pose, mode, energy, ammo, link) | each UAV → C2, peers, E3, ORC |
| `turret/state` | turret pose, arcs, magazine, thermal | turrets → C2, E3, ORC |
| `engagement/tasks` | `EngagementTask` list | C2/ORC → UAVs, turrets |
| `engagement/fire_request` | `FireRequest` + ROE evaluation | shooter → C2 → E3 (HMI-AUT-001) |
| `engagement/clearance` | signed `FireClearance` token | C2 (post-authorisation) → shooter |
| `engagement/fire` | release event | shooter → adjudicator (E2) |
| `engagement/result` | `EngagementResult` + debris footprint | adjudicator → C2, E3, ORC |
| `c2/decision_log` | TEWA decisions, rationales | C2/ORC → E3 audit |
| `env/conditions` | weather/lighting state | E2 (prod: met sensors) → E3 |
| `debris/state` | `DebrisArray` (live falling debris, predicted impact + zone) | E2 (prod: debris-tracking radar) → C2, E3, ORC |

- **ICD-001** — All channels shall carry versioned, schema-defined
  messages (dataclasses today, ROS 2 `.msg`/DDS at migration), transported
  over the simulated network layer (SIM-COM-001) when E2 is the peer.
- **ICD-003** — The v0.3 runtime wire-schema extensions are normative and
  documented in [ICD_RUNTIME.md](ICD_RUNTIME.md): building `kind` /
  `material` / `name` and `stations` in the scene payload; `kind`
  (interceptor/sentinel) on UAV states; a `debris` array, enriched
  attribution fields on engagement events and station occupancy in
  frames; `debris_intercepts`, `debris_saved_cost` and the
  `engagements` summary in evaluation metrics.

### 6.2 Evaluation-only channels (this stage; absent in production)

| Channel | Content | Producer → Consumer |
|---|---|---|
| `eval/ground_truth` | full true object list incl. unacquired threats, decoy truth, true kinematics | E2 → E3 ghost overlay (HMI-EVAL-\*), metrics |
| `eval/metrics` | live per-run metric stream | E2 → E3 dashboards |
| `scn/control` (ICD-SCN) | new-execution request / stop / pause / speed / snapshot; validation responses | E3 → E2 |

- **ICD-002** — `eval/*` channels shall be physically separable (distinct
  endpoint/port), enforcing HMI-EVAL-004 and ORC-006 by construction.

---

## 7. Verification and evaluation metrics

- **VER-001** — Every SIM-\* requirement shall be covered by deterministic
  unit/integration tests (extending the existing 21-test suite and the
  10-seed Monte-Carlo discipline).
- **VER-002** — System-level evaluation metrics computed per run and per
  batch (consumed by HMI-EVAL-005):
  - detection latency per threat (ghost-to-acquired time, by class,
    range, altitude, weather);
  - classification and decoy-discrimination timelines (time to correct
    class, decoy shots avoided);
  - engagement outcomes (attrition by class, time-to-intercept,
    Pk-realised vs Pk-predicted);
  - collateral performance (debris cost integral, critical-zone wrecks —
    invariant: 0 — stray-round impacts);
  - economics (ammo per kill, interceptors spent per threat, decoy
    expenditure);
  - human/agent loop (authorisation latency, escalations raised/approved,
    window expiries).
- **VER-003** — The v0.1 verified baseline (0 critical-zone wrecks, 0
  shots at identified decoys over the 10-seed reference raid) is the
  regression floor for all subsequent E2/E3 development.

---

## 8. Traceability

- **TRC-001** — A PHY→SIM traceability table (one row per PHY requirement:
  simulating model/mechanism, fidelity class {high / representative /
  placeholder}, known deviations) shall be created with the first E2
  increment under this SRS and maintained in `docs/`.
- **TRC-002** — Requirements in this SRS supersede conflicting statements
  in README/ARCHITECTURE v0.1; those documents remain as design rationale
  for the implemented baseline.
