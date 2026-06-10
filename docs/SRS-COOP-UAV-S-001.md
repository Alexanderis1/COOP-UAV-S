# SRS-COOP-UAV-S-001
# System Requirements Specification
## COOP-UAV-S — Cooperative Counter-UAS System

| Field | Value |
|---|---|
| Document ID | SRS-COOP-UAV-S-001 |
| Version | 0.5 — All open issues resolved; response time requirements, HEL effector, fleet sizing, ROE thresholds |
| Status | DRAFT |
| Date | 2026-06-10 |
| Classification | RESTRICTED |
| DO-178C DAL | B (Hazardous) |
| ARP-4754A System DAL | B |
| Prepared by | Requirements Engineering |
| Approved by | TBD |

---

## Document Change History

| Rev | Date | Author | Description |
|---|---|---|---|
| 0.1 | 2026-06-10 | Requirements Engineering | Initial draft — elicited from stakeholder and derived from hackathon baseline (`claude/uav-swarm-interception-hackathon-f0vqi0`) |
| 0.2 | 2026-06-10 | Requirements Engineering | OI-001 resolved: Class A+ engagement strategy defined as two-mode cooperative approach (relay interception primary; herding to anti-air gun kill zone secondary). Added SRS-COOP-007 through SRS-COOP-013, SRS-C2-011, SRS-IF-006/007, SRS-SAF-010/011. Updated threat table, traceability matrix, and OI list. |
| 0.3 | 2026-06-10 | Requirements Engineering | Stakeholder correction: threat trajectory adaptation attribute added; herding strategy restricted to maneuvering threats only (Class B/C); fixed-route threats (Class A/A+) use route-ambush gun coordination instead. Battery orchestration requirements added: charging stations, minimum deployment threshold, autonomous RTB/charge/redeploy cycle. Added SRS-CLS-009–012, SRS-C2-012, SRS-COOP-014–016, SRS-UAV-014–021, SRS-SIM-012–014. |
| 0.5 | 2026-06-10 | Requirements Engineering | All six open issues closed. OI-002/003: geometry-derived response time and detection range requirements adopted (SRS-PERF-001–008 TBDs resolved). OI-004: HEL (High-Energy Laser) selected; SRS-EFF-009/010 replaced by SRS-EFF-011–015. OI-005: 7 sectors / metropolitan area, 8 interceptors + 4 sentinels per sector (SRS-ARCH-002/003 updated). OI-006: HOTL pre-authorization window capped at 30 minutes (SRS-C2-009 updated). OI-007: ROE thresholds adopted as requirements; IHL legal review mandatory pre-deployment in PSSA (SRS-ROE-006 updated). C2 loop rate raised to ≥ 2 Hz (SRS-C2-001 updated). |
| 0.4 | 2026-06-10 | Requirements Engineering | Sentinel UAV role added (forward observer, RF-silent detection, outside defended zone). 3D coverage map with obstacle-aware LOS, GREEN/RED voxel status, staleness tracking, and coverage utility metric. Anti-air turret integration as ground-fixed effectors with fire control slaving, ROE enforcement, and turret-UAV deconfliction. Operator 3D situational awareness display expanded. Fleet role taxonomy (SENTINEL vs INTERCEPTOR) formalised. Added SRS-SENT-001–010, SRS-COV-001–010, SRS-TUR-001–007, SRS-C2-013–014, SRS-IF-008–009, SRS-SAF-012, SRS-SIM-015–017. |

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Operational Concept](#2-operational-concept)
3. [System Architecture Overview](#3-system-architecture-overview)
4. [Functional Requirements — Detection](#4-functional-requirements--detection)
5. [Functional Requirements — Tracking and Fusion](#5-functional-requirements--tracking-and-fusion)
6. [Functional Requirements — Classification and Threat Identification](#6-functional-requirements--classification-and-threat-identification)
7. [Functional Requirements — Command and Control (C2)](#7-functional-requirements--command-and-control-c2)
8. [Functional Requirements — Rules of Engagement (ROE)](#8-functional-requirements--rules-of-engagement-roe)
9. [Functional Requirements — Interceptor UAV](#9-functional-requirements--interceptor-uav)
10. [Functional Requirements — Effectors](#10-functional-requirements--effectors)
11. [Functional Requirements — Cooperative Engagement](#11-functional-requirements--cooperative-engagement)
12. [Performance Requirements](#12-performance-requirements)
13. [Interface Requirements](#13-interface-requirements)
14. [Safety Requirements](#14-safety-requirements)
15. [Cybersecurity Requirements](#15-cybersecurity-requirements)
16. [Environmental Requirements](#16-environmental-requirements)
17. [Simulation and Verification Platform Requirements](#17-simulation-and-verification-platform-requirements)
18. [Open Issues and TBDs](#18-open-issues-and-tbds)
19. [Requirements Traceability Matrix](#19-requirements-traceability-matrix)

---

## 1. Introduction

### 1.1 Purpose

This document defines the System Requirements Specification (SRS) for the **COOP-UAV-S** (Cooperative UAV System for Counter-UAS) programme. It establishes all mandatory, preferred, and optional requirements for the operational Counter-UAS (C-UAS) system and for the simulation and validation platform that supports its development and certification.

This SRS is the primary technical agreement between the stakeholder and the development team. All software and hardware design must be traceable to requirements in this document.

### 1.2 Scope

The COOP-UAV-S system provides layered, multi-domain detection, classification, tracking, and engagement of hostile unmanned aerial vehicles (UAVs) operating in urban and peri-urban environments. The system employs cooperative interceptor UAVs, a hierarchical Command and Control (C2) architecture, multi-modal sensing, and ground-risk-aware Rules of Engagement to minimise collateral damage in civilian-populated areas.

**In scope:**
- Operational C-UAS system: sensors, interceptor UAVs, base stations, C2 software, effectors, and communication infrastructure.
- Simulation and Validation (S&V) platform: the software simulation environment used to develop, test, and validate tactics, algorithms, and software components prior to hardware integration.

**Out of scope:**
- Strategic air defence against manned aircraft, ballistic missiles, or cruise missiles above 5,000 m AGL.
- Offensive UAV operations.
- Electronic warfare targeting of ground-based radio infrastructure.

### 1.3 Definitions and Abbreviations

| Term | Definition |
|---|---|
| AGL | Above Ground Level |
| C2 | Command and Control |
| C-UAS | Counter Unmanned Aerial System |
| DAL | Design Assurance Level (per DO-178C / ARP-4754A) |
| EO/IR | Electro-Optical / Infrared |
| EW | Electronic Warfare |
| FPV | First-Person View (tactical kamikaze UAV class) |
| GNN | Global Nearest Neighbour (data association algorithm) |
| HITL | Human-In-The-Loop (operator authorises every individual fire command) |
| HOTL | Human-On-The-Loop (system fires autonomously within pre-approved ROE; operator can veto) |
| IHL | International Humanitarian Law |
| IMM | Interacting Multiple Model (tracking filter variant) |
| KF | Kalman Filter |
| LOITERING | Loitering Munition threat class |
| MTBF | Mean Time Between Failures |
| OWA | One-Way Attack (UAS used in a single-direction strike, not recoverable) |
| Pk | Probability of Kill |
| PSSA | Preliminary System Safety Assessment |
| ROE | Rules of Engagement |
| RCS | Radar Cross Section |
| RF | Radio Frequency |
| RTB | Return To Base |
| SAFE/DANGEROUS/CRITICAL | Ground-risk zone classification (SORA/JARUS-derived) |
| S&V | Simulation and Validation |
| SRS | System Requirements Specification |
| STANAG | Standardisation Agreement (NATO) |
| TEWA | Threat Evaluation and Weapon Assignment |
| TTI | Time To Impact |
| VTOL | Vertical Take-Off and Landing |
| WTA | Weapon-Target Assignment |
| FIXED-ROUTE | Threat trajectory adaptation class: follows a pre-programmed GPS/INS route; does NOT respond to interceptor positioning or threats. Cannot be herded or lured. |
| Sentinel UAV | UAV operating in forward observer role outside the defended zone perimeter. Carries sensors, not effectors. Publishes coverage footprints and detections. Distinct from Interceptor UAV. |
| Interceptor UAV | UAV operating in defender role within or at the edge of the defended zone. Carries effectors (projectile, net). Executes engagement tasks from C2. |
| RF-silent threat | A hostile UAV that emits no detectable RF signal (no datalink, no active GPS receiver noise). Cannot be detected by RF-DF sensors. Detectable only by radar, EO/IR, acoustic, or onboard seeker. |
| Voxel | The smallest addressable unit of the 3D coverage map: a cuboid cell in the 3D grid with defined spatial extent. |
| GREEN cell | A coverage map voxel that is currently within the line-of-sight and detection range of at least one active Sentinel UAV sensor. |
| RED cell | A coverage map voxel not currently covered by any active Sentinel UAV sensor. |
| RED dwell time | Elapsed time since a voxel was last classified GREEN. Exceeding the configured threshold triggers a coverage gap alert. |
| LOS | Line-of-Sight: an unobstructed straight line between a sensor and a point in space, used for coverage computation. |
| Anti-air Turret | A fixed, ground-mounted kinetic effector (gun system) with an automated or remote-operated fire control, integrated with the C2 for track slaving and ROE enforcement. Distinct from a manually aimed human gun crew. |
| Coverage utility | Scalar metric [0,1] representing the fraction of critical surveillance zone voxels currently GREEN. Used as patrol planning objective. |
| MANEUVERING | Threat trajectory adaptation class: AI or human-guided; can actively respond to interceptor positioning by adjusting route. Herding strategies are applicable. |
| Route-Ambush | Gun engagement coordination strategy for FIXED-ROUTE threats: gun crew is positioned at the safest available point ON the threat's predicted route (no trajectory deflection expected). |
| Herding | Engagement shaping strategy for MANEUVERING threats: interceptors create threat corridors that exploit the threat's evasion capability to channel it toward a designated kill zone. |

### 1.4 Applicable and Reference Documents

| ID | Title |
|---|---|
| DO-178C | Software Considerations in Airborne Systems and Equipment Certification |
| ARP-4754A | Guidelines for Development of Civil Aircraft and Systems |
| ARP-4761 | Guidelines and Methods for Conducting the Safety Assessment Process |
| JARUS SORA | Specific Operations Risk Assessment (Joint Authorities for Rulemaking of Unmanned Systems) |
| STANAG 4586 | Standard Interfaces of UAV Control System |
| IEC 62443 | Security for Industrial Automation and Control Systems (cybersecurity reference) |
| GJB 2786 | Military Software Development (reference for military DAL equivalence) |

### 1.5 Document Structure and Conventions

Requirements are structured as follows:
- **[SRS-xxx-NNN]** — unique requirement identifier (category code + three-digit number).
- **SHALL** — mandatory requirement. Non-compliance constitutes a defect.
- **SHOULD** — preferred requirement. Non-compliance requires documented justification.
- **MAY** — optional capability. Implementation at developer discretion.
- **TBD** — value or decision to be determined before PDR (Preliminary Design Review).
- **TBC** — value or decision to be confirmed by stakeholder review.
- **⚠ OPEN ISSUE** — see Section 18 for the full issue description.

### 1.6 Summary of Open Issues

All open issues are resolved at document version 0.5. The SRS may now be elevated to DRAFT BASELINE status pending formal stakeholder review. No SRS-level TBDs remain. Pre-deployment conditions (PSSA IHL review, validated Pk surfaces, validated debris model) are noted in the relevant requirements as operational release gates.

| ID | Summary | Impact | Status |
|---|---|---|---|
| OI-001 | Class A+ Jet OWA (100 m/s) vs. VTOL-only interceptor fleet | **CLOSED v0.2** — Two-mode cooperative strategy adopted: relay interception primary, route-ambush gun coordination secondary (v0.3 correction: herding prohibited for FIXED-ROUTE threats). See SRS-COOP-007 to SRS-COOP-016, SRS-C2-011/012, SRS-IF-006/007, SRS-SAF-010/011. | **CLOSED v0.2/v0.3** |
| OI-002 | Class-specific response time requirements not formally derived | High — performance requirements incomplete | **CLOSED v0.5** — Geometry-derived values adopted: Class A ≥ 90 s / 6 km; A+ ≥ 60 s / 9 km; B ≥ 90 s / 7 km; C ≥ 60 s / 7 km. See SRS-PERF-001/002, SRS-C2-001/002. |
| OI-003 | Detection and tracking KPIs not complete | High — performance requirements incomplete | **CLOSED v0.5** — Track accuracy ≤ 50 m at 5 km (Class A/A+/C); ≤ 30 m at 2 km (Class B). See SRS-PERF-005. |
| OI-004 | Directed energy effector type not specified | Medium — impacts platform, power, safety | **CLOSED v0.5** — HEL (High-Energy Laser) selected. SRS-EFF-009/010 replaced by SRS-EFF-011–015. |
| OI-005 | Metropolitan fleet sizing not defined | Medium — scales comms and C2 | **CLOSED v0.5** — 7 sectors / metropolitan area; 8 interceptors + 4 sentinels per sector. See SRS-ARCH-002/003. |
| OI-006 | HOTL pre-authorization window duration not specified | Medium — safety and legal implication | **CLOSED v0.5** — Maximum 30 minutes. See SRS-C2-009. |
| OI-007 | ROE collateral damage threshold values not validated | High — safety and IHL compliance | **CLOSED v0.5** — Thresholds adopted as requirements; mandatory IHL legal review and PSSA sign-off required before operational deployment. See SRS-ROE-006. |

---

## 2. Operational Concept

### 2.1 Mission Objective

The COOP-UAV-S system shall detect, track, classify, and neutralise hostile UAV threats operating in urban and peri-urban environments, while minimising collateral damage to civilians, infrastructure, and non-combatant assets.

The system shall achieve this through coordinated operation of multiple interceptor UAVs, multi-modal ground and airborne sensors, and a hierarchical Command and Control architecture that enforces ground-risk-aware Rules of Engagement at all times.

### 2.2 Operational Environment

The system shall be designed to operate in the following environment:

**Geographic context:**
- Urban and peri-urban areas containing civilian populations, critical infrastructure, and mixed military/civilian structures.
- Defended area size: between 1 km² (point defence) and 500 km² (metropolitan area defence).
- Altitude band: 50 m to 5,000 m AGL.

**Threat density:**
- Nominal sustained raid: up to 50 simultaneous hostile tracks per sector.
- Peak metropolitan raid density: up to 400+ UAVs per night across all sectors.

**Operational baseline:**
The system design is informed by the Ukrainian theatre of operations (2022–2026), which represents the first large-scale conflict where drone consumption rivals artillery expenditure and where adversarial drone tactics evolve on a monthly cycle. This does not constrain the system to any one geographic theatre.

### 2.3 Threat Taxonomy

The system shall address the following threat classes. All four classes are mandatory in-scope engagement targets.

| Class ID | Name | Representative Platform | Speed | Altitude AGL | Mass | Warhead | Trajectory Adaptation | Notes |
|---|---|---|---|---|---|---|---|---|
| A | Strategic OWA | Shahed-136 / Geran-2 | 50–65 m/s | 50 m – 5,000 m (adaptive) | ~200 kg | Yes, 50–90 kg | **FIXED-ROUTE** — GPS/INS programmed waypoints; lateral weave is pre-programmed, not threat-responsive. Cannot be herded or lured. | Saturation swarm; decoy mixing; terminal dive |
| A+ | Jet OWA | Geran-3 / Shahed-238 | 95–110 m/s | 2,000–5,000 m | ~200 kg | Yes | **FIXED-ROUTE** — same GPS-programmed route model. High speed makes relay interception geometry tight. Route-ambush is the secondary strategy. See SRS-COOP-007 to SRS-COOP-016. | High-speed; direct VTOL pursuit infeasible |
| B | Tactical FPV | Quadcopter kamikaze | 30–40 m/s | 0–200 m | 1–5 kg | Yes | **MANEUVERING** — human-in-loop (fiber-optic) or onboard AI. Actively evades interceptors. Herding to kill zone is valid when feasible. | Fiber-optic variants are RF-jam-immune; agile; AI variants may perform evasive manoeuvres |
| C | Loitering Munition | Lancet-3 | 70–90 m/s | 50–500 m | 10–15 kg | Yes | **MANEUVERING (terminal phase)** — AI-guided seeker adjusts approach vector in terminal phase. Pre-terminal cruise is largely fixed-route. Strategy must account for phase transition. | AI-guided terminal seeker; precision strike |
| D | RF Decoy | Gerbera-type | Matches Class A | Matches Class A | ~18 kg | No | **FIXED-ROUTE** — mirrors Class A profile, including trajectory. | Shares OWA radar signature; decoy fraction up to 60% of salvos |

**Notes on Class D (Decoys):** Class D decoys are not engagement targets and shall not consume interceptor resources. However, they cannot be positively distinguished from Class A at initial detection. The system shall implement decoy discrimination algorithms and shall not withhold engagement resources from potential Class A contacts solely on the basis of unconfirmed decoy probability.

### 2.4 Tactical Threat Patterns

The system design shall account for the following adversarial tactical patterns, which have been operationally validated:

| Pattern | Description |
|---|---|
| Altitude switching | Drones adapt flight profile (low-level terrain masking vs. high-altitude dive) based on detected defences |
| Saturation attacks | 24-hour attack cycles designed to exhaust interceptor and missile reserves |
| Decoy integration | Unarmed replicas with identical radar/RF signatures mixed into strike packages at up to 60% ratio |
| Fiber-optic FPV | Radio-jamming-immune, human-piloted short-range munitions (Class B) — EW is ineffective |
| Two-phase strikes | Secondary hit timed to target first responders at the site of an initial strike |
| Terrain masking | Low-altitude flight profile (< 80 m AGL) used to hide below radar horizon |

### 2.5 System Boundary and Partitioning

The COOP-UAV-S programme encompasses two distinct but interdependent systems, both within SRS scope:

**Partition A — Operational C-UAS System (OPS):** The deployable system consisting of physical sensors, interceptor UAVs, base station hardware/software, communication infrastructure, and effectors. This partition is the primary product.

**Partition B — Simulation and Validation Platform (S&V):** The software simulation environment (currently implemented in Python with ROS 2-shaped interfaces) used to develop, test, and validate algorithms, tactics, and software components. This partition is a development and certification tool, not a fielded product, and carries its own (lower) DAL obligations. See Section 17.

Requirements in Sections 4–16 apply to **Partition A** unless explicitly noted otherwise.

---

## 3. System Architecture Overview

> **Note:** This section describes the required architectural framework. It does not prescribe implementation unless a specific architectural pattern is itself a requirement (e.g., hierarchical C2, published interfaces).

### 3.1 Hierarchical C2 Topology

[SRS-ARCH-001] The system shall implement a three-tier hierarchical Command and Control architecture:

- **Tier 1 — Metropolitan C2 (City-Level):** Overall area defence strategy, threat prioritisation across sectors, ROE policy enforcement, cross-sector track fusion, and engagement authority management.
- **Tier 2 — Sector-Level Base Station:** Local TEWA loop, weapon-target assignment, fire authorisation per sector, interceptor fleet management within the sector.
- **Tier 3 — UAV-Level Autonomy:** Execution of assigned engagement tasks, cooperative manoeuvre, and fail-safe autonomous operation during comms degradation.

[SRS-ARCH-002] The baseline metropolitan deployment shall comprise **7 Tier 2 sector base stations** per metropolitan area. The system architecture shall scale to a minimum of 4 and maximum of 16 independent Tier 2 nodes without requiring changes to Tier 1 or Tier 3 software. OI-005 closed.

[SRS-ARCH-003] Each Tier 2 sector shall operate a minimum baseline fleet of **8 interceptor UAVs** and **4 sentinel UAVs**. This fleet size is the operational baseline derived from metropolitan threat density analysis (OI-005 closed). Sector fleet size shall be configurable and may be increased for higher-threat sectors.

### 3.2 Message-Based Interface Architecture

[SRS-ARCH-004] All inter-component communication within each tier shall be implemented via a typed publish/subscribe message bus. Message schemas shall be defined independently of the transport layer to allow migration to ROS 2 or equivalent middleware without modifying tactical software.

[SRS-ARCH-005] The following logical topics and message types shall be defined as part of the system interface contract. All message types shall carry a monotonic timestamp field (`stamp`) used for latency measurement and track data freshness assessment:

| Topic | Message Type | Producer | Consumers |
|---|---|---|---|
| `detections` | `Detection` | All sensors, Sentinel UAV seekers | FusionNode |
| `tracks` | `TrackArray` | FusionNode | C2 (all tiers), UAVs, Turrets, Recorder |
| `uav/state` | `UavState` | Each interceptor UAV | C2, peer UAVs, Recorder |
| `sentinel/state` | `SentinelState` | Each sentinel UAV | C2, Recorder |
| `sentinel/coverage_footprint` | `CoverageFootprint` | Each sentinel UAV | Coverage Map node |
| `sentinel/contact_report` | `ContactReport` | Each sentinel UAV | Sector C2 |
| `coverage/map` | `CoverageMap` (sparse diff) | Coverage Map node | C2, Operator display, Recorder |
| `coverage/alert` | `CoverageGapAlert` | Coverage Map node | Sector C2, Operator display |
| `turret/state` | `TurretState` | Each anti-air turret | C2, Recorder, Operator display |
| `turret/fire_request` | `FireRequest` | Anti-air turret | Sector C2 |
| `turret/clearance` | `FireClearance` | Sector C2 | Anti-air turret |
| `turret/fire` | `FireRequest` | Anti-air turret | Engagement Adjudicator |
| `engagement/tasks` | `EngagementTask` | Sector C2 | Interceptor UAVs |
| `engagement/fire_request` | `FireRequest` | Shooter UAV | Sector C2 |
| `engagement/clearance` | `FireClearance` | Sector C2 | Shooter UAV |
| `engagement/fire` | `FireRequest` | Shooter UAV | Engagement Adjudicator |
| `engagement/result` | `EngagementResult` | Engagement Adjudicator | Sector C2 |
| `c2/situation` | `SituationReport` | Sector C2 | Metropolitan C2 |
| `c2/directive` | `EngagementDirective` | Metropolitan C2 | Sector C2 |

[SRS-ARCH-006] The `Detection` message shall include a full 3×3 position covariance matrix. Bearing-only sensors (RF-DF, acoustic) shall encode their measurement geometry as an anisotropic covariance rather than as a special message type, so that the fusion node requires no per-sensor conditional logic.

---

## 4. Functional Requirements — Detection

### 4.1 Sensor Complement

[SRS-DET-001] The system shall include the following sensor modalities, operating simultaneously:

| Sensor | Primary Function | Key Weakness (shall be compensated by other sensors) |
|---|---|---|
| Radar (short/medium range) | Long-range range and Doppler velocity measurement | R⁴ power falloff; minimum elevation angle creates low-altitude detection gap |
| RF Direction-Finding (RF-DF) | Very long range bearing detection; threat signature correlation | Bearing-only (no range); Class A and Class D share the same RF signature |
| Electro-Optical / Infrared (EO/IR) | Visual/thermal classification; decoy discrimination at close range | Performance degrades in fog and dense precipitation; range limited |
| Acoustic | Detection of low-altitude threats below radar horizon | Short range; coarse bearing |
| Onboard Seeker (carried by interceptor) | Terminal-range precision tracking and classification | Only available where an interceptor is already deployed |

[SRS-DET-002] The radar subsystem shall implement a detection probability model that accounts for:
- R⁴ signal-to-noise falloff with range.
- Target RCS (Class A/D: ≥ 0.5 m²; Class B FPV: ≥ 0.01 m²; Class C: ≥ 0.05 m²).
- A minimum elevation angle below which low-altitude targets are not detected. The minimum elevation angle shall be ≤ 2°.

[SRS-DET-003] The RF-DF subsystem shall detect threats at ranges exceeding the radar subsystem's maximum detection range and shall output bearing estimates with associated angular uncertainty. The RF-DF subsystem shall flag contacts whose RF signature matches the Class A/D profile as requiring multi-sensor confirmation before classification.

[SRS-DET-004] The EO/IR subsystem shall provide positive classification evidence at close range (≥ 4 km detection range; ≥ 1.2 km full positive identification range, both derived from validated simulation baseline; OI-003 closed). Classification quality shall improve monotonically as range decreases. At maximum EO/IR classification range, the system shall provide a meaningful update to decoy probability.

[SRS-DET-005] The acoustic sensor network shall detect Class B (FPV) threats flying below the radar horizon at altitudes ≤ 200 m AGL and at ranges sufficient to provide a track confirmation within the Class B response time requirement (see OI-002). Acoustic sensors shall output bearing estimates and engine-class cues.

[SRS-DET-006] The onboard seeker on each interceptor UAV shall provide high-accuracy position and class estimates within the interceptor's terminal engagement range. Seeker measurements shall be published to the shared `detections` topic and fused with ground sensor data.

[SRS-DET-007] All sensors shall produce a `Detection` message on every positive detection cycle, regardless of sensor type. The message shall include: sensor identity, timestamp, estimated 3D position, full 3×3 covariance, and any modality-specific additional field (e.g., radial velocity for radar Doppler, RF signature hash for RF-DF, acoustic engine class for acoustic).

### 4.2 Detection Coverage

[SRS-DET-008] The combined sensor network shall provide no detection coverage gap in the altitude band 50 m to 5,000 m AGL within the defended area. Coverage gaps at low altitude (< 200 m) shall be compensated by the acoustic sensor picket network.

[SRS-DET-009] Sensor deployment shall be based on a formal coverage analysis that demonstrates detection probability ≥ 90% (OI-003 closed) for each threat class at the minimum required detection range.

---

## 5. Functional Requirements — Tracking and Fusion

### 5.1 Track Lifecycle

[SRS-TRK-001] The fusion node shall maintain a common operational picture by associating detections from all sensor modalities into a single set of tracks. Each track shall have a unique identifier that persists for the lifetime of the contact.

[SRS-TRK-002] A new contact shall be promoted from tentative to confirmed track status after receiving a minimum of 3 independent sensor updates within 5 seconds. Both threshold values shall be configurable per deployment scenario.

[SRS-TRK-003] A confirmed track shall be maintained (coasted) for a maximum of 5 seconds after the last sensor update. After this period, with no new detections associated, the track shall be dropped. The coast duration shall be configurable.

[SRS-TRK-004] Track confirmations and drops shall be logged with timestamps for post-mission analysis and V&V purposes.

### 5.2 Tracking Filter

[SRS-TRK-005] The tracking filter shall be a minimum 6-state (position + velocity) Kalman Filter per track. The filter shall predict track state forward in time at each sensor update cycle.

[SRS-TRK-006] The tracking filter shall implement an Interacting Multiple Model (IMM) architecture including at minimum: a constant-velocity model, a coordinated-turn model, and a terminal-dive model. This is required to accurately track Class A drones that execute altitude-adaptive profiles and terminal dive manoeuvres. ⚠ Note: the v0.1 draft uses constant-velocity only — this is a known limitation and shall be corrected.

[SRS-TRK-007] The data association algorithm shall use a Mahalanobis-gated Hungarian assignment (Global Nearest Neighbour) as the baseline. The gate threshold shall be configurable. For high-density raids exceeding 20 simultaneous tracks, the system should upgrade to a Joint Probabilistic Data Association (JPDA) or Random Finite Sets (RFS) tracker.

[SRS-TRK-008] Track-to-detection association shall be performed per sensor scan (one detection per target per scan). Precision sensors (radar) shall be processed before bearing-only sensors (RF-DF, acoustic) so that accurate sensors seed new tracks preferentially.

[SRS-TRK-009] The fusion output (`TrackArray`) shall be published at a minimum rate of 5 Hz. Each published track shall include: position estimate, velocity estimate, full covariance, class belief distribution, decoy probability, hit count, track age, and time since last update.

### 5.3 Track Data Freshness

[SRS-TRK-010] Any software component that consumes a track for engagement purposes shall assess track data freshness before acting. A track update older than **5 seconds** shall be treated as stale and shall not trigger a fire request. This matches the track coast window (SRS-TRK-003) — a stale track beyond this limit has no confirmed update and may have manoeuvred significantly. OI-002 closed.

[SRS-TRK-011] Interceptor UAV software shall extrapolate the most recently received track state to the current time before computing intercept geometry. Staleness beyond the configurable threshold shall cause the UAV to enter HOLD mode and await a fresh track update.

---

## 6. Functional Requirements — Classification and Threat Identification

### 6.1 Class Belief and Decoy Probability

[SRS-CLS-001] The system shall maintain a Bayesian class belief distribution per track. The distribution shall cover all threat classes (A, A+, B, C) and the decoy class (D). The belief shall be updated recursively as new sensor evidence arrives.

[SRS-CLS-002] The system shall compute a scalar decoy probability `p_decoy ∈ [0,1]` for each track. This value shall be published with every track update and shall be directly usable by the assignment and ROE modules.

[SRS-CLS-003] The classification algorithm shall incorporate evidence from all available sensor modalities. RF-signature evidence shall flag Class A and Class D contacts as jointly ambiguous (both share the same RF hash). Resolution between Class A and Class D shall require additional evidence from EO/IR (at close range), acoustic (engine class), kinematic consistency, or onboard seeker.

[SRS-CLS-004] Kinematic classification evidence (e.g., speed distribution consistency with threat class profiles) shall be blended into the class belief at readout time only and shall not be accumulated into the stored posterior. This prevents double-counting of persistent motion state.

[SRS-CLS-005] The classification output shall include a confidence measure. Contacts with classification confidence below a configurable threshold shall be flagged as requiring additional sensor attention before assignment.

### 6.2 Decoy Management

[SRS-CLS-006] A track with `p_decoy ≥ 0.85` shall not be assigned an interceptor as the primary engagement target. This threshold shall be configurable.

[SRS-CLS-007] The system shall never assign zero engagement resources to a contact solely on the basis of decoy probability below the threshold in [SRS-CLS-006], because residual probability represents lethal risk. The threat score for contacts with elevated `p_decoy` shall be scaled down proportionally but not zeroed.

[SRS-CLS-008] If an engaged track is subsequently confirmed as a Class D decoy (post-intercept or via late sensor evidence), the interceptor shall receive a ABORT command, and the track shall be removed from the engagement queue. Any ammo expended shall be logged.

### 6.3 Trajectory Adaptation Classification

[SRS-CLS-009] The system shall maintain a **trajectory adaptation estimate** per confirmed track, with two possible values: `FIXED_ROUTE` and `MANEUVERING`. This estimate is distinct from and orthogonal to the threat class belief: a FIXED_ROUTE estimate means the track does not respond to UAV positioning; a MANEUVERING estimate means it does. This attribute is the primary gate for engagement strategy selection (SRS-C2-012).

[SRS-CLS-010] The trajectory adaptation estimate shall be initialised from the current class belief prior:
- Tracks with dominant Class A or A+ belief → initialise as `FIXED_ROUTE`.
- Tracks with dominant Class B or C belief → initialise as `MANEUVERING`.
- Tracks with ambiguous class belief → initialise as `MANEUVERING` (conservative default: assume the system can respond, avoid committing to a fixed-route strategy prematurely).

[SRS-CLS-011] The trajectory adaptation estimate shall be updated continuously from observed track behaviour:
- **Evidence for FIXED_ROUTE:** Predicted trajectory at T−N seconds matches observed position at T within **50 m**. The track does not deviate when interceptors approach within **500 m** proximity.
- **Evidence for MANEUVERING:** Observed track position diverges from predicted by more than **50 m** after an interceptor approach within **500 m** proximity. Or: track velocity changes abruptly in a way inconsistent with pre-programmed weave profiles (Class A/D weave is sinusoidal at known frequency; a sudden aperiodic heading change is evidence of active evasion).

The adaptation estimate shall be a Bayesian probability `p_maneuvering ∈ [0,1]`. A threshold (default 0.4) separates FIXED_ROUTE (below) from MANEUVERING (above). Thresholds shall be configurable.

[SRS-CLS-012] The trajectory adaptation estimate shall be included in every `Track` message published by the fusion node. The C2 shall use it as a direct input to engagement strategy selection (SRS-C2-012). It shall also be displayed on the operator tactical console per track.

---

## 7. Functional Requirements — Command and Control (C2)

### 7.1 TEWA Loop

[SRS-C2-001] The Tier 2 Sector Base Station shall execute a continuous Threat Evaluation and Weapon Assignment (TEWA) loop. The loop shall:
1. Ingest the current confirmed track picture.
2. Evaluate a threat score for each confirmed track.
3. Assign interceptors to tracks based on threat priority and kinematic feasibility.
4. Publish engagement tasks to interceptors.

The TEWA planning loop shall run at a minimum rate of **2 Hz**. This rate is derived from the Class B (FPV) worst-case engagement geometry: at 30–40 m/s closing speed and an engagement envelope of 40–200 m, a 500 ms re-planning cycle is required to maintain intercept geometry validity. OI-002 closed.

[SRS-C2-002] Fire requests from shooter UAVs shall be answered by the C2 immediately (out-of-band, not at the planning rate), because the engagement envelope against a 55 m/s threat can last only a few seconds. The maximum latency from fire request receipt to clearance issuance shall be **≤ 500 ms**. This is derived from the Class B engagement window: at 30–40 m/s closing speed and a 40 m net-effector envelope, the engagement window lasts approximately 1–1.5 s; 500 ms latency leaves ≥ 50% of the window for effector release. OI-002 closed.

[SRS-C2-003] The Tier 2 C2 shall maintain the following internal state per planning cycle:
- Current track picture (from fusion node).
- Threat assessment per track.
- Interceptor state (position, velocity, mode, battery, ammo, current task).
- Denied track set (tracks for which engagement has been DENIED by ROE; excluded from future allocation).
- Confirmed kill set (tracks confirmed destroyed; excluded from future allocation).
- Incumbent shooter map (current shooter ID per track; used for hysteresis).

### 7.2 Threat Evaluation

[SRS-C2-004] The threat score for each track shall be a scalar value ∈ [0,1] computed as a function of at minimum:
- **Lethality**: `1 − p_decoy`. Decoys are deprioritised but not zeroed.
- **Urgency**: a monotonically increasing function of decreasing time-to-impact.
- **Asset value**: the value of the asset most likely to be struck, based on the track's predicted impact point.
- **Ground zone factor**: a multiplier reflecting the risk classification of the predicted impact zone.

[SRS-C2-005] Predicted impact point shall be computed in the horizontal plane using current track position and velocity extrapolated to the asset asset capture radius. Altitude shall not be used to mask horizontal threat heading — a cruising OWA at 1,500 m AGL is still heading toward its ground target.

[SRS-C2-006] Tracks shall be ranked in descending threat score order for weapon assignment. In case of equal scores, track age (older tracks first) shall be the tiebreaker.

### 7.3 Engagement Authority Modes

[SRS-C2-007] The system shall support two engagement authority modes:

- **Human-In-The-Loop (HITL):** Every individual fire clearance requires explicit, real-time operator approval. The C2 software presents the fire request to the operator console and awaits a positive confirmation before issuing clearance. No automated clearance is issued in HITL mode.

- **Human-On-The-Loop (HOTL):** The C2 software may issue fire clearances autonomously within pre-approved ROE parameters. The operator monitors all engagements and may veto any pending clearance at any time. All autonomous clearances shall be logged and flagged for post-mission review.

[SRS-C2-008] The current engagement authority mode shall be clearly displayed on the operator console at all times. Mode changes shall require a deliberate operator action (no accidental mode changes).

[SRS-C2-009] The following hybrid rule shall govern mode transitions:

**Automatic HOTL activation (per track):** HOTL shall activate automatically for an individual engagement when:
- The track's time-to-impact (TTI) falls below 15 seconds AND the track's threat score exceeds the last-resort threshold (see [SRS-ROE-009]).

**Automatic HOTL activation (system-wide):** HOTL shall activate system-wide when:
- The number of confirmed hostile tracks simultaneously requiring engagement exceeds a configurable threshold.
- The operator may pre-authorize HOTL for a configurable time window during high-density raid operations. **The maximum pre-authorization window is 30 minutes.** After expiry, the system reverts to HITL; the operator must re-authorise explicitly. OI-006 closed.

**Return to HITL:** The operator may at any time return to HITL mode system-wide. Per-track automatic activations expire when TTI resets (target destroyed or track dropped).

[SRS-C2-010] In HITL mode, if the operator does not respond to a fire request within the remaining TTI minus the minimum intercept setup time, the C2 shall alert the operator that the engagement window is closing. If the operator still does not respond and TTI falls below the last-resort threshold, the system shall automatically escalate the request but shall not autonomously fire unless the hybrid rule in [SRS-C2-009] is met.

[SRS-C2-011] **Class A+ engagement strategy arbitration (SHALL):** For each confirmed Class A+ track, the TEWA loop shall evaluate and continuously update the preferred engagement strategy at every planning cycle. The strategy selection logic shall be:

1. Assess relay interception feasibility (SRS-COOP-007).
2. If relay is feasible AND the expected relay-chain Pk exceeds a configurable threshold: assign relay interceptors (SRS-COOP-008).
3. If relay is not feasible OR Pk is below threshold: activate route-ambush gun coordination (SRS-COOP-014) — Class A/A+ are FIXED-ROUTE and cannot be herded.
4. If neither strategy is currently executable (no relay geometry, no available gun zone on the predicted route): hold all interceptors at best available monitoring positions; escalate to operator and Tier 1 C2 with THREAT UNENGAGEABLE alert; continue updating trajectory prediction for opportunity reassessment.

The operator shall be informed of the current strategy in execution for each Class A+ track on the tactical display.

[SRS-C2-012] **Trajectory-adaptation-aware strategy routing (SHALL):** The C2 shall route every engagement to one of two strategy families based on the track's `p_maneuvering` value (SRS-CLS-011):

| p_maneuvering | Strategy family | Eligible secondary strategies |
|---|---|---|
| < 0.4 (`FIXED_ROUTE`) | Relay interception primary | Route-ambush gun coordination secondary (SRS-COOP-014). **Herding is PROHIBITED** — a fixed-route threat ignores UAV corridor threats entirely; assigning UAVs to herding posts is a waste of fleet resources with zero tactical effect. |
| ≥ 0.4 (`MANEUVERING`) | Direct pursuit or relay interception primary | Herding to kill zone secondary (SRS-COOP-010/011/012/013). Route-ambush coordination may also be used if a gun crew happens to be on the predicted approach vector. |

When `p_maneuvering` is in the transition zone (0.35–0.45), the system shall default to the FIXED_ROUTE strategy family (conservative: avoids wasting UAVs on herding posts for a threat that may not respond) while continuing to update the estimate.

Class C (Loitering Munition) transitions from fixed-route behaviour in cruise to maneuvering in the terminal phase. The C2 shall monitor Class C tracks for phase transition evidence and update the strategy accordingly at each TEWA cycle.

### 7.4 Coverage-Aware Threat Processing

[SRS-C2-013] **Coverage gap urgency adjustment (SHALL):** When a new detection arrives from a Sentinel UAV operating in a zone that was previously RED (uncovered), the C2 shall compute an adjusted urgency score for the resulting track. The effective advance warning time is shorter than nominal because the threat was undetected during the RED dwell period. The threat score urgency component shall be recomputed as:

```
effective_warning_time = TTI − RED_dwell_at_detection
adjusted_urgency = 1 / (1 + effective_warning_time / 60.0)
```

If the RED dwell period is unknown, the system shall assume maximum plausible dwell time equal to the zone's configured maximum RED dwell threshold (SRS-COV-006). This ensures conservative prioritisation of threats emerging from coverage gaps.

[SRS-C2-014] **Coverage map consumption and alerts (SHALL):** The C2 shall subscribe to the 3D coverage map topic (`coverage/map`) and shall:
- Display current coverage utility metric on the operator console at all times.
- Identify and log all coverage gap alerts (RED cells in critical surveillance zones exceeding dwell threshold).
- Recommend Sentinel UAV repositioning to the operator when a persistent coverage gap exists that could be resolved by redeploying an available sentinel. The recommendation shall include: gap location, duration, and which sentinel is best positioned to cover it.

---

## 8. Functional Requirements — Rules of Engagement (ROE)

### 8.1 Fire Authorization Framework

[SRS-ROE-001] The system shall enforce a probabilistic, ground-risk-aware fire authorization process for every effector release. No munition shall be released without a valid authorization token issued by the C2's ROE module. This constraint is absolute and applies in all operating modes including HOTL. See also [SRS-SAF-001].

[SRS-ROE-002] The ROE module shall evaluate the expected collateral impact of each proposed engagement by running a Monte-Carlo debris footprint model against the ground risk map at the proposed intercept point. The debris model shall account for effector type (net vs. projectile exhibit significantly different debris dispersion characteristics).

[SRS-ROE-003] The ground risk map shall classify every cell of the defended area as SAFE, DANGEROUS, or CRITICAL. Default classification for any unclassified cell shall be DANGEROUS, on the assumption that urban ground is populated. Critical infrastructure cells (hospitals, schools, shelters, dense housing) shall be designated CRITICAL.

[SRS-ROE-004] Zone classification weights for collateral cost computation shall be: SAFE ≈ 0.02, DANGEROUS = 1.0, CRITICAL ≥ 20.0. These values are adopted as requirements (OI-007 closed). They shall be validated against the population casualty model in the PSSA before operational deployment — this is a mandatory pre-deployment gate per SRS-ROE-006.

### 8.2 Authorization Decision Logic

[SRS-ROE-005] The ROE module shall evaluate a fire request and issue one of the following decisions:

| Decision | Meaning |
|---|---|
| `AUTHORIZED (geometry_safe)` | Expected collateral cost ≤ base threshold AND probability of any debris on a CRITICAL cell ≤ base hard cap |
| `AUTHORIZED (now_or_never)` | Above base threshold, but the target is flying toward worse ground and the current intercept point minimises debris cost over the predicted trajectory. Holding can only increase collateral. |
| `AUTHORIZED (last_resort)` | TTI ≤ last-resort time threshold AND threat score ≥ last-resort threshold AND collateral cost ≤ relaxed cap AND p(CRITICAL hit) ≤ relaxed cap |
| `HOLD` | Collateral unsafe now, but geometry may improve — herding or repositioning may move the intercept to safer ground |
| `DENIED` | Decoy-grade threat AND unsafe geometry — engagement would waste ammo and risk collateral for a low-probability armed target |

[SRS-ROE-006] The following ROE threshold parameters shall be configurable at scenario load time and shall not be hard-coded in software:

| Parameter | Description | Default (Baseline) |
|---|---|---|
| `max_expected_collateral` | Zone-weighted cost cap for normal authorization | 0.30 |
| `max_p_critical` | Hard cap on P(CRITICAL hit) in normal mode | 0.01 (1%) |
| `last_resort_time` | TTI threshold for last-resort authorization (seconds) | 25.0 s |
| `last_resort_threat` | Minimum threat score for last-resort | 0.35 |
| `last_resort_collateral` | Relaxed cost cap for last-resort | 2.0 |
| `last_resort_p_critical` | Relaxed P(CRITICAL hit) cap for last-resort | 0.05 (5%) |
| `lookahead_times` | Horizon points for now-or-never evaluation (seconds) | [5, 10, 20] |

**OI-007 closed (v0.5):** These threshold values are **adopted as requirements** at this SRS revision following stakeholder confirmation. They originated from the simulation engineering baseline and shall be treated as binding requirements for system design and V&V. However, **a mandatory IHL legal review and population casualty model validation shall be completed and documented in the Preliminary System Safety Assessment (PSSA) before any operational deployment**. Until PSSA approval is granted by both the legal authority and the safety authority, these thresholds shall not be applied in a live operational context. The PSSA shall specifically address the `last_resort_p_critical = 0.05` parameter, which permits up to 5% probability of a CRITICAL ground zone hit in last-resort engagements, and shall confirm or modify this value based on IHL proportionality analysis.

[SRS-ROE-007] The now-or-never evaluation shall compute the expected collateral cost at the target's predicted future positions (by extrapolating position with current velocity at each lookahead time). The now-or-never authorization shall only be issued if the target is still airborne (z > 0) at the lookahead positions.

[SRS-ROE-008] For the DENIED decision: a contact shall be denied engagement when its decoy probability exceeds the [SRS-CLS-006] threshold AND the engagement geometry is unsafe. A contact that is merely a probable decoy but presents safe geometry shall receive a `HOLD` rather than `DENIED`, to allow for the residual armed probability.

[SRS-ROE-009] All ROE decisions shall be logged with: timestamp, track ID, decision, reason code, computed collateral cost, computed P(CRITICAL hit), TTI at decision time, effector type, and proposed intercept point. This log is a safety record.

### 8.3 Collateral Risk Model

[SRS-ROE-010] The debris model shall generate a sampled set of probabilistic ground impact points for each proposed engagement. The model shall account for:
- Target airspeed and heading at intercept.
- Effector type: net capture results in substantially lower horizontal velocity retention than a projectile kill.
- Target altitude at intercept (higher altitude → larger lateral dispersion).
- Target mass (Class A/A+ at ~200 kg produces a larger footprint than Class B at ~3 kg).

[SRS-ROE-011] The debris model shall be validated against known ballistic data or accepted engineering references before operational deployment. Use of the current simulation-calibrated parameters in an operational system is NOT authorised without this validation.

---

## 9. Functional Requirements — Interceptor UAV

### 9.1 Flight Modes

[SRS-UAV-001] Each interceptor UAV shall implement the following operating modes as a finite state machine (FSM):

| Mode | Description |
|---|---|
| `IDLE` | Holding at launch pad; awaiting tasking from C2 |
| `TRANSIT` | Flying to assigned patrol area or intercept corridor |
| `PURSUIT` | Shooter: pursuing an assigned track toward the effector engagement envelope |
| `ENGAGE` | Shooter: within engagement envelope; fire request pending or clearance received |
| `BLOCKING` | Support: holding a cutoff post on the target's predicted corridor |
| `HERDING` | Support: holding a flank post opposite the designated kill box to constrain target lateral movement |
| `RTB` | Return to base: battery low OR ammo expended OR task aborted |

[SRS-UAV-002] A UAV shall enter RTB mode when battery level falls below 15% or ammo count reaches zero (for shooter-role UAVs). The RTB threshold shall be configurable.

[SRS-UAV-003] Mode transitions shall be driven by C2 task assignments and autonomous engagement logic. No mode transition shall result in an effector release without a valid `FireClearance` token.

### 9.2 Guidance

[SRS-UAV-004] The shooter UAV guidance law shall implement lead-pursuit with target state extrapolation: the fire control solution shall use the track state extrapolated from the last update timestamp to the current time, not the stale track fix. Staleness above the threshold in [SRS-TRK-010] shall abort the engagement.

[SRS-UAV-005] The intercept geometry computation shall solve the intercept-triangle analytically (intercept-time quadratic). When no analytic solution exists (the target is faster than the interceptor in all directions), the UAV shall be assigned a support role rather than a shooter role.

[SRS-UAV-006] A fire request shall be submitted to the C2 only when the computed Pk (probability of kill) for the current engagement geometry meets or exceeds a minimum threshold of 0.25. This threshold shall be configurable.

[SRS-UAV-007] If a valid `FireClearance` is received but the geometry has degraded below a minimum Pk of 0.15 before the effector is released, the release shall be aborted. The UAV shall re-enter PURSUIT mode and re-request clearance when the envelope is re-established. The abort threshold shall be configurable.

### 9.3 Autonomous Operation During Comms Degradation

[SRS-UAV-008] Each interceptor UAV shall continue executing its last assigned task autonomously for up to **30 seconds** after loss of C2 uplink. This limit is set to: (a) allow for brief communication dropouts without aborting an active engagement, and (b) ensure the UAV does not operate in an unsupervised armed state for longer than the Class B worst-case engagement window. After this timeout, the UAV shall:
1. Safe all effectors (no new fire requests shall be submitted without C2 connectivity).
2. Execute RTB to its home pad.

[SRS-UAV-009] During comms-degraded autonomous operation, the UAV shall not cross the defended area boundary in an armed state. On approaching the boundary, the UAV shall turn back regardless of task assignment.

[SRS-UAV-010] On regaining C2 connectivity, the UAV shall transmit a full state report and await re-tasking. It shall not autonomously re-engage a previously assigned track without a new `EngagementTask` message.

### 9.4 Navigation

[SRS-UAV-011] Interceptor UAV navigation shall not rely solely on GPS. The UAV shall implement a GPS-independent navigation fallback (inertial navigation, visual odometry, or equivalent) capable of maintaining position accuracy sufficient to execute the RTB manoeuvre in the event of GPS jamming or spoofing.

[SRS-UAV-012] The GPS-independent navigation system shall maintain position error below **50 m** for the duration of the comms-degraded autonomous operation window. This is consistent with SRS-PERF-005 track accuracy at engagement ranges; it is sufficient for RTB navigation to a known charging station. OI-002 closed.

### 9.5 Platform Performance Constraints (VTOL Multirotor)

[SRS-UAV-013] The interceptor UAV platform shall be a Vertical Take-Off and Landing (VTOL) multirotor. The following minimum performance parameters are required:

| Parameter | Minimum Requirement |
|---|---|
| Maximum speed | ≥ 45 m/s. Class A+ (95–110 m/s) is not engaged by direct pursuit; see SRS-COOP-007 to SRS-COOP-016 for the cooperative relay (primary) and route-ambush gun coordination (secondary) strategies. |
| Maximum acceleration | ≥ 15 m/s² |
| Operational endurance | ≥ 25 minutes at cruise speed |
| Payload mass (effector + seeker) | ≥ 2 kg — TBD by hardware platform selection (net gun ≈ 0.8 kg, projectile launcher ≈ 1.2 kg, seeker ≈ 0.4 kg; combined load varies by effector config) |
| Operating altitude | 50 m – 5,000 m AGL |
| Operating temperature | −25°C to +45°C |

Class A+ direct-pursuit engagement is by design not required. Class A+ (FIXED-ROUTE) is addressed through cooperative relay interception (primary) and route-ambush gun coordination (secondary). Herding is inapplicable to Class A+ because it cannot respond to UAV corridor positioning. OI-001 closed; v0.3 correction applied.

### 9.6 Battery Management and Charging Station Orchestration

> The interceptor UAV fleet operates from a set of charging stations distributed across the defended area. UAVs autonomously manage their own charge/deploy cycle in coordination with the C2, with the constraint that a minimum number of UAVs must remain deployed at all times during active operations.

[SRS-UAV-014] **Continuous battery monitoring (SHALL):** Each interceptor UAV shall continuously measure and report its battery state of charge (SoC, expressed as a percentage 0–100%) and estimated remaining flight time in every `UavState` message. The estimation shall account for: current flight mode (hover vs. cruise vs. high-speed pursuit consume different power), current altitude, and ambient temperature (battery performance degrades below −10°C per [SRS-ENV-002]).

[SRS-UAV-015] **Minimum deployment threshold (SHALL):** A configurable minimum number of interceptor UAVs (`min_deployed`) shall be maintained in airborne, operationally ready status during any active threat scenario. "Operationally ready" means: airborne AND in mode IDLE, TRANSIT, PURSUIT, ENGAGE, BLOCKING, or HERDING. UAVs in modes RTB, CHARGING, or MAINTENANCE do NOT count toward the minimum. The default `min_deployed` value and any changes shall be authorised by the Tier 2 operator. The system shall alert the operator if the deployed count falls or is predicted to fall below `min_deployed` within a configurable time horizon.

[SRS-UAV-016] **Autonomous RTB request and C2 arbitration (SHALL):** When a UAV's estimated remaining flight time falls below the configurable RTB-trigger threshold (default: time required to reach nearest charging station plus a safety margin of **5 minutes**), the UAV shall autonomously request permission to RTB. The C2 shall evaluate the request against the following rules:

| Condition | C2 Response |
|---|---|
| UAV is not the assigned shooter in an active engagement AND return would not drop deployed count below `min_deployed` | Approve RTB immediately; assign nearest available charging station |
| UAV is the assigned shooter in an active engagement AND relay/substitute shooter is available | Approve RTB; reassign engagement to substitute before UAV departs |
| UAV is the assigned shooter AND no substitute is available AND engagement is active | Defer RTB approval for up to **60 seconds** (within safe battery margin); concurrently seek substitute |
| Battery SoC falls below emergency threshold (default 10%) | Unconditional RTB regardless of any of the above; C2 cannot override emergency RTB |
| Return would drop deployed count below `min_deployed` AND no UAVs are currently charging | Alert operator; approve RTB anyway if SoC ≤ 15% (cannot hold UAV in air below safe margin) |

[SRS-UAV-017] **Charging station assignment (SHALL):** The C2 shall maintain a real-time map of all charging stations: location (ENU), total charging capacity (number of simultaneous UAVs), current occupancy, and per-UAV estimated charge completion time. When approving a UAV RTB, the C2 shall assign it to the charging station that minimises transit time subject to: available capacity at the station when the UAV is predicted to arrive. The UAV shall fly directly to the assigned station and not divert without a new C2 assignment.

[SRS-UAV-018] **Charging cycle orchestration — staggered recharge (SHALL):** The C2 shall orchestrate the fleet's charge/deploy cycle such that UAVs do not all reach low battery simultaneously, which would cause mass RTB and a catastrophic drop below `min_deployed`. The orchestration shall:
- Track predicted RTB time for every deployed UAV based on current SoC and consumption rate.
- Proactively send UAVs to charge before the RTB trigger is reached, if a replacement is available and doing so prevents a future deployment gap.
- Ensure that no more than `fleet_size − min_deployed` UAVs are simultaneously absent from deployment (charging, in transit to station, or in transit from station).
- Prefer staggered charging: send UAVs one or two at a time, not in a batch, so that charging completions are distributed over time.

[SRS-UAV-019] **Emergency low-battery behaviour (SHALL — inviolable):** When a UAV's SoC falls below the emergency threshold (default 10%, configurable), the UAV shall:
1. Immediately safe all effectors.
2. Issue an emergency RTB declaration to the C2 (cannot be overridden).
3. Navigate directly to the nearest charging station or, if insufficient range to reach any station, execute an emergency landing at the nearest safe ground point within range.

No C2 command, mode, or engagement task shall override emergency RTB once triggered. Emergency landings outside charging stations shall be logged and the UAV shall be marked MAINTENANCE until manually recovered.

[SRS-UAV-020] **Post-charge redeployment (SHALL):** When a UAV completes charging (SoC ≥ configurable deployment-ready threshold, default 90%), the charging station controller shall notify the C2. The C2 shall evaluate the current threat picture and deployed count, and either:
- Issue a DEPLOY command assigning the UAV to a patrol area or pre-positioned relay post; OR
- Issue a STANDBY command keeping the UAV at the station if `deployed_count ≥ min_deployed` and no immediate tasking is needed.

The decision shall be re-evaluated at every TEWA planning cycle. A UAV shall not remain on standby indefinitely during an active raid; the C2 shall deploy it proactively to maintain the relay chain coverage.

[SRS-UAV-021] **Charging station interface (SHALL):** Each charging station shall expose a machine-readable status interface to the C2 reporting: occupancy (UAV IDs present), per-UAV SoC, per-UAV estimated charge completion time, and station availability status (OPERATIONAL / DEGRADED / OFFLINE). The C2 shall poll or subscribe to this interface at the TEWA planning rate. Station status changes shall be reflected in the fleet management display on the operator console.

---

## 10. Functional Requirements — Effectors

### 10.1 Kinetic Effectors

[SRS-EFF-001] The system shall support two kinetic effector types carried by interceptor UAVs:

**Projectile effector:** Ballistic or explosive projectile providing a longer engagement envelope and higher closing-speed tolerance. The debris footprint of a projectile kill results in significant horizontal velocity retention of the target wreckage (~65% of target airspeed).

**Capture net effector:** Net gun providing a debris-friendly engagement option for low-speed, low-altitude threats (primarily Class B FPV). Effective range is substantially shorter than the projectile effector. Net engagements result in much lower horizontal velocity retention of captured debris (~15% of target airspeed), making them the preferred effector in populated areas when geometry permits.

[SRS-EFF-002] The ROE module shall explicitly consider effector type when computing the debris footprint for fire authorization. The net effector shall be preferred over the projectile effector in DANGEROUS and CRITICAL zones when both options are kinematically available.

[SRS-EFF-003] Each effector instance shall carry a configurable number of munitions (rounds or net cartridges). When ammo is exhausted, the UAV shall transition to RTB mode.

[SRS-EFF-004] Each effector shall define an engagement envelope characterised by: maximum range, maximum off-axis angle, maximum closing speed, and Pk as a function of these parameters. The Pk surface shall be derived from validated test data before operational deployment. The current simulation Pk surfaces are engineering estimates and shall not be used operationally without validation.

### 10.2 Non-Kinetic Effectors — EW / RF Jamming

[SRS-EFF-005] The system shall include an Electronic Warfare (EW) / RF jamming module for disruption of datalink-dependent threats (Classes A, A+, C). The EW module shall be capable of jamming:
- GPS/GNSS navigation signals.
- Common UAV RF control frequencies (2.4 GHz, 5.8 GHz, and others TBD).
- Satellite communication uplinks used by OWA threats for navigation updates.

[SRS-EFF-006] The EW module shall NOT target fiber-optic guided threats (Class B FPV, fiber-optic variant). The system shall correctly identify fiber-optic FPVs as jam-immune and shall not allocate EW resources to them.

[SRS-EFF-007] EW engagement authorisation shall follow the same ROE framework as kinetic engagements, including ROE decision logging and operator clearance in HITL mode. EW effects on civilian communications infrastructure shall be assessed and documented before deployment.

[SRS-EFF-008] The EW module shall implement deconfliction to avoid jamming friendly communication links and GPS receivers used by the interceptor UAVs.

### 10.3 Non-Kinetic Effectors — High-Energy Laser (HEL)

> OI-004 closed. Stakeholder decision (2026-06-10): **High-Energy Laser (HEL)** selected as the directed energy effector. HEL may be deployed as a ground-mounted or airborne (large-platform) module. Requirements SRS-EFF-011 through SRS-EFF-015 replace the previous placeholders SRS-EFF-009/010.

[SRS-EFF-011] **HEL effector integration (SHALL):** The system shall include a High-Energy Laser (HEL) effector module. The HEL shall be integrated with the C2 as a non-kinetic engagement option for all threat classes where kinematic and weather conditions permit. The HEL shall submit `FireRequest` and receive `FireClearance` through the same ROE pipeline as kinetic effectors (SRS-ROE-001 through SRS-ROE-011).

[SRS-EFF-012] **HEL performance parameters (SHALL — minimum):**

| Parameter | Requirement |
|---|---|
| Minimum output power | ≥ 10 kW continuous-wave equivalent |
| Effective engagement range (air-to-air or ground-to-air, clear conditions) | ≥ 1 km; ≥ 2 km goal |
| Dwell time for Class B FPV (1–5 kg frame) neutralisation | ≤ 3 s at 1 km, clear sky |
| Dwell time for Class A/A+ (~200 kg airframe) structural damage | ≤ 10 s at 1 km, clear sky |
| Weather-degraded range (light fog, visibility 2 km) | ≥ 500 m effective |

Precise performance curves shall be derived from validated atmospheric attenuation models and empirical test data before operational deployment.

[SRS-EFF-013] **HEL weather sensitivity and operational limits (SHALL):** The HEL effector shall automatically report a `WEATHER_DEGRADED` or `WEATHER_DENIED` status to the C2 based on real-time atmospheric visibility and humidity data. The C2 shall not assign HEL engagement tasks when status is `WEATHER_DENIED`. When status is `WEATHER_DEGRADED`, the C2 shall reduce the HEL's assigned maximum engagement range accordingly and prefer kinetic effectors.

[SRS-EFF-014] **HEL beam safety zone and no-fly cone (SHALL — safety-critical):** The HEL beam path defines a lethal no-fly volume. Before issuing HEL fire clearance, the C2 ROE module shall verify:
1. No friendly UAV (sentinel or interceptor) is located within the beam path volume (defined as: cylinder of radius **≥ 10 m** centred on the beam axis, from emitter to predicted dwell point; exact exclusion radius subject to optical engineering analysis before operational deployment).
2. No civil aviation contact is within the beam path.
3. The beam path does not intersect any declared protected area (hospital, civilian shelter) at ground level.

If any condition is violated, the ROE shall issue `HOLD` (not `DENIED`) and re-evaluate at the next TEWA cycle, because beam geometry changes as the target moves. No HEL clearance shall be issued while the beam-path safety check is unsatisfied. This constraint is inviolable.

[SRS-EFF-015] **HEL engagement debris model (SHALL):** Unlike kinetic effectors, a successful HEL engagement does not produce significant debris horizontal dispersion from the effector itself, but it may cause the target to become uncontrolled mid-flight and crash. The ROE debris model shall model HEL-neutralised targets as transitioning to free-fall with the target's last known velocity (no net capture effect, no explosive fragmentation unless the target warhead detonates). The probability of warhead detonation upon HEL dwell shall be derived from threat class (Class B FPV warhead is typically small; Class A/A+ warhead may detonate on sustained HEL exposure). The warhead detonation probability shall be included in the collateral cost computation for HEL engagements.

### 10.4 Fixed Ground Effectors — Anti-Air Turrets

> Anti-air turrets are fixed, ground-mounted gun systems with automated or remote-operated fire control. They are distinct from manually aimed human gun crews (SRS-IF-006/007): turrets have their own fire control computers, slaved to the C2 track picture, and do not require a human to manually aim and fire. ROE enforcement, UAV deconfliction, and track assignment all apply.

[SRS-TUR-001] **Turret definition and registration (SHALL):** Each anti-air turret shall be registered in the Tier 2 sector C2 configuration with: unique turret ID, ground position (ENU metres), 3D engagement envelope (azimuth min/max, elevation min/max, maximum range), calibre and projectile ballistics (used by debris model), maximum fire rate, and ammo capacity. Turret configuration shall be updateable by the Tier 2 operator without software redeployment.

[SRS-TUR-002] **Turret state interface (SHALL):** Each turret shall report a real-time state message to the C2, equivalent in structure to `UavState` but for a fixed emplacement. The turret state shall include: current azimuth/elevation aim point, ammo count, availability status (OPERATIONAL / DEGRADED / OFFLINE / RELOADING), and current assigned track ID (if any). This state shall be published on a `turret/state` topic and displayed on the operator console.

[SRS-TUR-003] **Track slaving (SHALL):** The C2 assignment module shall be capable of assigning a confirmed track to a turret as the primary or backup effector. When assigned, the turret's fire control shall receive a continuous stream of track state updates and shall independently compute and update its aim point using a predictive lead solution. Track assignment to turrets shall follow the same threat prioritisation as UAV assignment (SRS-C2-004 through SRS-C2-006).

[SRS-TUR-004] **Turret ROE enforcement (SHALL — identical to UAV):** Turret fire shall be subject to the full ROE framework (SRS-ROE-001 through SRS-ROE-011). A `FireRequest` originating from a turret shall be evaluated by the ROE module using the turret's ground position as the effector origin. The debris model (SRS-ROE-010) shall be applied to the turret's projectile characteristics. No turret shall fire without a valid `FireClearance` token. This constraint is inviolable.

[SRS-TUR-005] **Turret–UAV deconfliction (SHALL — inviolable):** Before issuing fire clearance to a turret, the C2 shall verify that no friendly UAV (sentinel or interceptor) is currently within the turret's firing cone for the proposed firing solution. The firing cone shall be defined as the volume swept by the projectile trajectory plus a safety buffer (TBD metres). The C2 shall command any UAV within the cone to exit before issuing clearance, using the same mechanism as SRS-SAF-010. No exception is permitted.

[SRS-TUR-006] **Turret vs. UAV effector selection (SHALL):** The C2 assignment module shall evaluate turrets alongside interceptor UAVs when allocating effectors to tracks. The selection shall prefer:
- **Turrets** for: threats within the turret's fixed engagement arc, low/slow threats where turret fire is debris-safe, and situations where UAV ammo is depleted or UAV pursuit would enter dangerous ground.
- **Interceptor UAVs** for: threats outside the turret arc, threats requiring active pursuit (no fixed-route solution within turret range), and high-altitude targets beyond turret elevation limits.
Combined turret + UAV assignment for a single high-priority track (layered engagement) is permitted and shall be coordinated by the C2 to ensure deconfliction before any clearance is issued.

[SRS-TUR-007] **Turret ammo management (SHALL):** The C2 shall track turret ammo counts in real time. When a turret's ammo count falls below a configurable threshold, the C2 shall alert the operator and shall not assign new tracks to that turret unless no other effector is available. Ammo resupply events shall be reported by the turret to the C2 via the turret state interface.

---

## 11. Functional Requirements — Cooperative Engagement

### 11.1 Cooperative Interception Geometry

[SRS-COOP-001] When an assigned threat cannot be intercepted by a single shooter UAV (the intercept-triangle equation has no solution, meaning the threat is kinematically faster than the shooter in all pursuit geometries), the C2 shall automatically form a cooperative engagement package consisting of:
- One **primary shooter** UAV placed on the best reachable intercept corridor point.
- Up to **two support** UAVs assigned blocking or herding roles.

[SRS-COOP-002] **Blocking (cutoff) role:** A support UAV shall occupy a corridor point that the threat must pass through, at which the UAV CAN achieve an intercept. Corridor points shall be computed using Apollonius-circle geometry: a point P is a valid cutoff post if a UAV at P can reach the target's predicted corridor before the target arrives. Multiple blockers shall occupy successive corridor points to form a relay interception chain.

[SRS-COOP-003] **Herding role:** A support UAV shall occupy a flank post on the opposite side of the target's trajectory from the designated kill box, to constrain lateral escape and drive the target toward the kill box.

[SRS-COOP-004] The kill box for each engagement shall be computed as the nearest safe (SAFE zone) ground cell to the target's current track position, minimising debris cost for the eventual engagement.

[SRS-COOP-005] Budget rule: the C2 shall not assign support UAVs to a lower-priority track if doing so would deny a shooter to a higher-priority queued track. The available support budget for a task is: `max(0, available_UAVs − remaining_queued_tasks)`.

[SRS-COOP-006] Incumbent hysteresis: re-assignment of a shooter from one track to another shall require the candidate replacement to offer a strictly better intercept cost by a margin factor (default 0.7). This prevents rapid shooter-swapping on estimate jitter that would disrupt converged pursuit geometry.

### 11.2 Class A+ Jet OWA — Dedicated Cooperative Engagement Strategy

> Applies exclusively to Class A+ (Jet OWA) threats. OI-001 resolution: direct tail-chase pursuit is geometrically impossible for the VTOL interceptor fleet. The system shall address Class A+ through two strategies applied in priority order: (1) cooperative relay interception and (2), when relay geometry is not achievable, herding to a pre-designated anti-air gun kill zone.

[SRS-COOP-007] **Relay feasibility assessment (SHALL):** On confirmation of a Class A+ track, the C2 shall immediately assess whether relay interception geometry is achievable. Feasibility is defined as: at least one available interceptor UAV can reach a valid corridor cutoff point *before* the Class A+ threat arrives at that point, determined by the Apollonius criterion:

```
UAV_distance_to_cutoff / UAV_speed < A+_distance_to_cutoff / A+_speed
```

The feasibility assessment shall be completed within 20 seconds of track confirmation and shall be re-evaluated at every TEWA planning cycle as the A+ trajectory evolves.

[SRS-COOP-008] **Relay interception execution (PRIMARY strategy — SHALL when feasible):** When relay feasibility is confirmed, the C2 shall assign a chain of relay interceptors along the predicted A+ flight corridor. Each relay interceptor shall be assigned a cutoff post it can reach before the A+ arrives. The relay chain shall be designed such that if the first interceptor misses, the next in the chain is already in position. Relay posts shall be spaced so that the A+ cannot outrun the chain without deviating significantly from its programmed trajectory.

[SRS-COOP-009] **A+ trajectory prediction (SHALL):** Class A+ trajectory prediction shall be computed in 3D, accounting for the A+'s known flight profile (high-altitude cruise with terminal dive). The predicted trajectory shall be extrapolated a minimum of 90 seconds forward to allow relay posts to be pre-positioned. The trajectory prediction shall be updated at every TEWA cycle with the latest track state.

[SRS-COOP-010] **Herding to anti-air gun kill zone (SECONDARY strategy for MANEUVERING threats only — SHALL when relay is infeasible or insufficient AND p_maneuvering ≥ 0.4):**

> **Prerequisite:** This strategy SHALL ONLY be activated for tracks classified as MANEUVERING (SRS-CLS-011). For FIXED-ROUTE threats (p_maneuvering < 0.4), herding has no effect — the threat will ignore UAV corridor positioning and maintain its programmed route regardless. Apply SRS-COOP-014 (route-ambush) for fixed-route threats instead.

When relay interception is assessed as infeasible for a MANEUVERING threat, the C2 shall activate the herding-to-kill-zone strategy:

1. **Kill zone selection:** The C2 shall select the anti-air gun kill zone that minimises the lateral angular deviation required from the threat's current projected trajectory, among all zones currently marked AVAILABLE (gun crew confirmed present and ready). If no kill zone is available, the system shall alert the Tier 1 Metropolitan C2 and the operator immediately.

2. **Herding formation assignment:** Available interceptors shall be assigned positions that create engagement threats on all A+ approach corridors except the one leading to the selected kill zone. Interceptors in herding positions shall be placed at posts the A+ would need to fly through on alternative routes — making those routes kinematically costly or geometrically risky. The kill-zone approach corridor shall be left uncontested.

3. **Gun crew alert:** Simultaneously with herding formation assignment, the C2 shall issue a TRACK ALERT to the gun crew associated with the selected kill zone, including: current A+ position and velocity, predicted time of arrival (PTA) at kill zone entry point, and confidence interval on trajectory prediction.

[SRS-COOP-011] **Anti-air gun kill zone management (SHALL):** Kill zones designated for Class A+ engagement shall be defined as part of the defended area configuration and shall meet all of the following criteria:
- Located in a SAFE ground-risk zone (per the ground risk map classification).
- Positioned such that anti-air gun engagement within the zone does not create debris risk over DANGEROUS or CRITICAL ground.
- Reachable by the MANEUVERING threat with ≤ 30 degrees of heading change from its predicted course (for herding strategies — the herded threat must not need to deviate more than 30° to reach the kill zone, otherwise herding channel posts are impractical to set up). For FIXED-ROUTE threats, use SRS-COOP-014 proximity criterion instead.
- Associated with a specific gun crew identifier and a real-time availability status (AVAILABLE / UNAVAILABLE).

Multiple kill zones may be designated. The C2 shall maintain kill zone status in real time and shall not attempt to herd a Class A+ toward an UNAVAILABLE zone.

[SRS-COOP-012] **Gun crew engagement handoff protocol (SHALL):** The formal handoff of Class A+ engagement responsibility from the UAV C2 system to the human anti-air gun crew shall follow this sequence:

| Step | Actor | Action |
|---|---|---|
| 1 | C2 | Issues TRACK ALERT to gun crew with A+ track data and PTA |
| 2 | Gun crew | Acknowledges alert; responds with READY or NOT-READY |
| 3a | C2 (if READY) | Continues herding formation; begins UAV evacuation from gun engagement cone (see SRS-SAF-010) |
| 3b | C2 (if NOT-READY or no response within **30 s**) | Selects alternate kill zone OR escalates to relay strategy with remaining UAVs OR alerts operator |
| 4 | C2 | Issues CLEARED HOT signal to gun crew once all UAVs are confirmed outside the gun engagement cone |
| 5 | Gun crew | Engages A+ when it enters the kill zone |
| 6 | C2 | Updates track: marks as ENGAGED BY GUN; suspends UAV engagement tasks for this track |

[SRS-COOP-013] **Strategy transition continuity (SHALL):** The system shall be capable of transitioning between relay interception, herding, and route-ambush strategies mid-engagement if the tactical situation changes (e.g., relay feasibility is lost; a gun crew becomes available; `p_maneuvering` crosses the classification threshold mid-flight). Strategy transitions shall be executed within one TEWA planning cycle. During transition, at least one interceptor shall maintain visual/sensor contact with the threat to preserve track continuity.

### 11.3 Fixed-Route Threat — Route-Ambush Gun Coordination

[SRS-COOP-014] **Route-ambush coordination for FIXED-ROUTE threats (SHALL when relay is infeasible AND p_maneuvering < 0.4):**

When relay interception is not achievable and the track is classified as FIXED-ROUTE, herding is prohibited (SRS-C2-012). The secondary strategy is **route-ambush**: coordinate human anti-air gun crews to engage the threat at the safest point on its own predicted trajectory. The C2 shall:

1. **Ambush point selection:** Compute all points on the A+'s predicted trajectory (extrapolated ≥ 90 s ahead per SRS-COOP-009) that fall within a configurable proximity (**≤ 2,000 m**, representing the gun's effective engagement range) of a gun kill zone currently marked AVAILABLE. Select the point that minimises debris risk (nearest to SAFE ground zone, consistent with gun crew having clear field of fire and sufficient engagement lead time).

2. **Lead-time validation:** Verify that the selected ambush point allows the gun crew at least **45 seconds** of preparation time (tracking acquisition + traverse/elevation to target bearing) before the threat arrives. If no point satisfies the lead-time constraint, escalate to operator.

3. **Gun crew alert:** Issue the TRACK ALERT to the selected gun crew with: threat class, current position and velocity, predicted ambush point coordinates, predicted time of arrival (PTA) at ambush point, and lateral position uncertainty at that point. The alert format is identical to the herding handoff (SRS-IF-006) — the gun crew interface is the same; only the strategic context differs.

4. **UAV supporting tasks:** While gun crew coordination proceeds, assign available interceptors to: (a) relay posts if any partial geometry exists, and (b) close-escort positions that maintain track continuity and provide late detection updates to narrow trajectory uncertainty. Interceptors shall NOT attempt shots unless relay geometry is confirmed viable, to avoid debris over uncleared ground.

[SRS-COOP-015] **Trajectory uncertainty management for route-ambush (SHALL):** The accuracy of the route-ambush strategy depends entirely on trajectory prediction fidelity. The C2 shall:
- Report trajectory prediction confidence (expressed as a lateral spread at the ambush point, in metres) to the gun crew in the TRACK ALERT.
- Re-issue updated TRACK ALERTs at every TEWA cycle with refined position and PTA as the threat approaches.
- If trajectory uncertainty at the ambush point exceeds a configurable threshold, alert the gun crew and assess whether an alternate ambush point with lower uncertainty is available.

[SRS-COOP-016] **Strategy transition — maneuverability classification update mid-engagement (SHALL):** If a track's `p_maneuvering` crosses the classification threshold during an active engagement (e.g., a threat initially classified FIXED-ROUTE begins exhibiting evasion behaviour), the C2 shall:
- Immediately re-evaluate the engagement strategy using SRS-C2-012.
- If transitioning from FIXED-ROUTE to MANEUVERING: cancel route-ambush gun coordination (issue STAND DOWN to gun crew); reassign UAVs to herding formation if feasible.
- If transitioning from MANEUVERING to FIXED-ROUTE: cancel herding formation (herding posts are now wasted resources); switch to route-ambush coordination.
- Log the transition event with timestamp, track ID, old and new p_maneuvering, and new strategy assigned.

### 11.4 Sentinel UAV Role — Forward Observer and RF-Silent Threat Detection

> Sentinel UAVs are a distinct role from Interceptor UAVs. They carry sensors but not primary effectors, operate outside the defended zone perimeter, and are the primary means of detecting RF-silent threats that cannot be found by RF-DF sensors. The C2 maintains two separate fleets: SENTINEL and INTERCEPTOR. Battery orchestration (SRS-UAV-014–021) applies to both roles but with different operational thresholds.

[SRS-SENT-001] **Sentinel UAV role definition (SHALL):** A Sentinel UAV operates as a forward airborne observer. Its primary mission is: (a) extend detection coverage to RF-silent threats beyond the range of ground-based radar and acoustic sensors, and (b) maintain continuous 3D area coverage outside the defended zone perimeter to minimise the dwell time of unobserved voxels. Sentinels shall NOT be assigned as primary kinetic shooters. They may carry a secondary lightweight effector for self-defence only, which shall not be engaged by C2 assignment and shall remain inhibited during normal sentinel operations.

[SRS-SENT-002] **Deployment zone (SHALL):** Sentinels shall be deployed outside the defended zone perimeter at a stand-off distance sufficient to provide advance warning for the most time-critical RF-silent threat class (Class B FPV at 30–40 m/s, approaching at low altitude). The minimum stand-off distance shall be derived from the Class B response time requirement (OI-002) as: `d_min = V_FPV × T_response + d_sensor_range`. Until OI-002 is resolved, sentinels shall be deployed at a minimum of 5 km outside the defended perimeter as a conservative estimate.

[SRS-SENT-003] **Sentinel sensor suite (SHALL):** Each sentinel UAV shall carry at minimum:
- **Wide-field EO/IR imager**: for area surveillance and initial detection of thermal signatures. Field-of-view and sensitivity shall be sufficient to detect a Class B FPV (heat signature, 1–5 kg) at range **≥ 4 km** under the environmental conditions of SRS-ENV-003/005/006. This is the airborne EO/IR detection range from the validated simulation baseline; OI-003 closed.
- **Acoustic sensor**: for detection of low-altitude, low-RCS threats (Class B FPV) below the radar horizon. Acoustic sensors on an airborne platform have different noise characteristics than ground-based sensors — the sensor design shall account for rotor noise masking and implement appropriate noise cancellation.

Sentinels should also carry:
- **Narrow-field EO/IR imager**: for target classification once a contact is detected by the wide-field channel.
- **Miniaturised radar or LiDAR**: for obstacle awareness (required for 3D LOS computation and autonomous flight near buildings/terrain).

[SRS-SENT-004] **RF-silent threat detection responsibility (SHALL):** The sentinel network is the primary detection layer for RF-silent threats. For any hostile contact that produces no RF signature detectable by the RF-DF network, the system shall rely on: radar (range and altitude permitting), acoustic ground pickets (short range, low altitude), and — critically — Sentinel UAV sensors as the early-warning layer. Sentinel detections shall feed the standard `detections` topic and be processed identically by the fusion node (SRS-TRK-001 through SRS-TRK-011). All `Detection` messages from sentinels shall include a `sensor_platform_id` field identifying the reporting sentinel's position and timestamp, so the fusion node can apply the correct position uncertainty.

[SRS-SENT-005] **Sentinel coverage publishing (SHALL):** At each update cycle (minimum 2 Hz), each sentinel shall compute and publish its current **perceived volume**: the set of voxels in the 3D coverage grid that are within its sensor line-of-sight and within its detection range for each sensor type. Publication shall be on the `sentinel/coverage_footprint` topic and shall include: sentinel ID, timestamp, set of GREEN voxel coordinates, and per-voxel confidence (a function of range and viewing angle). The coverage map node (SRS-COV-003) subscribes to this topic to build the aggregate map.

[SRS-SENT-006] **Patrol autonomy — coverage-driven (SHALL):** Sentinels shall autonomously plan and execute patrol routes that maximise the coverage utility metric (SRS-COV-008) within their assigned patrol sector. The patrol planner shall re-plan whenever:
- The coverage utility of the assigned sector falls below a configurable minimum threshold.
- A RED dwell alert is issued for a voxel within the sentinel's reachable coverage range.
- A contact is detected, requiring the sentinel to transition to target-tracking mode and hold position.
- Battery or weather constraints change the reachable patrol volume.

The patrol planner is centrally coordinated at the Tier 2 C2 level (baseline): the C2 assigns patrol sectors and waypoints to avoid redundant coverage between sentinels. A decentralised patrol coordination mode (sentinels negotiate sectors autonomously) is a future capability (see ROADMAP).

[SRS-SENT-007] **Multi-sentinel coverage coordination (SHALL):** The C2 shall maintain awareness of all sentinel positions and coverage footprints and shall ensure that no two sentinels are assigned to sectors with significant overlap, unless a specific area requires redundant coverage (e.g., a high-threat approach corridor). When a sentinel RTBs for charging, the C2 shall assess whether the resulting coverage gap exceeds the RED dwell threshold and shall either: reassign an adjacent sentinel to expand its sector, or expedite redeployment of the charging sentinel or a replacement.

[SRS-SENT-008] **Contact report and target handoff (SHALL):** When a sentinel detects a new contact:
1. It publishes a `Detection` to the `detections` topic (standard pipeline).
2. It simultaneously issues a `ContactReport` to the C2 flagging the detection as **forward observer** origin. The report includes: estimated threat class, confidence, and the current RED dwell time at the contact's location (indicating how long it may have been undetected).
3. The C2 shall treat forward observer contacts with elevated urgency per SRS-C2-013.
4. The sentinel shall transition to tracking mode: maintaining sensor focus on the contact, publishing updated detections at maximum sensor rate until a confirmed track is established in the fusion node (SRS-TRK-002).
5. Once the track is confirmed and interceptors are assigned, the sentinel shall resume patrol — unless its track quality contribution is irreplaceable (the track would drop to tentative without the sentinel's updates), in which case it shall continue tracking until another sensor or interceptor seeker can maintain the contact.

[SRS-SENT-009] **Sentinel–interceptor deconfliction (SHALL):** When an interceptor is assigned to engage a track that a sentinel is tracking:
- The C2 shall share the sentinel's current position with the interceptor.
- The interceptor's guidance shall maintain a configurable minimum separation from the sentinel.
- The sentinel shall not enter the interceptor's engagement corridor (the cone defined by the intercept geometry) to avoid being mistaken for the target or interfering with the shot.
- These deconfliction constraints shall be enforced at the C2 task assignment level, not left to the individual UAV agents.

[SRS-SENT-010] **Sentinel-specific battery thresholds (SHALL):** Because sentinels are deployed farther from charging stations than interceptors, their RTB trigger threshold (SRS-UAV-016) shall account for the greater return transit distance. The minimum battery reserve at RTB initiation shall guarantee arrival at the assigned charging station with ≥ TBD% SoC remaining as a landing buffer. The `min_deployed` sentinel count (SRS-UAV-015) shall be defined separately from the `min_deployed` interceptor count — both must be maintained independently during operations.

### 11.5 Three-Dimensional Coverage Map

> The coverage map is the shared situational awareness layer that shows the C2 and operators which portions of the airspace are currently observed by the sentinel network. It is 3D, obstacle-aware, and updated in real time.

[SRS-COV-001] **3D voxel grid definition (SHALL):** The coverage map shall be a 3D voxel grid over the union of the defended area and the sentinel patrol zone. Grid parameters:

| Parameter | Requirement |
|---|---|
| Lateral resolution | ≤ 100 m × 100 m per cell (configurable per deployment) |
| Vertical resolution | ≤ 50 m per cell (configurable) |
| Vertical extent | 0 m AGL to 5,500 m AGL (full threat altitude band plus margin) |
| Lateral extent | Defended zone bounds plus sentinel patrol zone plus **5 km** buffer (matching sentinel stand-off distance; ensures all sentinel patrol positions are within the map) |
| Coordinate system | ENU metres, origin at the Tier 1 Metropolitan C2 reference point |

[SRS-COV-002] **3D obstacle model (SHALL):** The coverage map system shall maintain a 3D obstacle model used for LOS raycasting. The obstacle model shall incorporate at minimum:
- Building footprints and heights (from scenario configuration or GIS data).
- Terrain digital elevation model (DEM) — the ground is an obstacle at terrain height.
- Declared no-fly zones (treated as opaque obstacles for coverage computation).

The obstacle model shall be loaded at system initialisation and shall be updateable during operation (e.g., a building collapse or new obstacle declared). Obstacle model updates shall propagate to all coverage map consumers within one map update cycle.

[SRS-COV-003] **Line-of-sight computation (SHALL):** For each sentinel UAV at its current position, the coverage map system shall compute sensor LOS to each voxel by raycasting from the sensor position to the voxel centre against the 3D obstacle model. A voxel is within LOS if the ray does not intersect any obstacle voxel. LOS computation shall account for:
- Sensor detection range (range-dependent: GREEN only within effective detection range for the relevant threat class).
- Sensor field of view (the sentinel's sensor frustum — not all visible voxels are within the sensor FOV).
- Sentinel heading and sensor gimbal orientation (if a gimballed sensor is modelled).

LOS computation shall be updated at every sentinel position update (minimum 2 Hz).

[SRS-COV-004] **GREEN/RED cell classification (SHALL):** A voxel is classified **GREEN** if and only if: at least one active sentinel has LOS to that voxel AND the voxel is within that sentinel's effective sensor detection range. A voxel is classified **RED** if no sentinel currently satisfies these conditions. GREEN/RED status shall be updated within one coverage map publication cycle of a sentinel position change.

[SRS-COV-005] **Coverage staleness and RED dwell tracking (SHALL):** The coverage map system shall track per-voxel:
- `last_green_ts`: timestamp of the last time the voxel was GREEN.
- `red_dwell_s`: elapsed time since `last_green_ts` (continuously updated while the voxel is RED).
- A voxel that has never been GREEN since system start shall have `red_dwell_s = +∞` (never observed).

These fields shall be included in the published coverage map and displayed in the operator interface.

[SRS-COV-006] **Coverage gap alert (SHALL):** The system shall maintain a configurable set of **critical surveillance zones** — 3D sub-volumes of the coverage map designated as high-priority observation areas (e.g., known threat approach corridors, asset protection zones outside the defended perimeter). For each cell within a critical surveillance zone:
- When `red_dwell_s` exceeds a configurable threshold T_red_alert (default: **60 seconds** — derived from OI-002: a threat at 30 m/s that enters an unobserved zone can travel 1,800 m in 60 s; alerting at this threshold preserves ≥ 30 s of response margin assuming sentinel redeployment or re-routing begins immediately. OI-002 closed.), the C2 shall issue a **COVERAGE GAP ALERT** to the operator, including: voxel coordinates, zone name, and RED dwell duration.
- Coverage gap alerts shall be logged and shall appear as highlighted volumes on the operator's 3D situational awareness display.

[SRS-COV-007] **Coverage utility metric (SHALL):** The system shall compute and publish a scalar coverage utility U ∈ [0,1] at every TEWA planning cycle:

```
U = (number of GREEN voxels in all critical surveillance zones) /
    (total voxels in all critical surveillance zones)
```

This metric shall be displayed on the operator console and shall be used by the Sentinel patrol planner as the primary optimisation objective. The C2 shall alert the operator when U falls below a configurable minimum threshold (default: **0.70** — alert when less than 70% of critical zone voxels are GREEN).

[SRS-COV-008] **Coverage map publication (SHALL):** The full 3D coverage map shall be published on the `coverage/map` topic at a configurable rate (default 1 Hz). To minimise bandwidth, the map shall be serialised using a sparse differential encoding: only voxels that changed status since the last publication are included. Consumers shall maintain a local cached map and apply the differential update. A full-map snapshot shall be provided on demand and at system start or reconnection.

[SRS-COV-009] **Obstacle-aware sentinel patrol planning (SHALL):** Sentinel patrol waypoints shall be computed with awareness of the 3D obstacle model. Sentinels shall not be assigned waypoints that require flight through an obstacle. Patrol route planning shall account for terrain masking: a waypoint at low altitude near a hill may provide less coverage than a higher-altitude waypoint, even if geometrically closer to the target zone.

[SRS-COV-010] **Coverage map integration with threat detection (SHALL):** When a threat detection arrives from any sensor, the C2 shall query the coverage map to determine the RED dwell time at the detection location. If `red_dwell_s > 0`, this indicates the threat may have been present in the area longer than the detection timestamp suggests. The C2 shall include `red_dwell_at_detection` in the `ThreatAssessment` for the resulting track, and shall use it in the urgency computation per SRS-C2-013.

---

## 12. Performance Requirements

> OI-002 and OI-003 closed (v0.5). Response time and detection range requirements in this section are **binding requirements** derived from intercept geometry analysis (see Section 18, OI-002/003 resolution). Analysis basis: sentinel at 5 km stand-off with 4 km EO/IR sensor provides 9 km total detection range; response time budget is allocated across detection, confirmation, TEWA, ROE, transit, and engagement sub-phases.

[SRS-PERF-001] **Minimum advance warning time per threat class (SHALL):** The system shall guarantee the following minimum advance warning times from first sensor detection to fire authorisation, at worst-case approach geometry with sentinel network in nominal deployment:

| Threat Class | Speed | Nominal Detection Range | Minimum Advance Warning Time |
|---|---|---|---|
| A — Strategic OWA | 55 m/s | ≥ 6 km (ground radar primary) | **≥ 90 seconds** |
| A+ — Jet OWA | 100 m/s | ≥ 9 km (sentinel EO/IR at 5 km stand-off) | **≥ 60 seconds** |
| B — Tactical FPV | 33 m/s | ≥ 7 km (sentinel at 5 km stand-off; ground acoustic provides secondary confirmation at ≤ 900 m) | **≥ 90 seconds** |
| C — Loitering Munition | 80 m/s | ≥ 7 km (sentinel primary; radar secondary above radar horizon) | **≥ 60 seconds** |

These values are derived from: detection range divided by threat speed, minus the confirmed track-establishment time (≤ 5 s per SRS-TRK-002), TEWA planning cycle (≤ 0.5 s at 2 Hz), ROE evaluation (≤ 0.5 s), and interceptor transit/setup margin. Sentinel deployment at ≥ 5 km stand-off is required to satisfy Class A+ and Class B advance warning requirements. OI-002 closed.

[SRS-PERF-002] **Response time budget allocation (SHALL):** The total response time budget (detection → fire authorization) for each threat class shall be allocated as follows:

| Phase | Time Budget |
|---|---|
| First detection to confirmed track (3 sensor hits, SRS-TRK-002) | ≤ 5 s |
| Confirmed track to TEWA assignment | ≤ 0.5 s (one 2 Hz cycle) |
| ROE fire authorization evaluation | ≤ 0.5 s |
| Fire request to clearance issuance (SRS-C2-002) | ≤ 0.5 s |
| Total C2 overhead (track → clearance) | ≤ 6.5 s |
| Remaining time available for interceptor transit to engagement envelope | Class A: ≥ 83.5 s; A+: ≥ 53.5 s; B: ≥ 83.5 s; C: ≥ 53.5 s |

For Class A+, relay interceptors shall be **pre-positioned** during the advance warning period; transit time to a pre-positioned cutoff post is negligible.

[SRS-PERF-003] The Tier 2 C2 TEWA planning loop latency (end-to-end: track received → engagement tasks published) shall be **≤ 500 ms** at 50 simultaneous tracks (consistent with the 2 Hz loop rate). This shall be measured and verified in the simulation platform before hardware integration.

[SRS-PERF-004] Fire request latency (fire request submitted by UAV → clearance issued by C2) shall be **≤ 500 ms**. This is out-of-band (not at the TEWA planning rate) and must be met irrespective of current TEWA load. OI-002 closed.

[SRS-PERF-005] Track position accuracy (root-mean-square error vs. truth) shall be **≤ 50 m at 5 km range** for Class A/A+/C, and **≤ 30 m at 2 km range** for Class B (FPV). These values are derived from the engagement envelope dimensions: the net effector optimal range is 18 m with a 40 m maximum; track accuracy must be sufficient to cue the interceptor into the engagement envelope. OI-003 closed.

[SRS-PERF-006] The system shall sustain full TEWA operational capability when simultaneously tracking up to 50 confirmed hostile tracks per sector. Performance degradation above this limit shall be logged but the system shall remain functional.

[SRS-PERF-007] At peak metropolitan load (400+ threats per night across all sectors), cross-sector track fusion at Tier 1 shall provide a consistent common operational picture with **≤ 2 s** staleness across sectors. Individual sector C2 nodes shall continue to operate independently during any Tier 1 downtime.

[SRS-PERF-008] Interceptor UAV minimum engagement Pk (at optimal engagement geometry within the effector envelope) shall be ≥ 0.50 for Class A/C and ≥ 0.35 for Class B (FPV), based on validated effector Pk surfaces (see [SRS-EFF-004]).

---

## 13. Interface Requirements

### 13.1 External Interfaces

[SRS-IF-001] The Tier 1 Metropolitan C2 shall expose a standardised external interface compliant with STANAG 4586 for integration with existing NATO and national military air defence command networks. The interface shall support: receipt of track data from external sensors, transmission of engagement status, and receipt of engagement directives from higher command.

[SRS-IF-002] The operator console interface shall present a real-time tactical display showing: all confirmed tracks with class belief and threat score, all interceptor positions and modes, active engagement tasks, kill box locations, ROE decisions (with reason codes), and current engagement authority mode (HITL/HOTL).

[SRS-IF-003] The operator console shall provide explicit controls for: mode transition (HITL ↔ HOTL), per-track engagement approval (HITL mode), pre-authorization of HOTL window (maximum 30 minutes; OI-006 closed), manual track overriding, and emergency HOLD (suspend all engagements immediately).

[SRS-IF-004] All datalinks between the C2 and interceptor UAVs shall use authenticated and encrypted communications. Authentication shall use a public-key infrastructure or equivalent. Encryption shall use AES-256 or equivalent. ⚠ See Section 15 (Cybersecurity) for full requirements.

[SRS-IF-005] The system shall provide a replay interface for mission debrief. The replay shall present the same visual representation as the live console and shall be driven by the mission recorder log.

[SRS-IF-006] **Anti-air gun crew alert interface (SHALL):** The system shall maintain a real-time, bidirectional communication interface with designated human anti-air gun crew positions. This interface shall support:

- **C2 → Gun crew:** TRACK ALERT message containing: threat class (A+), current 3D position, velocity vector, predicted time of arrival at kill zone entry point, prediction confidence interval, and selected kill zone identifier.
- **C2 → Gun crew:** CLEARED HOT signal, issued after UAV evacuation from the gun engagement cone is confirmed (SRS-SAF-010).
- **C2 → Gun crew:** STAND DOWN signal, issued when the threat is destroyed by other means or the herding strategy is aborted.
- **Gun crew → C2:** READY / NOT-READY acknowledgement with optional reason code.
- **Gun crew → C2:** ENGAGED confirmation, issued when the gun crew fires at the threat.
- **Gun crew → C2:** KILL / MISS report after engagement.

All messages on this interface shall carry timestamps and shall be logged as operational records. Latency of READY/NOT-READY acknowledgement shall be monitored; failure to acknowledge within a configurable timeout shall be treated as NOT-READY.

[SRS-IF-007] **Kill zone designation interface (SHALL):** Authorised operators at Tier 1 and Tier 2 shall be able to designate, modify, and remove anti-air gun kill zones on the tactical map. Each kill zone record shall specify:
- Geographic centre and radius (ENU metres).
- Associated gun crew identifier.
- Coverage altitude band (min/max AGL).
- Current availability status (AVAILABLE / UNAVAILABLE / DEGRADED).
- Maximum debris safe zone for gun engagement (used by ROE to confirm zone-safe shots by gun crew).

Kill zone definitions shall be stored in the Tier 1 Metropolitan C2 and propagated to all Tier 2 sector nodes. Changes to kill zone status shall propagate within one Tier 1 update cycle.

[SRS-IF-008] **3D Situational Awareness Display (SHALL):** The operator console shall provide a real-time interactive 3D map displaying all of the following layers simultaneously, each independently togglable:

| Layer | Content |
|---|---|
| Coverage map | Semi-transparent GREEN/RED voxel volume overlay; RED dwell intensity (brighter = longer unobserved); critical surveillance zone outlines |
| Obstacle model | Building footprints and heights; terrain surface |
| Sentinel UAVs | Position, heading, battery %, current sensor coverage frustum (as a transparent cone), mode (PATROL / TRACKING / RTB / CHARGING), assigned sector outline |
| Interceptor UAVs | Position, heading, battery %, ammo count, current mode (IDLE / PURSUIT / ENGAGE / BLOCKING / HERDING / RTB), assigned track ID |
| Threat tracks | Track position (trail), velocity vector, class belief (colour coded by dominant class), `p_maneuvering` indicator, threat score, TTI countdown, current strategy assigned (relay / route-ambush / herding) |
| Anti-air turrets | Ground position, engagement arc (3D cone frustum), current aim point, assigned track ID, ammo count, availability status |
| Kill zones / Ambush points | Marked on map with gun crew status (AVAILABLE / NOT READY) and current coordination phase |
| Coverage utility | Scalar meter and per-zone RED dwell heatmap overlay |

The display shall update at minimum 5 Hz for dynamic elements (UAV positions, tracks) and 1 Hz for coverage map. The display shall support zoom, pan, rotation, and altitude slice (show a horizontal cross-section of the 3D map at a configurable AGL).

[SRS-IF-009] **Anti-air turret fire control interface (SHALL):** Each anti-air turret shall have a dedicated fire control interface channel between the turret's fire control computer and the C2. This interface shall support:
- **C2 → Turret:** Track assignment (target track ID and latest track state).
- **C2 → Turret:** Fire clearance (`FireClearance` token — same structure as UAV clearance, SRS-MSG-004).
- **C2 → Turret:** HOLD command (suspend firing immediately).
- **C2 → Turret:** STAND DOWN (disengage from current track).
- **Turret → C2:** Fire request (`FireRequest` — same structure as UAV fire request, SRS-MSG-003).
- **Turret → C2:** Engagement result (`EngagementResult` — HIT/MISS/ABORT).
- **Turret → C2:** State update (SRS-TUR-002 fields, minimum 1 Hz).

The turret interface shall use the same authenticated, encrypted datalink requirements as the UAV link (SRS-SEC-001/002). A compromised or spoofed turret fire command is a critical safety risk.

### 13.2 Internal Message Bus Interface

[SRS-MSG-001] All message schemas shall be version-controlled and backward-compatible within a minor version series. Breaking schema changes shall require a major version increment and a migration plan.

[SRS-MSG-002] Message timestamps shall use a monotonic clock. Timestamps shall be included on all messages and shall be used to compute end-to-end latencies for performance monitoring and V&V.

[SRS-MSG-003] The `FireRequest` message shall include: task ID, UAV ID, track ID, effector type, predicted intercept point (3D, ENU metres), computed Pk at request time, and request timestamp.

[SRS-MSG-004] The `FireClearance` message shall include: task ID, UAV ID, decision enum (`AUTHORIZED`, `HOLD`, `DENIED`), authorization sub-type (`geometry_safe`, `now_or_never`, `last_resort` where applicable), expected collateral cost, clearance timestamp, and reason string. This message is a safety-critical record.

[SRS-MSG-005] The `EngagementResult` message shall include: task ID, UAV ID, track ID, outcome (`HIT`, `MISS`, `ABORT`), effector type, actual intercept point (for HIT/MISS), and timestamp. This message is used to update the kill set and drive post-engagement track management.

---

## 14. Safety Requirements

### 14.1 Fundamental Safety Constraints

[SRS-SAF-001] **No release without authorization token (SHALL — inviolable):** No interceptor effector shall be released under any circumstances without a valid `FireClearance` token issued by the C2 ROE module for that specific fire request. This constraint shall be enforced in hardware interlocks where technically feasible, and in software at both the UAV and C2 layers. It is not bypassable by any mode, configuration parameter, or software state. ⚠ Note: the existing draft enforces this in software only. Hardware enforcement is a requirement for the operational system.

[SRS-SAF-002] **Collision avoidance — friendlies (SHALL):** Each interceptor UAV shall implement a deconfliction function that prevents entry into a collision corridor with another friendly UAV. Deconfliction shall use the state of all known friendly UAVs (published on `uav/state`) and shall be computed at the UAV guidance layer, independently of C2 task assignments. C2 task assignments shall not override deconfliction.

[SRS-SAF-003] **Collision avoidance — civil aviation (SHALL):** The system shall interface with relevant air traffic management systems or maintain a civil aircraft exclusion zone. No interceptor shall be assigned to a track within the civil traffic deconfliction volume without explicit authorisation from the relevant ATC authority. Details of this interface are TBD and shall be addressed before any live flight operations.

[SRS-SAF-004] **Positive control — boundary (SHALL):** No armed interceptor UAV shall cross the designated defended area boundary without explicit, per-flight-crossing authorisation from the Tier 2 operator. On approaching the boundary in an armed state without such authorisation, the UAV shall abort its current task and return to base.

[SRS-SAF-005] **Comms loss — safe default (SHALL):** In the event of loss of C2 uplink beyond the timeout defined in [SRS-UAV-008], the UAV shall enter a safe state: safe all effectors, proceed to RTB on GPS/INS navigation. The safe state shall be the default behaviour on any unrecognised or corrupted command.

[SRS-SAF-006] **Mode display integrity (SHALL):** The engagement authority mode (HITL/HOTL) displayed on the operator console shall reflect the actual system mode at all times. Any discrepancy between displayed and actual mode shall trigger an audible and visual alarm and shall suspend engagement activity until the discrepancy is resolved.

[SRS-SAF-010] **UAV / anti-air gun deconfliction — gun engagement cone (SHALL — inviolable):** No interceptor UAV shall be present within the gun engagement cone of a human anti-air gun emplacement when that gun is in CLEARED HOT status. Before the CLEARED HOT signal is issued to the gun crew (step 4 of the handoff protocol in SRS-COOP-012), the C2 shall:
1. Command all interceptors to exit the gun engagement cone.
2. Receive confirmed position reports from all interceptors placing them outside the cone.
3. Only then issue the CLEARED HOT signal.

If any interceptor cannot exit the cone in time, the CLEARED HOT signal shall be withheld and the gun crew shall be notified with a HOLD message. This constraint is not bypassable under any mode or scenario.

The gun engagement cone shall be defined as: a 3D volumetric envelope centred on the gun emplacement, extending to the gun's maximum range, covering the full azimuth/elevation firing arc. The cone geometry shall be pre-configured per gun emplacement in the kill zone definition (SRS-IF-007).

[SRS-SAF-011] **UAV self-suppression during herding (SHALL):** Interceptor UAVs assigned to herding positions against a Class A+ track (SRS-COOP-010) shall have their effector release inhibited by default while executing the herding role. Effector release inhibit shall be lifted only if:
- The A+ trajectory has deviated from the herding funnel AND relay intercept geometry has simultaneously become viable for that UAV; OR
- The C2 explicitly issues a SHOOT AUTHORIZATION for that UAV after re-evaluating the engagement geometry.

This prevents uncoordinated UAV shots against a Class A+ from creating debris over uncleared ground while the gun-crew coordination is in progress.

[SRS-SAF-012] **Sentinel self-defence effector inhibit (SHALL):** Any secondary effector carried by a Sentinel UAV for self-defence purposes shall be inhibited by the C2 at all times during normal sentinel operations. The inhibit shall only be lifted by an explicit SELF-DEFENCE AUTHORIZATION issued by the Tier 2 operator — never autonomously. This ensures sentinels, which operate outside the defended perimeter and potentially within civilian airspace, do not inadvertently engage targets without operator authorisation. Sentinel `FireRequest` messages shall be treated by the ROE module as requiring HITL authorisation regardless of the system's current HITL/HOTL mode.

### 14.2 Safety Assurance

[SRS-SAF-007] The software implementing requirements [SRS-SAF-001] through [SRS-SAF-012] shall be developed and verified to DO-178C DAL B. The safety requirements list shall be included in the Functional Hazard Assessment (FHA) and the Preliminary System Safety Assessment (PSSA) as Hazard Mitigations.

[SRS-SAF-008] The following items from the v0.1 draft are NOT safety requirements and shall be classified as design-level parameters pending safety review:
- The ROE threshold values in [SRS-ROE-006] — adopted as requirements in v0.5 (OI-007 closed); PSSA IHL review is a mandatory pre-deployment gate, not a SRS-level open item.
- The DECOY_IGNORE_THRESHOLD value (0.85) — this is a resource allocation parameter, not a safety constraint.
- The 2 Hz TEWA loop rate — this is a minimum performance parameter, not a safety limit.

[SRS-SAF-009] The absence of a hard requirement for zero critical-zone debris impacts is a deliberate stakeholder decision (see Section 8.2 last-resort logic, [SRS-ROE-005]). The ROE allows, as a last resort, engagements where P(CRITICAL hit) ≤ 5% when the alternative is an unimpeded warhead strike. This trade-off involves IHL implications and shall be reviewed and approved by a legal authority before operational use. This is a mandatory pre-deployment gate documented in SRS-ROE-006 (OI-007 closed).

---

## 15. Cybersecurity Requirements

### 15.1 Communication Security

[SRS-SEC-001] All RF datalinks between C2 infrastructure and interceptor UAVs shall use AES-256 or equivalent symmetric encryption. Encryption keys shall be rotated at intervals not exceeding **24 hours** (subject to confirmation by cryptographic security review before operational deployment).

[SRS-SEC-002] All commands sent over the UAV datalink shall carry an HMAC authentication token computed with a shared secret. The UAV shall reject any command without a valid HMAC. Replay attacks shall be prevented by including a monotonic sequence number in each command message.

[SRS-SEC-003] The C2 software shall authenticate each sensor data source before incorporating its data into the fusion pipeline. Unauthenticated or anomalous sensor inputs shall be quarantined and flagged, not silently dropped.

[SRS-SEC-004] All network interfaces on C2 systems shall be configured to accept connections only from known, authenticated nodes. Default-deny network policies shall be implemented. Remote administration shall require multi-factor authentication.

### 15.2 GPS Anti-Spoofing

[SRS-SEC-005] The interceptor UAV navigation system shall implement GPS anti-spoofing measures. At minimum, the system shall detect anomalous GPS signal behaviour (sudden large position or velocity jumps, signal-strength anomalies inconsistent with satellite geometry) and alert the C2 and operator.

[SRS-SEC-006] In the event of suspected GPS spoofing, the UAV shall transition to GPS-independent navigation (per [SRS-UAV-011]) and shall reject the spoofed GPS input. The spoofing event shall be logged and reported to Tier 1 C2.

### 15.3 Software Integrity

[SRS-SEC-007] All flight-critical and safety-critical software components shall be cryptographically signed. The UAV shall verify software signatures at boot and shall refuse to execute unsigned or tampered software.

[SRS-SEC-008] The system shall implement runtime monitoring for anomalous command patterns (e.g., high-rate command injection, out-of-range parameter values). Anomalous patterns shall trigger an alert and may cause the affected UAV to enter a safe hold state.

### 15.4 Denial of Service Resilience

[SRS-SEC-009] The C2 message bus shall implement rate limiting per message topic to prevent a compromised sensor node or UAV from flooding the bus and disrupting the TEWA loop.

[SRS-SEC-010] Tier 2 sector base stations shall continue to operate independently if the Tier 1 metropolitan C2 link is disrupted. This architectural requirement also satisfies the availability resilience objective: no single communication link failure shall disrupt more than one sector's engagement capability.

---

## 16. Environmental Requirements

[SRS-ENV-001] The system shall be operational across the following ambient temperature range: −25°C to +45°C. Equipment shall survive (non-operational) temperatures of −40°C to +55°C.

[SRS-ENV-002] The interceptor UAV battery system shall maintain ≥ 80% rated capacity (flight endurance) at −10°C and ≥ 60% rated capacity at −25°C. Battery performance degradation at sub-zero temperatures shall be accounted for in battery-based RTB decision logic.

[SRS-ENV-003] The interceptor UAV shall be operational in wind speeds up to 15 m/s (approximately Beaufort Force 7). The system shall survive (on-ground) wind speeds up to 30 m/s.

[SRS-ENV-004] All ground-based sensors and base station equipment shall operate in rain and snow. Electronic enclosures shall meet a minimum ingress protection rating of IP54.

[SRS-ENV-005] The system shall maintain detection and tracking capability in all weather conditions (rain, snow, dense fog). Radar shall be the primary all-weather sensor of record. EO/IR performance may be degraded in fog and dense precipitation — the system shall remain operational (though at reduced classification confidence) without EO/IR contribution.

[SRS-ENV-006] The system shall operate in primary nocturnal conditions. Thermal imaging shall be the primary EO modality for nighttime operations.

[SRS-ENV-007] The system shall be designed to operate in a contested radio frequency environment. RF interference susceptibility analysis shall be conducted and documented before operational deployment.

---

## 17. Simulation and Verification Platform Requirements

> This section applies to **Partition B — Simulation and Validation Platform** (see Section 2.5).

### 17.1 Simulation Fidelity

[SRS-SIM-001] The simulation platform shall model all threat classes defined in Section 2.3 with sufficient fidelity to validate cooperative engagement tactics, sensor coverage gaps, and ROE decision logic. "Sufficient fidelity" means that the simulated sensor model and engagement model produce statistically consistent results with available operational data.

[SRS-SIM-002] The simulation shall implement a ground-truth quarantine principle: tactical software components (fusion, C2, interceptors) shall have access only to messages on the message bus. They shall not directly access the simulation world state. This ensures that sim-to-real migration does not require changes to tactical software.

[SRS-SIM-003] The simulation world shall execute at a deterministic fixed time step (default 50 ms, 20 Hz). Given a fixed random seed, any two runs of the same scenario shall produce byte-identical results. This property shall be verified by an automated regression test.

[SRS-SIM-004] The simulation shall support YAML-defined scenarios that specify: map geometry, ground risk zones, asset locations, sensor placement, interceptor fleet configuration, ROE parameters, and threat wave definitions. Scenarios shall be portable across simulation backend implementations.

[SRS-SIM-005] The simulation platform shall support Monte-Carlo batch execution over N seeds (configurable, minimum N=10) and shall produce per-run and aggregate metrics including: number of critical-zone debris impacts, number of interceptor resources expended on confirmed decoys, threat attrition by class, and number of asset hits.

### 17.2 Software-Hardware Migration Path

[SRS-SIM-006] The message bus and node interface (Partition B) shall mirror ROS 2 topic names, message field names, and publish/subscribe patterns such that migration to a ROS 2 / Gazebo / PX4 SITL environment requires changes only to the two infrastructure files (`bus.py`, `node.py`) and sensor/physics plugins. All tactical software shall migrate without modification.

[SRS-SIM-007] YAML scenario files shall be reusable across simulation backends (Python-native sim, ROS 2/Gazebo). Scenario parameters shall not embed backend-specific implementation details.

### 17.3 Visualisation

[SRS-SIM-008] The simulation platform shall provide a real-time 3D visualisation dashboard displaying: zone-classified ground map, hostile UAV positions and class, confirmed tracks (with decoy probability indicator), interceptor positions and modes, active engagement tasks, fire events, and event log.

[SRS-SIM-009] The visualisation shall support both live streaming (during simulation execution) and replay from a recorded session file. Both modes shall present identical visual representations.

### 17.4 V&V Test Suite

[SRS-SIM-010] The simulation platform shall include a minimum automated test suite covering:
- Deterministic reproducibility (given identical seed, identical output).
- Threat evaluation correctness (threat score ordering matches expected priority).
- Assignment algorithm: no resource starvation under saturation scenarios.
- ROE logic: all five decision paths exercised and verified correct.
- Cooperative interception: relay geometry confirmed for Class A+ (jet OWA) scenario.
- Guidance: intercept-triangle solution correctness for catchable and uncatchable cases.
- Risk zones: debris cost computation for each zone class.
- End-to-end: multi-threat scenario with known expected outcomes.

[SRS-SIM-011] The test suite shall be executed on every software commit (CI/CD). Any test regression shall block integration.

[SRS-SIM-012] **Charging station and battery cycle modelling (SHALL):** The simulation platform shall model:
- A configurable number of charging stations at defined positions, each with configurable simultaneous charging capacity and charge-time profile per UAV (SoC → time-to-full).
- Per-UAV SoC depletion as a function of flight mode (cruise, high-speed pursuit, hover) and ambient temperature. A linear drain model is acceptable for V0 but shall be replaced by a mode-weighted model before Phase 2 (ROADMAP §1).
- The C2 orchestration logic (SRS-UAV-015 through SRS-UAV-020) shall be fully exercised in simulation, including: RTB arbitration under active engagements, staggered recharge scheduling, and emergency low-battery RTB.

[SRS-SIM-013] **Minimum deployment threshold verification (SHALL):** Every simulation run shall record the deployed UAV count at every TEWA planning tick. Post-run analysis shall verify and report: minimum deployed count reached, number of ticks below `min_deployed`, and whether any engagement was attempted with zero deployed UAVs. These metrics shall be included in the Monte-Carlo batch report.

[SRS-SIM-014] **Maneuverability classification test scenarios (SHALL):** The simulation shall include dedicated scenarios exercising the trajectory adaptation classification system (SRS-CLS-009 to SRS-CLS-012):
- A scenario with all FIXED-ROUTE threats where the system must select route-ambush coordination (never herding).
- A scenario with MANEUVERING threats where herding is applicable.
- A scenario with a Class C loitering munition that transitions from fixed-route cruise to maneuvering terminal phase, verifying that the strategy updates correctly at transition.
- A scenario where a FIXED-ROUTE threat is initially misclassified as MANEUVERING and the system corrects its strategy when behavioural evidence accumulates.

[SRS-SIM-015] **Sentinel UAV and coverage map simulation (SHALL):** The simulation platform shall model:
- Sentinel UAV agents with a distinct software profile from interceptors: patrol autonomy, coverage footprint publication, contact report on detection, tracking mode, and sentinel-specific battery thresholds.
- 3D obstacle model integrated with the simulation world (building footprints and terrain surface from the scenario YAML must generate an obstacle voxel map used by both the coverage LOS engine and the sentinel flight planner).
- LOS raycasting from each sentinel's current position and sensor FOV to compute GREEN/RED voxel classifications at every sentinel update cycle.
- Per-voxel RED dwell time tracking and coverage gap alerts.

[SRS-SIM-016] **RF-silent threat detection scenario (SHALL):** The simulation shall include at least one scenario where all hostile threats emit no RF signature (RF-DF detections suppressed). In this scenario, the system shall demonstrate: (a) ground radar provides detection above the radar horizon; (b) ground acoustic sensors provide detection at low altitude within their range; (c) sentinel UAVs provide the primary early-warning detection for low-altitude threats outside acoustic sensor range. The scenario shall verify that track confirmation times are within the Class B response time requirement once OI-002 is resolved.

[SRS-SIM-017] **Anti-air turret simulation (SHALL):** The simulation shall model anti-air turrets as fixed-position effector nodes that participate in the C2 assignment and ROE pipeline identically to UAV shooters. Turret simulation shall include: engagement envelope check (target within azimuth/elevation arc and range), fire request submission, ROE clearance receipt, engagement adjudication (using the same probabilistic Pk model as UAV effectors), turret-UAV deconfliction verification, and ammo depletion. At least one scenario shall include both interceptor UAVs and turrets competing for the same tracks, verifying that the priority assignment and deconfliction rules produce correct, non-conflicting outcomes.

---

## 18. Open Issues and TBDs

All open issues are resolved at version 0.5. This section documents the resolution record for each issue. No unresolved items remain at SRS level; residual pre-deployment gates (IHL review, validated test data) are captured in the affected requirements.

### OI-001 — Class A+ Jet OWA Interception with VTOL-Only Fleet — **CLOSED v0.2**

**Resolution (stakeholder, 2026-06-10):** Direct pursuit engagement of Class A+ by VTOL interceptors is acknowledged as geometrically infeasible and is not required. The system shall address Class A+ through a two-mode cooperative strategy:

1. **Primary — Cooperative relay interception:** Pre-position relay interceptors along the predicted A+ corridor using Apollonius cutoff geometry. Relay posts are valid if the interceptor can reach the post before the A+ arrives, regardless of whether the interceptor can outrun the A+. Multiple relay stages form a chain; even slow VTOL platforms can contribute if pre-positioned far enough ahead.

2. **Secondary — Route-ambush gun coordination (v0.3 correction — NOT herding):** Class A+ is FIXED-ROUTE and cannot be herded. When relay geometry is not achievable, the C2 finds the safest point on the A+'s predicted route and coordinates human anti-air gun crews to be positioned there for an ambush engagement. UAVs continue to provide track continuity and any partial relay coverage. See SRS-COOP-014 to SRS-COOP-016.

**New requirements generated:**
- SRS-COOP-007 through SRS-COOP-016 (cooperative engagement for A+ and fixed-route threats)
- SRS-C2-011 (strategy arbitration in TEWA loop)
- SRS-IF-006 (anti-air gun crew alert interface)
- SRS-IF-007 (kill zone designation interface)
- SRS-SAF-010 (UAV/gun deconfliction — inviolable)
- SRS-SAF-011 (UAV effector inhibit during herding)

**Residual accepted risk:** In scenarios where relay geometry is infeasible AND no kill zone is available (UNAVAILABLE gun crews), the system cannot guarantee engagement of the A+ threat. This shall be treated as a capability gap requiring operational mitigation (pre-positioning gun crews, maintaining kill zone availability). The C2 shall issue a THREAT UNENGAGEABLE alert in these scenarios.

---

### OI-002 — Class-Specific Response Time Requirements — **CLOSED v0.5**

**Resolution (stakeholder, 2026-06-10):** Geometry-derived analysis conducted based on sentinel deployment at ≥ 5 km stand-off with 4 km EO/IR detection range, yielding 9 km total detection range for Class A+. The analysis applied the time-budget model `T_total = T_detect + T_confirm + T_TEWA + T_ROE + T_transit`, with sensor parameters taken from the validated simulation baseline.

**Adopted values:**

| Threat Class | Speed | Detection Range | Min Advance Warning |
|---|---|---|---|
| A — Strategic OWA | 55 m/s | ≥ 6 km | ≥ 90 s |
| A+ — Jet OWA | 100 m/s | ≥ 9 km (sentinel at 5 km stand-off) | ≥ 60 s |
| B — Tactical FPV | 33 m/s | ≥ 7 km (sentinel primary) | ≥ 90 s |
| C — Loitering Munition | 80 m/s | ≥ 7 km (sentinel primary) | ≥ 60 s |

**Requirements updated:** SRS-PERF-001, SRS-PERF-002, SRS-PERF-003, SRS-PERF-004, SRS-C2-001 (TEWA loop to ≥ 2 Hz), SRS-C2-002 (fire request latency ≤ 500 ms), SRS-UAV-008 (comms timeout = 30 s).

**Key derived constraints:**
- Ground radar minimum elevation angle (1.5°) creates a blind zone for Class B FPV (80 m AGL) beyond 2.5 km. Sentinel UAVs are the primary detection layer for this gap.
- Class A+ radar detection range ≈ 5.8 km (R⁴ model, RCS 0.5 m²); combined with sentinel, total range = 9 km → 60 s warning at 100 m/s after 5.5 s confirmation.
- TEWA loop must be ≥ 2 Hz (500 ms cycles) to maintain intercept geometry validity for Class B.

---

### OI-003 — Detection and Tracking KPIs — **CLOSED v0.5**

**Resolution (stakeholder, 2026-06-10):** Values derived from OI-002 analysis and effector engagement envelope requirements.

**Adopted values:**
- Track position accuracy: ≤ 50 m RMSE at 5 km for Class A/A+/C; ≤ 30 m RMSE at 2 km for Class B (FPV). These bounds are set to cue the interceptor within the effector engagement envelope (net optimal: 18 m; projectile optimal: 80 m).
- Detection probability: ≥ 0.90 for all classes at the minimum required detection range with nominal atmospheric conditions.
- Track confirmation time: ≤ 5 s (3 hits within 5 s, per SRS-TRK-002).

**Requirements updated:** SRS-PERF-005, SRS-DET-004 (EO/IR range context), SRS-DET-009 (detection probability now ≥ 90%).

---

### OI-004 — Directed Energy Effector Type — **CLOSED v0.5**

**Resolution (stakeholder, 2026-06-10):** **High-Energy Laser (HEL)** selected. The HEL is chosen for its narrow-beam precision, compatibility with ground-mounted and large-airframe-mounted deployment, and debris model advantage over kinetic effectors in urban environments. Weather sensitivity is accepted as an operational limitation; the C2 shall flag HEL-degraded status and revert to kinetic effectors automatically.

**Requirements updated:** SRS-EFF-009 and SRS-EFF-010 **replaced** by SRS-EFF-011 (HEL integration), SRS-EFF-012 (performance parameters), SRS-EFF-013 (weather sensitivity), SRS-EFF-014 (beam safety zone — safety-critical), SRS-EFF-015 (debris model for HEL engagements).

**Traceability matrix updated:** SRS-EFF-009/010 references in Section 19 replaced with SRS-EFF-011–015.

---

### OI-005 — Metropolitan Fleet Sizing — **CLOSED v0.5**

**Resolution (stakeholder, 2026-06-10):**
- **Sector count:** 7 sectors per metropolitan area (baseline).
- **Per-sector fleet:** 8 interceptor UAVs + 4 sentinel UAVs.
- **Total metropolitan fleet:** 56 interceptors + 28 sentinels.

This sizing is derived from a metropolitan area of approximately 500 km² divided into 7 sectors of ~70 km² each, with threat density up to 50 simultaneous tracks per sector. The 8 interceptor figure provides: 4 simultaneously deployed shooters, 2 support/relay UAVs, 2 in transit or charging, meeting the `min_deployed` constraint.

**Requirements updated:** SRS-ARCH-002 (7 sectors), SRS-ARCH-003 (8 interceptors + 4 sentinels per sector). Scalability range (4–16 sectors) retained in SRS-ARCH-002 for non-standard deployments.

---

### OI-006 — HOTL Pre-Authorization Window Duration — **CLOSED v0.5**

**Resolution (stakeholder, 2026-06-10):** **Maximum HOTL pre-authorization window: 30 minutes.**

Rationale: 30 minutes balances operator cognitive load relief during high-density raids (400+/night, which span multiple hours) against the legal obligation to maintain meaningful human oversight. The 30-minute cap requires the operator to make a deliberate re-authorization decision on a cadence that aligns with operational situation reassessment cycles. Shorter windows were rejected as operationally impractical; longer windows were rejected pending IHL guidance.

**Requirements updated:** SRS-C2-009 (HOTL pre-authorization window = 30 minutes max). SRS-IF-003 operator console control also updated accordingly.

---

### OI-007 — ROE Threshold Values and IHL Review — **CLOSED v0.5**

**Resolution (stakeholder, 2026-06-10):** ROE threshold values in [SRS-ROE-006] are **adopted as requirements** effective v0.5. Values were confirmed by the stakeholder with full awareness that they originated from simulation engineering estimates.

**Adopted as requirements (no change to values; status change only):**

| Parameter | Value |
|---|---|
| `max_expected_collateral` | 0.30 |
| `max_p_critical` | 0.01 (1%) |
| `last_resort_time` | 25.0 s |
| `last_resort_threat` | 0.35 |
| `last_resort_collateral` | 2.0 |
| `last_resort_p_critical` | 0.05 (5%) |

**Pre-deployment gate (mandatory — not waivable):** A qualified IHL legal authority and a certified safety authority shall jointly review these values against:
1. A population casualty model validated for the deployment theatre.
2. IHL proportionality requirements (additional protocol I, art. 51).
3. The PSSA collateral damage hazard analysis.

The PSSA review shall specifically assess the `last_resort_p_critical = 0.05` allowance, which constitutes an intentional trade of up to 5% CRITICAL zone hit probability against the alternative of allowing an armed threat to strike unimpeded. This trade shall be documented and approved by a competent legal authority before any live operational use.

**Requirements updated:** SRS-ROE-006 (status changed from "engineering estimates" to "requirements" with mandatory PSSA gate). SRS-SAF-008 and SRS-SAF-009 cross-reference updated.

---

## 19. Requirements Traceability Matrix

### 19.1 Threat Class to Detection/Engagement Requirements

| Threat Class | Detection | Tracking | Classification | C2/TEWA | ROE | Interceptor | Effector | Cooperation |
|---|---|---|---|---|---|---|---|---|
| A — Strategic OWA | DET-001, DET-002, DET-003 | TRK-001–011 | CLS-001–012 | C2-001–012 | ROE-001–011 | UAV-001–021 | EFF-001–004 | COOP-001–006; COOP-014–016 if relay fails (FIXED-ROUTE — no herding) |
| A+ — Jet OWA | DET-001, DET-002 | TRK-001–011 | CLS-001–012 | C2-001–012 | ROE-001–011 | UAV-001–021 (no direct pursuit) | EFF-001–004, EFF-005–008, EFF-011–015 (HEL), IF-006, IF-007 | COOP-007–016 (relay primary; route-ambush secondary; herding PROHIBITED); SAF-010, SAF-011 |
| B — FPV | DET-001, DET-005 (acoustic critical) | TRK-001–011 | CLS-001–012 | C2-001–012 | ROE-001–011 | UAV-001–021 | EFF-001–004 (net preferred) | COOP-001–006; COOP-010–013 (herding valid — MANEUVERING); SAF-010, SAF-011 |
| C — Loitering | DET-001–007 | TRK-001–011 | CLS-001–012 | C2-001–012 | ROE-001–011 | UAV-001–021 | EFF-001–004 | COOP-001–006; phase-dependent: COOP-014–016 in cruise (fixed-route), COOP-010–013 in terminal (maneuvering) |
| D — Decoy | DET-001–007 | TRK-001–011 | CLS-001–012 | C2-003–012 | ROE-008 | No engagement | N/A | N/A |

### 19.2 Safety Requirements to Draft Code Review Items

The following items in the v0.1 simulation draft require correction or validation against the SRS requirements before integration:

| SRS Requirement | Draft Status | Corrective Action Required |
|---|---|---|
| SRS-TRK-006 (IMM filter) | NOT IMPLEMENTED — constant-velocity KF only | Implement IMM with CV + coordinated-turn + dive models |
| SRS-SAF-001 (HW interlock) | Software only | Hardware interlock required for operational system |
| SRS-EFF-004 (validated Pk surface) | Simulation estimates only | Validated by empirical test data |
| SRS-ROE-011 (validated debris model) | Simulation estimates only | Validated against ballistic references |
| SRS-UAV-011 (GPS-independent nav) | Not modelled | Implement for real system |
| SRS-SEC-001–010 (cybersecurity) | Not modelled | Implement for real system |
| SRS-ARCH-001 (hierarchical C2) | Single base station only | Extend to 3-tier hierarchy |
| SRS-EFF-005–008 (EW jamming) | Stub / not implemented | Full implementation required |
| SRS-EFF-011–015 (HEL effector) | Not implemented | HEL integration, performance characterisation, beam safety zone enforcement, weather degradation logic, HEL-specific debris model. OI-004 resolved. |
| SRS-C2-007–010 (HITL/HOTL operator console) | Not implemented | Operator console required for operational system |
| SRS-C2-011 (A+ strategy arbitration) | Not implemented — current draft treats A+ same as Class A | Implement relay feasibility check and herding-to-gun-zone branch in TEWA loop |
| SRS-COOP-007–013 (A+ relay and herding) | Partially — cutoff geometry exists (cooperation.py) but no two-mode strategy, no gun zone handoff | Add A+ engagement mode FSM; implement gun kill zone management and handoff protocol |
| SRS-IF-006/007 (gun crew alert interface) | Not implemented | New external interface component required |
| SRS-SAF-010 (gun engagement cone deconfliction) | Not implemented | Hard safety function — highest priority; requires gun cone geometry model and UAV evacuation command |
| SRS-SAF-011 (herding effector inhibit) | Not implemented | Add effector inhibit flag to UAV mode FSM for herding role |
| SRS-CLS-009–012 (trajectory adaptation classification) | Not implemented — no maneuverability attribute in current track model | Add `p_maneuvering` field to Track message; implement behavioural update logic in fusion/classification module |
| SRS-C2-012 (strategy routing by maneuverability) | Not implemented — current draft has no routing by adaptation class | Add strategy gate in TEWA loop; prohibit herding for FIXED_ROUTE tracks |
| SRS-COOP-014–016 (route-ambush coordination) | Not implemented | New C2 module: ambush point selection on predicted trajectory, gun crew alert integration |
| SRS-UAV-014–021 (battery orchestration) | Partially — battery drain and RTB on low battery exist (uav.py); no C2 orchestration, no charging stations, no minimum deployment threshold | Add charging station model, C2 fleet orchestration loop, staggered recharge scheduler; extend `UavState` with estimated remaining flight time |
| SRS-SIM-012–014 (simulation: charging, maneuverability scenarios) | Not implemented | Add charging station nodes to sim; add mode-weighted battery depletion; add trajectory adaptation test scenarios |
| SRS-SENT-001–010 (Sentinel UAV role and patrol autonomy) | Not implemented — no sentinel role in current draft; all UAVs are interceptors | Add `SentinelUav` node class; patrol planner; coverage footprint publisher; contact report; sentinel-specific battery thresholds; role separation from InterceptorUav |
| SRS-COV-001–010 (3D coverage map) | Not implemented — no coverage map concept in current draft | New `CoverageMapNode`: 3D voxel grid, obstacle model, LOS raycasting, GREEN/RED status, RED dwell tracking, sparse diff publication, coverage utility metric, gap alerts |
| SRS-TUR-001–007 (anti-air turrets) | Not implemented — no turret concept in current draft | New `AntiAirTurret` node; integrate with C2 assignment (alongside UAVs); ROE enforcement using same pipeline; turret-UAV deconfliction (extension of SAF-010); turret state interface |
| SRS-IF-008 (3D situational awareness display) | Partially — Three.js 3D display exists in viz/ but has no coverage map, no sentinel layer, no turret layer; 2D-centric | Extend Three.js dashboard: add coverage voxel volume rendering, sentinel FOV frustums, turret engagement arcs, RED dwell heatmap, coverage utility meter |
| SRS-C2-013/014 (coverage-aware urgency, coverage alerts) | Not implemented | Extend BaseStation to consume coverage/map and coverage/alert topics; adjust urgency scoring with RED dwell |
| SRS-SAF-012 (sentinel effector inhibit) | Not applicable to current draft (no sentinels) | Implement inhibit flag in SentinelUav; HITL-only authorisation for sentinel self-defence |
| SRS-SIM-015–017 (sentinel/coverage/turret simulation) | Not implemented | Add sentinel nodes, LOS engine, obstacle voxel map, turret nodes to sim; add RF-silent-only scenario |

---

*End of SRS-COOP-UAV-S-001 v0.5*

*This document is a DRAFT BASELINE. All open issues are resolved. No SRS-level TBDs remain. Pre-deployment release gates (PSSA IHL review, validated Pk surfaces, validated debris model) are captured in the affected requirements and are mandatory before operational use. Formal stakeholder review and approval required to elevate to BASELINE status.*
