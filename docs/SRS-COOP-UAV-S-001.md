# SRS-COOP-UAV-S-001
# System Requirements Specification
## COOP-UAV-S — Cooperative Counter-UAS System

| Field | Value |
|---|---|
| Document ID | SRS-COOP-UAV-S-001 |
| Version | 0.1 — Initial Release for Stakeholder Review |
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

The following open issues are identified at document version 0.1. They must be resolved before this SRS is elevated to baseline status.

| ID | Summary | Impact | Target Resolution |
|---|---|---|---|
| OI-001 | Class A+ Jet OWA (100 m/s) vs. VTOL-only interceptor fleet | Critical — engagement of Class A+ cannot be guaranteed by VTOL pursuit alone | PDR |
| OI-002 | Class-specific response time requirements not yet formally derived | High — performance requirements incomplete | Analysis due before PDR |
| OI-003 | Detection/tracking KPIs derived from response time analysis not yet complete | High — performance requirements incomplete | Depends on OI-002 |
| OI-004 | Directed energy effector type not specified (laser vs. HPM) | Medium — impacts platform, power, safety requirements | TBD |
| OI-005 | Number of sector-level base stations and interceptors per metropolitan area | Medium — scales requirements for comms and C2 | CDR |
| OI-006 | HOTL pre-authorization time window duration | Medium — safety and legal implication | Stakeholder review |
| OI-007 | Collateral damage threshold values (RoeConfig parameters) | High — safety and IHL compliance | Legal/safety review |

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

| Class ID | Name | Representative Platform | Speed | Altitude AGL | Mass | Warhead | Notes |
|---|---|---|---|---|---|---|---|
| A | Strategic OWA | Shahed-136 / Geran-2 | 50–65 m/s | 50 m – 5,000 m (adaptive) | ~200 kg | Yes, 50–90 kg | Saturation swarm; decoy mixing; terminal dive |
| A+ | Jet OWA | Geran-3 / Shahed-238 | 95–110 m/s | 2,000–5,000 m | ~200 kg | Yes | High-speed; reduced intercept window; ⚠ OI-001 |
| B | Tactical FPV | Quadcopter kamikaze | 30–40 m/s | 0–200 m | 1–5 kg | Yes | Fiber-optic guided variants are RF-jam-immune; agile |
| C | Loitering Munition | Lancet-3 | 70–90 m/s | 50–500 m | 10–15 kg | Yes, AI-guided seeker | AI-guided terminal seeker; precision strike |
| D | RF Decoy | Gerbera-type | Matches Class A profile | Matches Class A profile | ~18 kg | No | Shares OWA radar signature; decoy fraction up to 60% of salvos |

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

[SRS-ARCH-002] The number of Tier 2 sector base stations required to cover a metropolitan area shall be TBD (⚠ OI-005), but the system architecture shall scale to a minimum of 4 and maximum of 16 independent Tier 2 nodes without requiring changes to Tier 1 or Tier 3 software.

[SRS-ARCH-003] Each Tier 2 sector shall operate a minimum fleet of 4 kinetic interceptors (projectile-effector) and 2 non-kinetic interceptors (net-effector) per 10 km² of defended airspace. TBC.

### 3.2 Message-Based Interface Architecture

[SRS-ARCH-004] All inter-component communication within each tier shall be implemented via a typed publish/subscribe message bus. Message schemas shall be defined independently of the transport layer to allow migration to ROS 2 or equivalent middleware without modifying tactical software.

[SRS-ARCH-005] The following logical topics and message types shall be defined as part of the system interface contract. All message types shall carry a monotonic timestamp field (`stamp`) used for latency measurement and track data freshness assessment:

| Topic | Message Type | Producer | Consumers |
|---|---|---|---|
| `detections` | `Detection` | All sensors | FusionNode |
| `tracks` | `TrackArray` | FusionNode | C2 (all tiers), UAVs, Recorder |
| `uav/state` | `UavState` | Each interceptor UAV | C2, peer UAVs, Recorder |
| `engagement/tasks` | `EngagementTask` | Sector C2 | UAVs |
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

[SRS-DET-004] The EO/IR subsystem shall provide positive classification evidence at close range (TBD km — see OI-003). Classification quality shall improve monotonically as range decreases. At maximum EO/IR classification range, the system shall provide a meaningful update to decoy probability.

[SRS-DET-005] The acoustic sensor network shall detect Class B (FPV) threats flying below the radar horizon at altitudes ≤ 200 m AGL and at ranges sufficient to provide a track confirmation within the Class B response time requirement (see OI-002). Acoustic sensors shall output bearing estimates and engine-class cues.

[SRS-DET-006] The onboard seeker on each interceptor UAV shall provide high-accuracy position and class estimates within the interceptor's terminal engagement range. Seeker measurements shall be published to the shared `detections` topic and fused with ground sensor data.

[SRS-DET-007] All sensors shall produce a `Detection` message on every positive detection cycle, regardless of sensor type. The message shall include: sensor identity, timestamp, estimated 3D position, full 3×3 covariance, and any modality-specific additional field (e.g., radial velocity for radar Doppler, RF signature hash for RF-DF, acoustic engine class for acoustic).

### 4.2 Detection Coverage

[SRS-DET-008] The combined sensor network shall provide no detection coverage gap in the altitude band 50 m to 5,000 m AGL within the defended area. Coverage gaps at low altitude (< 200 m) shall be compensated by the acoustic sensor picket network.

[SRS-DET-009] Sensor deployment shall be based on a formal coverage analysis that demonstrates detection probability ≥ TBD% (see OI-003) for each threat class at the minimum required detection range.

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

[SRS-TRK-010] Any software component that consumes a track for engagement purposes shall assess track data freshness before acting. A track update older than TBD seconds (see OI-002 — this limit is tied to response time requirements) shall be treated as stale and shall not trigger a fire request.

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

---

## 7. Functional Requirements — Command and Control (C2)

### 7.1 TEWA Loop

[SRS-C2-001] The Tier 2 Sector Base Station shall execute a continuous Threat Evaluation and Weapon Assignment (TEWA) loop. The loop shall:
1. Ingest the current confirmed track picture.
2. Evaluate a threat score for each confirmed track.
3. Assign interceptors to tracks based on threat priority and kinematic feasibility.
4. Publish engagement tasks to interceptors.

The TEWA planning loop shall run at a minimum rate of 1 Hz. ⚠ See OI-002 — for Class B (FPV) threats, 1 Hz may be insufficient and a higher rate may be required pending response time analysis.

[SRS-C2-002] Fire requests from shooter UAVs shall be answered by the C2 immediately (out-of-band, not at the planning rate), because the engagement envelope against a 55 m/s threat can last only a few seconds. The maximum latency from fire request receipt to clearance issuance shall be ≤ TBD ms (see OI-002).

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
- The operator may pre-authorize HOTL for a configurable time window during high-density raid operations (⚠ OI-006 — duration of pre-authorization window TBD).

**Return to HITL:** The operator may at any time return to HITL mode system-wide. Per-track automatic activations expire when TTI resets (target destroyed or track dropped).

[SRS-C2-010] In HITL mode, if the operator does not respond to a fire request within the remaining TTI minus the minimum intercept setup time, the C2 shall alert the operator that the engagement window is closing. If the operator still does not respond and TTI falls below the last-resort threshold, the system shall automatically escalate the request but shall not autonomously fire unless the hybrid rule in [SRS-C2-009] is met.

---

## 8. Functional Requirements — Rules of Engagement (ROE)

### 8.1 Fire Authorization Framework

[SRS-ROE-001] The system shall enforce a probabilistic, ground-risk-aware fire authorization process for every effector release. No munition shall be released without a valid authorization token issued by the C2's ROE module. This constraint is absolute and applies in all operating modes including HOTL. See also [SRS-SAF-001].

[SRS-ROE-002] The ROE module shall evaluate the expected collateral impact of each proposed engagement by running a Monte-Carlo debris footprint model against the ground risk map at the proposed intercept point. The debris model shall account for effector type (net vs. projectile exhibit significantly different debris dispersion characteristics).

[SRS-ROE-003] The ground risk map shall classify every cell of the defended area as SAFE, DANGEROUS, or CRITICAL. Default classification for any unclassified cell shall be DANGEROUS, on the assumption that urban ground is populated. Critical infrastructure cells (hospitals, schools, shelters, dense housing) shall be designated CRITICAL.

[SRS-ROE-004] Zone classification weights for collateral cost computation shall be: SAFE ≈ 0.02, DANGEROUS = 1.0, CRITICAL ≥ 20.0. Exact values shall be reviewed and confirmed by the safety authority before baseline (⚠ OI-007).

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

⚠ OI-007: These values are engineering estimates from the simulation baseline. They have NOT been validated against casualty models or reviewed by a legal/IHL authority. They shall not be used operationally without such review.

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

[SRS-UAV-008] Each interceptor UAV shall continue executing its last assigned task autonomously for up to TBD seconds (⚠ OI-002) after loss of C2 uplink. After this timeout, the UAV shall:
1. Safe all effectors (no new fire requests shall be submitted without C2 connectivity).
2. Execute RTB to its home pad.

[SRS-UAV-009] During comms-degraded autonomous operation, the UAV shall not cross the defended area boundary in an armed state. On approaching the boundary, the UAV shall turn back regardless of task assignment.

[SRS-UAV-010] On regaining C2 connectivity, the UAV shall transmit a full state report and await re-tasking. It shall not autonomously re-engage a previously assigned track without a new `EngagementTask` message.

### 9.4 Navigation

[SRS-UAV-011] Interceptor UAV navigation shall not rely solely on GPS. The UAV shall implement a GPS-independent navigation fallback (inertial navigation, visual odometry, or equivalent) capable of maintaining position accuracy sufficient to execute the RTB manoeuvre in the event of GPS jamming or spoofing.

[SRS-UAV-012] The GPS-independent navigation system shall maintain position error below TBD m (⚠ OI-002) for the duration of the comms-degraded autonomous operation window.

### 9.5 Platform Performance Constraints (VTOL Multirotor)

[SRS-UAV-013] The interceptor UAV platform shall be a Vertical Take-Off and Landing (VTOL) multirotor. The following minimum performance parameters are required:

| Parameter | Minimum Requirement |
|---|---|
| Maximum speed | ≥ 45 m/s (TBC — higher speeds preferred; see OI-001) |
| Maximum acceleration | ≥ 15 m/s² |
| Operational endurance | ≥ 25 minutes at cruise speed |
| Payload mass (effector + seeker) | ≥ TBD kg |
| Operating altitude | 50 m – 5,000 m AGL |
| Operating temperature | −25°C to +45°C |

⚠ OI-001: The maximum speed requirement of ≥ 45 m/s is insufficient to pursue Class A+ Jet OWA (95–110 m/s) in direct tail-chase geometry. The cooperative cutoff / relay interception architecture partially mitigates this, but cannot guarantee engagement in all geometries. This is a critical open issue requiring stakeholder decision.

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

### 10.3 Non-Kinetic Effectors — Directed Energy (DE)

[SRS-EFF-009] The system shall include provisions for a Directed Energy (DE) effector module. The specific DE technology (high-energy laser or high-power microwave) is TBD (⚠ OI-004). The SRS shall be updated with specific performance and safety requirements once the technology is selected.

[SRS-EFF-010] Until OI-004 is resolved, the DE module shall be treated as a reserved interface. The software architecture shall include the message bus interface for a DE effector without implementing specific DE engagement logic.

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

---

## 12. Performance Requirements

> ⚠ OI-002, OI-003: Response time and detection range requirements are TBD pending formal intercept geometry analysis. The following requirements specify the analysis method and will be updated with numeric values before PDR.

[SRS-PERF-001] Class-specific response time requirements shall be formally derived from intercept geometry analysis before PDR. The analysis shall determine, for each threat class at worst-case approach geometry, the maximum tolerable elapsed time from first confirmed track to fire authorization, consistent with achieving minimum required Pk. The analysis shall account for: sensor detection range, track confirmation time, TEWA loop cycle time, ROE evaluation time, interceptor transit time to engagement envelope, and fire request latency.

[SRS-PERF-002] Until OI-002 is resolved, the following indicative (non-binding) target response times shall guide architecture development:

| Threat Class | Indicative Total Response Time (detection → fire authorization) |
|---|---|
| A — Strategic OWA | ≤ 120 s |
| A+ — Jet OWA | ≤ 45 s (highly sensitive to detection range — see OI-001) |
| B — Tactical FPV | ≤ 30 s |
| C — Loitering Munition | ≤ 60 s |

[SRS-PERF-003] The Tier 2 C2 TEWA planning loop latency (end-to-end: track received → engagement tasks published) shall be ≤ 1.0 s at 50 simultaneous tracks. This shall be measured and verified in the simulation platform before hardware integration.

[SRS-PERF-004] Fire request latency (fire request submitted by UAV → clearance issued by C2) shall be ≤ TBD ms (see OI-002). This is driven by the engagement window duration against the fastest threat class.

[SRS-PERF-005] Track position accuracy (root-mean-square error vs. truth) shall be ≤ TBD m at TBD km range (see OI-003, derived from weapon Pk requirements).

[SRS-PERF-006] The system shall sustain full TEWA operational capability when simultaneously tracking up to 50 confirmed hostile tracks per sector. Performance degradation above this limit shall be logged but the system shall remain functional.

[SRS-PERF-007] At peak metropolitan load (400+ threats per night across all sectors), cross-sector track fusion at Tier 1 shall provide a consistent common operational picture with ≤ TBD s staleness across sectors. Individual sector C2 nodes shall continue to operate independently during any Tier 1 downtime.

[SRS-PERF-008] Interceptor UAV minimum engagement Pk (at optimal engagement geometry within the effector envelope) shall be ≥ 0.50 for Class A/C and ≥ 0.35 for Class B (FPV), based on validated effector Pk surfaces (see [SRS-EFF-004]).

---

## 13. Interface Requirements

### 13.1 External Interfaces

[SRS-IF-001] The Tier 1 Metropolitan C2 shall expose a standardised external interface compliant with STANAG 4586 for integration with existing NATO and national military air defence command networks. The interface shall support: receipt of track data from external sensors, transmission of engagement status, and receipt of engagement directives from higher command.

[SRS-IF-002] The operator console interface shall present a real-time tactical display showing: all confirmed tracks with class belief and threat score, all interceptor positions and modes, active engagement tasks, kill box locations, ROE decisions (with reason codes), and current engagement authority mode (HITL/HOTL).

[SRS-IF-003] The operator console shall provide explicit controls for: mode transition (HITL ↔ HOTL), per-track engagement approval (HITL mode), pre-authorization of HOTL window (⚠ OI-006), manual track overriding, and emergency HOLD (suspend all engagements immediately).

[SRS-IF-004] All datalinks between the C2 and interceptor UAVs shall use authenticated and encrypted communications. Authentication shall use a public-key infrastructure or equivalent. Encryption shall use AES-256 or equivalent. ⚠ See Section 15 (Cybersecurity) for full requirements.

[SRS-IF-005] The system shall provide a replay interface for mission debrief. The replay shall present the same visual representation as the live console and shall be driven by the mission recorder log.

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

### 14.2 Safety Assurance

[SRS-SAF-007] The software implementing requirements [SRS-SAF-001] through [SRS-SAF-006] shall be developed and verified to DO-178C DAL B. The safety requirements list shall be included in the Functional Hazard Assessment (FHA) and the Preliminary System Safety Assessment (PSSA) as Hazard Mitigations.

[SRS-SAF-008] The following items from the v0.1 draft are NOT safety requirements and shall be classified as design-level parameters pending safety review:
- The ROE threshold values in [SRS-ROE-006] (⚠ OI-007).
- The DECOY_IGNORE_THRESHOLD value (0.85) — this is a resource allocation parameter, not a safety constraint.
- The 1 Hz TEWA loop rate — this is a minimum performance parameter, not a safety limit.

[SRS-SAF-009] The absence of a hard requirement for zero critical-zone debris impacts is a deliberate stakeholder decision (see Section 8.2 last-resort logic, [SRS-ROE-005]). The ROE allows, as a last resort, engagements where P(CRITICAL hit) ≤ 5% when the alternative is an unimpeded warhead strike. This trade-off involves IHL implications and shall be reviewed and approved by a legal authority before operational use (⚠ OI-007).

---

## 15. Cybersecurity Requirements

### 15.1 Communication Security

[SRS-SEC-001] All RF datalinks between C2 infrastructure and interceptor UAVs shall use AES-256 or equivalent symmetric encryption. Encryption keys shall be rotated at intervals not exceeding TBD hours.

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

---

## 18. Open Issues and TBDs

The following items are unresolved at document version 0.1 and must be resolved before this SRS is elevated to baseline status (before PDR unless noted otherwise).

### OI-001 — Class A+ Jet OWA Interception with VTOL-Only Fleet (CRITICAL)

**Description:** The stakeholder has specified that the system must engage all four threat classes (A, A+, B, C) AND that the interceptor platform shall be VTOL multirotor only. Class A+ Jet OWA operates at 95–110 m/s. The maximum speed of VTOL multirotor interceptors (SRS-UAV-013) is ≥ 45 m/s. The simulation baseline (v0.1 draft) explicitly documents that the jet OWA is "beyond the propeller interceptor tier" and shows 0% direct-pursuit attrition for this class.

**Impact:** A VTOL multirotor fleet cannot reliably engage Class A+ threats by direct pursuit. Cooperative cutoff (relay) geometry partially mitigates this but cannot guarantee engagement in all geometries, especially at high altitude or when the OWA has a straight, unobstructed flight path.

**Options for resolution:**
1. Add a fixed-wing or hybrid VTOL-FW interceptor tier capable of ≥ 120 m/s.
2. Accept that Class A+ engagement relies entirely on cooperative cutoff geometry and directed energy/EW effectors, with reduced Pk guarantee. Define an acceptable Pk for Class A+ accordingly.
3. Reduce Class A+ in-scope from engagement to detect-and-track only, with warning relay to other C2 systems.

**Stakeholder decision required before PDR.**

---

### OI-002 — Class-Specific Response Time Requirements (HIGH)

**Description:** The stakeholder specified that response time requirements shall be derived from formal intercept geometry analysis. This analysis has not been conducted. Until it is, the performance requirements in Section 12 contain TBD values.

**Analysis method:** For each threat class, compute: `R_min = V_threat × T_total`, where `T_total = T_detect + T_confirm + T_TEWA + T_ROE + T_transit + T_engage`. Back-solve for each time budget component. This analysis shall be the basis for requirements [SRS-PERF-001], [SRS-PERF-004], [SRS-UAV-008], [SRS-UAV-012].

**Target:** Complete analysis before PDR.

---

### OI-003 — Detection and Tracking KPIs (HIGH)

**Description:** Minimum detection range, track accuracy, and confirmation time requirements are TBD, pending OI-002. Requirements [SRS-PERF-005], [SRS-DET-009] are incomplete.

**Target:** Derived from OI-002; complete before PDR.

---

### OI-004 — Directed Energy Effector Type (MEDIUM)

**Description:** The stakeholder has specified directed energy (laser or HPM) as an in-scope effector. The specific technology has not been selected. The two options have very different platform integration, power, safety, and ROE implications.

| Aspect | High-Energy Laser (HEL) | High-Power Microwave (HPM) |
|---|---|---|
| Power source | ≥ 10 kW, duty cycle limited | Pulsed, potentially lower average power |
| Range | 1–3 km (air-to-air, weather dependent) | Wider area effect, shorter range |
| Collateral | Narrow beam, precise | Wide lobe can affect bystanders |
| Weather sensitivity | High (fog, clouds attenuate) | Lower |
| UAV integration | Very challenging for VTOL payload | Potentially ground-mounted only |

**Stakeholder decision required before PDR. Until resolved, [SRS-EFF-009] and [SRS-EFF-010] apply.**

---

### OI-005 — Metropolitan Fleet Sizing (MEDIUM)

**Description:** The number of Tier 2 sector base stations, interceptors per sector, and sensor deployments required for a full metropolitan-scale deployment has not been formally determined. [SRS-ARCH-002] specifies a scalability range (4–16 sectors) but does not define the baseline configuration.

**Target:** Completed as part of Operational Requirements Document (ORD) at CDR.

---

### OI-006 — HOTL Pre-Authorization Window Duration (MEDIUM)

**Description:** [SRS-C2-009] allows the operator to pre-authorize a HOTL window during high-density raids. The maximum duration of this window has not been specified. A long window reduces operator cognitive load but expands autonomous engagement authority. A short window may be impractical during 400+/night raids.

**Safety and legal review required. Stakeholder decision before CDR.**

---

### OI-007 — ROE Threshold Values and IHL Review (HIGH)

**Description:** The ROE configuration parameters in [SRS-ROE-006] (collateral cost thresholds, P(CRITICAL hit) caps) are currently engineering estimates from the simulation baseline. They have not been validated against population casualty models and have not been reviewed by a legal authority for IHL compliance. The allowance of up to 5% P(CRITICAL hit) in last-resort mode ([SRS-ROE-006]: `last_resort_p_critical = 0.05`) has direct IHL implications.

**Legal and safety authority review required before operational deployment. Values shall not be treated as operationally valid.**

---

## 19. Requirements Traceability Matrix

### 19.1 Threat Class to Detection/Engagement Requirements

| Threat Class | Detection | Tracking | Classification | C2/TEWA | ROE | Interceptor | Effector | Cooperation |
|---|---|---|---|---|---|---|---|---|
| A — Strategic OWA | DET-001, DET-002, DET-003 | TRK-001–011 | CLS-001–008 | C2-001–010 | ROE-001–011 | UAV-001–013 | EFF-001–004 | COOP-001–006 |
| A+ — Jet OWA | DET-001, DET-002 | TRK-001–011 | CLS-001–005 | C2-001–010 | ROE-001–011 | UAV-001–013 ⚠ OI-001 | EFF-001–004, EFF-009–010 | COOP-001–006 (primary mitigation) |
| B — FPV | DET-001, DET-005 (acoustic critical) | TRK-001–011 | CLS-001–005 | C2-001–010 | ROE-001–011 | UAV-001–013 | EFF-001–004 (net preferred) | COOP-001–006 |
| C — Loitering | DET-001–007 | TRK-001–011 | CLS-001–005 | C2-001–010 | ROE-001–011 | UAV-001–013 | EFF-001–004 | COOP-001–006 |
| D — Decoy | DET-001–007 | TRK-001–011 | CLS-001–008 | C2-003–010 | ROE-008 | No engagement | N/A | N/A |

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
| SRS-EFF-009–010 (directed energy) | Not implemented | Interface reservation after OI-004 resolution |
| SRS-C2-007–010 (HITL/HOTL operator console) | Not implemented | Operator console required for operational system |

---

*End of SRS-COOP-UAV-S-001 v0.1*

*This document is a DRAFT. It has not been formally reviewed or approved. All requirements are subject to change following stakeholder review of the open issues listed in Section 18.*
