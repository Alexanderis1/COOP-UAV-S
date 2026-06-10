# PHY → SIM Traceability Table (TRC-001)

> Living table required by [SRS](SRS.md) TRC-001 / SIM-001: one row per
> physical-segment requirement, the simulation mechanism that reproduces
> it, and an honest fidelity class.
>
> Fidelity classes: **high** (model grounded in the specified physics/
> behaviour), **representative** (right shape and coupling, invented
> parameters), **placeholder** (seam exists, behaviour stubbed).

| PHY req | Simulated by | Fidelity | Known deviations |
|---|---|---|---|
| PHY-UAV-001 (two tiers, speeds, endurance) | `interceptors/uav.py` point-mass kinematics, per-UAV `max_speed`; fleet defined per scenario | representative | Single tier in reference scenario; Tier-F (150 m/s) not yet in the preset fleet (ROADMAP fast-interceptor item). Point-mass, no airframe dynamics (SIM-PHX-005 upgrade path). |
| PHY-UAV-002 (environmental envelope) | `sim/weather.py` wind displacement on all airborne objects; sensor degradation factors | representative | Temperature/icing effects not yet modelled; battery-vs-cold coupling absent. |
| PHY-UAV-003 (recoil/release tolerance) | Not modelled — effector release has no flight-dynamics effect | placeholder | Irrelevant at point-mass fidelity. |
| PHY-UAV-004 (60 s scramble, autonomous recovery) | Instant launch from home on tasking; RTB + landing at home pad | representative | No explicit scramble latency parameter yet (ROADMAP CAP-station item covers launch latency). |
| PHY-UAV-010 (FCU + mission computer split) | Single agent node per UAV (`InterceptorUav`) at its own update rate | placeholder | Compute split irrelevant until SIL of real flight stack (SIM-SIL upgrade). |
| PHY-UAV-011 (nav sensors, GNSS-denied) | UAV truth state used directly for own-ship navigation | placeholder | Nav error model and GNSS-denial fault injection not yet implemented (SIM-SIL-003 partial: comms faults only). |
| PHY-UAV-012 (EO/LWIR seeker + ranging) | `sensors/seeker.py` onboard seeker: range-limited detections with close-range ID quality | representative | Single combined seeker model rather than separate EO/IR channels; no gimbal FOV constraint. |
| PHY-UAV-013 (health telemetry ≥ 1 Hz) | `UavState` (battery, ammo, mode, link) published at node rate, recorded in frames | high | Per-cell battery / ESC detail abstracted to one battery scalar. |
| PHY-UAV-020 (net vs projectile effectors) | `interceptors/effectors.py` Pk envelopes; mechanism-dependent debris in `risk/debris.py` | representative | Pk surfaces are plausible inventions — no public data exists (RESEARCH §5). |
| PHY-UAV-021 (no release without clearance) | Shooter FSM requires AUTHORIZED `FireClearance`; turrets identically interlocked; orchestrator/posture gate northbound | high | Cryptographic token signing abstracted to message identity. |
| PHY-UAV-022 (calibrated envelope = sim Pk model) | Same `EngagementEnvelope` object used by fire control and adjudicator | high | — |
| PHY-UAV-030 (ROS 2-shaped node middleware) | `core/bus.py` / `core/node.py` pub-sub with typed dataclasses, 1:1 ROS 2 mapping | high | In-process bus; DDS QoS semantics not modelled beyond comms layer. |
| PHY-UAV-031 (software functions as nodes) | guidance / cooperation / FSM / fire control / telemetry all in `interceptors/`, on-bus | high | Single Python class hosts the functions; split into separate nodes at ROS 2 port. |
| PHY-UAV-032 (onboard AI models) | Seeker detection + classification likelihoods feeding Bayesian belief and `p_decoy` | representative | Statistical stand-ins for the neural models; no inference-latency budget yet (SIM-SIL-002 partial). |
| PHY-UAV-033 (link-loss autonomy, no self-authorisation) | Comms model drops/delays clearances; shot discipline holds/aborts; RTB behaviours | representative | Pre-authorised-constraint continuation policy minimal (current task continues; no timeout-RTB parameter yet). |
| PHY-UAV-034 (software runs off-vehicle) | Whole tactical stack runs as plain Python (the SIL mechanism itself) | high | — |
| PHY-UAV-040 (encrypted low-latency C2 link) | `core/comms.py` latency/jitter/loss model on C2↔UAV topics | representative | Security properties asserted, not simulated. |
| PHY-UAV-041 (UAV-UAV mesh ≥ 2 Hz) | `uav/state` peer subscription through comms model | high | — |
| PHY-UAV-042 (auth, replay protection, signed tokens) | Message-identity abstraction; single-clearance consumption in shooter FSM | placeholder | Crypto out of simulation scope. |
| PHY-UAV-043 (link degradation detection/reporting) | Per-UAV link quality (delivery ratio / jam state) in `UavState.link`, displayed in E3 | representative | — |
| PHY-GCS-001 (C2 + fusion stacks) | `c2/` TEWA + ROE + orchestrator; `perception/` fusion — full stack on-bus | high | — |
| PHY-GCS-002 (≤ 1 s TEWA at 400 tracks) | BaseStation at 1 Hz node rate | representative | Compute load not modelled; 400-track stress untested. |
| PHY-GCS-003 (sensor network ownership) | Scenario sensor laydown (`radar/rf/eo_ir/acoustic` nodes) → `detections` | high | — |
| PHY-GCS-004 (northbound ICD, clearance authority) | `viz/server.py` /ops endpoint + orchestrator clearance issuance | high | — |
| PHY-GCS-005 (24 h ops, displacement) | Not simulated | placeholder | Out of engagement-timescale scope. |
| PHY-TUR-001 (turrets slaved to GCS, same interlock) | `sim/turret.py` fire-request/clearance flow identical to UAVs | high | — |
| PHY-TUR-002 (slew/range/dispersion characterisation) | Rate-limited az/el FSM, range gate, dispersion-based per-round hit model, TOF lead | representative | Parameter values invented; thermal limits simplified. |
| PHY-TUR-003 (fused-track fire solutions, ROE applies) | Turret targets only fused `tracks`; requests pass the same ROE + posture gate | high | — |
| PHY-GCS-006 (debris-intercept tasking, kinetic only, red > yellow) | `c2/base_station.py` debris assessments from `debris/state`; `c2/assignment.py` kinetic-only eligibility; turret debris targeting; `c2/roe.py` `debris_mitigation` branch | high | Debris ROE auto-authorises (time-critical defensive act) — documented posture exception. |
| PHY-GCS-007 (Pk-aware assignment) | `c2/assignment.py` envelope-feasibility filter + Pk-proxy-weighted shooter cost | high | Pk proxy uses track speed vs closing-speed envelope, not full geometry. |
| PHY-SEN-001 (radar R⁴, horizon, Doppler) | `sensors/radar.py` R⁴ Pd, radar-horizon mask, radial velocity; building occlusion via `sim/occlusion.py` (two-way transmittance) | high | Terrain (ground elevation) occlusion still absent; buildings only. |
| PHY-SEN-002 (RF bearing-only, signature hash) | `sensors/rf.py` anisotropic covariance bearings, shared decoy/OWA hashes; material-attenuated by `sim/occlusion.py` | high | — |
| PHY-SEN-003 (EO/IR towers, weather/illumination degradation) | `sensors/eo_ir.py` range-ramped ID + `weather.eo_ir_range_factor`; hard-blocked by solid buildings (`sim/occlusion.py`) | representative | Single-channel model of the EO-vs-IR crossover. |
| PHY-SEN-004 (acoustic below-horizon) | `sensors/acoustic.py` + wind/precip range factor; mild diffraction attenuation per crossed building | high | — |
| PHY-SEN-005 (LOS masking by buildings/terrain) | `sim/occlusion.py` 2.5D ray-vs-building grid with per-material, per-channel transmittance | representative | Material transmission coefficients are plausible inventions; no terrain elevation model. |
| PHY-SNT-001 (unarmed sentinel UAVs, EO/IR + RF payload) | `interceptors/sentinel.py` `SentinelUav` with mounted `EoIrSensor` + `RfSensor` feeding `detections` | high | — |
| PHY-SNT-002 (patrol orbits into common picture) | Orbit controller (centre/radius/alt/speed per scenario); detections fuse identically to fixed sensors | high | — |
| PHY-SNT-003 (sentinel endurance/turnaround) | Shared `UavAirframe` battery + RTB/REARM cycle; patrol auto-resume | high | — |
| PHY-CHG-001 (rooftop/adjacent charging stations) | `ChargingStation` objects in `sim/environment.py`; UAV homes resolved to stations; citygen sites them on rooftops/pads | representative | Charge model is the existing turnaround timer; no power/queueing model. |

## Coverage summary

- **high:** 19 — the interlock chain, middleware shape, sensing layer,
  C2 stack, turret integration, sentinel overwatch and debris-intercept
  tasking: the seams the SRS declares load-bearing.
- **representative:** 14 — physical performance models with invented
  parameters (flagged honestly; tuning is this stage's purpose),
  including building-material transmittance and charging-station siting.
- **placeholder:** 5 — crypto, compute-platform split, nav-error,
  recoil, logistics: all parked behind explicit seams with ROADMAP items.

Maintenance rule: any PR that adds or changes a SIM model must update the
affected rows in the same commit (TRC-001).
