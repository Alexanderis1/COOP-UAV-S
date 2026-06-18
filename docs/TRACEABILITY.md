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

## Staged models — physics core (P1, not yet wired into the sim)

P1 delivered `src/coopuavs/physics/` standalone (plan Problem-1; wiring
arrives in P4 for the fleet and P6 for threats — the PHY rows above keep
describing the live legacy path until then). Equation citations:
RESEARCH.md "P1 physics core". Validation per model:

| Model | Will serve | Fidelity | Validation pins |
|---|---|---|---|
| `physics/rigid_body.py` batched quat 6DOF RK4 | PHY-UAV-001 dynamics (replaces point-mass via SIM-PHX-005), 6DOF threats | high | free-fall exact; analytic spin; 60 s torque-free energy/momentum drift < 1e-9 (diagonal and Jxz inertia); order-4 slope; state-dependent-wrench exponential-decay anchor; Hamilton-product literals; batch==scalar; scipy cross-checks |
| `physics/atmosphere.py` ISA | PHY-UAV-002 envelope | high | USSA-1976 table values at 0/1/11 km; > 11 km and non-finite altitude ValueError |
| `physics/dryden.py` MIL-F-8785C | PHY-UAV-002 wind/turbulence (upgrade of `sim/weather.py` displacement) | high | Welch PSD == analytic spectrum; variance; spec param table vs independent literals (Beard-McLain Table 4.1 cross-check); 10/1000 ft clamps; coeffs == scipy.signal.bilinear; per-vehicle child RNG streams (fleet-size invariant); stationary cold start; `gusts_to_world` body->world rotation pins |
| `physics/motor.py` + `physics/battery.py` | PHY-UAV-013 energy truth, SIM-SIL motor/battery faults | representative | tau in 15-50 ms band; w ceiling tracks sag; sag = I*R0; recovery exp(tau1); coulomb exact; throttle clip + SOC charge-side clamp. Standalone only — bus coupling is an unstable algebraic loop, wire through `powertrain.py` |
| `physics/powertrain.py` implicit motor+battery DC-bus coupling | PHY-UAV-013 energy truth (P4-4 wiring), battery-sag faults | representative | bus fixed point satisfies both component equations; explicit lagged loop diverges where Powertrain stays bounded; 10 s closed loop finite + SOC monotone; spin-up inrush clamped at `i_bus_max_a` (YAML sizing pinned); bus voltage in [3.0, 4.2] V/cell; batch==scalar |
| `physics/multirotor.py` + `interceptor_quad.yaml` / `fpv_quad.yaml` | PHY-UAV-001 Tier-P plant; FPV multirotor threat (P6) | representative (invented-but-self-consistent params) | hover trim 0.1%; Cheeseman-Bennett curve exact + max-gain clip in the singular band; 80 m/s terminal at 65 deg pin; rho-scaling of parasitic drag; drag dissipation; allocation signs + literal moment magnitudes; fpv_quad hover headroom 0.3-0.8 and T/W 2.0-4.5; RotorPy oracle gate 0.005 m / 0.01 deg over 10 s x 6 flights (measured <= 1.9e-4 m / <= 8.9e-5 deg) |
| `physics/fixedwing.py` + `shahed_fw/jet_owa_fw.yaml` | 6DOF threat classes (P6) | representative | cruise trim residual < 1e-3 mg; Cm_alpha < 0; stall bounded; damping/weathervane signs; literal q/r rate-term pins (c/2V vs b/2V); prop washout; 5 s closed-loop trim hold; Jxz aileron roll-yaw coupling |
| `physics/collision.py` | wreck/impact events for sitl/sixdof modes | high | analytic wall/roof/terrain hits; malformed-prism ValueError in every entry point; one `ground_z` datum shared by terrain + prisms (nonzero-datum pins); batch==scalar 1e-12 |

Perf: plant RK4 at 800 Hz is gated at 0.25 s CPU/sim-s for **both** N=20
and N=30 (`pytest -m perf`); the tighter ~0.2 s/sim-s N=30 budget-table
figure is informational only (printed by the test, never asserted).
Measured 2026-06-11: 0.19-0.22 s/sim-s, machine-dependent.

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
