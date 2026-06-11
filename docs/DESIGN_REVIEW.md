# Design Review — faults and limitations vs project objectives

*Scope: design-level audit of the simulation framework as of `feature/urban-environment`
(SRS v0.3, June 2026). Line-level bugs are out of scope (handled in PR review);
this document evaluates whether the **design** can deliver the stated objectives.
Findings already owned by [ROADMAP.md](ROADMAP.md) are marked **ACK**.*

The objectives, as stated by README and the SRS:

1. **Cooperation beats speed** — blocker/relay geometry lets a slower fleet defeat
   faster targets (the jet OWA is "included specifically to exercise cooperative cutoff").
2. **Collateral-damage-aware engagement** — debris footprint is part of every fire
   decision; invariant: zero wrecks on CRITICAL ground.
3. **Ukraine-baseline realism** — 400+ drones/night saturation, decoy mixing,
   all-weather (−25..+45 °C, 20 m/s wind, rain/snow/fog), primarily nocturnal
   (thermal dominant), 50 m–5 km band, fiber-FPV jam-immunity, altitude switching,
   two-phase strikes.
4. **Engineering** — deterministic seeded runs; ROS 2-shaped seams so migration
   "replaces two small classes".

**Verdict in one paragraph.** Objectives 1 and 2 are currently *neither proven nor
measurable*: the engagement model (Pk table + point mass) cannot distinguish a
relay-enabled kill from a lucky tail chase, and no metric counts cooperation's
contribution; the collateral chain holds at 9-threat scale but has unmodelled
failure modes. Objective 3's saturation figure is specified (PHY-GCS-002) but never
exercised, and the operator loop demonstrably collapses under it. Objective 4 is
true today but architecturally fragile against exactly the ROS 2 migration it is
designed for.

---

## 1. "Cooperation beats speed" — the claim is unfalsifiable as designed

The most important cluster: the project's headline thesis lacks both the mechanism
that would make it true and the measurement that would show it.

| # | Severity | Finding |
|---|----------|---------|
| 1.1 | HIGH | **Pk table hides whether cooperation matters** (`interceptors/effectors.py`, `sim/adjudicator.py`). No flyout or miss distance: range, off-axis and closing speed collapse into one scalar and one RNG roll. A blocker forcing geometry and a lucky tail-chase shot are indistinguishable in outcome data — the model cannot falsify the core thesis. |
| 1.2 | HIGH | **No proof the jet OWA is defeated.** No test kills the 103 m/s jet with the 80 m/s fleet; `test_support_roles` checks mode assignment, not intercept. The mock frontend even encodes the opposite doctrine ("outruns Tier-P — turret engagement only"). |
| 1.3 | HIGH | **Blocker reserve collapses exactly at saturation** (`c2/assignment.py`, `support_budget = available − remaining_tracks`): in dense raids the fast leaker gets zero support — precisely the case cooperation exists for. Acknowledged in a docstring as a trade; the failure mode is not documented or tested. |
| 1.4 | MED | **"Roles rotate" relay is not implemented** (`interceptors/cooperation.py`, UAV FSM). Blockers are static posts; no handoff transition exists. The 1 Hz replan *might* produce rotation incidentally — untested, and the code comment overpromises. |
| 1.5 | MED | **Metrics cannot see cooperation** (`sim/evaluation.py`): no blocker-contribution, prevented-leak, or intercept-geometry statistics. The research question "how much does cooperation improve outcomes against fast targets?" is unmeasured end to end. |
| 1.6 | MED **ACK** | **Point-mass kinematics, no turn-rate/g limits** (`sim/physics.py`): instant heading change makes interception artificially easy and deflates the value of blocking geometry. ROADMAP owns flight dynamics. |

## 2. Collateral-aware engagement

| # | Severity | Finding |
|---|----------|---------|
| 2.1 | MED | **Assignment is collateral-blind** (`c2/assignment.py`): shooter choice optimises Pk/intercept time; the ROE sees debris cost only *after* the shooter is fixed. Net (velocity retention 0.15) vs projectile (0.65) matters enormously over a city, and the planner never prefers the net for that reason. |
| 2.2 | MED | **No ceiling on debris engagement load** (`c2/base_station.py`, CRITICAL debris score 0.90): wreck mitigation can outrank live fast threats and drain the shooter pool. The live-threat-vs-debris trade is untested. |
| 2.3 | MED | **Debris hazard model is one point at one terminal speed** (`risk/debris.py`, `sim/debris_objects.py`): no fragmentation, no fuel, mechanism difference reduced to a retention scalar; turret stray rounds pass through buildings (`adjudicator._stray_rounds`, partially acknowledged in code). "Zero wrecks on CRITICAL" is verified only against this simplified hazard, at 9-threat scale. |
| 2.4 | LOW | **`zone_source` toggle flips the conservative default** (rects→DANGEROUS vs buildings→SAFE): one config line silently changes the entire ROE baseline. Related: legacy v0.1 scenarios with decorative buildings silently gain occlusion in v0.3. |

## 3. Ukraine-baseline realism

| # | Severity | Finding |
|---|----------|---------|
| 3.1 | HIGH | **Fiber-FPV is not jam-immune by construction** (`sensors/rf.py` + `threats/enemy_drone.py` RF_SIGNATURES): every FPV emits RF, so passive RF always sees them — the defining property of the threat class is absent. A `rf_signature=None` variant would fix the model. Not in ROADMAP. |
| 3.2 | HIGH **ACK** | **Flat terrain**: no elevation model, so terrain masking / valley routing — the stated low-altitude tactic — does not exist; occlusion is buildings-only. |
| 3.3 | MED **ACK** | **Threats do not react**: no altitude switching, no adaptation to detected defences; two-phase strikes deferred to Phase-4 stretch despite SIM-THR-003 listing them; herding is "positioning, not coercion" (ROADMAP's words). Cooperation geometry is being validated against a compliant opponent. |
| 3.4 | MED | **Nocturnal/thermal not modelled** (`sim/weather.py`, `sensors/eo_ir.py`): "thermal dominant" reduces to a ±10 % lighting multiplier; no EO-vs-IR crossover, no thermal-contrast advantage at night — yet night is the primary envelope. |
| 3.5 | MED | **Winter absent**: no temperature state, no battery cold-derating (PHY-UAV-002 specifies it; TRACEABILITY admits absence); precipitation is a single scalar — no snow/sleet/icing distinction. The operational baseline is "mostly nocturnal winter". |
| 3.6 | MED | **Sensor optimism at density**: radar has no urban clutter/multipath; acoustic has no 400-engine soundscape problem (each emitter resolved independently); comms default to a near-perfect link. |
| 3.7 | MED | **Decoy discrimination is all-or-nothing** (`sensors/eo_ir.py` likelihoods): uninformative beyond ID range, 0.98 inside — no graded evidence, so classification flips rather than degrades under noise. |

## 4. Saturation (400+/night) — specified, never exercised

| # | Severity | Finding |
|---|----------|---------|
| 4.1 | HIGH | **Operator loop collapses under `human_confirm`** (`c2/orchestrator.py`): dense waves produce tens of concurrent 12 s auth windows — the operator would need >7 decisions/second; there is no queue prioritisation, shedding or escalation; expiry→HOLD→re-request churn is the structural failure mode (the v0.3 pk-floor bug was one instance of this class). |
| 4.2 | MED | **No scaling validation anywhere**: PHY-GCS-002 (TEWA ≤ 1 s at 400 tracks) is marked "untested" in TRACEABILITY; reference raids are 9–14 threats. Synchronous bus with O(sensors×targets) sweeps, a 256-sample ROE Monte-Carlo per fire request, and a full 1 Hz allocation re-solve — with no profiling hooks or benchmark scenario, the sim would degrade silently. |
| 4.3 | LOW | **Denial-TTL re-queue cycling**: geometry-denied tracks re-enter allocation every 15 s and compete with new threats; overhead unmeasured. |

## 5. Determinism & ROS 2 migration engineering

| # | Severity | Finding |
|---|----------|---------|
| 5.1 | HIGH | **Single shared `world.rng` consumed in call order**: every sensor/adjudication/debris/comms draw depends on exact callback sequencing. ROS 2 executors do not guarantee that order → same seed, different battle. No per-node RNG streams. Together with process-global track-id/msg-seq resets, the *determinism contract* does not survive the migration as designed — "replaces two small classes" is understated. |
| 5.2 | MED | **Synchronous-bus semantics ≠ ROS 2** (`core/bus.py`, `core/node.py`): immediate in-stack delivery, ordering inherited from scenario construction order, no cycle guard, missed ticks rebased — behaviours the test suite currently depends on. |
| 5.3 | MED | **Display rate inside the control loop**: `DebrisReporter` publishes at `record_hz` — changing a recording knob changes C2 tasking timing (violates the spirit of SIM-003). |
| 5.4 | MED | **Truth-quarantine soft spot**: the turret reads `world.occlusion` directly as "survey data" — defensible, but it should be an explicit surveyed-geometry interface, not a world handle. Meanwhile C2 has *no* occlusion awareness, so planning can post LOS-blocked shooters it only discovers at fire time. |
| 5.5 | LOW | **Scenario YAML schema unformalised**: no schema document, version field, or validator — against the "experiments are data" principle. |

---

## Recommended priority order

1. **Make objective 1 measurable, then true** — cooperation metrics
   (blocker-contribution, prevented-leak, kill-geometry class), a jet-OWA defeat
   test, then a miss-distance / geometry-sensitive engagement model. Until then the
   thesis is an assertion.
2. **Saturation track** — benchmark scenarios at 50/100/400 tracks with TEWA-latency
   and operator-queue-depth metrics; auth-queue prioritisation/shedding design.
3. **Determinism architecture** — per-node seeded RNG streams and an explicit bus
   ordering contract *before* the ROS 2 port; decouple the debris reporter from
   `record_hz`.
4. **Cheap realism wins** — zero-RF fiber-FPV variant; threat reactivity
   (ROADMAP Phase 1); thermal/EO night crossover.
5. **Collateral chain** — effector debris cost in assignment, debris
   engagement-load ceiling, stray rounds vs buildings.

## Open questions

- Jet-OWA defeat: is fleet-only defeat required, or is turret-layer defeat
  acceptable doctrine? Changes the weight of finding 1.2.
- 400/night: must the sim hold 400 *concurrent* tracks, or staged waves with a
  lower concurrent ceiling? Determines how hard the scale work is.
