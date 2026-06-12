# PLAN — Problem 1: High-Fidelity Physics + SITL UAVs (CoopFC)

*Living checklist. Approved 2026-06-11. Source plan: Claude Code session
`based-on-last-pr-inherited-blossom`. Checkboxes updated every task; each task =
TDD unit (tests first, implement, suite green, tick). Stop-and-replan rule applies
per task — gates are never loosened silently.*

## Status log

- 2026-06-11 — Plan approved. Tier-F fixed-wing interceptor **confirmed out of
  Problem-1 scope** (user decision; ROADMAP Phase-1 item). Cadence: phase-by-phase,
  stop at each phase GATE for user review.
- 2026-06-11 — Pre-change baseline on `feature/urban-environment` (clean tree):
  **160 passed in 51.69s** (31 test files). P0 started.
- 2026-06-11 — **P0 COMPLETE, GATE PASSED**: 211 default + 1 slow, ruff clean.
  DESIGN_REVIEW 5.1 resolved, 5.2 doc'd, 5.3 resolved. Stopped for user review
  per cadence decision; next: P1 (physics core, standalone).
- 2026-06-11 — PR [#7](https://github.com/Alexanderis1/COOP-UAV-S/pull/7) opened
  (`feature/problem1-p0-foundations`, stacked on #6 / `feature/urban-environment`).
- 2026-06-11 — **P1 COMPLETE, GATE PASSED**: physics core standalone, 287 default +
  1 slow + 5 `@oracle` + 1 `@perf` green, ruff clean. Perf 0.188 s CPU/sim-s at
  N=20/N=30 (gate 0.25; first measure 0.406 → numpy hot-path work, no numba).
  RotorPy oracle ≤0.0002 m / ≤1e-4° over 10 s. 41-agent adversarial review (3-lens
  refutation per finding): 12 raw → 8 upheld → 6 distinct, all fixed same day
  (3 mutation-killing test pins for quat_rotate_inv / body-frame Faessler drag /
  control-surface polarity; Dryden v≤0 ValueError; collision closed hi-face bounds;
  oracle exporter t=0 init + sanctioned trace re-record). Stopped for user review
  per cadence; next: P2 (hw device models) or P6 lane B (both unblocked by P1).
- 2026-06-11 — **P1 gate review 2** (45-agent deep inspection of PR #8: confirmed
  findings incl. 1 critical motor↔battery algebraic-loop instability). Fixed in 9
  WPs, all done: **WP1** `physics/powertrain.py` implicit DC-bus solve + `i_bus_max_a`
  current clamp (350 A / 125 A new YAML `powertrain:` blocks) + [3.0, 4.2] V/cell
  bounds, closed-loop pins (explicit lagged loop pinned divergent). **WP2** dryden
  NaN-airspeed/dt ValueError, per-vehicle spawned RNG streams (fleet-size invariant),
  stationary cold start (closed-form discrete Lyapunov), MIL-8785C independent
  literals + 1000 ft upper-clamp pin, ISA non-finite/>11 km pins, `gusts_to_world`
  body→world converter. **WP3** rigid-body pins: frozen-wrench killer (linear-drag
  exponential decay), Hamilton-product literals, Jxz tumble invariants + solve_ivp
  cross-check. **WP4** fixed-wing pins: FRD r channel, c/2V-vs-b/2V + CL_q literals,
  washout threshold, 5 s closed-loop trim hold, Jxz aileron coupling; windmill-drag
  docstring note. **WP5** multirotor pins: GE max-gain clip in singular band, literal
  moment magnitudes, rho scaling. **WP6** collision: sign-preserving eps (negative-t
  fix), malformed-prism ValueError, `ground_z` threaded as one terrain+prism datum.
  **WP7** N=30 perf now gated at 0.25 s/sim-s (0.2 stays informational). **WP8** 6th
  oracle flight `roll_mix_pulse` (gyroscopic coupling), gates tightened 0.5 m/3° →
  0.005 m/0.01° (measured ≤1.9e-4 m / ≤8.9e-5°). **WP9** docs: TRACEABILITY staged
  table re-stated (gate-vs-margin, fpv_quad under multirotor, powertrain row, perf
  wording), ORDERING.md §6 micro-tick contract (gust draw + bus solve placement),
  RESEARCH.md powertrain equation + windmill/tip-Mach known limitations, this entry.
  No existing pins re-baselined; param values unchanged (comments/keys only).

- 2026-06-11 — **P2 COMPLETE, GATE PASSED**: `hw/` device models standalone
  (imu, gps, baro, mag, seeker_gimbal + GimbaledSeeker adapter, esc_telem +
  `hw/stoch.py` shared error processes; `interceptor_devices.yaml`
  invented-but-representative, pinned). Allan suite green: configured N/B/K
  recovered ±10% on all 6 IMU axes (worst 7.2%), valid because the
  vectorized `generate()` path is pinned bit-exact to the `sample()` loop.
  GPS latency exactly 96 ticks @ 800 Hz (integer-tick design, no float-time
  compares). Perf gate: full 20-vehicle sensor stack **0.020 s CPU/sim-s**
  (gate 0.1); N=30 0.027 (gated 0.15 per budget table); reps span 4 sim-s
  so readings resolve above the 15.625 ms Windows process_time quantum.
  86 new tests (default suite 416 + 2 slow + 2 perf + 7 oracle; heavy
  markers run as separate pytest processes — the @slow Allan suite's
  transient ~400 MB heap measurably degrades a same-process @perf
  measurement), ruff clean, legacy suite + golden pins untouched. Branch
  `feature/problem1-p2-hw-devices` stacked on PR #8. One upstream fix
  folded in: `stoch.avar_gauss_markov` initially transcribed the IEEE-952
  GM Allan curve a factor 2 low (their q-parameterization vs our
  stationary sigma) — caught by the Allan Monte-Carlo itself, re-derived
  from R(u) and documented in RESEARCH.md.
- 2026-06-11 — **P2 gate review** (69-agent adversarial workflow: 6 find
  lenses incl. worktree mutation testing, 3-lens refutation per finding):
  11 confirmed (+5 doc/number nits whose verifiers hit a session cap,
  re-judged by hand). All fixed same day: (1) saturation now clips to
  grid-aligned full scale `floor(range/lsb)*lsb` — shipped-yaml 34.9/1.06e-3
  previously rounded a saturated gyro to 34.9005 > range (non-commensurate
  pin added); (2) the surviving-mutant class (cross-channel/cross-component
  draw reuse passes every statistical gate incl. Allan) killed structurally
  by `tests/test_hw_draw_layout.py` — absolute bit-exact draw-layout pins
  for all 5 stochastic devices; (3) GimbaledSeeker servo now advances by
  elapsed sim time (was fixed 1/rate_hz per fire — time-warped under
  scheduler overload); byte-identical claim scoped to all-in-cone scans +
  multi-enemy skip-shifts-draws pin; (4) gimbal initial pose clipped into
  el travel band; (5) generate()==sample() pin now forced across internal
  chunk boundaries (monkeypatched budget); (6) GPS timing pins fail closed
  (exact fix counts); (7) perf reps lengthened to 4 sim-s (the recorded
  0.016 was exactly one Windows timer quantum, unresolved) — resolved
  figures 0.020/0.027; (8) esc rpm relabelled mechanical shaft rpm (eRPM
  pole-pair conversion is the driver's), baro sigma comment 0.25 m.
  Stopped for user review per cadence; next: P3 (CoopFC, critical path);
  P6 lane B remains unblocked.
- 2026-06-11 — **P2 open questions resolved** (user delegated to
  fidelity-optimal): gimbal cue = engaged target's fused track
  (`InterceptorUav.seeker_cue()` additive seam, estimate-only; the interim
  nearest-truth auto-cue was a SIM-GT-001 truth leak and is gone); N=30
  sensor-stack perf gated at the same 0.1 s/sim-s as N=20 (0.15
  budget-table figure informational). See Resolved questions 2-3.

## Context

Based on PR #6 (`feature/urban-environment`, SRS v0.3). Simulator today: 20 Hz fixed-step,
synchronous bus, point-mass kinematics (`sim/physics.py` self-declared "physics-lite"); UAVs are
high-level mode-FSM Python nodes with perfect nav, no autopilot, no onboard sensors, no health
monitoring; engagement = single Pk roll (DESIGN_REVIEW 1.1: core thesis unfalsifiable).

**Objective (Problem 1):** (a) physics modeled 1:1 — 6DOF dynamics, motors/ESC/battery,
atmosphere/wind, sensor physics; (b) each fleet UAV = running software instance ("virtual
microprocessor") executing its own high- and low-level software — SITL: embedded flight software,
sensor drivers, low-level control, CBIT. Other problems deferred until done.

**User decisions (binding):** flight stack = **CoopFC, custom in-repo flight software we author**
(not ArduPilot/PX4/Renode — those rejected/deferred after research); SITL scope = **fleet only**
(threats = vectorized 6DOF + scripted autonomy, preserves 400-threat saturation); physics core =
**custom in-repo, vectorized, source-traceable per equation**; runtime = pure Windows Python
(numpy/scipy/pyyaml); RotorPy + ArduPilot SITL (WSL2) = offline validation oracles only.
Legacy point-mass mode stays behind a scenario flag; all 31 legacy test files keep passing.
**Round 2 decisions:** SITL telemetry = EKF estimate (nav error visible on ops channel; truth on
/eval); design envelope = **30+ SITL UAVs** (vectorization + perf budgets sized for 30 from day
one); numba/C extension only on perf-gate failure w/ profile evidence + user approval; FCU-side
hard fire interlock = follow-up task in P5 (P4 ships MC-side interlock byte-equivalent to today).
**Round 3 decisions (2026-06-11):** Tier-F fixed-wing interceptor OUT of Problem-1 scope.

## Architecture (synthesis of 3 design agents)

### Instance model — in-process `VirtualMCU` (subprocess rejected)
20 UAVs × 2 processors (FCU + MC) = ~9,000 SW ticks/sim-s. Subprocess lockstep IPC on Windows
≈ 2.4 s sync overhead/sim-s + 60 s spawn + 3 GB RSS → rejected. In-process `VirtualMCU` with
isolation made mechanical:
- ctor `(params, clock: VirtualClock, rng, ports)` — no World/bus/env can physically enter
- import-fence test: AST-walk `coopfc/`+`mc/`, assert no import of `sim/`/`threats/`/`sensors/`/`risk/`
- mailbox-only comms (`core/ports.py`); bus callbacks only append to inbox, drained at own tick
- per-MCU exception fence = "processor crash" CBIT/world event (free SIM-SIL-003 fault mode)
- per-MCU VirtualClock + named RNG streams
Port/mailbox seam = future subprocess transport attach point (hybrid deferred, not designed away).

### Time — two-level clock
World macro-step `dt=0.05` stays the bus epoch (C2 1 Hz, fusion 5 Hz, recorder 5 Hz, RunController
untouched). Inside it, SITL micro-loop at `BASE_HZ=800` (scenario data, min 400; CI profile lower):
plant RK4 800 Hz; IMU+rate loop 400 Hz; attitude 100 Hz; EKF/velocity/position/baro/mag/link/CBIT-fast
50 Hz; GPS 10 Hz; CBIT-slow/health 1 Hz. Frozen micro-tick order (determinism contract):
devices sample truth → per-vehicle scheduler runs due tasks (drivers→estimator→controllers→mixer→
PWM→CBIT→link) → MC tick if due → latch actuators → ONE batched fleet RK4 → threat batch.
Rate ratios validated at scenario build (no silent rounding).

### Boundary refactor (strangler, legacy untouched)
`InterceptorUav` → 4 artifacts: `physics/multirotor.py` plant (world-owned, fleet-vectorized) +
`hw/` device models + CoopFC FCU instance + `mc/interceptor_app.py` (FSM/guidance/fire-control
ported near line-for-line, flying on EKF estimates, commanding velocity setpoints over modeled
FCU↔MC link instead of `body.command_velocity`). `sim/vehicle.py FriendlyVehicle` adapter keeps the
`world.friendlies` duck-type (`.position .velocity .mode .body.position .effector .link_quality`) —
comms registration, mounted sensors, enemy evasion, adjudicator, recorder, eval ALL unchanged.
Clearance interlock moves verbatim into `mc/fire_control.py` (same topics, constants MIN_PK=0.30,
CLEARANCE_TIMEOUT_S=3.0 — PHY-UAV-021 preserved). MC↔C2 rides existing `CommsModel` unchanged.

### Repo layout (new packages)
```
src/coopuavs/core/rng.py    RngRegistry: name-keyed SeedSequence streams (fixes DESIGN_REVIEW 5.1)
src/coopuavs/core/ports.py  Port/Mailbox isolation primitives
src/coopuavs/physics/       rigid_body (batched quat 6DOF RK4), multirotor (kf/km rotors, Faessler
                            drag, Cheeseman-Bennett ground effect), motor (ESC+Ke=1/KV+Rw lag),
                            battery (Thevenin 1-RC ECM, OCV(SOC) sag), fixedwing (Beard-McLain),
                            atmosphere (ISA), dryden (MIL-F-8785C), collision, params/*.yaml
src/coopuavs/hw/            imu, gps, baro, mag, seeker_gimbal, esc_telem — Kalibr/PX4 stochastic
                            models (noise density + bias RW + Gauss-Markov + turn-on bias), FIFOs,
                            quantization, latency, per-device RNG streams
src/coopuavs/coopfc/        flight SW (import-fenced, no numpy in >=100 Hz paths):
                            fcu.py (boot/PBIT/arming/modes: OFFBOARD/POS_HOLD/RTL/LAND/FAILSAFE_ATT),
                            sched.py (rate groups, overrun accounting), core/vec.py, core/topics.py
                            (uORB-style), params.py (CRC-checked), hal/ (HalIO seam — MCU-portable),
                            drivers/, estimation/ (Sola error-state 15-state quat EKF, PX4-EKF2-style
                            fusion, alignment), control/ (rate PID 400 Hz, Brescianini attitude P,
                            velocity/position), mixer.py (quad-X, prioritized desat),
                            cbit/ (~22-fault dictionary, monitors, PBIT/CBIT/IBIT engine, degraded
                            modes per PHY-UAV-033), link/coop_link.py (MAVLink-style framed FCU<->MC)
src/coopuavs/mc/            mission-computer apps: interceptor_app, sentinel_app, fire_control,
                            guidance, cooperation, telemetry, fcu_client (guidance/cooperation get
                            re-export shims in interceptors/ so legacy tests untouched)
src/coopuavs/sil/           clock.py (VirtualClock), host.py (VirtualMCU), vehicle.py
                            (FriendlyVehicle), fleet.py (SitlEngine micro-loop in world.step)
scripts/oracle/ + tests/fixtures/oracle/   RotorPy/ArduPilot trace exporters + committed CSVs
```
Scenario YAML: `fidelity: {fleet: pointmass|sitl, threats: pointmass|sixdof}` (defaults pointmass)
+ `sitl: {base_hz, fcu rates, mc tick, link {latency_s, bandwidth_bps}}`. Airframe params:
interceptor = 12 kg quad-X racer-class, 12S, ~320 KV, T/W≈3.6, drag tuned to 80 m/s at 65° tilt —
flagged invented-but-self-consistent, pinned by trim tests.

### CBIT (first-class deliverable)
Fault dictionary ~22 codes (IMU_STALE/RANGE/NOISE, GYRO_STUCK, GPS_LOSS/DEGRADED, BARO/MAG faults,
EKF_INNOV/DIVERGED, DR_BUDGET_LOW, MOTOR_RESPONSE(i)/SAT_PERSIST, BATT_LOW/CRITICAL/SAG_ANOM/
CELL_IMBALANCE, LINK_C2_LOSS/LINK_MC_LOSS, SCHED_OVERRUN, PARAM_CRC, ALIGN_FAIL, WDOG_MISS) with
severity/latching/inhibit_arming/inhibit_fire/degraded_mode. Monitors at 50 Hz + 1 Hz. Degraded
modes: GPS loss → dead-reckon + inhibit_fire + RTL on DR budget; mag fault → yaw-from-GPS-course;
batt critical → LAND; EKF diverged → FAILSAFE_ATT + inhibit_fire; C2 loss → never self-authorize.
Northbound `UavHealth` msg 1 Hz on routed topic; ICD frame `uavs[].health` additive only.

### ICD/recorder — additive only
`UavState` gains optional `attitude_q`/`health`/`nav_quality` (None in pointmass mode). Recorder
adds per-UAV `att`/`health` keys only when present; ICD_RUNTIME.md bumped additively same commit.
Frontend untouched. SITL telemetry reports EKF **estimates** (nav error operationally visible);
truth stays on /eval.

## Phases & tasks

Critical path P0→P1→P2→P3→P4 (~70% of effort). Lane B (P6) parallel after P1; Lane C (P7 comms/
debris) anytime after P0; P7 flyout last. Cadence: stop at each phase GATE for user review.

### P0 — Foundations (M) — only phase touching legacy behavior
- [x] P0-1 `docs/PLAN_PROBLEM1.md` living checklist (2026-06-11)
- [x] P0-2 pytest markers `slow`/`perf`/`oracle` in pyproject; default run excludes
      slow/perf/oracle per Stated Assumptions ("default pytest run stays fast") —
      note: task line originally said perf/oracle only; assumptions section governs.
      `--strict-markers` added. Tests: `tests/test_markers.py` (2026-06-11)
- [x] P0-3 characterization pins: SMALL_SCENARIO + urban_raid(60s, seed 7) event+summary
      golden files in `tests/fixtures/golden/`; test `tests/test_characterization.py`;
      re-record only via `scripts/record_golden.py` at P0-7 (2026-06-11)
- [x] P0-4 `sil/clock.py` VirtualClock + RateGroupScheduler + MicroScheduler; `World.micro`
      seam (None = legacy). Pins reproduce bit-for-bit with seam attached at K=1 AND K=4;
      integer-tick clock exact at 1e6 ticks; non-divisor rates rejected.
      Tests: `tests/test_sil_clock.py` (20) (2026-06-11)
- [x] P0-5 scenario `fidelity` flags parsed+validated; defaults pointmass/pointmass; sitl→
      NotImplementedError (P4), sixdof→NotImplementedError (P6); unknown keys/values rejected;
      stored in `Scenario.meta["fidelity"]` (recordings untouched until P4-7).
      Tests: `tests/test_fidelity_flags.py` (9) (2026-06-11)
- [x] P0-6 `core/rng.py` RngRegistry (streams pure fn of (seed, name), sha256 key);
      migrated one consumer per commit: weather ("weather") → comms ("comms") → sensors
      ("sensor/&lt;name&gt;") → adjudicator ("adjudicator") + DebrisModel ("debris") → threats
      ("threat/&lt;id&gt;"). Shared `world.rng` proven virgin through a full battle; order-independence
      capstone green (extra consumer, identical outcomes). One legacy test seeding idiom updated
      (test_kill_bookkeeping steers `adj._rng`, assertions untouched).
      Tests: `tests/test_rng_registry.py` (6) + `tests/test_rng_streams.py` (7) (2026-06-11)
- [x] P0-7 stochastic re-baseline: `docs/reports/rng_rebaseline.md` (+before/after JSON);
      floors re-affirmed (24→33 kills, 6.708→5.606 spk; floors ≥10 / ≤9.0); 0 CRITICAL
      wrecks all 20 runs; pins re-recorded ONCE; suite fully green (206) (2026-06-11)
- [x] P0-8 DebrisReporter own `debris_hz` scenario knob (default 5.0 = no behavior change);
      rate-decoupling test in `tests/test_debris_live.py` (2026-06-11)
- [x] P0-9 `docs/ORDERING.md` (step phases, bus semantics, node scheduling, RNG streams,
      ROS 2 preservation list) + `tests/test_ordering.py` (4 pins). DESIGN_REVIEW 5.1
      RESOLVED, 5.2 doc'd, 5.3 RESOLVED (2026-06-11)
- [x] GATE PASSED 2026-06-11: full suite 211 green + 1 `@slow` green (all 31 legacy files
      incl. hit-rate floors); order-independence capstone proves 5.1 fixed; ruff clean.
      Awaiting user review before P1.

### P1 — Physics core, standalone (L) — vectorized `(N,·)` from day one
- [x] P1-1 `physics/rigid_body.py` batched quat RK4: free-fall/quat-rotation analytic, energy drift
      <1e-9/60 s vacuum, RK4 order slope test. 13 tests `tests/test_rigid_body.py` (2026-06-11)
- [x] P1-2 `atmosphere.py` ISA + `dryden.py` MIL-F-8785C (PSD matches analytic spectrum via Welch).
      13 tests `tests/test_atmosphere_dryden.py` (2026-06-11)
- [x] P1-3 `motor.py` (step τ in 15-50 ms band, ω ceiling tracks sagging V) + `battery.py` ECM
      (instant sag = I·R0, recovery τ1, coulomb integral exact — exact-ZOH discretization).
      12 tests `tests/test_motor_battery.py` (2026-06-11)
- [x] P1-4 `multirotor.py`: hover trim Σkfω²=mg ±0.1%, ground-effect curve at z/R∈{0.6,1,2},
      terminal speed 80±5 m/s at 65° tilt (cdA tuned → 80.0, pins airframe params), Faessler
      drag signs. `params/interceptor_quad.yaml`. 12 tests `tests/test_multirotor.py` (2026-06-11)
- [x] P1-5 `fixedwing.py` Beard-McLain: trim at cruise (residual <1e-3·mg; shahed α 7.1° δt 0.74,
      jet α 3.0° δt 0.23), C_mα<0, stall bounded; shahed_fw/jet_owa_fw/fpv_quad param files.
      FRD↔FLU flip M=diag(1,-1,-1) doc'd+tested. 11 tests `tests/test_fixedwing.py` (2026-06-11)
- [x] P1-6 `collision.py` prism/terrain + batch==scalar equivalence (1e-12) + perf microbench
      (`@perf`: 20-vehicle RK4 @800 Hz ≤0.25 s CPU/sim-s) — gate first measured 0.406, numpy
      hot-path optimization (no numba) → **0.188 s/sim-s at N=20 and N=30**. 10 tests (2026-06-11)
- [x] P1-7 oracle traces: `scripts/oracle/export_rotorpy.py` → 5 committed CSVs; `@oracle` tests
      pos RMSE <0.5 m / att <3° over 10 s matched-param drag-free flights — measured ≤0.0002 m /
      ≤1e-4°. rotorpy 2.1.2 offline-only. pitch flight = doublet (one-sided pulse tumbled through
      ground plane: our GE clamp vs RotorPy no-GE; flight-design fix) (2026-06-11)
- [x] P1-8 TRACEABILITY (staged-models table) + RESEARCH.md "P1 physics core" citations per
      equation (TRC-001 same commit) (2026-06-11)

### P2 — Hardware device models (M)
- [x] P2-1 `hw/imu.py`: Allan-variance slope test recovers configured N/B/K ±10% (`@slow`) —
      measured worst 7.2% over 6 axes (32768 s @ 100 Hz; generate()==sample() pinned bit-exact);
      FIFO/quantization/turn-on-bias-per-seed tests. `hw/stoch.py` shared primitives
      (exact-ZOH GM + stationary cold start, bias RW, quantize, analytic AVAR curves) (2026-06-11)
- [x] P2-2 `hw/gps.py` noise + GM wander (h/v split), 10 Hz, 120 ms latency exact
      (integer-tick: 96 ticks @ 800 Hz, divisibility validated at build) + fix-type field
      (2026-06-11)
- [x] P2-3 `hw/baro.py` ISA round-trip exact (altitude_from_pressure inverse; sigma_h =
      sigma_p/(rho g0) pinned) + GM drift; `hw/mag.py` theater field (|B|/decl/incl -> ENU),
      hard-iron per power-up + GM bias + white, rotation pins vs quat_to_rotmat (2026-06-11)
- [x] P2-4 `hw/seeker_gimbal.py` FOV/slew/servo (PHY-UAV-012): rate-limited first-order
      2-axis servo (deadbeat for dt>tau, never overshoots), travel limits, closed FOV cone;
      `GimbaledSeeker` adapter (additive — OnboardSeeker untouched; detections byte-identical
      when every observed enemy is in-cone, FOV-skip shifts later draws same scan [pinned];
      servo advances by elapsed sim time; cued by the engaged target's fused track via
      `InterceptorUav.seeker_cue` — estimate-only, untasked = caged hold, P4 moves the call
      onto the FCU<->MC link) (2026-06-11)
- [x] P2-5 `hw/esc_telem.py` (BLHeli32-class rpm/V/A frames off Powertrain outputs, exact rpm
      conversion pin, quantization, powertrain-in-envelope smoke) + determinism/stream-uniqueness
      suite (run-twice, extra-consumer order-independence, removed-device invariance, fleet-growth
      prefix, shared-parent hazard pin) + `@perf` stack gate (2026-06-11)
- GATE: Allan suite green; 20-vehicle sensor stack ≤0.1 s CPU/sim-s — measured **0.020-0.023
  s/sim-s at N=20, 0.027 at N=30** (N=30 gated at the SAME 0.1, P1 precedent; budget-table 0.15
  informational; 4 sim-s reps, resolved above the Windows timer quantum)

### P3 — CoopFC flight stack in isolation (XL — largest phase)
`sil/bench.py` harness: physics + hw + one FCU, no tactical stack. Import fence enforced.
- [x] P3-1 `coopfc/core/vec.py` plain-float quat/vec math (validated vs scipy Rotation AND
      physics helpers; ZYX euler w/ gimbal-pole branch; exact-map quat_integrate) +
      `core/topics.py` uORB-style latest-value store + `params.py` CRC-checked overlay
      (bool≠int pinned) + import fence: AST walk, no `coopuavs.*` escapes coopfc (relative
      imports resolved), numpy only under `estimation/`. 42 tests `test_coopfc_{vec,topics,
      params,fence}.py` (2026-06-11)
- [x] P3-2 `coopfc/sched.py` rate groups: exact fire counts over 10 s (800/400/100/50/10/1 Hz),
      registration order = within-tick pipeline, integer-tick derived time, duplicate names
      rejected. Overruns MODELED not measured (declared `cost_ticks` busy window — wall-clock
      would be nondeterministic; P5 injects overload by raising cost): due-fire inside busy
      window skipped+counted, consecutive ≥ `overrun_fault_after` latches fault (CBIT
      SCHED_OVERRUN seam). Exceptions propagate (VirtualMCU crash fence is the host's, P4).
      14 tests `test_coopfc_sched.py` (2026-06-11)
- [x] P3-3 `coopfc/hal/` HalIO seq-stamped latest-frame ports (host writes, drivers read —
      MCU-portable seam) + `coopfc/drivers/` imu/gps/baro/mag/esc + `core/msgs.py` typed
      NamedTuples. Shared staleness contract (no new seq = stale tick; latches at
      `stale_after`, clears on fresh frame; GPS default 3 — first fix lands 2 driver ticks
      after boot at 120 ms latency). Unit round-trips: coopfc-owned ISA inverse pinned
      bit-near vs `hw.baro.altitude_from_pressure` over 0-10 km; esc rpm→rad/s inverts
      encoding; baro rejects non-finite/<=0 Pa without publishing (bad_frames tally = CBIT
      seam); GPS msg carries fix_stamp (OOSM key). 15 tests `test_coopfc_drivers.py`
      (2026-06-11)
- [x] P3-4 `estimation/alignment.py` (leveling, gyro bias, mag yaw, variance gate, honest P0) +
      `ekf.py` Sola 15-state error-state EKF: PX4-style delayed horizon (OOSM structural,
      exact-stamp fusion on the IMU lattice), chi-square gates, Joseph form, divergence latch.
      Colored-error honesty: R inflation + mag yaw information floor + baro PARTIAL update
      (gain masked to vertical channel — 15k correlated baro-drift fusions otherwise suppress
      claimed sigma_vel 20x via maneuver-built cross-covariances; caught by the 4-sigma honesty
      gate, isolated by baro-on/off A/B) + unmodeled-error budget on every reported sigma with
      one-shot GNSS-denial injection (RESEARCH.md "P3 CoopFC flight stack"). NEES/NIS 25-seed
      MC vs the real P2 devices (`@slow`, bounds not precision); GPS-denied 5 min: drift
      km-class free-inertial (worst 5472 m, regression gate 7000 m, first-principles ~3.4 km
      RSS scale) AND inside the filter's own 4-sigma claim — PHY-UAV-011 partial, VIO/datalink
      fallback out of sim scope; 50 m spoof step gated (>=25 consecutive rejections). 17 tests
      `test_coopfc_{alignment,ekf}.py` + 2 `@slow` `test_coopfc_ekf_mc.py` (2026-06-12)
- [x] P3-5 `control/` cascade (rate PID 400 Hz w/ conditional anti-windup vs own clip AND
      mixer axis_sat feedback; quaternion attitude P, Brescianini law, yaw_weight 0.4;
      velocity PI -> (q_sp, thrust) via flatness map + u_hover sqrt thrust curve) +
      `control/mixer.py` quad-X sequential desat, priority rp > collective > yaw, per-axis
      directional saturation flags. Acceptance vs REAL plant (truth-fed; powertrain motor lag
      in loop, plant RK4 800 Hz / ctl 400 / vel 50): roll+pitch rate rise <60 ms overshoot
      <20%; yaw rate 0.5 rad/s settle 0.138 s, gated <0.20 s regression-style (user decision
      2026-06-12 gate review, RESEARCH.md "P3-5 yaw rate gate": yaw authority ~30x weaker —
      drag-torque actuation — 60 ms figure is a roll/pitch spec); 30° att step settle <0.5 s;
      vel zero SS error calm + 5 m/s wind; integrator frozen-while-saturated white-box pin +
      2.5 s saturated-dash recovery; mixer analytic desat pins; run-twice bit-identical.
      14 tests `test_coopfc_control.py` (2026-06-12)
- [x] P3-6 `fcu.py` (sched-wired pipeline drivers→est_intake 400→est_update 50→batt→pbit→
      nav 50→rate_mix 400; FCU-owned ParamTable; rate feedback = latest gyro − EKF bias) +
      `control/position.py` P→vel_sp + `battery_monitor.py` (upward-latching NORMAL→LOW→CRIT,
      1 s debounce). FSM BOOT(align, auto-retry on variance gate)→STANDBY(PBIT: align/stale/
      EKF/no-GPS-fusion/battery/PARAM_CRC/sched-faults)→ARMED modes OFFBOARD/POS_HOLD/RTL/LAND;
      failsafe priority BATT_CRIT→LAND > LINK_LOSS→RTL > BATT_LOW→RTL > OFFBOARD_TIMEOUT→
      POS_HOLD, first reason latched; touchdown = altitude-floor of home datum (documented
      bench placeholder until P4 ground). Pins: PBIT-blocks-arming + recovery; vibration
      align retry; setpoint-timeout→POS_HOLD; link-loss→RTL tick-exact timeline; battery
      debounce + LOW→RTL→CRIT→LAND + CRIT-beats-link-loss; disarmed actuators zero; RTL
      integration flights through HAL+EKF+cascade vs real plant (perfect frames): 120 m fast,
      2 km under 6 m/s crosswind `@slow` (41.9 s wall, lands <190 s, disarms). 10 tests
      `test_coopfc_fcu.py` (2026-06-12)
- [x] P3-7 `link/coop_link.py`: MAVLink-shaped framing (sync|len u16|id u8|payload|crc32,
      streaming decoder, corrupt frame costs exactly one frame + resync, bad_frames CBIT
      tally); struct-packed msg registry (HEARTBEAT/ARM/DISARM/SET_MODE/VEL_SP/SET_HOME/
      STATUS/NAV); Channel = pure-arithmetic FIFO wire (serialization 8n/bps behind previous
      tx + fixed latency; bounded in-flight bytes, send REFUSED deterministically when over
      budget). Pins: per-type round-trips, byte-at-a-time chunking, corruption+resync,
      garbage skip, arrival times exact closed-form (not a tick early), back-to-back burst
      spacing, backpressure refuse+drain, idle-wire no history, run-twice. 11 tests
      `test_coopfc_link.py` (2026-06-12)
- [x] P3-8 `sil/bench.py` (physics + P2 hw devices w/ real noise/latency/quantization + one
      FCU; frozen-stand boot, devices-sample-truth→FCU→actuators→plant micro-tick; Dryden
      world-rotated) + acceptance flights. Hover gate SPLIT (user decision 2026-06-12,
      RESEARCH.md): CONTROL error |est−hold| gets the plan numbers — <0.15 m calm / <1.0 m
      in 8 m/s+Dryden w20=8 (measured 0.07–0.08 m both); TRUTH error gated at the GNSS
      device budget 2.0 m RMS (measured 0.5–0.9 m; GM wander floor, RTK-class hover is out
      of suite scope by design). 200 m waypoint square via MC-role OFFBOARD velocity guidance
      on NAV telemetry: TRUTH cross-track <2 m (passes at face value). Run-twice bit-identical
      (truth + nav + actuators). PERF gate re-scoped (user decision 2026-06-12, RESEARCH.md):
      1-vehicle RTF ≥3× (measured 3.6–3.7×; the pre-P1 "≥20×" died with the N-independent
      ~0.2 s/sim-s plant floor) + 20-instance projection ≥1× per the P4 fleet architecture
      C20 = C_phys+dev(N=20) + 20·C_fcu(direct) — measured RTF 1.24–1.38×; enabled by a
      sha256-verified VALUE-IDENTICAL selection-indexed EKF fusion refactor (`_fuse_sel`).
      5 tests `test_coopfc_bench.py` (2 fast + 2 @slow + 1 @perf) (2026-06-12)
- [x] P3-9 `@oracle` ArduPilot SITL (WSL2, official prebuilt stable ArduCopter, EKF3) flies
      the same 200 m square via `scripts/oracle/export_ardupilot_square.py` (pymavlink,
      offline-oracle policy, fixture committed); `test_oracle_ardupilot.py` envelope bands:
      both complete, lap ratio [0.5,2.0] (35 vs 38 s), leg cross-track same class (1.81 vs
      0.67 m — bench flies real GNSS GM wander), cruise ±30%, alt band ±4 m. Setup +
      re-baseline procedure in tests/fixtures/oracle/README.md (2026-06-12)
- [x] P3-10 tuning-stop rule: EXERCISED 2026-06-12 — P3-8 hover-truth-RMS and RTF-20× gates
      were physically unreachable (GNSS GM wander floor; N-independent plant cost); stopped,
      raised to user, resolved by explicit decision (gate split + re-scope, RESEARCH.md) —
      never loosened silently
- GATE: bench ✓ + NEES ✓ + oracle ✓ (RotorPy + ArduPilot) + determinism ✓; perf per the
  2026-06-12 user re-scope: 1-vehicle RTF ≥3× (meas. 3.6–3.7×), 20-instance projection ≥1×
  (meas. 1.24–1.38×). STOPPED for user gate review per cadence.
- [x] P3-R gate-review fixes (2026-06-12, 7-angle/20-verifier review of PR #10; 10 confirmed,
      all fixed TDD-first, 13 new tests): **F1** GPS driver poll 50 Hz (was 10 — off-phase
      from the 120 ms fix delivery, every bench fix reached the EKF 200 ms old, 60 ms past
      lag_s, silently fused ~42 ms stale; new `ekf.late_meas` CBIT seam counts behind-horizon
      stamps, pinned ==0 through real device timing; gps stale_after 15 keeps the 300 ms
      window); **F2** touchdown datum frozen at LAND entry + armed `cmd_set_home` refuses
      home z at/above vehicle (TOCTOU → mid-air disarm); **F3** EKF pos0/vel0 seeded from the
      latest fix (origin prior chi-gated out any spawn >~870 m: PBIT NO_GPS_FUSION forever);
      **F4** link decoder rejects length fields > registry MAX_PAYLOAD (corrupt len byte
      stalled decode ~9 s → spurious LINK_LOSS RTL); **F5** arming seeds `_last_hb` (no-
      heartbeat-ever flight had LINK_LOSS structurally disabled); **F6** param-table mag
      declination threaded into EkfParams (split defaults = persistent yaw bias under
      overlay); **F7** cmd_arm resets `_q_sp/_thrust/_sat` (re-arm ran up to 15 ticks on the
      previous flight's terminal setpoints); **F8** EKF intake early-returns when diverged
      (unbounded gps/baro/mag deque growth post-divergence); **F9** esc driver rejects
      non-finite/non-positive frames like baro (NaN v_bus sustains the battery debounce —
      NaN >= x is False — into latched CRITICAL → forced LAND); **F10** wire enum tables
      (STATE/MODE/FAILSAFE/BATT codes) pinned in the link registry, cross-checked against the
      fcu vocabulary. Full fast suite 550 + ruff + @slow/@perf/@oracle re-run green.
- [x] P3-R2 cut-findings pass (2026-06-12, user-directed "fix all + decide yaw"): **(a)** hover
      gates grew a VERTICAL channel — control z plan-class <0.15/1.0 m (measured 0.028-0.037),
      truth z 3.0 m RMS vertical device budget (gps_gm_v 2.4 ⊕ baro drift 1.25 filter-blended;
      measured 1.21-2.43 over seeds, +23% regression headroom); **(b)** `_fuse_sel` equivalence
      now a COMMITTED default-suite pin vs a test-side dense Joseph reference (all 4 sensor
      blocks + masked baro partial update + spoof-gate case); **(c)** Joseph update expanded to
      rank-m selection form (~5x fewer multiplies; algebraic identity, pinned by (b)) + baro
      gain mask precomputed; **(d)** strapdown deduplicated — one `_strapdown_step` feeds both
      mainline and output replay (sha256 BIT-IDENTICAL pre/post on a 6 s maneuvering-EKF and a
      3 s bench run); output predictor kept FULL replay (exact) over PX4-style incremental
      delta (approximate) per the fidelity goal — perf headroom delegated to (c), @perf green;
      **(e)** `mag_yaw` deduplicated alignment↔EKF (same sha256 proof); **(f)** bench hot-loop
      preallocation (u/z buffers, hoisted zeros; sha256 bit-identical); **(g)** yaw settle gate
      re-stamped as user decision and TIGHTENED 0.40 → 0.20 s regression gate (+45% over the
      deterministic 0.138 s; RESEARCH.md "P3-5 yaw rate gate"). Suites re-run green.

### P4 — Fleet integration (XL — riskiest; staged strangler)
- [x] P4-1 `sil/vehicle.py` FriendlyVehicle truth adapter + protocol-conformance test pinning the
      full world.friendlies duck-type (consumer map in test_sil_vehicle.py: adjudicator/evasion/
      comms/recorder/seeker accesses; legacy InterceptorUav + SentinelUav held to the same
      contract) + `sil/fleet.py` SitlEngine behind World.micro: N×(P2 device banks w/ registry
      `sensor/*` parents + `dryden`) + N FCUs, frozen ORDERING §6 order pinned structurally;
      wind = plant force (WeatherState.mean_wind_at + per-vehicle Dryden, OU excluded;
      world.step skips wind_displaced=False friendlies). User decisions 2026-06-12: Dryden over
      OU; IMU accel = exact wrench force_world/m (dv/dt placeholder closed in the engine);
      ground contact deferred (stand convention: frozen non-ARMED rows, pre-spin at arm).
      Pins: §6 first-tick order, run-twice bit-identical (wind+gusts), fleet-size invariance
      (bitwise gust draws + 1e-9 trajectory — ULP kernel finding, RESEARCH.md), exact-wrench
      IMU, stand freeze w/ live devices, ekf.late_meas==0 through fleet timing, world-clock
      lockstep + wind-skip seam. 20 tests test_sil_{vehicle,fleet}.py (2026-06-12)
- [x] P4-2 Stage 1 velocity passthrough: `mc/fcu_client.py` FcuClient (HEARTBEAT/VEL_SP/
      autonomous ARM→OFFBOARD flow off STATUS; wire enum tables, never literals) + SitlBody
      (PointMass duck: command_velocity→VEL_SP, position/velocity = NAV ESTIMATE — agent never
      reads truth); engine hosts FCU side in the §6 pipeline tail (drain 50 Hz, NAV 25 / STATUS
      10 down; bench heartbeat placeholder now unlinked-only); scenario sitl build path live
      (`sitl: {base_hz, link, fcu}` validated; SitlBody swap + FriendlyVehicle into friendlies/
      adjudicator/seekers/comms, link_quality forwarded to tactical telemetry; sentinels legacy
      until P4-5). Pins: arm→OFFBOARD over the wire, velocity passthrough closes <1 m/s, link
      silence → OFFBOARD_TIMEOUT latched first + RTL escalation (P3 priority contract), pursuit
      twin of test_guidance closest TRUTH approach <10 m through the full stack, build wiring,
      2-interceptor FPV kill in SITL_SMALL_SCENARIO (kill t≈49 s, SAFE-zone debris, truth
      quarantine visibly held: est≠truth bounded <10 m). 8 tests test_sitl_stage1.py +
      fidelity-flag flip (2026-06-12)
- [x] P4-3 Stage 2 MC split (PHY-UAV-010/011): `core/ports.py` bounded Mailbox/Ports +
      `sil/host.py` VirtualMCU ((clock,rng,ports) ctor, exception fence latches processor crash,
      clock freezes — SIM-SIL-003 fault mode free) [P4-3a]; guidance/cooperation moved to `mc/`
      (re-export shims keep legacy imports); clearance interlock moved VERBATIM to
      `mc/fire_control.py` FireControl — ONE state machine driven by BOTH the legacy node and
      `mc/interceptor_app.py` (same effector object, ammo cannot fork; uav.py keeps `_clearance`
      property views for tests); InterceptorApp = near line-for-line FSM port on mailbox I/O
      (tasks/tracks/debris/peers/clearance/command/link_quality in; uav_state/fire_request/fire/
      cue out); `SitlShellUav` thin shell ferries bus↔mailboxes + mirrors mode/battery, body =
      app's estimate body; engine hosts MCU in the §6 step-3 slot; scenario sitl path builds
      shell+MCU per interceptor (`sitl.mc_hz` default 10). Pins: 4 byte-equivalent clearance
      twins (same script → field-equal FireRequests both hosts), MC arms+flies from inside the
      loop on mailbox tasking, crash fence end-to-end (dead MC → silent → FCU failsafes home,
      sim never sees the exception), e2e kill re-validated through the MCU path. 7+6 tests
      test_sil_host.py + test_sitl_stage2.py; suite 591 fast green, ruff clean (2026-06-12)
- [x] P4-4 energy/telemetry rewire (user decisions 2026-06-12: voltage-proxy + full land-dock):
      STATUS wire msg += batt_frac f32 (BatteryMonitor.fraction(): loaded v_cell mapped
      crit→0..4.20→1, conservative under sag; real SOC = P5 CELL_IMBALANCE); app battery =
      telemetry property (synthetic drain model deleted); MC floor debounced 2 s (one-sample
      spool-up sag read 0.11); FCU failsafe leads, app follows it home (post-mortem latched
      reason ignored while disarmed). Rearm = physical cycle: RTB→LAND→touchdown+disarm on pad
      → engine pad charger (set_pad, SOC ramp over recharge_s=turnaround) → BATT_RESET wire msg
      (pack-swap semantics, ground-only, clears the upward latch) → re-arm → climb-out. Two
      FCU fixes shaken out: (1) touchdown drops the EKF for ground re-alignment (the stand-stop
      is IMU-unobservable; chi-gates locked out GPS/baro → 8 m/s free-running pad drift);
      (2) OFFBOARD setpoints clamped to vel_max_h/up/down (PX4 convention — fleet overlays size
      the envelope per airframe). MC loiter altitude 15 m (no ground contact: only FCU LAND
      approaches the surface; legacy point-mass keeps z=0). mc/ import fence joined the coopfc
      AST walk (allowed: mc.*, core.messages, core.ports, coopfc.link). KNOWN FINDING (out of
      P4-4 scope, converges): vertical brake from fast climbs holds near-hover thrust for
      seconds (~90 m overshoot at 20 m/s climb authority) — P3 velocity-controller envelope,
      flagged for review. 6+2 tests test_sitl_energy.py + fence; 598 fast + @slow/@perf
      coopfc flights re-run green, ruff clean (2026-06-12)
- [x] P4-5 sentinels as MC app: `mc/sentinel_app.py` (verbatim orbit guidance on NAV estimates;
      shared P4-4 land-dock/batt-telemetry machinery) + `SitlShellSentinel` ferry shell;
      sentinels join the SAME SitlEngine (shared fleet airframe — documented approximation;
      ops envelope = MC max_speed/orbit speed, which is what PHY-SNT pins); mounted EO/RF
      payloads ride the FriendlyVehicle TRUTH adapter in sitl mode (scenario passes platforms,
      not shells). Twins: orbit annulus on TRUTH + PATROL + sentinel telemetry (estimate,
      0.001<|est−truth|<15 m) + airborne-only track formation; drained pack breaks the orbit
      off home through the FCU failsafe. 2 tests test_sitl_sentinel.py; legacy test_sentinel
      untouched green; suite 600 fast, ruff clean (2026-06-12)
- [x] P4-6 `tests/test_sitl_end_to_end.py` over SITL_SMALL_SCENARIO (full stack: devices→EKF→
      MC VirtualMCUs→coop_link→FCU→batched plant + unchanged C2/ROE/adjudication). Baseline
      measured 2026-06-12 seeds 0..9: 10/10 kills, 0 leakers, 0 CRITICAL wrecks, t_end 32-66 s.
      OWN floors (tripped floor = stop-and-replan): 3-seed CI kills≥1 each + CRITICAL==0 +
      event-kind chain + in-flight truth-quarantine band (1e-3 < |est−truth| < 10 m); run-twice
      determinism (events + summary equal); 10-seed @slow total kills ≥9/10 + CRITICAL==0.
      Stage-1 kill smoke superseded into this suite. 4 tests; 601 fast + @slow e2e green,
      ruff clean (2026-06-12)
- [x] P4-7 recorder/ICD additive: UavState += attitude_q/nav_quality/health (None in pointmass;
      apps fill att+nav_q from NAV/STATUS telemetry — estimate-domain; health lands P5);
      recorder `_uav_entry` emits att/nav_q/health ONLY when present; ICD_RUNTIME bumped v0.4
      same commit (additive §2.2 note + sitl pos/vel=estimate clarification). Parse pins:
      pointmass recording keeps the EXACT v0.3 uav key set; sitl recording carries unit-quat
      att + nav_q in (0,10) m, no health yet, json round-trips. 2 tests test_sitl_recorder.py
      (2026-06-12)
- [x] P4-8 perf gate `@perf` test_sitl_perf.py: residential_raid sitl (8 FCU+MC pairs, full
      pipeline) **RTF 0.80-0.81× headless** (1.24 s CPU/sim-s, gate ≥0.5×, 60% headroom) over a
      20 sim-s boot+raid slice; committed profile docs/PERF_P4_SITL.md (FCUs 46% — EKF dominant;
      batched plant 29%; macro pipeline ~0.4 s/sim-s; consistent with the P3-8 C20 projection).
      Fallback levers untouched (2026-06-12)
- GATE: sitl twins ✓ (guidance pursuit, clearance×4 byte-equivalent, sentinel, energy cycle) +
  e2e ✓ (3-seed CI + 10-seed @slow, own floors, 0-CRITICAL invariant) + determinism ✓ (engine
  run-twice bitwise; e2e events+summary) + perf ✓ (RTF 0.80× vs 0.5 gate) + legacy suite
  untouched and green (601 fast incl. all 31 legacy files) + mc/ import fence. STOPPED for
  user gate review per cadence (2026-06-12).
- [x] P4-R gate-review resolutions (user delegated to fidelity/determinism-optimal, 2026-06-12):
      **(1) vertical-brake loss FIXED** — root cause: low-fz tilt cone + 50 Hz sign-flipping
      saturated horizontal demand → ±45° attitude-setpoint steps → rate-loop torque slam →
      mixer rp-priority desat pins average collective at hover (vertical priority lost in the
      actuator chain; EKF verified healthy). Fix: `VelParams.tilt_slew` 6 rad/s followable-
      setpoint limit (engages only on pathological steps; all P3 maneuver specs unchanged) +
      `mc/guidance.approach_velocity` braking-aware waypoint capture (MC apps only; legacy
      keeps goto_velocity). Deterministic reproducer pinned (pre-fix vz +3.4, post-fix < −5);
      fleet climb-out <30 m (pre-fix >90). E2e RE-BASELINED post-fix: 9/10 seed kills (seed-0
      5-shot pk≈0.5 miss streak, vehicles healthy), CI seeds 1-3, @slow floor 8/10 — documented
      in test docstring + RESEARCH.md. Residual honest behavior: sustained full-power climbs
      sag the 12S pack into the voltage-only monitor's band → FCU protects/lands/retries (P5
      CELL_IMBALANCE owns SOC estimation). 604 fast + @slow bench/fcu/e2e + @perf (re-read
      1.59×, machine-state sensitive per the documented caveat) green, ruff clean.

### P5 — CBIT + fault injection (M)
- [ ] P5-1 `cbit/` dictionary+engine+monitors: table-driven test per fault (detection latency,
      latch, degraded mode); `inhibit_fire` end-to-end suppression of staged fire request
- [ ] P5-2 scenario `faults:` block (sensor dropout, GPS denial, motor-out, link jam) injected at
      hw/link level on dedicated streams (SIM-SIL-003); no-fault scenarios bit-identical
- [ ] P5-3 degraded-mode scenarios: motor-out→controlled descent no-CRITICAL-wreck; GPS-denied
      5 min→DR bound+RTB; interlock holds under every injected fault
- [ ] P5-4 `UavHealth` ≥1 Hz to C2 + recorder + TRACEABILITY rows (PHY-UAV-013/033 → high)
- [ ] P5-5 FCU-side hard fire interlock: clearance token mirrored over coop_link; FCU refuses
      WEAPON_RELEASE without valid token (additive; MC-side interlock already live since P4)
- GATE: fault matrix 100% test-covered

### P6 — 6DOF threats + saturation (L; parallel after P1)
- [ ] P6-1 vectorized threat batch `(N,13)` fixed-wing + FPV multirotor; scripted autonomy as
      vectorized autopilot-lite (course/alt hold PD per Beard-McLain ch.6); per-class envelope pins
- [ ] P6-2 `EnemyDrone` adapter over batch rows; legacy threat mode behind flag; 6DOF twins of
      test_threats/test_threat_evasion
- [ ] P6-3 `benchmarks/saturation_400.yaml`: 400 threats + 20 sitl UAVs, RTF ≥0.2×, TEWA latency
      profiled (starts DESIGN_REVIEW 4.x evidence)
- [ ] P6-4 10-seed MC pointmass-vs-6DOF threat comparison report
- GATE: envelope pins + vector==scalar + full-scale run-twice determinism (`@slow`)

### P7 — Fidelity extras (L)
- [ ] P7-1 comms link budget: log-distance + shadowing + altitude-Rician → per-link SNR→loss
      replacing scalar (legacy mode kept); test_comms twins
- [ ] P7-2 debris drag-coefficient ballistics replacing retention scalar; predict==realize shared
      kernel consistency
- [ ] P7-3 munition flyout: projectile/net flyout + dispersion → miss distance → Pk(miss);
      adjudicator uses flyout in sitl mode, Pk-roll kept legacy; calibration vs envelope table
- [ ] P7-4 falsifiability closure (DESIGN_REVIEW 1.1): blocker-forced geometry measurably shifts
      miss-distance distribution vs tail chase over MC batch
- GATE: legacy adjudication untouched; cooperation measurable

## Performance budget & fallbacks
Design envelope: **30 SITL UAVs** (user decision). Estimate at 30 ≈1.0-1.3 s CPU/sim-s full
saturation (FCU plain-float hot path ~0.42, MC 0.08, batched plant 0.2, sensors 0.15, threats 0.2)
→ RTF gates (headless, 30-UAV fleet): ≥0.5× reference raid, ≥0.2× 400-threat saturation.
All perf microbenches sized at N=30. Rules: physics/hw vectorized across vehicles; no
numpy/allocation in ≥100 Hz paths; profiling gate each phase. Fallback levers in order:
(1) scenario rate profiles (CI 200/100/25 Hz documented), (2) mixed-fidelity fleets,
(3) IMU rate = control rate, (4) numba/C extension — only on gate failure, with committed profile
evidence, and explicit user approval (user-confirmed policy).

## Docs & process (every phase, same commit — TRC-001)
SRS: extend existing SIM-SIL-001..003 numbering (no new prefix). TRACEABILITY rows per model.
ICD additive only. RESEARCH.md citation per equation (Sola, Brescianini, Faessler,
Cheeseman-Bennett, Beard-McLain, MIL-F-8785C, Kalibr/PX4, Chen-Rincon-Mora ECM).
DESIGN_REVIEW 1.1/1.6/5.1/5.2/5.3 marked resolved as they close.

## Verification (end-to-end)
1. Per-task unit tests (TDD) — analytic physics, Allan variance, NEES/NIS, step responses, fault matrix.
2. Determinism: run-twice pins everywhere; order-independence suite invariant; legacy golden files.
3. Oracles: RotorPy trajectory diffs (CI via committed CSVs); ArduPilot SITL behavioral envelope (offline).
4. e2e: legacy suite green every phase; sitl twins (end_to_end, clearance_binding, energy_rearm,
   sentinel, guidance, comms); 0-critical-wrecks invariant; perf gates with committed profiles.
5. `coopuavs run scenarios/... --headless` in both fidelity modes; dashboard replay unchanged.

## Critical existing files modified (seams only)
- `src/coopuavs/sim/world.py` — SitlEngine insertion, RngRegistry, skip wind-displacement for sitl
- `src/coopuavs/sim/scenario.py` — fidelity flags, sitl build path, rate validation
- `src/coopuavs/interceptors/uav.py` — strangler source → `mc/` apps (legacy path kept)
- `src/coopuavs/core/messages.py` — additive UavHealth + optional UavState fields
- `src/coopuavs/viz/recorder.py` + `docs/ICD_RUNTIME.md` — additive frame fields, same commit
- `docs/TRACEABILITY.md`, `docs/SRS.md`, `docs/RESEARCH.md`, `docs/DESIGN_REVIEW.md` — per phase

## Top risks
1. Python perf at 9k ticks/sim-s → conservative gates + profiling each phase + approved fallback levers.
2. P0-6 RNG migration shifts MC baselines → one consumer per task, 10-seed before/after report,
   tripped floor = stop-and-replan, never tolerance bump.
3. SITL intercepts worse than perfect-nav point-mass → own floors, EKF/controller tuned in P3
   before any tactical assertion; staged strangler isolates cause.
4. EKF/controller tuning rabbit hole → tolerances are tests-first spec + explicit stop rule (P3-10).
5. Hidden truth coupling via `world.friendlies` duck-type → protocol-conformance test pins contract
   before sitl build path lands.
6. ICD drift → additive-only + legacy-recording parse test.

## Resolved questions
1. Tier-F fixed-wing interceptor out of Problem-1 scope — **CONFIRMED out** (user, 2026-06-11).
2. P2 gimbal cue source (user delegated "optimal for fidelity", 2026-06-11): **engaged target's
   fused track** via additive `InterceptorUav.seeker_cue()` seam (estimate-only, SIM-GT-001;
   untasked = caged hold). The earlier interim nearest-truth-threat auto-cue was a truth leak
   into tactical logic (plan risk #5) and was removed same day; P4 moves the call onto the
   modeled FCU<->MC link.
3. P2 perf gate at N=30 (user delegated, 2026-06-11): gated at the **same 0.1 s/sim-s as N=20**
   (P1 same-bound-both-N precedent); the 0.15 budget-table figure stays informational. Tight
   sensor gates protect the fidelity budget from the degrading fallback levers.
