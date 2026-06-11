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
      servo advances by elapsed sim time; interim nearest-threat auto-cue until P4 MC cueing)
      (2026-06-11)
- [x] P2-5 `hw/esc_telem.py` (BLHeli32-class rpm/V/A frames off Powertrain outputs, exact rpm
      conversion pin, quantization, powertrain-in-envelope smoke) + determinism/stream-uniqueness
      suite (run-twice, extra-consumer order-independence, removed-device invariance, fleet-growth
      prefix, shared-parent hazard pin) + `@perf` stack gate (2026-06-11)
- GATE: Allan suite green; 20-vehicle sensor stack ≤0.1 s CPU/sim-s — measured **0.020 s/sim-s
  at N=20, 0.027 at N=30** (N=30 gated 0.15 per budget table; 4 sim-s reps, resolved above the
  Windows timer quantum)

### P3 — CoopFC flight stack in isolation (XL — largest phase)
`sil/bench.py` harness: physics + hw + one FCU, no tactical stack. Import fence enforced.
- [ ] P3-1 `core/vec.py` (vs scipy Rotation) + `core/topics.py` + `params.py` CRC overlay
- [ ] P3-2 `sched.py` rate groups: exact fire counts over 10 s, overrun→fault, deterministic order
- [ ] P3-3 `hal/` + `drivers/`: staleness flags, unit round-trips
- [ ] P3-4 `estimation/alignment.py` (leveling accuracy, variance gate) + `ekf.py`: Sola F/Q
      predict, covariance symmetry/PD guard; GPS/baro/mag sequential fusion + chi-square gating +
      0.5 s ring-buffer OOSM; NEES/NIS 25-seed MC consistency (`@slow`); GPS-denied drift <envelope
      5 min (PHY-UAV-011); 50 m spoof step rejected
- [ ] P3-5 `control/` cascade + `mixer.py`: rate rise <60 ms overshoot <20%; 30° attitude step
      settle <0.5 s; velocity zero steady-state error; anti-windup ramp recovery; desat priority
- [ ] P3-6 `fcu.py` boot/PBIT/arming/modes + `battery_monitor`/failsafes: PBIT-blocks-arming,
      setpoint-timeout→POS_HOLD, link-loss→RTL timeline, RTL home from 2 km under wind
- [ ] P3-7 `link/coop_link.py`: framing/heartbeat/latency/bandwidth-queue determinism
- [ ] P3-8 bench acceptance flights: hover RMS <0.15 m calm / <1.0 m in 8 m/s+Dryden; 200 m
      waypoint square cross-track <2 m; run-twice pins
- [ ] P3-9 `@oracle` ArduPilot SITL (WSL2) waypoint-square envelope comparison, procedure doc'd
- [ ] P3-10 tuning-stop rule: tolerances unmet after budgeted tuning → STOP and replan (never
      loosen gates silently)
- GATE: bench + NEES + oracle + determinism; 1-vehicle RTF ≥20×, 20-instance projection ≥1×

### P4 — Fleet integration (XL — riskiest; staged strangler)
- [ ] P4-1 `sil/vehicle.py` FriendlyVehicle protocol-conformance test (pins full duck-type contract)
      + `sil/fleet.py` SitlEngine into `world.step` (wind becomes force, not displacement)
- [ ] P4-2 Stage 1 velocity passthrough: InterceptorUav keeps FSM, `command_velocity` routes over
      link to FCU OFFBOARD; sitl twin of guidance intercept test; 1-interceptor kill in
      SITL_SMALL_SCENARIO
- [ ] P4-3 Stage 2 MC split: tactical logic → `mc/` apps on own VirtualMCU (PHY-UAV-010/011);
      `interceptors/uav.py` thin shell in sitl mode; clearance-interlock sitl twins byte-equivalent
- [ ] P4-4 energy/telemetry rewire: ECM battery via FCU telemetry; UavState from MC estimates only
      (truth quarantine holds); import-boundary test
- [ ] P4-5 sentinels as MC app + sitl twin of test_sentinel
- [ ] P4-6 `tests/test_sitl_end_to_end.py`: ≥1 kill, 0 CRITICAL wrecks, determinism pin; sitl gets
      OWN re-baselined floors (3-seed CI + 10-seed `@slow`), never reuses pointmass pins
- [ ] P4-7 recorder/ICD additive fields + ICD_RUNTIME v0.4 same commit + legacy-recording parse test
- [ ] P4-8 perf gate `@perf`: residential_raid sitl RTF ≥0.5× headless + committed profile; miss →
      pull fallback levers before proceeding
- GATE: all sitl twins + e2e + determinism + perf; legacy suite untouched and green

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
