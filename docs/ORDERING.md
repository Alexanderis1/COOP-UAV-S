# Ordering contract — bus delivery and world stepping

*The execution-order guarantees the simulator makes, what deliberately is
NOT guaranteed, and what a ROS 2 port must preserve. Pinned by
`tests/test_ordering.py`; closes the documentation side of
DESIGN_REVIEW 5.2.*

## 1. The macro step (`World.step`, `sim/world.py`)

Each step, in this frozen order:

1. **Comms drain** — `CommsModel.step(t)` delivers every queued routed
   message whose latency has elapsed, before any node runs (SIM-COM-001).
2. **Weather** — OU gust process advances (own RNG stream; calm air draws
   nothing).
3. **Enemy spawns** — spawn-queue entries with `spawn_time <= t` pop in
   schedule order.
4. **Enemy step + wind** — each enemy integrates at world `dt`, then wind
   displaces it (truth-side kinematic displacement).
5. **Debris integration** — falling debris steps and lands *between* the
   enemies and the nodes, so sensors and C2 see this tick's fall state
   (SIM-DEB-001).
6. **SITL micro seam** — `World.micro.run_macro_step(t, dt)` if installed
   (SIM-SIL-002): K micro-ticks of the fleet engine, so the nodes below see
   this tick's flown state. `None` in pointmass scenarios.
7. **Nodes** — `node.maybe_update(t, dt)` in *registration order* (§3).
8. **Friendly wind** — truth-side displacement of airborne, non-REARM
   friendlies (SIM-PHX-003). Note the asymmetry: enemy wind applies before
   the nodes, friendly wind after.
9. **Clock** — `t += dt` (float accumulation; the SITL micro clock is
   integer-tick and immune, `sil/clock.py`).

## 2. Bus delivery (`core/bus.py`)

- `publish` is **synchronous and in-stack**: subscriber callbacks run inside
  the publisher's stack frame, in **subscription order** (list-append order).
- There is **no re-entrancy guard**: a callback that publishes recurses into
  `publish`. Keep handler chains shallow.
- **Routed-topic exception** (`core/comms.py`): topics in `ROUTED_TOPICS`
  with a non-None endpoint on either side are intercepted by the comms
  router — loss-rolled per hop (own RNG stream), then queued
  `(t + latency, seq)` and delivered at the **next** macro step's comms
  drain. Equal-deadline messages drain in send order (monotonic `seq`
  tiebreak). Self-delivery (sender == receiver) never routes: it stays
  synchronous.
- Subscription order is fixed at construction time and is **not** the node
  tick order: UAV/sentinel objects are constructed before the ground
  segment, so on shared topics (`tracks`, `uav/state`,
  `engagement/clearance`) their callbacks sit earlier in the subscriber
  lists even though their node ticks run later in the step.

## 3. Node scheduling (`core/node.py`)

- Registration order = within-step pipeline. `scenario.build` registers:
  sentinel-mounted EO → RF (per sentinel) → ground sensors (YAML order) →
  per-interceptor seekers → fusion → debris reporter → base station →
  orchestrator → turrets (YAML order) → interceptors (YAML order) →
  sentinels (YAML order) → adjudicator → eval tracker → recorder. That is:
  **sense → fuse → decide → act → adjudicate → evaluate → record**.
- `maybe_update` fires at most once per macro step. The next deadline
  advances from the previous deadline (no systematic under-rate); missed
  deadlines **rebase to `t + period`** — there is no catch-up burst.
- Every node fires its first update at `t = 0`.
- The adjudicator has no periodic work: it is event-driven via
  `engagement/fire`, which is unrouted — adjudication happens **inline in
  the shooter's own tick**.

## 4. Randomness (DESIGN_REVIEW 5.1 — fixed)

Every consumer draws from its own named stream
(`core/rng.py RngRegistry`, pure function of `(run_seed, name)`):
`weather`, `comms`, `sensor/<name>`, `adjudicator`, `debris`,
`threat/<id>`. The P2 hw device banks (wired by the P4-1 SitlEngine)
follow the same rule with one parent stream per device type —
`sensor/imu`, `sensor/gps`, `sensor/baro`, `sensor/mag`,
`sensor/esc_telem`, plus `dryden` for the fleet gust bank — from which
each bank spawns one child per vehicle (the Dryden pattern: a fleet-size
change leaves existing vehicles' draw histories identical; suites:
`tests/test_hw_determinism.py`, including the pin that spawning twice from
ONE parent is *not* an independent copy — names must be unique, and
`tests/test_sil_fleet.py` through the engine wiring. Note the contract is
*draw histories*: batched einsum/matmul kernels differ at the last ULP
across batch sizes, so trajectories agree to ~1e-14 relative, not
bitwise — RESEARCH.md "P4-1 fleet-size invariance").
Consequences:

- Execution order between consumers **no longer touches randomness** — an
  extra consumer, a removed sensor, or a reordered scan leaves every other
  stream's draws identical (capstone test in `tests/test_rng_streams.py`).
- Stream names must be unique per consumer: requesting an existing name
  returns the *same* generator (that is the registry's caching contract),
  so a name collision silently couples two consumers.
- The legacy shared `world.rng` stays for construction-time compatibility
  and is proven untouched through a full battle.

What still depends on ordering: message `seq` numbers
(`core/messages.py` process-global counter, reset per `World`), track ids
(`perception/tracking.py`, same), event list order, and comms queue
tiebreaks. These are reproducible given the step order above, but they are
**ordering-sensitive by design** — a ROS 2 port must either preserve the
pipeline order or stop asserting on them.

## 5. What a ROS 2 port must preserve

1. The §1 phase order (or an explicit barrier equivalent).
2. The sense → fuse → decide → act → adjudicate pipeline as *intra-step*
   sequencing — executors that interleave callbacks across phases will
   reproduce a different battle from the same seed.
3. Per-consumer RNG streams (already executor-safe).
4. The routed/unrouted split: unrouted topics assume same-tick synchronous
   delivery (the clearance interlock and inline adjudication depend on it).
5. The §6 micro-tick order once the SITL engine lands (P4).

## 6. SITL micro-tick contract (engine landed P4-1: `sil/fleet.py`)

Referenced normatively by the physics docstrings (`physics/motor.py`,
`physics/__init__.py`); the frozen order is the PLAN_PROBLEM1 "Time —
two-level clock" contract, extended here with where the Dryden gust draw
and the powertrain bus solve sit. Inside one §1 item-6 macro seam, each of
the K micro-ticks (BASE_HZ, plan: 800 Hz) runs, in this frozen order:

1. **Devices sample truth** (`hw/` models, P2; per-device-type parent
   streams with per-vehicle spawned children, §4). Devices are clocked at
   exact divisors of BASE_HZ and latencies are integer tick counts
   (`hw/gps.py` validates both at construction; 120 ms = 96 ticks at
   800 Hz) — a rate pairing that doesn't divide is a scenario error.
2. **Per-vehicle software scheduler** runs due tasks: drivers → estimator →
   controllers → mixer → PWM → CBIT → link.
3. **MC tick** if due.
4. **Latch actuators** — throttle/surface commands frozen for the rest of
   the micro-tick.
5. **Dryden gust draw** — one `DrydenGusts.step()` per fleet bank
   (per-vehicle child streams, §4-conformant: a fleet-size change leaves
   every other vehicle's gust history identical). The body-FLU gusts are
   rotated through each vehicle's pre-step attitude with
   `dryden.gusts_to_world` and composed into `wind_world`, which is then
   held (ZOH) across the plant RK4 stages.
6. **Powertrain bus solve + electrical advance** — `Powertrain.step()` at
   the latched throttle: closed-form implicit solve of the motor/battery
   algebraic bus loop at pre-step omega/SOC/V1, bus current and
   cell-voltage limits, then motor RK2 micro-step and battery SOC/V1
   update. The resulting rotor speeds are **latched** as plant inputs.
   Never wire `i_bus` → `battery.step` → `v_bus` explicitly one step late:
   that composition diverges at any dt (`physics/powertrain.py`).
7. **ONE batched fleet RK4** — `rigid_body.rk4_step` over all vehicles at
   the latched rotor speeds/wind; state-dependent wrench terms re-evaluate
   at every RK4 stage, latched inputs do not.
8. **Threat batch** (P6 6DOF threats, own streams).

Steps 5 and 6 have no data flow between them and draw from separate
streams, but the order is frozen anyway — determinism pins replay whole
ticks. Status: P1 ships the physics models and their unit pins; P2 ships
the step-1 device models (imu/gps/baro/mag/seeker_gimbal/esc_telem) and
their unit/determinism pins; P4-1 lands the fleet micro-loop
(`sil/fleet.py SitlEngine`) — the tick order is pinned structurally by
`tests/test_sil_fleet.py` (first-tick call-sequence pin plus run-twice
determinism), step 3 (MC tick) is a seam until P4-3, step 8 (threat
batch) until P6. P4-2 wires the coop-link into the step-2 pipeline tail
(50 Hz drain/dispatch using the wire enum tables, NAV 25 Hz / STATUS
10 Hz down); in stage 1 the MC side is the legacy tactical NODE (§1
item 7, 10 Hz) driving `mc/fcu_client.py SitlBody` — frames it sends at
node time t enter the FCU inside the NEXT macro step's micro window, so
command transport is honestly one macro step + serialization + latency. Two P4-1 wiring notes, both user decisions 2026-06-12:
the IMU samples the exact wrench `force_world / m` at the latched inputs
(the P3 dv/dt bench placeholder is closed in the engine; the
single-vehicle bench keeps its pinned form), and ground contact stays
deferred — a non-ARMED row is frozen truth ("stand convention") with
zeroed velocity/rates, devices keep sampling it with real noise, and the
world's truth-side wind displacement skips SITL friendlies
(`FriendlyVehicle.wind_displaced = False`; wind is a plant force here).
