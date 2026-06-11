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
`threat/<id>`. Consequences:

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
