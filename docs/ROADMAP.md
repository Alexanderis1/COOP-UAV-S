# COOP-UAV-S — Roadmap

Phases are ordered by research value per unit of effort. Items reference
the literature survey in [RESEARCH.md](RESEARCH.md).

## Phase 1 — Harden the baseline (post-hackathon weeks 1–2)


- [x] **Forward high-altitude CAP sentinels** (PHY-SNT-004) — pickets that
      stand up *already on station* above the ground radar's envelope and
      forward of the defended area, carrying a look-down airborne
      early-warning radar (`sensors/airborne_radar.py`), with a barrier-
      racetrack patrol option. Cuts diving-jet acquisition latency ~36× in
      `scenarios/high_diver_raid.yaml`. Placement is still hand-tuned;
      auto-optimising picket geometry against historical axes is the
      remaining piece.
- [x] **Fast-interceptor tier** — the 100 m/s jet OWA is beyond an 80 m/s
      propeller interceptor by physics, as in reality. Add a small number
      of 150+ m/s interceptors with their own cost/availability budget.
- [x] **Metrics module** — per-run scorecards (attrition by class,
      time-to-intercept distributions, debris cost integral, ammo per
      kill), batch report generation; promote the 10-seed Monte-Carlo from
      ad-hoc script to `coopuavs batch` output.

## Phase 2 — Deepen the two innovation pillars (weeks 3–6)

Cooperation:
- [x] **Apollonius-circle containment** (`mc/apollonius.py`): exact
      closed-form Apollonius rendezvous now drives the cooperative blocker
      relay (`cooperation.cutoff_points`), replacing the v0.1 time-stepping
      search; plus the game-theoretic containment arc and an escape-set
      (safe-fraction) area objective for manoeuvring evaders and for seeding
      the learned policy (RESEARCH.md §1: Garcia/Casbeer/Von Moll/Pachter).
      Remaining: iterative area-gradient contraction controller.
- [x] **Decentralised allocation (CBBA)** behind the existing
      `allocate(...)` interface; degrade gracefully when the base station
      link drops — the doctrinal argument for UAV autonomy.
- [x] **MARL benchmark** (MAPPO via a PettingZoo-shaped wrapper around the
      sim) vs the geometric baseline — implemented: `coopuavs/rl/` (env,
      shared-actor/centralised-critic MAPPO, CPU-parallel workers),
      `c2/learned_allocator.py` (drop-in behind the `allocate` seam), and
      `coopuavs eval` for the A/B. See docs/MARL.md.

Risk-aware engagement:
- [ ] **Intercept-point optimisation**: choose *where along the corridor*
      to take the shot by minimising expected debris cost subject to Pk —
      today the ROE only vetoes/approves the geometry the shooter offers.
      This closes the loop between kill-box selection, herding and ROE,
      and is the project's most defensible novelty (RESEARCH.md §6 notes
      the open-literature gap).
- [ ] **Population-density risk maps** (time-of-day layers, shelter
      states) replacing the three-class grid; casualty-expectation units.
- [ ] **Decoy-ratio sweep study**: defence performance as decoy fraction
      varies 0–60 % — directly addresses the Gerbera economics problem.

## Phase 3 — Fidelity & migration (months 2–4)

- [ ] **ROS 2 port**: reimplement `MessageBus`/`Node` on rclpy, generate
      `.msg` files from `core/messages.py`, run the same scenarios under
      ROS 2 with the sim as a node. (Aerostack2 as the multi-drone
      framework target — RESEARCH.md §7.)
- [ ] **Gazebo / PX4 SITL** flight dynamics for a 2–3 vehicle slice;
      keep the Python world for large-scale Monte-Carlo.
- [ ] **Sensor fidelity**: micro-Doppler classification features, terrain
      occlusion masks for radar/EO, weather effects (the operational
      envelope in the README is mostly nocturnal winter).
- [ ] **Effector flyout models**: net deployment dynamics, projectile
      dispersion — replace the Pk surface with Monte-Carlo flyout where it
      matters.

## Phase 4 — Stretch

- [ ] Human-on-the-loop C2 console (the FireClearance seam already exists).
- [ ] Hardware-in-the-loop with a real flight controller.
- [ ] Two-phase strike / re-attack tactics; EW degradation of own sensors;
      comms-denied operation studies.

## Known limitations of v0.1 (honest list)

- Constant-velocity tracker lags manoeuvres (mitigated by onboard seeker).
- Effector Pk surfaces are plausible inventions, not measured data — no
  public Pk data exists for any C-UAS interceptor (RESEARCH.md §5).
- Battery model is a linear drain; no recovery/rearm cycle (UAVs RTB and
  stay there).
- The live dashboard loads Three.js from a CDN — vendor it for offline
  demo environments.
