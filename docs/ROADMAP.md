# COOP-UAV-S — Roadmap

Phases are ordered by research value per unit of effort. Items reference
the literature survey in [RESEARCH.md](RESEARCH.md).

## Phase 1 — Harden the baseline (post-hackathon weeks 1–2)

- [ ] **IMM tracking** (CV + coordinated turn + dive models) — the CV
      filter lags terminal dives and weave; biggest single accuracy win.
      (RESEARCH.md §3, filterpy/Stone Soup.)
- [ ] **Reactive evaders** — give FPV/loitering threats evasion policies
      (dodge nearest interceptor, altitude drops) so herding pressure has
      something real to push against; today herding only pre-positions a
      second shooter.
- [ ] **CAP station optimisation** — picket placement against historical
      raid axes (today: hand-placed); launch latency was empirically the
      difference between intercepts over fields vs over the city.
- [ ] **Fast-interceptor tier** — the 100 m/s jet OWA is beyond an 80 m/s
      propeller interceptor by physics, as in reality. Add a small number
      of 150+ m/s interceptors with their own cost/availability budget.
- [ ] **Metrics module** — per-run scorecards (attrition by class,
      time-to-intercept distributions, debris cost integral, ammo per
      kill), batch report generation; promote the 10-seed Monte-Carlo from
      ad-hoc script to `coopuavs batch` output.

## Phase 2 — Deepen the two innovation pillars (weeks 3–6)

Cooperation:
- [ ] **Apollonius-circle containment** proper: escape-set computation and
      area-minimising blocker placement against *reactive* evaders
      (RESEARCH.md §1: Garcia/Casbeer/Von Moll/Pachter line of work).
- [ ] **Decentralised allocation (CBBA)** behind the existing
      `allocate(...)` interface; degrade gracefully when the base station
      link drops — the doctrinal argument for UAV autonomy.
- [ ] **MARL benchmark** (MAPPO via PettingZoo wrapper around the sim) vs
      the geometric baseline; publishable comparison either way.

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
      keep the Python world for large-scale Monte-Carlo. The
      `AirframeBody` seam (`sim/physics.py`) is the adapter boundary: a
      `Px4Body` implements the same four members over offboard
      setpoints. Interim step already in place: 3-DOF load-factor body
      with terminal PN behind the same seam.
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
- Enemy drones do not react to interceptors; herding is therefore
  positioning, not coercion.
- Interceptor flight dynamics are 3-DOF load-factor (lateral n_max g,
  terminal PN) — no attitude state or aero coefficients, and the n_max
  values are invented; threats remain point-mass. No terrain occlusion.
- Effector Pk surfaces are plausible inventions, not measured data — no
  public Pk data exists for any C-UAS interceptor (RESEARCH.md §5).
- Battery model is a linear drain; no recovery/rearm cycle (UAVs RTB and
  stay there).
- The live dashboard loads Three.js from a CDN — vendor it for offline
  demo environments.
