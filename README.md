# COOP-UAV-S
Cooperative UAVs + Base System to handle Drones swarms in crytical environments

> A research and engineering framework for the design, simulation, and validation of an articulated, AI-assisted counter-UAS (C-UAS) system operating across multiple physical and cyber domains in contested urban and semi-urban environments.

---

COOP-UAV-S is a systems engineering and research project aimed at building a modular, multi-domain system for counter-drone defence in complex, civilian-populated environments.

The system is based on cooperative UAVs deployment + Base Station and designed around a real-world operational baseline derived from the Ukrainian theatre of operations (2022–2026), the first large-scale conflict where drone consumption rivals artillery shell expenditure as a logistics challenge, and where adversarial drone tactics evolve faster than any single defensive technology can adapt.

## Operational Context & Threat Taxonomy

### Threat Classes

The system is designed around three operationally validated drone threat classes:

| Class | Example | Mass | Speed | Altitude (AGL) | Behaviour |
|-------|---------|------|-------|----------------|-----------|
| A — Strategic OWA | Shahed-136 / Geran-2 | ~200 kg | 50–65 m/s | 50 m – 5 km (adaptive) | Swarm saturation, decoy mixing, terminal dive |
| A+ — Jet OWA | Geran-3 / Shahed-238 | ~200 kg | ~103 m/s | 2–5 km | High-speed approach, low intercept window |
| B — Tactical FPV | Quadcopter kamikaze | 1–5 kg | 30–40 m/s | 0–200 m | Agile, fiber-optic guided (jam-resistant), human-in-loop or autonomous |
| C — Loitering Munition | Lancet-3 | 12 kg | ~80 m/s | 50–500 m | AI-guided terminal seeker, precision strike |

### Key Environmental Constraints

```
Temperature:     -25°C to +45°C (operational); battery degradation below -10°C
Wind:            Up to 20 m/s operational ceiling for small drones
Precipitation:   Rain, snow, dense fog — all-weather operation required
Illumination:    Primarily nocturnal operations; thermal imaging dominant
Engagement zone: Urban / peri-urban, mixed civilian/military structures
Altitude band:   50 m – 5,000 m AGL (multi-layer coverage required)
Attack density:  Up to 400+ drones/night over a metropolitan area
```

### Tactical Patterns Addressed

- **Altitude switching:** drones adapt flight profile (low-level terrain masking vs. high-altitude dive) based on detected defences
- **Saturation attacks:** 24-hour attack cycles designed to exhaust missile reserves
- **Decoy integration:** unarmed replicas with identical radar/visual signatures mixed into strike packages
- **Fiber-optic FPV:** radio-jamming-immune, human-piloted short-range munitions
- **Two-phase strikes:** secondary hit timed to target first responders

---

## Draft System Architecture

Note that in the architecture we still need to understand how to implement UAVs cooperation, orchestration and maneouvers management.

The infrastructure will need to specify exactly the responsabilities and capability of the base station with respect to the single drones.

Drones and base station hardware is yet to be decided.

Engagement will include new engagement tactics that do not use interceptors or jamming, like projectiles, nets or something else to reduce the costs.

```
┌─────────────────────────────────────────────────────────────┐
│                    C2 / Fusion Layer                        │
│         (multi-sensor correlation, threat grading,          │
│          ROE enforcement, engagement authorisation)         │
├──────────────────────────┬──────────────────────────────────┤
│   Detection Layer        │    Classification Layer          │
│   - Radar (short/mid)    │    - Deep learning (EO/IR)       │
│   - Acoustic signature   │    - Adversarial-robust models   │
│   - EO/IR cameras        │    - Decoy discrimination        │
│   - RF spectrum scan     │    - Behaviour-based filtering   │
├──────────────────────────┴──────────────────────────────────┤
│                    Tracking Layer                           │
│     (multi-object tracking, trajectory prediction,          │
│      Kalman filtering, ByteTrack / BoT-SORT variants)       │
├─────────────────────────────────────────────────────────────┤
│                  Engagement Layer (Kinetic / Non-Kinetic)   │
│    - EW / jamming module    - Interceptor drone dispatch    │
│    - Directed energy stub   - Net/collision systems         │
├─────────────────────────────────────────────────────────────┤
│                Infrastructure & Simulation                  │
│        ROS 2 / Gazebo sim │ SIL test harness │ HIL stubs    │
└─────────────────────────────────────────────────────────────┘
```
