"""Hardware device models (Problem 1, P2): imu, gps, baro, mag,
seeker_gimbal, esc_telem.

Standalone package like ``physics/`` — no imports from sim/, threats/,
sensors/ or risk/; the SITL world wiring arrives in P4 (micro-tick phase 1,
"devices sample truth", docs/ORDERING.md section 6). Every model is batched
over vehicles (leading axis n) and source-traceable per equation
(docs/RESEARCH.md, section "P2 hardware device models").

Conventions (frozen, inherited from physics/):
- World frame ENU z-up; body frame FLU; Hamilton scalar-first quaternion
  [w, x, y, z], body -> world. SI units unless a field name says otherwise
  (e.g. magnetometer microtesla ``_ut``, ESC telemetry ``rpm``).
- RNG: each device model takes one injected parent ``np.random.Generator``
  (a named registry stream, e.g. ``sensor/imu``) and spawns one child per
  vehicle (the Dryden/P0 contract: growing the fleet leaves existing
  vehicles' draw histories identical). Devices always consume the same
  number of draws per tick regardless of which noise terms are enabled, so
  re-tuning one sigma never shifts another consumer's stream.
- Devices are clocked externally at exactly their configured rate
  (P4 RateGroupScheduler); rates and latencies must come out as exact
  integer tick counts — a rate pairing that doesn't is a scenario error,
  never rounded silently (the sil/clock.py rule).
"""
