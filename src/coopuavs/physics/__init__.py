"""High-fidelity vectorized physics core (Problem 1, P1).

Standalone package: no imports from sim/, threats/, sensors/ or risk/ — the
world adapters arrive in P4/P6. Every model is batched over vehicles
(leading axis N) and source-traceable per equation (docs/RESEARCH.md).

Conventions (frozen, see docs/ORDERING.md for the macro/micro step contract):
- World frame: ENU, z up, metres / seconds / kilograms / radians.
- Body frame: FLU (x forward, y left, z up).
- Attitude: Hamilton unit quaternion, scalar-first [w, x, y, z], body -> world.
"""
