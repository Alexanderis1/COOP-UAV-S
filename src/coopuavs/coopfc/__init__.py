"""CoopFC — the in-repo flight control software (Problem 1, P3).

This package is the code that "runs on" each fleet UAV's flight control
unit inside a VirtualMCU. It is import-fenced: nothing from the simulator
(sim/, threats/, sensors/, risk/, physics/, hw/, sil/, core/, ...) may be
imported here — the FCU receives params, a clock, RNG and ports through
its constructor and can touch nothing else (tests/test_coopfc_fence.py).
numpy is allowed only under estimation/ (50 Hz); every >=100 Hz path is
plain-float (perf budget: no numpy/allocation in hot loops).

Conventions match the frozen P1 physics contract by value (not by
import): world ENU z-up, body FLU, Hamilton unit quaternion scalar-first
[w, x, y, z], body -> world, SI units.
"""

GRAVITY = 9.81  # m/s^2 — flight software's own constant; equals physics.GRAVITY by design
