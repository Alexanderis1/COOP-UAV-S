"""Software-in-the-loop runtime: virtual clocks, hosted flight software.

Foundations for SIM-SIL-001..003 — the fleet SITL engine plugs into the
world through `World.micro` using the clock machinery in `clock.py`.
P3-8 adds `bench.py` (one vehicle: physics + hw devices + one FCU, no
tactical stack); P4 adds `vehicle.py`/`fleet.py`.
"""
