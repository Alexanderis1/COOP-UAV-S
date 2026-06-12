"""Re-export shim: cooperation moved to ``mc/cooperation.py``
(PLAN_PROBLEM1 P4-3 — the MC apps own the tactical math)."""

from ..mc.cooperation import (  # noqa: F401
    catchable,
    cutoff_points,
    herding_post,
)
