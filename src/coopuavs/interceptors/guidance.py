"""Re-export shim: guidance moved to ``mc/guidance.py`` (PLAN_PROBLEM1
P4-3 — the MC apps own the tactical math; legacy imports stay valid)."""

from ..mc.guidance import (  # noqa: F401
    goto_velocity,
    intercept_time,
    predicted_intercept_point,
    pursuit_velocity,
    terminal_pursuit_velocity,
)
