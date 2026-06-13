"""Re-export shim: patrol geometry moved to ``mc/patrol.py`` (the MC apps
own the tactical math; keeping it under ``coopuavs.mc`` also respects the MC
import fence). Legacy ``coopuavs.interceptors.patrol`` imports stay valid."""

from ..mc.patrol import (  # noqa: F401
    PATTERNS,
    loop_length,
    orbit_waypoint,
    path_offset,
)
