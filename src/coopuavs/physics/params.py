"""Airframe parameter file loader.

Airframe YAMLs live in ``coopuavs/physics/params/`` and are flagged
invented-but-self-consistent (no public data exists for these classes);
their numbers are pinned by the trim/terminal/envelope tests, so editing a
param file intentionally breaks tests until re-pinned.
"""

from __future__ import annotations

from pathlib import Path

import yaml

_PARAMS_DIR = Path(__file__).parent / "params"


def load_airframe(name: str | Path) -> dict:
    """Load an airframe parameter dict by short name or explicit path."""
    path = Path(name)
    if path.suffix not in (".yaml", ".yml"):
        path = _PARAMS_DIR / f"{name}.yaml"
    return yaml.safe_load(path.read_text())
