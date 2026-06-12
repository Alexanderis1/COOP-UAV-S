"""Device-suite parameter file loader (mirrors physics/params.py).

Suites live in ``coopuavs/hw/params/`` — one YAML per airframe sensor fit,
flagged invented-but-representative (magnitudes sanity-checked against the
public MEMS/GNSS/baro/mag device classes named in the file comments, not
copied from any datasheet); their numbers are pinned by the hw tests, so
editing a file intentionally breaks pins until re-baselined.
"""

from __future__ import annotations

from pathlib import Path

import yaml

_PARAMS_DIR = Path(__file__).parent / "params"


def load_devices(name: str | Path) -> dict:
    """Load a device-suite parameter dict by short name or explicit path."""
    path = Path(name)
    if path.suffix not in (".yaml", ".yml"):
        path = _PARAMS_DIR / f"{name}.yaml"
    return yaml.safe_load(path.read_text())
