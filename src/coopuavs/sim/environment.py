"""Static description of the defended residential area."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..core.messages import ZoneClass
from ..risk.zones import RiskMap


@dataclass
class ProtectedAsset:
    """A point the enemy raid is heading for (substation, depot, shelter)."""

    name: str
    position: np.ndarray
    value: float = 1.0
    radius: float = 30.0   # enemy detonation radius


@dataclass
class Building:
    """Axis-aligned box, only for visualisation and occlusion later."""

    rect: tuple[float, float, float, float]   # xmin, ymin, xmax, ymax
    height: float = 15.0


@dataclass
class Environment:
    bounds: tuple[float, float, float, float]
    risk_map: RiskMap
    assets: list[ProtectedAsset] = field(default_factory=list)
    buildings: list[Building] = field(default_factory=list)

    @classmethod
    def from_config(cls, cfg: dict) -> "Environment":
        bounds = tuple(cfg["bounds"])
        rm = RiskMap(
            bounds,
            cell_size=cfg.get("cell_size", 50.0),
            default=ZoneClass[cfg.get("default_zone", "DANGEROUS")],
        )
        for z in cfg.get("zones", []):
            rm.set_rect(tuple(z["rect"]), ZoneClass[z["class"]])
        assets = [
            ProtectedAsset(
                name=a["name"],
                position=np.array(a["position"], dtype=float),
                value=a.get("value", 1.0),
                radius=a.get("radius", 30.0),
            )
            for a in cfg.get("assets", [])
        ]
        buildings = [
            Building(rect=tuple(b["rect"]), height=b.get("height", 15.0))
            for b in cfg.get("buildings", [])
        ]
        return cls(bounds=bounds, risk_map=rm, assets=assets, buildings=buildings)
