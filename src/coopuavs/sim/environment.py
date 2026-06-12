"""Static description of the defended residential area."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from ..core.messages import ZoneClass
from ..risk.zones import RiskMap, derive_zones


@dataclass
class ProtectedAsset:
    """A point the enemy raid is heading for (substation, depot, shelter)."""

    name: str
    position: np.ndarray
    value: float = 1.0
    radius: float = 30.0   # enemy detonation radius


class BuildingKind(str, Enum):
    """City fabric taxonomy (SIM-ENV-004): kind drives the civilian-presence
    zoning (SIM-ENV-005) and the default construction material."""

    RESIDENTIAL_HIGH = "residential_high"   # dense apartment blocks
    RESIDENTIAL_LOW = "residential_low"     # houses, 2-3 floors
    SCHOOL = "school"
    HOSPITAL = "hospital"
    COMMERCIAL = "commercial"               # offices, malls
    INDUSTRIAL = "industrial"               # restricted, civilian-free ground
    PARK = "park"                           # open green, not a solid structure
    WATER = "water"                         # river/lake, not a solid structure


class Material(str, Enum):
    """Construction material — drives per-channel occlusion (SIM-SEN-005)."""

    CONCRETE = "concrete"
    BRICK = "brick"
    GLASS_STEEL = "glass_steel"
    LIGHT_METAL = "light_metal"
    WOOD = "wood"
    NONE = "none"          # parks/water: no solid structure


KIND_DEFAULT_MATERIAL = {
    BuildingKind.RESIDENTIAL_HIGH: Material.CONCRETE,
    BuildingKind.RESIDENTIAL_LOW: Material.BRICK,
    BuildingKind.SCHOOL: Material.CONCRETE,
    BuildingKind.HOSPITAL: Material.CONCRETE,
    BuildingKind.COMMERCIAL: Material.GLASS_STEEL,
    BuildingKind.INDUSTRIAL: Material.LIGHT_METAL,
    BuildingKind.PARK: Material.NONE,
    BuildingKind.WATER: Material.NONE,
}


@dataclass
class Building:
    """Axis-aligned 2.5D box: footprint rect + height.

    ``kind`` classifies the city fabric (SIM-ENV-004), ``material`` the
    construction (occlusion, SIM-SEN-005). Parks and water are modelled as
    flat ``Building`` entries (height ~0, material NONE) so one list
    describes the whole ground fabric.
    """

    rect: tuple[float, float, float, float]   # xmin, ymin, xmax, ymax
    height: float = 15.0
    kind: BuildingKind = BuildingKind.COMMERCIAL
    material: Material | None = None
    name: str = ""

    def __post_init__(self) -> None:
        if self.material is None:
            self.material = KIND_DEFAULT_MATERIAL[self.kind]

    @property
    def solid(self) -> bool:
        """Whether the entry obstructs sight lines at all."""
        return self.material is not Material.NONE and self.height > 0.0


@dataclass
class ChargingStation:
    """An explicit recharge/rearm pad (PHY-CHG-001), sited on a rooftop or
    on the ground adjacent to a building; UAV ``home`` positions resolve
    to a station."""

    station_id: str
    position: np.ndarray                      # pad surface, map frame
    rooftop: bool = False
    capacity: int = 4


@dataclass
class Environment:
    bounds: tuple[float, float, float, float]
    risk_map: RiskMap
    assets: list[ProtectedAsset] = field(default_factory=list)
    buildings: list[Building] = field(default_factory=list)
    stations: list[ChargingStation] = field(default_factory=list)

    def station(self, station_id: str) -> ChargingStation:
        for st in self.stations:
            if st.station_id == station_id:
                return st
        raise KeyError(f"unknown charging station '{station_id}'")

    @classmethod
    def from_config(cls, cfg: dict) -> "Environment":
        bounds = tuple(cfg["bounds"])
        zone_source = cfg.get("zone_source", "rects")
        # Building-derived zoning starts from civilian-free ground and adds
        # presence where buildings imply it; legacy rect zoning keeps the
        # conservative all-DANGEROUS default.
        default_zone = ZoneClass[
            cfg.get("default_zone", "SAFE" if zone_source == "buildings" else "DANGEROUS")
        ]
        rm = RiskMap(bounds, cell_size=cfg.get("cell_size", 50.0), default=default_zone)
        buildings = [
            Building(
                rect=tuple(b["rect"]),
                height=b.get("height", 15.0),
                kind=BuildingKind(b.get("kind", "commercial")),
                material=Material(b["material"]) if b.get("material") else None,
                name=b.get("name", ""),
            )
            for b in cfg.get("buildings", [])
        ]
        if zone_source == "buildings":
            derive_zones(rm, buildings)
        # Hand-painted rects: the whole story for legacy scenarios, manual
        # overrides on top of the derived raster otherwise.
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
        stations = [
            ChargingStation(
                station_id=s["id"],
                position=np.array(s["pos"], dtype=float),
                rooftop=s.get("rooftop", False),
                capacity=s.get("capacity", 4),
            )
            for s in cfg.get("charging_stations", [])
        ]
        return cls(bounds=bounds, risk_map=rm, assets=assets,
                   buildings=buildings, stations=stations)
