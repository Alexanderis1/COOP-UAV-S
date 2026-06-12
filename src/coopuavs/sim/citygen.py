"""Deterministic city-scenario generator (SIM-ENV-006).

Lays out a realistic peri-urban defended area on a street grid: a river
through the centre, a dense residential core mixed with a commercial
spine, a low-residential ring, parks, one hospital, two schools and a
restricted industrial district hosting the protected assets. Emits a
complete scenario dict — environment (with ``zone_source: buildings``),
charging stations, sensor laydown, fleet (interceptors + sentinels),
turrets and the threat raid — ready to be dumped to YAML.

The generator is a pure function of the seed: the same seed always emits
the same scenario (SIM-003). ``scenarios/urban_raid.yaml`` is the checked-
in output of ``coopuavs citygen --seed 7``.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

# Block lattice: 230 m blocks separated by 70 m street corridors.
_BLOCK_STEP = 300.0
_BLOCK_HALF = 115.0
_CORE_R = 1300.0          # dense residential / commercial core
_RING_R = 2700.0          # low-residential ring
_RIVER_HALF = 150.0       # river half-width (y = 0 axis)
_RIVER_BANK = 120.0       # unbuilt banks on each side

_HEIGHTS = {
    "residential_high": (30.0, 70.0),
    "residential_low": (7.0, 14.0),
    "commercial": (20.0, 55.0),
    "industrial": (8.0, 16.0),
    "school": (10.0, 14.0),
    "hospital": (24.0, 32.0),
}


def _rect(cx: float, cy: float, hw: float, hh: float) -> list[float]:
    return [round(float(cx - hw), 1), round(float(cy - hh), 1),
            round(float(cx + hw), 1), round(float(cy + hh), 1)]


def _building(rng, cx, cy, kind, hw=None, hh=None, name="") -> dict:
    hw = hw if hw is not None else _BLOCK_HALF * float(rng.uniform(0.78, 0.95))
    hh = hh if hh is not None else _BLOCK_HALF * float(rng.uniform(0.78, 0.95))
    lo, hi = _HEIGHTS[kind]
    b = {"rect": _rect(cx, cy, hw, hh), "height": round(float(rng.uniform(lo, hi)), 1),
         "kind": kind}
    if name:
        b["name"] = name
    return b


def _city_fabric(rng) -> list[dict]:
    """The block lattice: kind chosen by radius, with parks sprinkled in."""
    buildings: list[dict] = [
        # The river crosses the whole map west-east through the centre.
        {"rect": [-6000.0, -_RIVER_HALF, 6000.0, _RIVER_HALF], "height": 0.0,
         "kind": "water", "name": "river"},
    ]
    # Landmark civic blocks claim their lattice cells first.
    landmarks = [
        _building(rng, 600.0, 1200.0, "hospital", hw=140.0, hh=110.0, name="city-hospital"),
        _building(rng, -900.0, -600.0, "school", hw=110.0, hh=90.0, name="school-south"),
        _building(rng, 1500.0, 2100.0, "school", hw=110.0, hh=90.0, name="school-north"),
    ]
    taken = [b["rect"] for b in landmarks]
    # Two designed parks flanking the core.
    parks = [
        {"rect": _rect(-1500.0, 900.0, 320.0, 260.0), "height": 0.0,
         "kind": "park", "name": "west-park"},
        {"rect": _rect(1800.0, -1200.0, 300.0, 240.0), "height": 0.0,
         "kind": "park", "name": "east-park"},
    ]
    taken += [p["rect"] for p in parks]
    buildings += landmarks + parks

    coords = np.arange(-_RING_R + _BLOCK_STEP / 2.0, _RING_R, _BLOCK_STEP)
    for cy in coords:
        if abs(cy) < _RIVER_HALF + _RIVER_BANK + _BLOCK_HALF:
            continue   # river corridor and its banks stay unbuilt
        for cx in coords:
            r = float(np.hypot(cx, cy))
            if r > _RING_R:
                continue
            if any(not (cx + _BLOCK_HALF < t[0] or cx - _BLOCK_HALF > t[2]
                        or cy + _BLOCK_HALF < t[1] or cy - _BLOCK_HALF > t[3])
                   for t in taken):
                continue   # landmark/park already occupies this cell
            roll = float(rng.random())
            if roll < 0.07:
                buildings.append({"rect": _rect(cx, cy, 100.0, 100.0),
                                  "height": 0.0, "kind": "park"})
                continue
            if abs(cx) < 260.0:
                kind = "commercial"             # north-south commercial spine
            elif r < _CORE_R:
                kind = "residential_high" if roll < 0.8 else "commercial"
            else:
                kind = "residential_low" if roll < 0.9 else "commercial"
            buildings.append(_building(rng, cx, cy, kind))

    # Restricted industrial district (civilian-free) south-east, hosting
    # the substation; warehouses on their own coarser lattice.
    for iy, cy in enumerate(np.arange(-4500.0, -3000.0, 480.0)):
        for ix, cx in enumerate(np.arange(3300.0, 5200.0, 520.0)):
            buildings.append(_building(rng, cx, cy, "industrial",
                                       hw=190.0, hh=160.0,
                                       name=f"warehouse-{iy * 4 + ix + 1}"))
    return buildings


def generate(seed: int = 7) -> dict:
    rng = np.random.default_rng(seed)
    buildings = _city_fabric(rng)

    # Rooftop pads go on sturdy flat-roof stock: the tallest warehouse and
    # two commercial blocks nearest the base; ground pads sit next to
    # buildings on the southern approach where the GCS lives.
    def _roof(b):
        r = b["rect"]
        return [round((r[0] + r[2]) / 2, 1), round((r[1] + r[3]) / 2, 1), b["height"]]

    commercial = [b for b in buildings if b["kind"] == "commercial"]
    south_comm = sorted(commercial, key=lambda b: (b["rect"][1] + b["rect"][3]) / 2)[:2]
    warehouses = [b for b in buildings if b["kind"] == "industrial"]
    tall_wh = max(warehouses, key=lambda b: b["height"])
    stations = [
        {"id": "cs-base-1", "pos": [-450.0, -3250.0, 0.0], "rooftop": False, "capacity": 6},
        {"id": "cs-base-2", "pos": [450.0, -3250.0, 0.0], "rooftop": False, "capacity": 6},
        {"id": "cs-comm-1", "pos": _roof(south_comm[0]), "rooftop": True, "capacity": 5},
        {"id": "cs-comm-2", "pos": _roof(south_comm[1]), "rooftop": True, "capacity": 5},
        {"id": "cs-ind-1", "pos": _roof(tall_wh), "rooftop": True, "capacity": 4},
        {"id": "cs-north-1", "pos": [-2950.0, 2950.0, 0.0], "rooftop": False, "capacity": 4},
    ]

    interceptors = []
    # 14 projectile-gun hawks and 6 net carriers spread over the stations.
    hawk_stations = ["cs-base-1", "cs-base-2", "cs-comm-1", "cs-comm-2",
                     "cs-ind-1", "cs-north-1"]
    for i in range(14):
        interceptors.append({"id": f"hawk-{i + 1}",
                             "station": hawk_stations[i % len(hawk_stations)],
                             "effector": "projectile", "max_speed": 80.0})
    net_stations = ["cs-base-1", "cs-base-2", "cs-comm-1", "cs-comm-2"]
    for i in range(6):
        interceptors.append({"id": f"net-{i + 1}",
                             "station": net_stations[i % len(net_stations)],
                             "effector": "net", "max_speed": 50.0})

    # 10 sentinels on a ring of patrol orbits covering the approaches and
    # the rooftop-masked core volumes.
    sentinels = []
    sent_stations = ["cs-base-1", "cs-base-2", "cs-comm-1", "cs-comm-2",
                     "cs-ind-1", "cs-north-1"]
    for i in range(10):
        ang = np.deg2rad(i * 36.0)
        c = 3300.0 * np.array([np.sin(ang), np.cos(ang)])
        sentinels.append({
            "id": f"sent-{i + 1}",
            "station": sent_stations[i % len(sent_stations)],
            "orbit": {"center": [round(float(c[0]), 1), round(float(c[1]), 1)],
                      "radius": 800.0, "alt": 350.0, "speed": 25.0},
        })

    cfg = {
        "name": "urban_raid",
        "seed": seed,
        "dt": 0.05,
        "duration": 600.0,
        "record_hz": 5.0,
        "weather": {"wind_speed": 4.0, "wind_dir_deg": 250.0, "fog": 0.1,
                    "precip": 0.0, "daylight": 0.25},
        "comms": {"latency_s": 0.01, "jitter_s": 0.002, "base_loss": 0.005,
                  "loss_per_km": 0.002, "base_pos": [0.0, -3200.0, 0.0]},
        "occlusion": {"enabled": True},
        "environment": {
            "bounds": [-6000.0, -6000.0, 6000.0, 6000.0],
            "cell_size": 50.0,
            "zone_source": "buildings",
            "assets": [
                {"name": "substation", "position": [4150.0, -3750.0, 0.0],
                 "value": 1.0, "radius": 40.0},
                {"name": "depot", "position": [-2950.0, 3100.0, 0.0],
                 "value": 0.8, "radius": 40.0},
            ],
            "buildings": buildings,
            "charging_stations": stations,
        },
        "base_station": {
            "debris_policy": {"engage_zones": ["CRITICAL", "DANGEROUS"]},
        },
        "sensors": [
            {"type": "radar", "name": "radar-main",
             "position": [0.0, -3200.0, 18.0], "max_range": 12000.0},
            {"type": "rf", "name": "rf-df",
             "position": [0.0, -3200.0, 18.0], "max_range": 15000.0},
            {"type": "eo_ir", "name": "eo-base", "position": [0.0, -3200.0, 25.0]},
            {"type": "eo_ir", "name": "eo-substn", "position": [4150.0, -3750.0, 22.0]},
            {"type": "eo_ir", "name": "eo-depot", "position": [-2950.0, 3100.0, 22.0]},
            {"type": "eo_ir", "name": "eo-hospital", "position": [780.0, 1200.0, 35.0]},
            {"type": "acoustic", "name": "ac-w1", "position": [-4500.0, -1000.0, 5.0]},
            {"type": "acoustic", "name": "ac-w2", "position": [-4000.0, 800.0, 5.0]},
            {"type": "acoustic", "name": "ac-n1", "position": [-1500.0, 4200.0, 5.0]},
            {"type": "acoustic", "name": "ac-n2", "position": [1500.0, 4200.0, 5.0]},
            {"type": "acoustic", "name": "ac-e1", "position": [4200.0, 500.0, 5.0]},
            {"type": "acoustic", "name": "ac-s1", "position": [0.0, -4500.0, 5.0]},
        ],
        "interceptors": interceptors,
        "sentinels": sentinels,
        "turrets": [
            {"id": "turret-base", "position": [0.0, -3400.0, 6.0]},
            {"id": "turret-substn", "position": [4350.0, -3550.0, 6.0]},
            {"id": "turret-north", "position": [-300.0, 3300.0, 6.0]},
        ],
        "threats": [
            # wave 1 — strategic OWAs with decoy mixing from the north-west
            {"time": 10.0, "class": "OWA_STRATEGIC", "spawn": [-5800.0, 5600.0, 1400.0], "target": "substation"},
            {"time": 16.0, "class": "DECOY", "spawn": [-5600.0, 5800.0, 1450.0], "target": "substation"},
            {"time": 24.0, "class": "OWA_STRATEGIC", "spawn": [-5900.0, 5200.0, 1350.0], "target": "depot"},
            {"time": 31.0, "class": "OWA_STRATEGIC", "spawn": [-5500.0, 5900.0, 1500.0], "target": "substation"},
            {"time": 38.0, "class": "DECOY", "spawn": [-5700.0, 5400.0, 1400.0], "target": "depot"},
            # wave 2 — low-level penetration from the west
            {"time": 70.0, "class": "LOITERING", "spawn": [-5900.0, 600.0, 420.0], "target": "substation"},
            {"time": 80.0, "class": "FPV", "spawn": [-5800.0, -700.0, 90.0], "target": "depot"},
            {"time": 88.0, "class": "FPV", "spawn": [-5900.0, 300.0, 70.0], "target": "substation"},
            {"time": 96.0, "class": "LOITERING", "spawn": [-5800.0, -300.0, 380.0], "target": "depot"},
            # wave 3 — jets from due north, short window
            {"time": 130.0, "class": "OWA_JET", "spawn": [-600.0, 5900.0, 3000.0], "target": "substation"},
            {"time": 142.0, "class": "OWA_JET", "spawn": [600.0, 5900.0, 3000.0], "target": "depot"},
            # wave 4 — second strategic push with a decoy from the north-east
            {"time": 180.0, "class": "OWA_STRATEGIC", "spawn": [5600.0, 5700.0, 1400.0], "target": "depot"},
            {"time": 188.0, "class": "DECOY", "spawn": [5800.0, 5500.0, 1450.0], "target": "substation"},
            {"time": 196.0, "class": "OWA_STRATEGIC", "spawn": [5400.0, 5900.0, 1500.0], "target": "substation"},
        ],
    }
    return cfg


def write_yaml(cfg: dict, path: str | Path) -> Path:
    path = Path(path)
    header = (
        "# Generated by `coopuavs citygen --seed {seed}` (SIM-ENV-006).\n"
        "# Realistic urban scenario: building-typed city fabric, zones\n"
        "# derived from building kinds (zone_source: buildings), charging\n"
        "# stations, 20 interceptors + 10 sentinels, 3 turrets.\n"
    ).format(seed=cfg["seed"])
    path.write_text(header + yaml.safe_dump(cfg, sort_keys=False, width=100),
                    encoding="utf-8")
    return path
