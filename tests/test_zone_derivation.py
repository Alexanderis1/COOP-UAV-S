"""Civilian-presence zoning derived from building kinds (SIM-ENV-005)."""

import numpy as np

from coopuavs.core.messages import ZoneClass
from coopuavs.sim.environment import Environment

BASE = {
    "bounds": [-2000.0, -2000.0, 2000.0, 2000.0],
    "cell_size": 50.0,
    "zone_source": "buildings",
}


def env_with(buildings, **extra):
    cfg = dict(BASE)
    cfg["buildings"] = buildings
    cfg.update(extra)
    return Environment.from_config(cfg)


def test_hospital_core_and_buffer_critical():
    env = env_with([{"rect": [-100, -100, 100, 100], "height": 25, "kind": "hospital"}])
    assert env.risk_map.zone_at(0, 0) == ZoneClass.CRITICAL
    assert env.risk_map.zone_at(180, 0) == ZoneClass.CRITICAL      # +100 m buffer
    assert env.risk_map.zone_at(0, 250) == ZoneClass.DANGEROUS     # outer halo
    assert env.risk_map.zone_at(0, 1500) == ZoneClass.SAFE         # open ground


def test_park_safe_residential_buffer_dangerous():
    env = env_with([
        {"rect": [-500, -500, -300, -300], "height": 10, "kind": "residential_low"},
        {"rect": [300, 300, 500, 500], "height": 0, "kind": "park"},
    ])
    assert env.risk_map.zone_at(-400, -400) == ZoneClass.DANGEROUS
    assert env.risk_map.zone_at(-400, -260) == ZoneClass.DANGEROUS  # +60 m buffer
    assert env.risk_map.zone_at(400, 400) == ZoneClass.SAFE


def test_critical_wins_overlap():
    env = env_with([
        {"rect": [-100, -100, 100, 100], "height": 40, "kind": "residential_high"},
        {"rect": [60, -100, 260, 100], "height": 0, "kind": "park"},
    ])
    # dense-residential CRITICAL core paints last: wins over the park edge
    assert env.risk_map.zone_at(90, 0) == ZoneClass.CRITICAL


def test_industrial_is_civilian_free():
    env = env_with([{"rect": [-200, -200, 200, 200], "height": 12, "kind": "industrial"}])
    assert env.risk_map.zone_at(0, 0) == ZoneClass.SAFE


def test_manual_rect_overrides_derived():
    env = env_with(
        [{"rect": [-100, -100, 100, 100], "height": 25, "kind": "hospital"}],
        zones=[{"rect": [-2000, -2000, -1500, -1500], "class": "CRITICAL"}],
    )
    assert env.risk_map.zone_at(-1700, -1700) == ZoneClass.CRITICAL


def test_legacy_rects_unchanged():
    cfg = {
        "bounds": [-1000.0, -1000.0, 1000.0, 1000.0],
        "cell_size": 50.0,
        "zones": [{"rect": [-1000, -1000, 0, 0], "class": "SAFE"}],
        "buildings": [{"rect": [100, 100, 300, 300], "height": 20}],
    }
    env = Environment.from_config(cfg)
    # default stays DANGEROUS, buildings paint nothing in rects mode
    assert env.risk_map.zone_at(-500, -500) == ZoneClass.SAFE
    assert env.risk_map.zone_at(200, 200) == ZoneClass.DANGEROUS
    assert env.risk_map.zone_at(500, -500) == ZoneClass.DANGEROUS
