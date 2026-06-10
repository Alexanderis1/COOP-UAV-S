import numpy as np

from coopuavs.core.messages import EffectorType, ZoneClass
from coopuavs.risk.debris import DebrisModel
from coopuavs.risk.zones import RiskMap


def make_map() -> RiskMap:
    rm = RiskMap((-1000, -1000, 1000, 1000), cell_size=50.0, default=ZoneClass.SAFE)
    rm.set_rect((-200, -200, 200, 200), ZoneClass.CRITICAL)
    rm.set_rect((300, 300, 600, 600), ZoneClass.DANGEROUS)
    return rm


def test_zone_lookup():
    rm = make_map()
    assert rm.zone_at(0, 0) == ZoneClass.CRITICAL
    assert rm.zone_at(450, 450) == ZoneClass.DANGEROUS
    assert rm.zone_at(-800, -800) == ZoneClass.SAFE


def test_collateral_cost_orders_zones():
    rm = make_map()
    safe = rm.collateral_cost(np.array([[-800.0, -800.0]]))
    dang = rm.collateral_cost(np.array([[450.0, 450.0]]))
    crit = rm.collateral_cost(np.array([[0.0, 0.0]]))
    assert safe < dang < crit


def test_nearest_safe_cell_avoids_critical():
    rm = make_map()
    cell = rm.nearest_safe_cell(0.0, 0.0)
    assert rm.zone_at(cell[0], cell[1]) == ZoneClass.SAFE


def test_debris_net_drops_shorter_than_projectile():
    rng = np.random.default_rng(1)
    model = DebrisModel(rng, n_samples=2000)
    pos = np.array([0.0, 0.0, 500.0])
    vel = np.array([55.0, 0.0, 0.0])
    net = model.footprint(pos, vel, EffectorType.NET)
    proj = model.footprint(pos, vel, EffectorType.PROJECTILE)
    # A netted target mostly drops; a shot one carries forward.
    assert np.mean(net[:, 0]) < np.mean(proj[:, 0])


def test_debris_spread_grows_with_altitude():
    rng = np.random.default_rng(2)
    model = DebrisModel(rng, n_samples=2000)
    vel = np.array([55.0, 0.0, 0.0])
    low = model.footprint(np.array([0.0, 0.0, 100.0]), vel, EffectorType.NET)
    high = model.footprint(np.array([0.0, 0.0, 2000.0]), vel, EffectorType.NET)
    assert np.std(high[:, 1]) > np.std(low[:, 1])
