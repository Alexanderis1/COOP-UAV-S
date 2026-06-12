"""Building LOS occlusion (SIM-SEN-005 / SIM-EFF-006)."""


from coopuavs.sim.environment import Building, BuildingKind, Material
from coopuavs.sim.occlusion import OcclusionGrid

BOUNDS = (-1000.0, -1000.0, 1000.0, 1000.0)


def grid(*buildings, enabled=True):
    return OcclusionGrid(list(buildings), BOUNDS, enabled=enabled)


def wall(kind=BuildingKind.RESIDENTIAL_HIGH, height=30.0):
    return Building(rect=(-50.0, -50.0, 50.0, 50.0), height=height, kind=kind)


def test_through_wall_blocked_over_roof_clear():
    occ = grid(wall())
    assert occ.transmittance([-200, 0, 10], [200, 0, 10], "eo_ir") == 0.0
    assert occ.transmittance([-200, 0, 60], [200, 0, 80], "eo_ir") == 1.0
    assert occ.clear([-200, 0, 60], [200, 0, 80])
    assert not occ.clear([-200, 0, 10], [200, 0, 10])


def test_beside_footprint_clear():
    occ = grid(wall())
    assert occ.transmittance([-200, 200, 10], [200, 200, 10], "eo_ir") == 1.0


def test_material_channel_transmittance():
    concrete = grid(wall(BuildingKind.RESIDENTIAL_HIGH))   # concrete default
    industrial = grid(wall(BuildingKind.INDUSTRIAL))       # light metal
    p0, p1 = [-200, 0, 10], [200, 0, 10]
    assert concrete.transmittance(p0, p1, "radar") == 0.0
    assert industrial.transmittance(p0, p1, "radar") == 0.35
    assert industrial.transmittance(p0, p1, "rf") == 0.5
    # acoustic diffracts at a flat per-crossing factor
    assert concrete.transmittance(p0, p1, "acoustic") == 0.6


def test_park_and_water_never_obstruct():
    park = Building(rect=(-50, -50, 50, 50), height=0.0, kind=BuildingKind.PARK)
    occ = grid(park)
    assert occ.transmittance([-200, 0, 5], [200, 0, 5], "eo_ir") == 1.0


def test_mount_host_exemption():
    """A sensor placed inside its host footprint below the roof is mounted
    ON the structure: the host does not occlude its own instrument."""
    occ = grid(wall())
    assert occ.clear([0, 0, 6], [400, 0, 50])        # from inside the host
    assert not occ.clear([400, 0, 6], [0, 0, 6])     # toward a target inside


def test_multiple_crossings_multiply():
    b1 = Building(rect=(-300, -50, -200, 50), height=30.0, kind=BuildingKind.INDUSTRIAL)
    b2 = Building(rect=(200, -50, 300, 50), height=30.0, kind=BuildingKind.INDUSTRIAL)
    occ = grid(b1, b2)
    t = occ.transmittance([-500, 0, 10], [500, 0, 10], "radar")
    assert abs(t - 0.35 * 0.35) < 1e-9


def test_disabled_restores_v01():
    occ = grid(wall(), enabled=False)
    assert occ.transmittance([-200, 0, 10], [200, 0, 10], "eo_ir") == 1.0
    assert occ.clear([-200, 0, 10], [200, 0, 10])


def test_deterministic_pure_geometry():
    occ = grid(wall())
    vals = {occ.transmittance([-200, 33, 12], [200, -41, 18], "rf") for _ in range(5)}
    assert len(vals) == 1


def test_diagonal_dda_finds_offset_building():
    b = Building(rect=(380, 380, 480, 480), height=60.0,
                 kind=BuildingKind.RESIDENTIAL_HIGH, material=Material.CONCRETE)
    occ = grid(b)
    assert not occ.clear([0, 0, 10], [800, 800, 10])
