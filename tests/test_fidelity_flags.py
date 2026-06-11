"""P0-5: scenario `fidelity` flags (parse + validate, no behavior yet).

`fidelity: {fleet: pointmass|sitl, threats: pointmass|sixdof}` defaults to
pointmass/pointmass. The non-default modes are declared but land later
(fleet sitl in P4, threats sixdof in P6), so today they must refuse loudly
— NotImplementedError, never a silent fallback. Bad values and unknown
sub-keys are scenario errors at build time.
"""

from __future__ import annotations

import copy

import pytest

from coopuavs.sim import scenario as scenario_mod
from test_end_to_end import SMALL_SCENARIO


def _cfg(fidelity=None):
    cfg = copy.deepcopy(SMALL_SCENARIO)
    if fidelity is not None:
        cfg["fidelity"] = fidelity
    return cfg


def test_default_is_pointmass_pointmass():
    sc = scenario_mod.build(_cfg())
    assert sc.meta["fidelity"] == {"fleet": "pointmass", "threats": "pointmass"}


def test_explicit_pointmass_builds():
    sc = scenario_mod.build(_cfg({"fleet": "pointmass", "threats": "pointmass"}))
    assert sc.meta["fidelity"] == {"fleet": "pointmass", "threats": "pointmass"}


def test_partial_block_fills_defaults():
    sc = scenario_mod.build(_cfg({"fleet": "pointmass"}))
    assert sc.meta["fidelity"]["threats"] == "pointmass"


def test_fleet_sitl_refuses_loudly_for_now():
    with pytest.raises(NotImplementedError, match="sitl"):
        scenario_mod.build(_cfg({"fleet": "sitl"}))


def test_threats_sixdof_refuses_loudly_for_now():
    with pytest.raises(NotImplementedError, match="sixdof"):
        scenario_mod.build(_cfg({"threats": "sixdof"}))


@pytest.mark.parametrize("block", [
    {"fleet": "fancy"},
    {"threats": "pointless"},
    {"fleet": 1},
])
def test_invalid_values_rejected(block):
    with pytest.raises(ValueError, match="fidelity"):
        scenario_mod.build(_cfg(block))


def test_unknown_subkey_rejected():
    with pytest.raises(ValueError, match="fidelity"):
        scenario_mod.build(_cfg({"fleeet": "pointmass"}))
