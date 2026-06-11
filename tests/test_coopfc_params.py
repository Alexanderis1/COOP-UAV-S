"""P3-1: coopfc/params.py — CRC-checked parameter table with overlay.

Boot-time contract: defaults are declared in code, a scenario overlay may
override values but never invent keys or change types, and the frozen
table carries a CRC32 that the CBIT PARAM_CRC monitor (P5) recomputes to
detect in-flight corruption.
"""

from __future__ import annotations

import pytest

from coopuavs.coopfc.params import ParamTable

DEFAULTS = {
    "rate.roll.p": 0.15,
    "rate.roll.i": 0.2,
    "ekf.gps_gate": 5.0,
    "fcu.arm_timeout_s": 10,
    "fcu.allow_offboard": True,
    "frame.type": "quad_x",
}


def test_defaults_and_overlay():
    t = ParamTable(DEFAULTS, {"rate.roll.p": 0.25, "fcu.allow_offboard": False})
    assert t.get("rate.roll.p") == 0.25
    assert t.get("fcu.allow_offboard") is False
    assert t.get("rate.roll.i") == 0.2  # untouched default
    assert t.get("frame.type") == "quad_x"


def test_no_overlay_is_pure_defaults():
    t = ParamTable(DEFAULTS)
    assert t.get("ekf.gps_gate") == 5.0


def test_unknown_overlay_key_rejected():
    with pytest.raises(ValueError, match="rate.roll.pp"):
        ParamTable(DEFAULTS, {"rate.roll.pp": 0.3})


def test_unknown_get_rejected():
    t = ParamTable(DEFAULTS)
    with pytest.raises(KeyError):
        t.get("nope")


def test_type_mismatch_rejected():
    with pytest.raises(ValueError, match="frame.type"):
        ParamTable(DEFAULTS, {"frame.type": 3})
    with pytest.raises(ValueError, match="fcu.arm_timeout_s"):
        ParamTable(DEFAULTS, {"fcu.arm_timeout_s": 2.5})


def test_int_overlay_coerces_to_float_default():
    # YAML writes `5` for a float param; that must work and become float.
    t = ParamTable(DEFAULTS, {"ekf.gps_gate": 7})
    assert t.get("ekf.gps_gate") == 7.0
    assert type(t.get("ekf.gps_gate")) is float


def test_bool_is_not_an_int():
    # bool subclasses int in Python; a YAML `true` must never satisfy an
    # int/float param or vice versa.
    with pytest.raises(ValueError):
        ParamTable(DEFAULTS, {"fcu.arm_timeout_s": True})
    with pytest.raises(ValueError):
        ParamTable(DEFAULTS, {"fcu.allow_offboard": 1})
    with pytest.raises(ValueError):
        ParamTable(DEFAULTS, {"ekf.gps_gate": True})


def test_non_scalar_values_rejected():
    with pytest.raises(ValueError):
        ParamTable({"a": [1, 2]})
    with pytest.raises(ValueError):
        ParamTable(DEFAULTS, {"ekf.gps_gate": None})


def test_table_is_frozen():
    t = ParamTable(DEFAULTS)
    with pytest.raises(AttributeError):
        t.crc = 0
    assert not hasattr(t, "set")


def test_crc_stable_and_order_independent():
    t1 = ParamTable(DEFAULTS, {"rate.roll.p": 0.25, "ekf.gps_gate": 7.0})
    t2 = ParamTable(dict(reversed(list(DEFAULTS.items()))),
                    {"ekf.gps_gate": 7.0, "rate.roll.p": 0.25})
    assert t1.crc == t2.crc


def test_crc_sensitive_to_any_change():
    base = ParamTable(DEFAULTS).crc
    assert ParamTable(DEFAULTS, {"rate.roll.p": 0.151}).crc != base
    assert ParamTable(DEFAULTS, {"fcu.allow_offboard": False}).crc != base
    renamed = {("rate.roll.x" if k == "rate.roll.p" else k): v for k, v in DEFAULTS.items()}
    assert ParamTable(renamed).crc != base
    # 1 vs 1.0 vs True must hash differently (type is part of identity).
    assert ParamTable({"a": 1}).crc != ParamTable({"a": 1.0}).crc
    assert ParamTable({"a": 1}).crc != ParamTable({"a": True}).crc


def test_verify_detects_corruption():
    t = ParamTable(DEFAULTS)
    assert t.verify() is True
    assert t.compute_crc() == t.crc
    # Simulate a bit-flip in the live table (the CBIT seam: PARAM_CRC).
    t._values["rate.roll.p"] = 999.0
    assert t.verify() is False


def test_names_listing_sorted():
    t = ParamTable(DEFAULTS)
    assert t.names() == sorted(DEFAULTS)
