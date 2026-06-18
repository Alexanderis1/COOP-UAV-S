"""CRC-checked flight-software parameter table (boot-frozen overlay).

Defaults are declared in code by the modules that own them; a scenario
overlay may *change values* but never invent keys or change types (a
typo'd key or a string where a gain belongs is a build error, not a
silent default). The frozen table carries a CRC32 of the canonical
encoding; the CBIT PARAM_CRC monitor (P5) recomputes it in flight to
detect table corruption.

Values are scalars only: bool / int / float / str. bool is checked
*before* int everywhere — Python's bool subclasses int, and a YAML
``true`` must never satisfy a numeric gain (pinned in tests).
"""

from __future__ import annotations

import zlib
from collections.abc import Mapping

_SCALARS = (bool, int, float, str)


def _check_scalar(name: str, value, who: str) -> None:
    if type(value) not in _SCALARS:
        raise ValueError(
            f"{who} param {name!r}: values must be bool/int/float/str, "
            f"got {type(value).__name__}"
        )


def _coerce(name: str, default, value):
    """Overlay value matching the default's type; int -> float allowed."""
    dt, vt = type(default), type(value)
    if dt is float and vt is int:
        return float(value)
    if vt is not dt:
        raise ValueError(
            f"overlay param {name!r}: expected {dt.__name__}, "
            f"got {vt.__name__} ({value!r})"
        )
    return value


class ParamTable:
    """Immutable after construction; `get` is the only read API."""

    __slots__ = ("_values", "_crc", "_frozen")

    def __init__(self, defaults: Mapping, overlay: Mapping | None = None):
        values = {}
        for name, default in defaults.items():
            _check_scalar(name, default, "default")
            values[name] = default
        if overlay:
            for name, value in overlay.items():
                if name not in values:
                    raise ValueError(f"overlay param {name!r} is not a declared default")
                _check_scalar(name, value, "overlay")
                values[name] = _coerce(name, values[name], value)
        self._values = values
        self._crc = _crc32(values)
        self._frozen = True

    def __setattr__(self, attr, value):
        if getattr(self, "_frozen", False):
            raise AttributeError("ParamTable is frozen after construction")
        object.__setattr__(self, attr, value)

    def get(self, name: str):
        return self._values[name]

    def names(self) -> list[str]:
        return sorted(self._values)

    @property
    def crc(self) -> int:
        """CRC32 frozen at boot."""
        return self._crc

    def compute_crc(self) -> int:
        """CRC32 of the *live* table (CBIT PARAM_CRC recomputes this)."""
        return _crc32(self._values)

    def verify(self) -> bool:
        return self.compute_crc() == self._crc


def _crc32(values: Mapping) -> int:
    # Canonical encoding: sorted keys, type tag + repr (float repr is the
    # shortest round-trip form, stable across runs/platforms in py3).
    parts = [f"{name}={type(values[name]).__name__}:{values[name]!r}"
             for name in sorted(values)]
    return zlib.crc32("\n".join(parts).encode("utf-8"))
