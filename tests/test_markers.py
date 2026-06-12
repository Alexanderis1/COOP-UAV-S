"""P0-2: marker registration and default-run exclusion contract.

The default pytest run must stay fast: ``slow``/``perf``/``oracle`` suites run
only when asked for explicitly (e.g. ``pytest -m slow``). Registration is
strict so a typo like ``@pytest.mark.slws`` fails collection instead of
silently always-running.
"""

from __future__ import annotations

import tomllib
from pathlib import Path

import pytest

PYPROJECT = Path(__file__).resolve().parents[1] / "pyproject.toml"


def _pytest_ini() -> dict:
    with PYPROJECT.open("rb") as fh:
        return tomllib.load(fh)["tool"]["pytest"]["ini_options"]


def test_markers_registered():
    ini = _pytest_ini()
    registered = {entry.split(":")[0].strip() for entry in ini.get("markers", [])}
    assert {"slow", "perf", "oracle"} <= registered


def test_default_run_excludes_slow_perf_oracle():
    ini = _pytest_ini()
    addopts = ini.get("addopts", "")
    assert "not slow" in addopts
    assert "not perf" in addopts
    assert "not oracle" in addopts


def test_strict_markers_enforced():
    assert "--strict-markers" in _pytest_ini().get("addopts", "")


@pytest.mark.slow
def test_slow_is_excluded_from_default_run(request):
    """If this executes, the caller opted in via ``-m``; the default ``-m``
    expression from addopts must not be in effect unchanged."""
    mexpr = request.config.getoption("-m")
    assert mexpr != "not slow and not perf and not oracle"
