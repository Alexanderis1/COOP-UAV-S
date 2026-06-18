"""P3-1: coopfc import fence (architecture isolation contract).

CoopFC is the flight software that runs *inside* a VirtualMCU: nothing of
the simulator may physically enter it. The fence is an AST walk over every
module in ``coopuavs/coopfc`` asserting

1. no import of any ``coopuavs.*`` module outside ``coopuavs.coopfc``
   (relative imports are resolved before checking, so ``from ...physics
   import x`` cannot sneak out), and
2. no numpy/scipy outside ``coopfc/estimation`` — the >=100 Hz hot paths
   (vec, sched, drivers, control, mixer) are plain-float by the perf
   budget; only the 50 Hz estimator is allowed numpy.

When ``mc/`` lands in P4 it joins the same walk.
"""

from __future__ import annotations

import ast
from pathlib import Path

import coopuavs.coopfc as coopfc_pkg

COOPFC_ROOT = Path(coopfc_pkg.__file__).parent
PKG_PREFIX = "coopuavs.coopfc"

# Only these third-party roots may appear anywhere in coopfc; numpy/scipy
# additionally only under estimation/.
NUMERIC_ROOTS = {"numpy", "scipy"}
BANNED_PREFIX = "coopuavs"


def _modules() -> list[Path]:
    return sorted(COOPFC_ROOT.rglob("*.py"))


def _dotted_name(py: Path) -> str:
    parts = py.relative_to(COOPFC_ROOT).with_suffix("").parts
    name = ".".join((PKG_PREFIX, *parts))
    if name.endswith(".__init__"):
        name = name[: -len(".__init__")]
    return name


def _resolve_imports(py: Path) -> list[str]:
    """All imported module names in `py`, relative imports resolved."""
    tree = ast.parse(py.read_text(encoding="utf-8"))
    mod_name = _dotted_name(py)
    is_pkg = py.name == "__init__.py"
    out: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            out.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level == 0:
                out.append(node.module or "")
            else:
                # Package the import is relative to: the module's own
                # package, then up (level - 1) more for each extra dot.
                pkg_parts = mod_name.split(".")
                if not is_pkg:
                    pkg_parts = pkg_parts[:-1]
                drop = node.level - 1
                assert drop <= len(pkg_parts), f"{py}: relative import escapes package root"
                base = pkg_parts[: len(pkg_parts) - drop]
                resolved = ".".join(base + ([node.module] if node.module else []))
                out.append(resolved)
    return out


def test_walker_sees_the_package():
    # An empty walk would vacuously pass everything below.
    assert len(_modules()) >= 4


def test_no_simulator_import_escapes_coopfc():
    for py in _modules():
        for name in _resolve_imports(py):
            root = name.split(".")[0]
            if root == BANNED_PREFIX:
                assert name == PKG_PREFIX or name.startswith(PKG_PREFIX + "."), (
                    f"{py.relative_to(COOPFC_ROOT.parent)} imports {name!r}: "
                    "flight software must not import the simulator"
                )


def test_numpy_only_under_estimation():
    for py in _modules():
        in_estimation = "estimation" in py.relative_to(COOPFC_ROOT).parts
        if in_estimation:
            continue
        for name in _resolve_imports(py):
            root = name.split(".")[0]
            assert root not in NUMERIC_ROOTS, (
                f"{py.relative_to(COOPFC_ROOT.parent)} imports {name!r}: "
                "numpy/scipy are allowed only in coopfc/estimation (50 Hz); "
                "hot paths are plain-float"
            )


# -- mc/ joins the walk (P4-4 import-boundary test) -----------------------------
#
# Mission-computer software runs on a VirtualMCU behind the mailbox seam:
# it may import its own package, the shared message vocabulary and port
# primitives (core.messages / core.ports) and the wire protocol
# (coopfc.link) — never the simulator, the world bus, devices or physics
# (truth quarantine: an mc/ module that can import sim/ can read truth).

MC_ALLOWED_PREFIXES = ("coopuavs.mc", "coopuavs.core.messages",
                       "coopuavs.core.ports", "coopuavs.coopfc.link")


def _mc_modules() -> list[Path]:
    import coopuavs.mc as mc_pkg
    return sorted(Path(mc_pkg.__file__).parent.rglob("*.py"))


def test_mc_walker_sees_the_package():
    assert len(_mc_modules()) >= 5


def test_no_simulator_import_escapes_mc():
    import coopuavs.mc as mc_pkg
    mc_root = Path(mc_pkg.__file__).parent

    global COOPFC_ROOT, PKG_PREFIX
    saved = COOPFC_ROOT, PKG_PREFIX
    COOPFC_ROOT, PKG_PREFIX = mc_root, "coopuavs.mc"   # reuse the resolver
    try:
        for py in _mc_modules():
            for name in _resolve_imports(py):
                if name.split(".")[0] != BANNED_PREFIX:
                    continue
                ok = any(name == p or name.startswith(p + ".")
                         for p in MC_ALLOWED_PREFIXES)
                assert ok, (
                    f"{py.relative_to(mc_root.parent)} imports {name!r}: "
                    "MC software may only reach coopuavs.mc, core.messages, "
                    "core.ports and coopfc.link — never the simulator"
                )
    finally:
        COOPFC_ROOT, PKG_PREFIX = saved
