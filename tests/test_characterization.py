"""P0-3 characterization pins: golden event-stream files.

These pin the exact event stream + summary of the two reference runs so the
P0 foundation work (VirtualClock scheduler, RngRegistry migration, debris_hz
decoupling) is provably behavior-preserving. A mismatch means behavior
changed: investigate first. Re-record via `python scripts/record_golden.py`
ONLY at the sanctioned P0-7 re-baseline.
"""

from __future__ import annotations

import pytest

import golden_util as gu


@pytest.mark.parametrize("name", sorted(gu.RUNS))
def test_events_match_golden(name):
    path = gu.golden_path(name)
    if not path.exists():
        pytest.fail(
            f"golden file missing: {path}. Record it with "
            "`python scripts/record_golden.py` (sanctioned re-baseline only)."
        )
    fresh = gu.to_json(gu.RUNS[name]())
    golden = path.read_text(encoding="utf-8")
    if fresh != golden:
        f_lines, g_lines = fresh.splitlines(), golden.splitlines()
        divergence = next(
            (i for i, (a, b) in enumerate(zip(f_lines, g_lines)) if a != b),
            min(len(f_lines), len(g_lines)),
        )
        ctx_fresh = f_lines[divergence : divergence + 3]
        ctx_gold = g_lines[divergence : divergence + 3]
        pytest.fail(
            f"{name} diverges from golden at line {divergence + 1}:\n"
            f"  fresh:  {ctx_fresh}\n  golden: {ctx_gold}\n"
            "Behavior changed — investigate before any re-record (P0-7 only)."
        )
