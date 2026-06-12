"""Record the P0-3 characterization golden files (tests/fixtures/golden/).

Run ONLY at a sanctioned re-baseline (docs/PLAN_PROBLEM1.md P0-7): the
goldens pin current behavior; re-recording over a regression destroys the
safety net.

Usage: python scripts/record_golden.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests"))

import golden_util  # noqa: E402

if __name__ == "__main__":
    golden_util.record_all()
    sys.exit(0)
