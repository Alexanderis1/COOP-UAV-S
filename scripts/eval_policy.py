"""A/B a trained cooperation policy against the classical allocator.

Thin wrapper over ``coopuavs eval``. Runs the learned policy and the greedy
baseline over a seed sweep on the same scenario (CAP sentinels included) and
prints the outcome comparison — armed leakers, kills, jet leak rate,
collateral, ammo economy. Example::

    python scripts/eval_policy.py scenarios/high_diver_raid.yaml \
        --policy runs/marl/policy.pt -n 20
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from coopuavs.cli import main  # noqa: E402

if __name__ == "__main__":
    main(["eval", *sys.argv[1:]])
