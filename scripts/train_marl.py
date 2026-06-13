"""Train the learned cooperation (WTA) policy with MAPPO.

Thin wrapper over ``coopuavs train`` for users who prefer a script. Requires
the training extra::

    pip install -e ".[train]"

Saturate a many-core CPU box (pin BLAS so N workers don't oversubscribe)::

    OMP_NUM_THREADS=1 python scripts/train_marl.py scenarios/high_diver_raid.yaml \
        --n-envs 30 --steps 8000000 --out runs/marl

See docs/MARL.md for the full recipe, hyper-parameters and A/B evaluation.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from coopuavs.cli import main  # noqa: E402

if __name__ == "__main__":
    main(["train", *sys.argv[1:]])
