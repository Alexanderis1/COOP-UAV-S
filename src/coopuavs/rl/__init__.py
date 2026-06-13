"""Reinforcement-learning stack for the learned weapon-target policy.

Layering by dependency so the pure-Python core install is never burdened:

* :mod:`spaces`, :mod:`reconcile`, :mod:`env` — numpy only. The observation/
  action encoding, the action->task reconciliation, and the PettingZoo-shaped
  training environment. Importable in the base install and unit-tested there.
* :mod:`models`, :mod:`mappo`, :mod:`vec_env` — require PyTorch (the
  ``[train]`` optional dependency). The shared-actor / centralised-critic
  networks, the MAPPO trainer, and the CPU-parallel vectorised env. Imported
  lazily so ``import coopuavs.rl`` works without torch.

See docs/MARL.md for the design and the remote-box training recipe.
"""

from __future__ import annotations

from . import reconcile, spaces            # numpy-only, always importable
from .env import CoopWtaParallelEnv

__all__ = ["CoopWtaParallelEnv", "spaces", "reconcile"]
