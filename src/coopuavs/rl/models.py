"""MAPPO networks: a parameter-shared actor and a centralised critic.

CTDE (centralised training, decentralised execution): one **shared actor**
is applied to each agent's ego-centric observation — the same weights run on
every interceptor, so the policy generalises across platforms and fleet
sizes and deploys decentrally. A **centralised critic** sees the joint
observation (all agents concatenated) during *training only*, which stabilises
the value estimate under the non-stationarity of many learning agents.

Pure PyTorch, CPU-first (small MLPs — the bottleneck is env stepping, not the
net). Requires the ``[train]`` optional dependency; importing this module
without torch raises a clear error.
"""

from __future__ import annotations

try:
    import torch
    import torch.nn as nn
except ImportError as exc:    # pragma: no cover - optional dependency
    raise ImportError(
        "coopuavs.rl.models requires PyTorch — install the training extras: "
        "pip install -e '.[train]'") from exc

from .spaces import NUM_ACTIONS, OBS_DIM

_NEG_INF = -1e9


def _mlp(sizes, activation=nn.Tanh):
    layers = []
    for i in range(len(sizes) - 1):
        layers.append(nn.Linear(sizes[i], sizes[i + 1]))
        if i < len(sizes) - 2:
            layers.append(activation())
    return nn.Sequential(*layers)


class Actor(nn.Module):
    """Shared policy: obs -> masked categorical over the WTA actions."""

    def __init__(self, obs_dim: int = OBS_DIM, n_actions: int = NUM_ACTIONS,
                 hidden=(256, 256)):
        super().__init__()
        self.net = _mlp([obs_dim, *hidden, n_actions])

    def logits(self, obs, mask=None):
        logits = self.net(obs)
        if mask is not None:
            logits = logits.masked_fill(~mask.bool(), _NEG_INF)
        return logits

    def dist(self, obs, mask=None):
        return torch.distributions.Categorical(logits=self.logits(obs, mask))

    @torch.no_grad()
    def act(self, obs, mask=None, *, deterministic=False, generator=None):
        """Sample (or argmax) an action with its log-prob — rollout/deploy."""
        d = self.dist(obs, mask)
        if deterministic:
            action = torch.argmax(d.logits, dim=-1)
        elif generator is not None:
            # Reproducible sampling from a seeded generator (the sim is
            # deterministic; policy stochasticity must be too).
            probs = d.probs
            action = torch.multinomial(probs, 1, generator=generator).squeeze(-1)
        else:
            action = d.sample()
        return action, d.log_prob(action)

    def evaluate(self, obs, mask, action):
        """Log-prob + entropy of given actions under the current policy."""
        d = self.dist(obs, mask)
        return d.log_prob(action), d.entropy()


class Critic(nn.Module):
    """Centralised state-value: V(joint observation) -> one team value."""

    def __init__(self, joint_dim: int, hidden=(256, 256)):
        super().__init__()
        self.net = _mlp([joint_dim, *hidden, 1])

    def forward(self, joint_obs):
        return self.net(joint_obs).squeeze(-1)


class ActorCritic(nn.Module):
    """Bundle for save/load with the metadata a deployment allocator needs to
    rebuild the actor (obs/action widths, agent count)."""

    def __init__(self, n_agents: int, obs_dim: int = OBS_DIM,
                 n_actions: int = NUM_ACTIONS, hidden=(256, 256)):
        super().__init__()
        self.n_agents = int(n_agents)
        self.obs_dim = int(obs_dim)
        self.n_actions = int(n_actions)
        self.actor = Actor(obs_dim, n_actions, hidden)
        self.critic = Critic(n_agents * obs_dim, hidden)
        self.hidden = tuple(hidden)

    def save(self, path) -> None:
        torch.save({
            "actor": self.actor.state_dict(),
            "critic": self.critic.state_dict(),
            "meta": {"n_agents": self.n_agents, "obs_dim": self.obs_dim,
                     "n_actions": self.n_actions, "hidden": list(self.hidden)},
        }, path)

    @classmethod
    def load(cls, path, map_location="cpu") -> "ActorCritic":
        ckpt = torch.load(path, map_location=map_location, weights_only=False)
        m = ckpt["meta"]
        model = cls(m["n_agents"], m["obs_dim"], m["n_actions"], tuple(m["hidden"]))
        model.actor.load_state_dict(ckpt["actor"])
        model.critic.load_state_dict(ckpt["critic"])
        model.eval()
        return model
