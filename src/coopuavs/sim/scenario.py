"""Scenario loader: a YAML file fully describes a battle.

Everything tunable lives in the scenario — map and zones, sensor laydown,
interceptor fleet, raid composition, ROE thresholds — so experiments are
data, not code. See ``scenarios/residential_raid.yaml`` for the reference
scenario and the inline documentation of every field.
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from ..c2.base_station import BaseStation
from ..c2.roe import RoeConfig
from ..core.messages import ThreatClass
from ..interceptors.effectors import EFFECTOR_FACTORIES
from ..interceptors.uav import InterceptorUav
from ..perception.fusion import FusionNode
from ..sensors.acoustic import AcousticSensor
from ..sensors.eo_ir import EoIrSensor
from ..sensors.radar import Radar
from ..sensors.rf import RfSensor
from ..sensors.seeker import OnboardSeeker
from ..threats.enemy_drone import EnemyDrone
from ..viz.recorder import Recorder
from .adjudicator import EngagementAdjudicator
from .environment import Environment
from .world import World

SENSOR_TYPES = {
    "radar": Radar,
    "rf": RfSensor,
    "eo_ir": EoIrSensor,
    "acoustic": AcousticSensor,
}


@dataclass
class Scenario:
    name: str
    duration: float
    world: World
    recorder: Recorder
    uavs: dict[str, InterceptorUav] = field(default_factory=dict)

    def run(self, **kwargs) -> dict:
        return self.world.run(self.duration, **kwargs)


def load(path: str | Path, seed: int | None = None) -> Scenario:
    cfg = yaml.safe_load(Path(path).read_text())
    return build(cfg, seed=seed)


def build(cfg: dict, seed: int | None = None) -> Scenario:
    env = Environment.from_config(cfg["environment"])
    world = World(
        env,
        dt=cfg.get("dt", 0.05),
        seed=cfg.get("seed", 0) if seed is None else seed,
    )
    assets = {a.name: a for a in env.assets}

    # Node order fixes the within-step pipeline: sense -> fuse -> decide ->
    # act -> adjudicate -> record.
    uavs: dict[str, InterceptorUav] = {}
    for u in cfg.get("interceptors", []):
        u = dict(u)
        uav = InterceptorUav(
            uav_id=u.pop("id"),
            bus=world.bus,
            home=np.array(u.pop("home"), dtype=float),
            effector=EFFECTOR_FACTORIES[u.pop("effector")](),
            **u,
        )
        uavs[uav.uav_id] = uav

    for s in cfg.get("sensors", []):
        s = dict(s)
        cls = SENSOR_TYPES[s.pop("type")]
        name = s.pop("name")
        position = np.array(s.pop("position"), dtype=float)
        world.add_node(cls(name, world, position, **s))
    if cfg.get("seekers", True):
        for uav in uavs.values():
            world.add_node(OnboardSeeker(f"seeker-{uav.uav_id}", world, uav))

    world.add_node(FusionNode(world.bus, **cfg.get("fusion", {})))

    bs_cfg = dict(cfg.get("base_station", {}))
    roe = RoeConfig(**bs_cfg.pop("roe", {}))
    world.add_node(
        BaseStation(
            world.bus, env, world.debris_model,
            uav_speeds={uid: u.max_speed for uid, u in uavs.items()},
            roe_config=roe, **bs_cfg,
        )
    )
    for uav in uavs.values():
        world.add_node(uav)
    world.add_node(EngagementAdjudicator(world, uavs))

    recorder = Recorder(world, rate_hz=cfg.get("record_hz", 5.0))
    world.add_node(recorder)

    counters: dict[str, itertools.count] = {}
    for th in cfg.get("threats", []):
        tc = ThreatClass[th["class"]]
        n = counters.setdefault(tc.value, itertools.count(1))
        drone_id = th.get("id", f"{tc.value}-{next(n)}")
        target = (
            assets[th["target"]].position
            if isinstance(th.get("target"), str)
            else np.array(th["target"], dtype=float)
        )
        spawn = np.array(th["spawn"], dtype=float)
        world.schedule_enemy(
            th.get("time", 0.0),
            _enemy_factory(drone_id, tc, spawn, target, world),
        )

    return Scenario(
        name=cfg.get("name", "unnamed"),
        duration=cfg.get("duration", 600.0),
        world=world,
        recorder=recorder,
        uavs=uavs,
    )


def _enemy_factory(drone_id, threat_class, spawn, target, world):
    def make() -> EnemyDrone:
        return EnemyDrone(drone_id, threat_class, spawn, target, world.rng)
    return make
