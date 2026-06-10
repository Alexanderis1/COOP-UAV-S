"""Scenario loader: a YAML file or a parametric request fully describes a battle.

Everything tunable lives in the scenario — map and zones, sensor laydown,
interceptor fleet, turrets, weather, raid composition, ROE thresholds — so
experiments are data, not code (SIM-RT-004). See
``scenarios/residential_raid.yaml`` for the reference scenario and the
inline documentation of every field.

Two entry points:

* :func:`load` / :func:`build` — the YAML path, unchanged from v0.1;
* :func:`build_parametric` — the ICD_RUNTIME §3 ``start_run`` request
  (per-class counts, objectives, approach axes, wave timing, weather,
  seed) applied on top of a preset's map/laydown/fleet (SIM-THR-002,
  HMI-SCN-002/003).
"""

from __future__ import annotations

import copy
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
from ..threats.enemy_drone import THREAT_PROFILES, EnemyDrone
from ..viz.recorder import Recorder
from .adjudicator import EngagementAdjudicator
from .environment import Environment
from .evaluation import EvalTracker
from .turret import GroundTurret
from .weather import WeatherState
from .world import World

SENSOR_TYPES = {
    "radar": Radar,
    "rf": RfSensor,
    "eo_ir": EoIrSensor,
    "acoustic": AcousticSensor,
}

# Default wave timing for parametric raids (overridable per class group).
DEFAULT_FIRST_TIME = 10.0
DEFAULT_SPACING = 8.0


@dataclass
class Scenario:
    name: str
    duration: float
    world: World
    recorder: Recorder
    uavs: dict[str, InterceptorUav] = field(default_factory=dict)
    turrets: dict[str, GroundTurret] = field(default_factory=dict)
    eval_tracker: EvalTracker | None = None
    meta: dict = field(default_factory=dict)

    def run(self, **kwargs) -> dict:
        return self.world.run(self.duration, **kwargs)


def load(path: str | Path, seed: int | None = None) -> Scenario:
    cfg = yaml.safe_load(Path(path).read_text())
    return build(cfg, seed=seed)


def build(cfg: dict, seed: int | None = None) -> Scenario:
    env = Environment.from_config(cfg["environment"])
    run_seed = cfg.get("seed", 0) if seed is None else seed
    world = World(env, dt=cfg.get("dt", 0.05), seed=run_seed)
    world.weather = WeatherState.from_config(cfg.get("weather"), world.rng)
    assets = {a.name: a for a in env.assets}

    # Node order fixes the within-step pipeline: sense -> fuse -> decide ->
    # act -> adjudicate -> evaluate -> record.
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
        world.friendlies[uav.uav_id] = uav

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

    turrets: dict[str, GroundTurret] = {}
    for tcfg in cfg.get("turrets", []):
        tcfg = dict(tcfg)
        turret = GroundTurret(
            turret_id=tcfg.pop("id"),
            world=world,
            position=np.array(tcfg.pop("position"), dtype=float),
            **tcfg,
        )
        turrets[turret.turret_id] = turret
        world.turrets[turret.turret_id] = turret
        world.add_node(turret)

    for uav in uavs.values():
        world.add_node(uav)
    world.add_node(EngagementAdjudicator(world, uavs, turrets))

    tracker = EvalTracker(world)
    world.add_node(tracker)

    recorder = Recorder(world, rate_hz=cfg.get("record_hz", 5.0))
    recorder.eval_tracker = tracker
    world.add_node(recorder)

    counters: dict[str, itertools.count] = {}
    for th in cfg.get("threats", []):
        tc = ThreatClass[th["class"]]
        n = counters.setdefault(tc.value, itertools.count(1))
        drone_id = th.get("id", f"{tc.value}-{next(n)}")
        target_name = th["target"] if isinstance(th.get("target"), str) else ""
        target = (
            assets[th["target"]].position
            if isinstance(th.get("target"), str)
            else np.array(th["target"], dtype=float)
        )
        spawn = np.array(th["spawn"], dtype=float)
        world.schedule_enemy(
            th.get("time", 0.0),
            _enemy_factory(drone_id, tc, spawn, target, world, target_name),
        )

    name = cfg.get("name", "unnamed")
    duration = cfg.get("duration", 600.0)
    recorder.run_meta = {"name": name, "seed": run_seed,
                         "duration": duration, "eval": True}
    return Scenario(
        name=name,
        duration=duration,
        world=world,
        recorder=recorder,
        uavs=uavs,
        turrets=turrets,
        eval_tracker=tracker,
        meta={"seed": run_seed, "speed": 1.0,
              "posture": cfg.get("posture", "human_confirm"), "eval": True},
    )


def build_parametric(request: dict, preset_cfg: dict, seed: int) -> Scenario:
    """Build a scenario from an ICD §3 ``start_run`` request over a preset.

    ``request["threats"]`` maps ThreatClass value strings to
    ``{count, target, axis_deg, first_time, spacing}``; the preset supplies
    map, zones, sensors, fleet, turrets and ROE. Spawn points are placed
    outside the map on the approach bearing at the class cruise altitude.
    Invalid class or asset names raise ``ValueError`` with a structured
    message (HMI-SCN-003).
    """
    cfg = copy.deepcopy(preset_cfg)
    cfg["seed"] = seed
    rng = np.random.default_rng(seed)

    assets = [a["name"] for a in cfg.get("environment", {}).get("assets", [])]
    if not assets:
        raise ValueError("preset has no protected assets to target")
    asset_cycle = itertools.cycle(assets)

    bounds = cfg["environment"]["bounds"]
    centre = np.array([(bounds[0] + bounds[2]) / 2.0, (bounds[1] + bounds[3]) / 2.0])
    spawn_radius = 0.5 * float(np.hypot(bounds[2] - bounds[0], bounds[3] - bounds[1])) + 200.0

    preset_axes = _preset_threat_axes(cfg, centre)
    axis_cycle = itertools.cycle(preset_axes) if preset_axes else None

    threats: list[dict] = []
    for cls_key, group in (request.get("threats") or {}).items():
        tc = _parse_class(cls_key)
        group = dict(group or {})
        count = int(group.get("count", 0))
        if count < 0:
            raise ValueError(f"threat class '{cls_key}': count must be >= 0")
        if count == 0:
            continue
        target = group.get("target", "auto")
        if target not in ("auto", None) and target not in assets:
            raise ValueError(
                f"unknown target asset '{target}' for class '{cls_key}'; "
                f"available assets: {', '.join(assets)}"
            )
        first_time = float(group.get("first_time") or DEFAULT_FIRST_TIME)
        spacing = float(group.get("spacing") or DEFAULT_SPACING)
        axis = group.get("axis_deg")
        alt = THREAT_PROFILES[tc].cruise_alt

        for i in range(count):
            if axis is not None:
                bearing = float(axis)
            elif axis_cycle is not None:
                bearing = next(axis_cycle)
            else:
                bearing = float(rng.uniform(0.0, 360.0))
            bearing += float(rng.normal(0.0, 1.5))   # lateral spread in the wave
            b = np.deg2rad(bearing)
            spawn = centre + spawn_radius * np.array([np.sin(b), np.cos(b)])
            threats.append({
                "time": first_time + i * spacing,
                "class": tc.name,
                "spawn": [float(spawn[0]), float(spawn[1]), float(alt)],
                "target": next(asset_cycle) if target in ("auto", None) else target,
            })

    threats.sort(key=lambda th: th["time"])
    cfg["threats"] = threats

    if request.get("weather"):
        weather = dict(cfg.get("weather") or {})
        weather.update(request["weather"])
        cfg["weather"] = weather
    if request.get("duration"):
        cfg["duration"] = float(request["duration"])
    if request.get("posture"):
        cfg["posture"] = request["posture"]

    scenario = build(cfg, seed=seed)
    scenario.meta.update({
        "speed": float(request.get("speed") or 1.0),
        "posture": request.get("posture") or "human_confirm",
        "request": request,
    })
    return scenario


def _parse_class(key: str) -> ThreatClass:
    try:
        return ThreatClass(str(key).lower())
    except ValueError:
        pass
    try:
        return ThreatClass[str(key).upper()]
    except KeyError:
        valid = [c.value for c in ThreatClass if c != ThreatClass.UNKNOWN]
        raise ValueError(
            f"unknown threat class '{key}'; valid classes: {', '.join(valid)}"
        ) from None


def _preset_threat_axes(cfg: dict, centre: np.ndarray) -> list[float]:
    """Approach bearings of the preset's reference raid (deduplicated to
    ~10 degrees) — the default axes for parametric raids."""
    axes: list[float] = []
    for th in cfg.get("threats", []):
        spawn = np.asarray(th.get("spawn", (0.0, 0.0))[:2], dtype=float)
        rel = spawn - centre
        if float(np.linalg.norm(rel)) < 1.0:
            continue
        bearing = float(np.degrees(np.arctan2(rel[0], rel[1]))) % 360.0
        if all(min(abs(bearing - a), 360.0 - abs(bearing - a)) > 10.0 for a in axes):
            axes.append(round(bearing, 1))
    return axes


def _enemy_factory(drone_id, threat_class, spawn, target, world, target_name=""):
    def make() -> EnemyDrone:
        return EnemyDrone(drone_id, threat_class, spawn, target, world.rng,
                          world=world, target_name=target_name)
    return make
