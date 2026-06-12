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
from ..c2.orchestrator import Orchestrator
from ..c2.roe import RoeConfig
from ..core.comms import CommsModel
from ..core.messages import ThreatClass
from ..interceptors.effectors import EFFECTOR_FACTORIES
from ..interceptors.sentinel import SentinelUav, SitlShellSentinel
from ..interceptors.uav import InterceptorUav, SitlShellUav
from ..mc.fcu_client import FcuClient
from ..perception.fusion import FusionNode
from ..sensors.acoustic import AcousticSensor
from ..sensors.base import mounted
from ..sensors.eo_ir import EoIrSensor
from ..sensors.radar import Radar
from ..sensors.rf import RfSensor
from ..sensors.seeker import OnboardSeeker
from ..sil.fleet import SitlEngine
from ..sil.vehicle import FriendlyVehicle
from ..threats.enemy_drone import THREAT_PROFILES, EnemyDrone
from ..viz.recorder import Recorder
from .adjudicator import EngagementAdjudicator
from .debris_objects import DebrisReporter
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

# Caps on client-supplied parametric request values (resource safety: the
# request arrives over the /ops websocket and must not be able to exhaust
# the backend).
MAX_GROUP_COUNT = 200
MAX_TOTAL_THREATS = 500
MAX_DURATION_S = 7200.0


@dataclass
class Scenario:
    name: str
    duration: float
    world: World
    recorder: Recorder
    uavs: dict[str, InterceptorUav] = field(default_factory=dict)
    sentinels: dict[str, SentinelUav] = field(default_factory=dict)
    turrets: dict[str, GroundTurret] = field(default_factory=dict)
    eval_tracker: EvalTracker | None = None
    orchestrator: Orchestrator | None = None
    meta: dict = field(default_factory=dict)

    def run(self, **kwargs) -> dict:
        return self.world.run(self.duration, **kwargs)


def load(path: str | Path, seed: int | None = None) -> Scenario:
    cfg = yaml.safe_load(Path(path).read_text())
    return build(cfg, seed=seed)


# Fidelity modes (PLAN_PROBLEM1): pointmass is the v0.x behavior; the
# alternatives are declared here so scenarios can name them, and refuse
# loudly until their build paths land (fleet sitl: P4, threats sixdof: P6).
_FIDELITY_ALLOWED = {"fleet": ("pointmass", "sitl"),
                     "threats": ("pointmass", "sixdof")}


def _parse_fidelity(raw) -> dict:
    fid = dict(raw or {})
    unknown = set(fid) - set(_FIDELITY_ALLOWED)
    if unknown:
        raise ValueError(f"unknown fidelity keys: {sorted(unknown)}")
    out = {key: fid.get(key, "pointmass") for key in _FIDELITY_ALLOWED}
    for key, allowed in _FIDELITY_ALLOWED.items():
        if out[key] not in allowed:
            raise ValueError(
                f"fidelity.{key} must be one of {list(allowed)}, got {out[key]!r}")
    if out["threats"] == "sixdof":
        raise NotImplementedError(
            "fidelity.threats=sixdof lands with the 6DOF threat batch (PLAN_PROBLEM1 P6)")
    return out


# SITL engine knobs (`sitl:` block; only legal with fidelity.fleet=sitl).
_SITL_LINK_KEYS = {"latency_s", "bandwidth_bps", "queue_max_bytes"}


def _parse_sitl(raw) -> dict:
    cfg = dict(raw or {})
    out = {
        "base_hz": int(cfg.pop("base_hz", 800)),
        "mc_hz": int(cfg.pop("mc_hz", 10)),
        "link": dict(cfg.pop("link", None) or {}),
        "fcu": dict(cfg.pop("fcu", None) or {}) or None,
    }
    if cfg:
        raise ValueError(f"unknown sitl keys: {sorted(cfg)}")
    unknown = set(out["link"]) - _SITL_LINK_KEYS
    if unknown:
        raise ValueError(f"unknown sitl.link keys: {sorted(unknown)}")
    return out


# `faults:` block (P5-2b, SIM-SIL-003): a list of timed fault windows
# injected at the hw/link level by the SitlEngine schedule. Structure is
# validated here (loud, before any build work); semantics (rotor range,
# sensor names) by SitlEngine.schedule_fault. Only legal with
# fidelity.fleet=sitl — point-mass vehicles have no hw to fault.
_FAULT_COMMON_KEYS = {"t", "uav", "kind", "until"}


def _parse_faults(raw, ids: set) -> list[dict]:
    from ..sil.fleet import SitlEngine

    out = []
    for k, entry in enumerate(raw or []):
        f = dict(entry)
        missing = {"t", "uav", "kind"} - set(f)
        if missing:
            raise ValueError(f"faults[{k}]: missing keys {sorted(missing)}")
        kind = f["kind"]
        required = SitlEngine.FAULT_KINDS.get(kind)
        if required is None:
            raise ValueError(
                f"faults[{k}]: unknown kind {kind!r}; "
                f"one of {sorted(SitlEngine.FAULT_KINDS)}")
        unknown = set(f) - _FAULT_COMMON_KEYS - required
        if unknown:
            raise ValueError(f"faults[{k}] ({kind}): unknown keys "
                             f"{sorted(unknown)}")
        missing = required - set(f)
        if missing:
            raise ValueError(f"faults[{k}] ({kind}): missing params "
                             f"{sorted(missing)}")
        if f["uav"] not in ids:
            raise ValueError(f"faults[{k}]: unknown uav {f['uav']!r}")
        out.append(f)
    return out


def build(cfg: dict, seed: int | None = None) -> Scenario:
    fidelity = _parse_fidelity(cfg.get("fidelity"))
    env = Environment.from_config(cfg["environment"])
    run_seed = cfg.get("seed", 0) if seed is None else seed
    world = World(env, dt=cfg.get("dt", 0.05), seed=run_seed)
    world.weather = WeatherState.from_config(
        cfg.get("weather"), world.rng_registry.stream("weather"))
    world.occlusion.enabled = dict(cfg.get("occlusion") or {}).get("enabled", True)
    assets = {a.name: a for a in env.assets}

    # Node order fixes the within-step pipeline: sense -> fuse -> decide ->
    # act -> adjudicate -> evaluate -> record.
    icfgs = []
    for u in cfg.get("interceptors", []):
        u = dict(u)
        icfgs.append((u.pop("id"), _resolve_home(u, env),
                      u.pop("effector"), u))
    scfgs = []
    for s in cfg.get("sentinels", []):
        s = dict(s)
        scfgs.append((s.pop("id"), _resolve_home(s, env),
                      s.pop("orbit"), s))

    # Fleet fidelity (PLAN_PROBLEM1 P4-3 stage 2): in sitl mode the
    # tactical stack runs as mc/interceptor_app.py on a VirtualMCU inside
    # the engine micro-loop, flying a SitlEngine vehicle over the
    # coop-link on EKF estimates; the world-side SitlShellUav node only
    # ferries bus traffic across the mailbox boundary, and every sim-side
    # consumer (adjudicator, comms radio, seekers, evasion, recorder)
    # sees the FriendlyVehicle TRUTH adapter. P4-5: sentinels ride the
    # same engine (shared fleet airframe, documented approximation) with
    # mc/sentinel_app.py on their own VirtualMCU; the mounted sensor
    # payload mounts on the truth adapter.
    uavs: dict[str, InterceptorUav] = {}
    sentinels: dict[str, SentinelUav] = {}
    platforms: dict[str, object] = uavs
    sent_platforms: dict[str, object] = sentinels
    if fidelity["fleet"] == "sitl" and (icfgs or scfgs):
        from ..mc.interceptor_app import InterceptorApp
        from ..mc.sentinel_app import SentinelApp
        from ..sil.host import VirtualMCU

        sitl_cfg = _parse_sitl(cfg.get("sitl"))
        # Airframe classes (P4 gate-review resolution 2): interceptors fly
        # the engine-default racer quad, sentinels the endurance-pack
        # variant — one batched plant per class inside the one engine.
        engine = SitlEngine(
            [(uid, tuple(home)) for uid, home, _, _ in icfgs]
            + [(uid, tuple(home), "sentinel_quad")
               for uid, home, _, _ in scfgs],
            world.rng_registry, weather=world.weather, world_dt=world.dt,
            base_hz=sitl_cfg["base_hz"], fcu_overlay=sitl_cfg["fcu"],
            heartbeat_hz=0.0)
        world.micro = engine

        def _host(uid, home, extra, make_app):
            up, down = engine.attach_link(uid, **sitl_cfg["link"])
            client = FcuClient(up, down)
            app_kw = {k: v for k, v in extra.items() if k != "rate_hz"}

            def factory(clock, rng, ports):
                return make_app(clock, rng, ports, client, app_kw)

            mcu = VirtualMCU(f"mc/{uid}", tick_hz=sitl_cfg["mc_hz"],
                             base_hz=sitl_cfg["base_hz"], app_factory=factory,
                             rng=world.rng_registry.stream(f"mc/{uid}"))
            engine.attach_mc(uid, mcu)
            # Pad charger (P4-4): the recharge matches the MC turnaround.
            engine.set_pad(uid, home,
                           recharge_s=extra.get("turnaround_s", 90.0))
            return mcu

        platforms = {}
        sent_platforms = {}
        for uid, home, eff_name, extra in icfgs:
            effector = EFFECTOR_FACTORIES[eff_name]()
            mcu = _host(uid, home, extra,
                        lambda c, r, p, cl, kw, _u=uid, _h=home, _e=effector:
                        InterceptorApp(c, r, p, uav_id=_u, home=_h,
                                       effector=_e, fcu_client=cl, **kw))
            shell = SitlShellUav(uid, world.bus, home, effector, mcu=mcu,
                                 **extra)
            uavs[uid] = shell
            platforms[uid] = FriendlyVehicle(engine, uid, home,
                                             tactical=shell)
        for uid, home, orbit, extra in scfgs:
            mcu = _host(uid, home, extra,
                        lambda c, r, p, cl, kw, _u=uid, _h=home, _o=orbit:
                        SentinelApp(c, r, p, uav_id=_u, home=_h, orbit=_o,
                                    fcu_client=cl, **kw))
            shell = SitlShellSentinel(uid, world.bus, home, orbit, mcu=mcu,
                                      **extra)
            sentinels[uid] = shell
            sent_platforms[uid] = FriendlyVehicle(engine, uid, home,
                                                  tactical=shell)
        # Timed fault windows (P5-2b): scheduled after every link/MCU
        # is attached so jam targets exist; a scenario without the
        # block schedules nothing (bit-identical, SIM-SIL-003).
        for f in _parse_faults(cfg.get("faults"), set(engine.ids)):
            engine.schedule_fault(
                f.pop("t"), f.pop("uav"), f.pop("kind"),
                until=f.pop("until", None), **f)
    else:
        if cfg.get("sitl"):
            raise ValueError("a `sitl:` block requires fidelity.fleet=sitl")
        if cfg.get("faults"):
            raise ValueError(
                "a `faults:` block requires fidelity.fleet=sitl "
                "(point-mass vehicles have no hw to fault, SIM-SIL-003)")
        for uid, home, eff_name, extra in icfgs:
            uavs[uid] = InterceptorUav(
                uav_id=uid, bus=world.bus, home=home,
                effector=EFFECTOR_FACTORIES[eff_name](), **extra)
        for uid, home, orbit, extra in scfgs:
            sentinels[uid] = SentinelUav(
                uav_id=uid, bus=world.bus, home=home, orbit=orbit, **extra)
    for uid, platform in platforms.items():
        world.friendlies[uid] = platform
    for uid, platform in sent_platforms.items():
        world.friendlies[uid] = platform

    # Simulated network layer (SIM-COM-001/002): every C2<->UAV and UAV<->UAV
    # topic rides it. The default config is a near-perfect link, preserving
    # the v0.1 verified baseline; scenarios may degrade it (`comms:` block).
    comms = CommsModel(world, **dict(cfg.get("comms") or {}))
    for uid, platform in platforms.items():
        # The radio rides the airframe: truth adapter in sitl mode (it
        # forwards link_quality to the tactical node's telemetry).
        comms.register_endpoint(uid, platform)
    for uid, platform in sent_platforms.items():
        comms.register_endpoint(uid, platform)

    # Mounted sentinel payloads (PHY-SNT-*) ride the airframe truth —
    # the adapter in sitl mode, the legacy body otherwise.
    for uid, platform in sent_platforms.items():
        world.add_node(mounted(EoIrSensor)(
            f"eo-{uid}", world, platform, max_range=3000.0,
            full_id_range=1000.0))
        world.add_node(mounted(RfSensor)(
            f"rf-{uid}", world, platform, max_range=8000.0))

    for s in cfg.get("sensors", []):
        s = dict(s)
        cls = SENSOR_TYPES[s.pop("type")]
        name = s.pop("name")
        position = np.array(s.pop("position"), dtype=float)
        world.add_node(cls(name, world, position, **s))
    if cfg.get("seekers", True):
        for uid, platform in platforms.items():
            # Seekers mount on the truth airframe; the cue stays the
            # tactical node's estimate-only picture (seeker_cue forward).
            world.add_node(OnboardSeeker(f"seeker-{uid}", world, platform))

    world.add_node(FusionNode(world.bus, **cfg.get("fusion", {})))
    # Debris-tracking picture (SIM-DEB-002): published before the C2 plans,
    # so intercept tasking sees this tick's fall state. Own `debris_hz` knob
    # (DESIGN_REVIEW 5.3): the recording rate must not steer C2 tasking.
    world.add_node(DebrisReporter(world, rate_hz=cfg.get("debris_hz", 5.0)))

    bs_cfg = dict(cfg.get("base_station", {}))
    roe = RoeConfig(**bs_cfg.pop("roe", {}))
    world.add_node(
        BaseStation(
            world.bus, env, world.debris_model,
            uav_speeds={uid: u.max_speed for uid, u in uavs.items()},
            uav_effectors={uid: u.effector.type.value for uid, u in uavs.items()},
            roe_config=roe, **bs_cfg,
        )
    )

    # Orchestration agent (SRS ORC-*): holds the autonomy posture and turns
    # the C2's ROE verdicts into clearances. It gets the world's log_event
    # callable only — never the world itself (ORC-006).
    orchestrator = Orchestrator(
        world.bus,
        posture=cfg.get("posture", "human_confirm"),
        log_event=world.log_event,
        **dict(cfg.get("orchestrator") or {}),
    )
    world.add_node(orchestrator)

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
    for sent in sentinels.values():
        world.add_node(sent)
    # The referee costs true geometry: truth adapters in sitl mode.
    world.add_node(EngagementAdjudicator(world, platforms, turrets))

    tracker = EvalTracker(world)
    world.add_node(tracker)

    recorder = Recorder(world, rate_hz=cfg.get("record_hz", 5.0))
    recorder.eval_tracker = tracker
    world.add_node(recorder)

    counters: dict[str, itertools.count] = {}
    seen_ids: set[str] = set()
    for th in cfg.get("threats", []):
        tc = ThreatClass[th["class"]]
        n = counters.setdefault(tc.value, itertools.count(1))
        drone_id = th.get("id")
        if drone_id is None:
            drone_id = f"{tc.value}-{next(n)}"
            while drone_id in seen_ids:   # an explicit id took this slot
                drone_id = f"{tc.value}-{next(n)}"
        elif drone_id in seen_ids:
            # world.enemies is keyed by id: the second airframe would
            # silently replace the first one in flight at spawn time.
            raise ValueError(f"duplicate threat id '{drone_id}' in scenario")
        seen_ids.add(drone_id)
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
        sentinels=sentinels,
        turrets=turrets,
        eval_tracker=tracker,
        orchestrator=orchestrator,
        meta={"seed": run_seed, "speed": 1.0,
              "posture": cfg.get("posture", "human_confirm"), "eval": True,
              "fidelity": fidelity},
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
        if count > MAX_GROUP_COUNT:
            raise ValueError(
                f"threat class '{cls_key}': count {count} exceeds the "
                f"per-class maximum of {MAX_GROUP_COUNT}"
            )
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
    if len(threats) > MAX_TOTAL_THREATS:
        raise ValueError(
            f"raid totals {len(threats)} threats, exceeding the "
            f"maximum of {MAX_TOTAL_THREATS}"
        )
    cfg["threats"] = threats

    if request.get("weather"):
        weather = dict(cfg.get("weather") or {})
        weather.update(request["weather"])
        cfg["weather"] = weather
    if request.get("duration"):
        duration = float(request["duration"])
        if not 0.0 < duration <= MAX_DURATION_S:    # also rejects nan
            raise ValueError(
                f"duration must be in (0, {MAX_DURATION_S:.0f}] s, "
                f"got {duration}"
            )
        cfg["duration"] = duration
    if request.get("posture"):
        cfg["posture"] = request["posture"]

    scenario = build(cfg, seed=seed)
    scenario.meta.update({
        "speed": float(request.get("speed") or 1.0),
        "posture": request.get("posture") or "human_confirm",
        "request": request,
    })
    return scenario


def _resolve_home(u: dict, env: Environment) -> np.ndarray:
    """A UAV is homed either to an explicit `home` point (legacy) or to a
    charging station by id (`station`, PHY-CHG-001)."""
    if "home" in u:
        u.pop("station", None)
        return np.array(u.pop("home"), dtype=float)
    return env.station(u.pop("station")).position.copy()


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
        # Per-threat stream (DESIGN_REVIEW 5.1): spawn order stops mattering.
        rng = world.rng_registry.stream(f"threat/{drone_id}")
        return EnemyDrone(drone_id, threat_class, spawn, target, rng,
                          world=world, target_name=target_name)
    return make
