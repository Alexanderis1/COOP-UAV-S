"""P4-1 fleet SITL engine: N vehicles of physics + hw devices + one FCU each.

The ``sil/bench.py`` micro-tick shape, fleet-vectorized and installed as
``World.micro`` (ORDERING §1 item 6). Each of the K micro-ticks per world
macro step runs the frozen ORDERING §6 order:

1. devices sample truth — vectorized P2 banks with their real error
   processes, latency and quantization; one registry parent stream per
   device type (``sensor/imu`` .. ``sensor/esc_telem``, §4) from which
   each bank spawns one child per vehicle, so a fleet-size change leaves
   every existing vehicle's draw history identical;
2. per-vehicle FCU ``run_tick()`` in vehicle order (the registration-
   order determinism contract);
3. MC tick if due — seam only until P4-3;
4. actuators latched from each FCU's HAL port;
5. ONE Dryden bank draw (``dryden`` stream), body-FLU gusts rotated
   through each vehicle's pre-step attitude;
6. powertrain implicit bus solve + electrical advance (batched);
7. ONE batched fleet RK4.

IMU acceleration is the exact wrench ``force_world / m`` at the latched
inputs (gravity included — the ``hw.Imu.sample`` contract), closing the
P3 dv/dt bench placeholder (user decision 2026-06-12; the single-vehicle
bench keeps its pinned finite-difference form).

Wind enters as a plant FORCE: per-vehicle sheared mean wind
(``WeatherState.mean_wind_at``) plus MIL-F-8785C Dryden gusts when the
scenario wind is nonzero (user decision 2026-06-12: Dryden replaces the
legacy OU gust contribution for SITL vehicles; the world's truth-side
displacement skips them via ``FriendlyVehicle.wind_displaced``). The
Dryden bank steps every tick from t=0 so per-vehicle gust histories
never depend on when anyone armed.

Stand convention (ground contact deferred, user decision 2026-06-12): a
non-ARMED row is frozen truth with zero velocity/rates; devices keep
sampling it with real noise so alignment and the EKF converge on honest
data. Motors pre-spin to hover speed on the disarmed→ARMED transition.

Heartbeats: until the P4-2 MC coop-link wiring, the engine feeds each
FCU ``on_heartbeat`` at ``heartbeat_hz`` (the bench convention) so a
hovering fleet does not trip the LINK_LOSS failsafe.
"""

from __future__ import annotations

import math

import numpy as np

from coopuavs.coopfc.fcu import ARMED, TICK_HZ, Fcu
from coopuavs.coopfc.hal import HalIO
from coopuavs.coopfc.link.coop_link import (
    BATT_CODES,
    DEGRADED_CODES,
    FAILSAFE_CODES,
    MODE_CODES,
    MODE_NAMES,
    MSG,
    STATE_CODES,
    Channel,
    FrameDecoder,
    decode_msg,
    encode_msg,
)
from coopuavs.hw import params as hw_params
from coopuavs.hw.baro import Baro, BaroParams
from coopuavs.hw.esc_telem import EscTelem, EscTelemParams
from coopuavs.hw.gps import Gps, GpsParams
from coopuavs.hw.imu import Imu, ImuParams
from coopuavs.hw.mag import Mag, MagParams
from coopuavs.physics.battery import BatteryEcm
from coopuavs.physics.dryden import DrydenGusts, gusts_to_world
from coopuavs.physics.motor import MotorEsc
from coopuavs.physics.multirotor import MultirotorParams, MultirotorPlant
from coopuavs.physics.params import load_airframe
from coopuavs.physics.powertrain import Powertrain

from .clock import MicroScheduler

RHO = 1.225
G = 9.81

# FCU-pipeline device poll rates (coopfc/fcu.py scheduler registration);
# the host writes raw frames at the matching device rates, bench-style.
BARO_MAG_HZ = 50
ESC_HZ = 10
# FCU-side coop-link rates (P4-2): drain + dispatch sits in the §6
# per-vehicle pipeline slot at the plan's 50 Hz link rate group;
# telemetry down the wire at NAV 25 Hz / STATUS 10 Hz (the MC node runs
# at 10 Hz — fresher STATUS buys nothing, NAV headroom feeds guidance).
LINK_HZ = 50
NAV_HZ = 25
STATUS_HZ = 10
HEALTH_HZ = 1     # CBIT northbound (P5-1c; PHY-UAV-013 needs >= 1 Hz)


class _FcuLink:
    """FCU-side wire state for one linked vehicle."""

    __slots__ = ("up", "down", "dec")

    def __init__(self, up: Channel, down: Channel):
        self.up = up          # MC -> FCU (engine drains)
        self.down = down      # FCU -> MC (engine sends)
        self.dec = FrameDecoder()


class _AirframeGroup:
    """One airframe class within the fleet: a batched plant + powertrain
    over the global state rows in ``rows`` (P4 gate-review resolution 2)."""

    def __init__(self, airframe: str, rows: np.ndarray):
        raw = load_airframe(airframe)
        self.airframe = airframe
        self.rows = rows
        self.cfg = MultirotorParams.from_dict(raw)
        gn = len(rows)
        self.plant = MultirotorPlant(self.cfg, gn)
        self.motor = MotorEsc(gn, self.cfg.n_rotors, **self.cfg.motor)
        battery = BatteryEcm(gn, **self.cfg.battery)
        self.pt = Powertrain(self.motor, battery,
                             i_bus_max_a=raw["powertrain"]["i_bus_max_a"])
        self.mass_col = self.plant.mass[:, None]
        self.w_hover = math.sqrt(
            self.cfg.mass * G / (self.cfg.n_rotors * self.cfg.kf))


class SitlEngine:
    """Fleet micro-loop behind ``World.micro`` (SIM-SIL-002)."""

    def __init__(self, vehicles, rng_registry, weather=None, *,
                 world_dt: float = 0.05, base_hz: int = TICK_HZ,
                 airframe: str = "interceptor_quad",
                 devices: str = "interceptor_devices",
                 fcu_overlay: dict | None = None,
                 heartbeat_hz: float = 10.0):
        if base_hz != TICK_HZ:
            # Scenario rate profiles are a documented perf fallback lever
            # (PLAN_PROBLEM1 §perf); the FCU tick rate is compile-time
            # today, so anything else would silently warp FCU time.
            raise ValueError(
                f"base_hz={base_hz} unsupported: the CoopFC tick rate is "
                f"{TICK_HZ} Hz (rate profiles are a fallback lever, not wired)")
        # Airframe classes (P4 gate-review resolution 2, fidelity-first):
        # a vehicle entry is (uid, start) for the engine default airframe
        # or (uid, start, airframe) for its own class — one batched
        # plant/powertrain per class, ONE RK4 per class per tick. Device
        # banks stay fleet-wide (one suite), so every class must share
        # the rotor count for the shared ESC telemetry bank.
        specs = [(v[0], v[1], v[2] if len(v) > 2 else airframe)
                 for v in vehicles]
        ids = [uid for uid, _, _ in specs]
        if len(set(ids)) != len(ids):
            raise ValueError(f"duplicate vehicle ids in {ids}")
        if not ids:
            raise ValueError("SitlEngine needs at least one vehicle")
        self.ids = ids
        self.index = {uid: i for i, uid in enumerate(ids)}
        self.n = len(ids)
        n = self.n
        self._world_dt = float(world_dt)
        self._dt = 1.0 / base_hz
        self._micro = MicroScheduler(world_dt, base_hz)
        self._micro.add("sitl_fleet", base_hz, self._tick)

        dev = hw_params.load_devices(devices)
        stream = rng_registry.stream
        self.imu = Imu(ImuParams.from_dict(dev["imu"]), n,
                       stream("sensor/imu"))
        self.gps = Gps(GpsParams.from_dict(dev["gps"]), n,
                       stream("sensor/gps"), clock_hz=base_hz)
        self.baro = Baro(BaroParams.from_dict(dev["baro"]), n,
                         stream("sensor/baro"))
        self.mag = Mag(MagParams.from_dict(dev["mag"]), n,
                       stream("sensor/mag"))

        by_airframe: dict[str, list[int]] = {}
        for i, (_, _, af) in enumerate(specs):
            by_airframe.setdefault(af, []).append(i)
        self.groups = [_AirframeGroup(af, np.asarray(rows, dtype=np.intp))
                       for af, rows in by_airframe.items()]
        self._group_of = np.zeros(n, dtype=np.intp)
        self._local_of = np.zeros(n, dtype=np.intp)
        for gi, g in enumerate(self.groups):
            self._group_of[g.rows] = gi
            self._local_of[g.rows] = np.arange(len(g.rows))
        rotor_counts = {g.cfg.n_rotors for g in self.groups}
        if len(rotor_counts) != 1:
            raise ValueError(
                f"airframe classes disagree on rotor count {rotor_counts}: "
                "the fleet shares one ESC telemetry bank")
        n_rotors = rotor_counts.pop()
        cell_counts = {g.pt.battery.n_series for g in self.groups}
        if len(cell_counts) != 1:
            raise ValueError(
                f"airframe classes disagree on series cells {cell_counts}: "
                "the fleet shares one ESC telemetry bank")
        self.esc = EscTelem(EscTelemParams.from_dict(dev["esc_telem"]), n,
                            n_rotors, stream("sensor/esc_telem"),
                            cells=cell_counts.pop())

        self._imu_every = self._divisor(base_hz, self.imu.params.rate_hz, "imu")
        self._baro_mag_every = self._divisor(base_hz, BARO_MAG_HZ, "baro/mag")
        self._esc_every = self._divisor(base_hz, ESC_HZ, "esc")
        self._link_every = self._divisor(base_hz, LINK_HZ, "link")
        self._nav_every = self._divisor(base_hz, NAV_HZ, "nav telemetry")
        self._status_every = self._divisor(base_hz, STATUS_HZ, "status telemetry")
        self._health_every = self._divisor(base_hz, HEALTH_HZ, "health telemetry")
        self.heartbeat_every = (round(base_hz / heartbeat_hz)
                                if heartbeat_hz else 0)
        self.links: list[_FcuLink | None] = [None] * n
        self.mcs: list = [None] * n            # VirtualMCU per vehicle (P4-3)
        self._pads: list = [None] * n          # pad chargers (P4-4)

        starts = np.asarray([s for _, s, _ in specs], dtype=float)
        self.state = np.zeros((n, 13))
        self.state[:, 0:3] = starts
        self.state[:, 6] = 1.0

        self.weather = weather
        self.dryden = None
        if weather is not None and weather.wind_speed > 0.0:
            self.dryden = DrydenGusts(
                n, self._dt, max(weather.wind_speed, 1.0), starts[:, 2],
                weather.wind_speed, stream("dryden"))

        self.hals = [HalIO() for _ in range(n)]
        self.fcus = [Fcu(hal, overlay=fcu_overlay) for hal in self.hals]
        self._act_ports = [hal.port("actuators") for hal in self.hals]

        self._flying = np.zeros(n, dtype=bool)
        self._omega_r = np.zeros((n, n_rotors))
        self._wind = np.zeros((n, 3))
        self._wind_zero = np.zeros((n, 3))
        self._accel_buf = np.zeros((n, 3))
        esc_v = np.empty(n)
        for g in self.groups:
            esc_v[g.rows] = g.pt.battery.ocv(g.pt.battery.soc) - g.pt.battery.v1
        self._esc_out = (np.zeros((n, n_rotors)), esc_v, np.zeros(n))
        self._u_buf = np.zeros((n, n_rotors))
        self._z_buf = np.empty(n)
        self._cells_buf = np.empty((n, self.esc.cells))

    @staticmethod
    def _divisor(base_hz: int, rate_hz: float, name: str) -> int:
        div = base_hz / rate_hz
        if abs(div - round(div)) > 1e-9 or div < 1.0:
            raise ValueError(
                f"{name} rate {rate_hz} Hz does not divide base_hz={base_hz}")
        return round(div)

    @property
    def clock(self):
        return self._micro.clock

    # Single-airframe conveniences (the P3-bench shape every harness uses;
    # mixed fleets address self.groups directly).
    @property
    def cfg(self):
        return self.groups[0].cfg

    @property
    def plant(self):
        return self.groups[0].plant

    @property
    def motor(self):
        return self.groups[0].motor

    @property
    def pt(self):
        return self.groups[0].pt

    def attach_link(self, uav_id: str, *, latency_s: float = 0.02,
                    bandwidth_bps: float = 57600.0,
                    queue_max_bytes: int = 4096):
        """Wire one vehicle's FCU<->MC coop-link (P4-2). Returns the MC
        side ``(up, down)`` channel pair (up = MC->FCU). A linked vehicle
        gets heartbeats only over the wire — the engine's bench-style
        heartbeat placeholder applies to unlinked vehicles alone."""
        i = self.index[uav_id]
        if self.links[i] is not None:
            raise ValueError(f"vehicle {uav_id!r} already linked")
        link = _FcuLink(Channel(latency_s, bandwidth_bps, queue_max_bytes),
                        Channel(latency_s, bandwidth_bps, queue_max_bytes))
        self.links[i] = link
        return link.up, link.down

    def attach_mc(self, uav_id: str, mcu) -> None:
        """Host one vehicle's mission computer (sil/host.py VirtualMCU)
        in the §6 step-3 slot — it ticks on the micro clock between the
        FCU pipeline and the actuator latch."""
        i = self.index[uav_id]
        if self.mcs[i] is not None:
            raise ValueError(f"vehicle {uav_id!r} already has an MC")
        self.mcs[i] = mcu

    def set_pad(self, uav_id: str, pos, *, recharge_s: float = 90.0,
                radius: float = 25.0) -> None:
        """Pad charger (P4-4 rearm cycle, user decision: full land-dock):
        a vehicle DISARMED within ``radius`` of its pad recharges the ECM
        pack linearly to full over ``recharge_s``. Boundary-condition
        model — the charger circuit itself is out of scope, so SOC is
        driven directly (the cell relaxation v1 decays on its own)."""
        if recharge_s <= 0.0 or radius <= 0.0:
            raise ValueError("recharge_s and radius must be positive")
        i = self.index[uav_id]
        self._pads[i] = (float(pos[0]), float(pos[1]), float(pos[2]),
                         float(radius) ** 2, 1.0 / float(recharge_s))

    # ------------------------------------------------------------- world seam

    def run_macro_step(self, t: float, dt: float) -> None:
        """K micro-ticks inside one world macro step (World.micro hook)."""
        if abs(dt - self._world_dt) > 1e-12:
            raise ValueError(
                f"macro dt={dt!r} != engine world_dt={self._world_dt!r}")
        if abs(t - self.clock.now) > 0.5 * self._dt:
            raise ValueError(
                f"engine clock {self.clock.now:.6f} s out of step with "
                f"world t={t:.6f} s (engine must be installed at build)")
        self._micro.run_macro_step(t, dt)

    # -------------------------------------------------------------- micro tick

    def _devices(self) -> None:
        st = self.state
        k = self.clock.tick
        if k % self._imu_every == 0:
            quat = st[:, 6:10]
            if self._flying.any():
                # Exact truth CoM acceleration: the wrench at the latched
                # rotor speeds and ZOH wind (gravity included — the
                # hw.Imu.sample contract), per airframe class. Stand rows
                # read zero: the ground reaction the plant does not model
                # balances them.
                a_w = self._accel_buf
                for g in self.groups:
                    force, _ = g.plant.wrench(st[g.rows],
                                              self._omega_r[g.rows],
                                              self._wind[g.rows], RHO)
                    a_w[g.rows] = force / g.mass_col
                a_w[~self._flying] = 0.0
            else:
                a_w = np.zeros((self.n, 3))
            gyro, accel = self.imu.sample(quat, st[:, 10:13], a_w)
            for i, hal in enumerate(self.hals):
                hal.port("imu").write((tuple(gyro[i]), tuple(accel[i])))
        fix = self.gps.tick(st[:, 0:3], st[:, 3:6])
        if fix is not None:
            for i, hal in enumerate(self.hals):
                hal.port("gps").write((tuple(fix.pos[i]), tuple(fix.vel[i]),
                                       int(fix.fix_type[i]), fix.stamp_s))
        if k % self._baro_mag_every == 0:
            self._z_buf[:] = st[:, 2]
            p = self.baro.sample(self._z_buf)
            field = self.mag.sample(st[:, 6:10])
            for i, hal in enumerate(self.hals):
                hal.port("baro").write(float(p[i]))
                hal.port("mag").write(tuple(field[i]))
        if k % self._esc_every == 0:
            omega_r, v_bus, i_bus = self._esc_out
            v_cells = self._cells_buf
            for g in self.groups:
                v_cells[g.rows] = g.pt.battery.cell_voltages(v_bus[g.rows])
            f = self.esc.sample(omega_r, v_bus, i_bus, v_cells)
            for i, hal in enumerate(self.hals):
                hal.port("esc").write((tuple(f.rpm[i]), float(f.voltage[i]),
                                       float(f.current[i]),
                                       tuple(f.cells[i])))

    def _link_task(self, fcu: Fcu, link: _FcuLink, now: float, k: int) -> None:
        """FCU side of the wire: dispatch arrived commands (P3-R F10 wire
        enum tables, never literals), stream NAV/STATUS telemetry."""
        for frame in link.up.recv(now):
            for mid, payload in link.dec.feed(frame):
                if mid not in MSG:
                    continue
                name, v = decode_msg(mid, payload)
                if name == "HEARTBEAT":
                    fcu.on_heartbeat()
                elif name == "ARM":
                    fcu.cmd_arm()
                elif name == "DISARM":
                    fcu.cmd_disarm()
                elif name == "SET_MODE":
                    mode = MODE_NAMES.get(v["mode"])
                    if mode:
                        fcu.cmd_set_mode(mode)
                elif name == "VEL_SP":
                    fcu.cmd_velocity((v["vx"], v["vy"], v["vz"]), v["yaw"])
                elif name == "SET_HOME":
                    fcu.cmd_set_home((v["x"], v["y"], v["z"]))
                elif name == "BATT_RESET":
                    fcu.cmd_batt_reset()
        nav = fcu.nav
        if nav is None:
            return    # nothing worth telemetering until alignment
        if k % self._nav_every == 0:
            q, p, vel = nav.q, nav.pos, nav.vel
            link.down.send(encode_msg(
                "NAV", now, q[0], q[1], q[2], q[3],
                p[0], p[1], p[2], vel[0], vel[1], vel[2]), now)
        if k % self._status_every == 0:
            link.down.send(encode_msg(
                "STATUS", now, STATE_CODES[fcu.state], MODE_CODES[fcu.mode],
                FAILSAFE_CODES[fcu.failsafe], BATT_CODES[fcu.batt.state],
                nav.sigma_pos_h, fcu.batt.fraction()), now)
        if k % self._health_every == 0:
            cbit = fcu.cbit
            flags = (int(cbit.inhibit_arming)
                     | (int(cbit.inhibit_fire) << 1))
            link.down.send(encode_msg(
                "HEALTH", now, cbit.word(), flags,
                DEGRADED_CODES[cbit.degraded_mode()]), now)

    def _tick(self, now: float) -> None:
        # 1. devices sample truth (pre-step state, ZOH inputs)
        self._devices()

        # 2. per-vehicle flight software, vehicle order = pipeline order;
        # the coop-link drain/telemetry is the pipeline's last stage (§6
        # "drivers→estimator→controllers→mixer→PWM→CBIT→link").
        k = self.clock.tick
        heartbeat = (self.heartbeat_every and k % self.heartbeat_every == 0)
        link_due = k % self._link_every == 0
        for i, fcu in enumerate(self.fcus):
            link = self.links[i]
            if heartbeat and link is None:
                fcu.on_heartbeat()    # bench placeholder, unlinked only
            fcu.run_tick()
            if link is not None and link_due:
                self._link_task(fcu, link, now, k)

        # 3. MC tick if due (P4-3): hosted mission computers on the micro
        # clock, behind their crash fences (a dead MC is silent, the
        # simulation keeps running).
        for mcu in self.mcs:
            if mcu is not None and mcu.due(k):
                mcu.run_tick()

        # 4-7. latch actuators, gusts, powertrain, ONE batched RK4
        st = self.state
        flying = np.fromiter((f.state == ARMED for f in self.fcus),
                             dtype=bool, count=self.n)
        rises = flying & ~self._flying
        if rises.any():
            # Stand convention: motors pre-spun to hover at arming.
            for g in self.groups:
                lm = rises[g.rows]
                if lm.any():
                    g.motor.omega[lm] = g.w_hover
                    self._omega_r[g.rows[lm]] = g.w_hover
        falls = self._flying & ~flying
        if falls.any():
            # Touchdown/disarm: the (unmodeled) ground stops the vehicle.
            st[falls, 3:6] = 0.0
            st[falls, 10:13] = 0.0

        u = self._u_buf
        u[:] = 0.0
        for i in np.flatnonzero(flying):
            _, frame = self._act_ports[i].read()
            if frame is not None:
                u[i] = frame

        if self.dryden is not None:
            # One bank draw per tick from t=0, armed or not: per-vehicle
            # gust histories must not depend on when anyone armed (§4).
            gusts = gusts_to_world(st[:, 6:10], self.dryden.step())
            wind = self.weather.mean_wind_at(st[:, 2]) + gusts
        elif self.weather is not None and self.weather.wind_speed > 0.0:
            wind = self.weather.mean_wind_at(st[:, 2])
        else:
            wind = self._wind_zero

        # Powertrain + ONE batched RK4 per airframe class.
        omega_r, v_bus, i_bus = self._esc_out
        new = np.empty_like(st)
        for g in self.groups:
            om, vb, ib = g.pt.step(self._dt, u[g.rows])
            omega_r[g.rows] = om
            v_bus[g.rows] = vb
            i_bus[g.rows] = ib
            new[g.rows] = g.plant.step(st[g.rows], self._dt, om,
                                       wind[g.rows], RHO)
        self._omega_r = omega_r

        if not flying.all():
            frozen = ~flying
            new[frozen] = st[frozen]
            # Pad chargers (P4-4): docked = disarmed within pad radius.
            for i in np.flatnonzero(frozen):
                pad = self._pads[i]
                if pad is None:
                    continue
                dx = new[i, 0] - pad[0]
                dy = new[i, 1] - pad[1]
                dz = new[i, 2] - pad[2]
                if dx * dx + dy * dy + dz * dz <= pad[3]:
                    batt = self.groups[self._group_of[i]].pt.battery
                    li = self._local_of[i]
                    batt.soc[li] = min(1.0, batt.soc[li] + self._dt * pad[4])
        self.state = new
        self._wind = wind
        self._flying = flying
