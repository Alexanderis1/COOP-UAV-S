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
        ids = [uid for uid, _ in vehicles]
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

        raw = load_airframe(airframe)
        cfg = MultirotorParams.from_dict(raw)
        self.cfg = cfg
        self.esc = EscTelem(EscTelemParams.from_dict(dev["esc_telem"]), n,
                            cfg.n_rotors, stream("sensor/esc_telem"))
        self.plant = MultirotorPlant(cfg, n)
        self.motor = MotorEsc(n, cfg.n_rotors, **cfg.motor)
        battery = BatteryEcm(n, **cfg.battery)
        self.pt = Powertrain(self.motor, battery,
                             i_bus_max_a=raw["powertrain"]["i_bus_max_a"])

        self._imu_every = self._divisor(base_hz, self.imu.params.rate_hz, "imu")
        self._baro_mag_every = self._divisor(base_hz, BARO_MAG_HZ, "baro/mag")
        self._esc_every = self._divisor(base_hz, ESC_HZ, "esc")
        self.heartbeat_every = (round(base_hz / heartbeat_hz)
                                if heartbeat_hz else 0)

        starts = np.asarray([s for _, s in vehicles], dtype=float)
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
        self._omega_r = np.zeros((n, cfg.n_rotors))
        self._wind = np.zeros((n, 3))
        self._wind_zero = np.zeros((n, 3))
        self._esc_out = (np.zeros((n, cfg.n_rotors)),
                         battery.ocv(battery.soc) - battery.v1,
                         np.zeros(n))
        self._mass_col = self.plant.mass[:, None]
        self._w_hover = math.sqrt(cfg.mass * G / (cfg.n_rotors * cfg.kf))
        self._u_buf = np.zeros((n, cfg.n_rotors))
        self._z_buf = np.empty(n)

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
                # hw.Imu.sample contract). Stand rows read zero: the
                # ground reaction the plant does not model balances them.
                force, _ = self.plant.wrench(st, self._omega_r,
                                             self._wind, RHO)
                a_w = force / self._mass_col
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
            f = self.esc.sample(*self._esc_out)
            for i, hal in enumerate(self.hals):
                hal.port("esc").write((tuple(f.rpm[i]), float(f.voltage[i]),
                                       float(f.current[i])))

    def _tick(self, now: float) -> None:
        # 1. devices sample truth (pre-step state, ZOH inputs)
        self._devices()

        # 2. per-vehicle flight software, vehicle order = pipeline order
        heartbeat = (self.heartbeat_every
                     and self.clock.tick % self.heartbeat_every == 0)
        for fcu in self.fcus:
            if heartbeat:
                fcu.on_heartbeat()
            fcu.run_tick()

        # 3. MC tick if due — P4-3 seam.

        # 4-7. latch actuators, gusts, powertrain, ONE batched RK4
        st = self.state
        flying = np.fromiter((f.state == ARMED for f in self.fcus),
                             dtype=bool, count=self.n)
        rises = flying & ~self._flying
        if rises.any():
            # Stand convention: motors pre-spun to hover at arming.
            self.motor.omega[rises] = self._w_hover
            self._omega_r[rises] = self._w_hover
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

        omega_r, v_bus, i_bus = self.pt.step(self._dt, u)
        self._esc_out = (omega_r, v_bus, i_bus)
        self._omega_r = omega_r

        new = self.plant.step(st, self._dt, omega_r, wind, RHO)
        if not flying.all():
            frozen = ~flying
            new[frozen] = st[frozen]
        self.state = new
        self._wind = wind
        self._flying = flying
