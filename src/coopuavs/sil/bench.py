"""P3-8 single-vehicle SIL bench: physics + P2 hw devices + one CoopFC FCU.

The micro-tick (800 Hz, the ORDERING contract shape):

1. devices sample the truth state (P2 models with their real error
   processes, latency and quantization) and write raw HAL frames at
   their own rates — IMU 400 Hz, GNSS device clock 800 Hz (fix delivery
   has the exact 120 ms latency), baro/mag 50 Hz, ESC telemetry 10 Hz;
2. ``Fcu.run_tick()`` (the flight software sees only HAL frames);
3. if armed: actuator port -> Powertrain -> plant RK4 step with
   ``wind = mean + Dryden gusts`` (gusts body-generated, rotated world).

While disarmed the plant is FROZEN (bench convention: the vehicle sits
on a stand at ``start``; devices keep sampling the static truth with
real noise, so alignment and EKF convergence run on honest data) and
the motors are pre-spun to hover speed at arming.

IMU ``accel_world`` is the finite-difference ``dv/dt`` over the last
plant step (the average kinematic acceleration over the sample period);
P4 threads the exact ``force_world/m`` of the wrench evaluation instead
(ORDERING §6 wiring note).

Determinism: one ``numpy.default_rng(seed)`` parent spawned per device
(the P2 fleet-size-invariant pattern; registry streams arrive with the
P4 world wiring), Dryden child included — run-twice is pinned by the
P3-8 suite.
"""

from __future__ import annotations

import math

import numpy as np

from coopuavs.coopfc.fcu import Fcu
from coopuavs.coopfc.hal import HalIO
from coopuavs.hw import params as hw_params
from coopuavs.hw.baro import Baro, BaroParams
from coopuavs.hw.esc_telem import EscTelem, EscTelemParams
from coopuavs.hw.gps import Gps, GpsParams
from coopuavs.hw.imu import Imu, ImuParams
from coopuavs.hw.mag import Mag, MagParams
from coopuavs.physics.dryden import DrydenGusts, gusts_to_world
from coopuavs.physics.battery import BatteryEcm
from coopuavs.physics.motor import MotorEsc
from coopuavs.physics.multirotor import MultirotorParams, MultirotorPlant
from coopuavs.physics.params import load_airframe
from coopuavs.physics.powertrain import Powertrain

TICK_HZ = 800
DT = 1.0 / TICK_HZ
RHO = 1.225
G = 9.81


class Bench:
    def __init__(self, seed: int = 0, start=(0.0, 0.0, 50.0),
                 wind_mean=(0.0, 0.0, 0.0), dryden_wind20: float | None = None,
                 heartbeat_hz: float = 10.0):
        rng = np.random.default_rng(seed)
        dev = hw_params.load_devices("interceptor_devices")
        self.imu = Imu(ImuParams.from_dict(dev["imu"]), 1, rng.spawn(1)[0])
        self.gps = Gps(GpsParams.from_dict(dev["gps"]), 1, rng.spawn(1)[0],
                       clock_hz=TICK_HZ)
        self.baro = Baro(BaroParams.from_dict(dev["baro"]), 1, rng.spawn(1)[0])
        self.mag = Mag(MagParams.from_dict(dev["mag"]), 1, rng.spawn(1)[0])

        cfg = MultirotorParams.from_dict(load_airframe("interceptor_quad"))
        self.cfg = cfg
        self.esc = EscTelem(EscTelemParams.from_dict(dev["esc_telem"]), 1,
                            cfg.n_rotors, rng.spawn(1)[0])
        self.plant = MultirotorPlant(cfg, 1)
        self.motor = MotorEsc(1, cfg.n_rotors, **cfg.motor)
        battery = BatteryEcm(1, **cfg.battery)
        self.pt = Powertrain(self.motor, battery, i_bus_max_a=350.0)

        self.wind_mean = np.array([wind_mean], dtype=float)
        self.dryden = None
        if dryden_wind20 is not None:
            airspeed = max(float(np.linalg.norm(wind_mean)), 1.0)
            self.dryden = DrydenGusts(1, DT, airspeed, start[2],
                                      dryden_wind20, rng.spawn(1)[0])

        self.hal = HalIO()
        self.fcu = Fcu(self.hal)
        self.heartbeat_every = (round(TICK_HZ / heartbeat_hz)
                                if heartbeat_hz else 0)

        self.state = np.zeros((1, 13))
        self.state[0, 0:3] = start
        self.state[0, 6] = 1.0
        self._v_prev2 = np.zeros((2, 3))     # velocity 1 and 2 steps back
        self._esc_out = (np.zeros((1, cfg.n_rotors)), np.array([50.0]),
                         np.array([0.0]))
        self.k = 0

    @property
    def now(self) -> float:
        return self.k * DT

    @property
    def flying(self) -> bool:
        return self.fcu.state == "ARMED"

    def _write_frames(self) -> None:
        s = self.state[0]
        k = self.k
        quat = self.state[:, 6:10]
        if k % 2 == 0:
            if self.flying:
                a_w = (self.state[0, 3:6] - self._v_prev2[0]) / (2.0 * DT)
                omega_b = self.state[:, 10:13]
            else:
                a_w = np.zeros(3)
                omega_b = np.zeros((1, 3))
            gyro, accel = self.imu.sample(quat, omega_b, a_w[None, :])
            self.hal.port("imu").write((tuple(gyro[0]), tuple(accel[0])))
        vel = self.state[:, 3:6] if self.flying else np.zeros((1, 3))
        fix = self.gps.tick(self.state[:, 0:3], vel)
        if fix is not None:
            self.hal.port("gps").write((tuple(fix.pos[0]), tuple(fix.vel[0]),
                                        int(fix.fix_type[0]), fix.stamp_s))
        if k % 16 == 0:
            p = self.baro.sample(np.array([s[2]]))
            self.hal.port("baro").write(float(p[0]))
            self.hal.port("mag").write(tuple(self.mag.sample(quat)[0]))
        if k % 80 == 0:
            omega_r, v_bus, i_bus = self._esc_out
            f = self.esc.sample(omega_r, v_bus, i_bus)
            self.hal.port("esc").write((tuple(f.rpm[0]), float(f.voltage[0]),
                                        float(f.current[0])))

    def tick(self) -> None:
        self._write_frames()
        if self.heartbeat_every and self.k % self.heartbeat_every == 0:
            self.fcu.on_heartbeat()
        self.fcu.run_tick()
        if self.flying:
            _, u = self.hal.port("actuators").read()
            self._v_prev2[0] = self._v_prev2[1]
            self._v_prev2[1] = self.state[0, 3:6]
            omega_r, v_bus, i_bus = self.pt.step(DT, np.array([u], dtype=float))
            self._esc_out = (omega_r, v_bus, i_bus)
            wind = self.wind_mean
            if self.dryden is not None:
                wind = wind + gusts_to_world(self.state[:, 6:10],
                                             self.dryden.step())
            self.state = self.plant.step(self.state, DT, omega_r, wind, RHO)
        self.k += 1

    def run(self, t_span: float, until=None) -> bool:
        for _ in range(round(t_span * TICK_HZ)):
            self.tick()
            if until is not None and until(self):
                return True
        return False

    def boot_and_arm(self, t_boot: float = 4.0) -> None:
        """Boot, align on real device noise, pass PBIT, arm, pre-spin."""
        ok = self.run(t_boot, until=lambda b: b.fcu.pbit_ok)
        if not ok:
            raise RuntimeError(
                f"PBIT not green after {t_boot} s: {self.fcu.pbit_reasons}")
        armed, why = self.fcu.cmd_arm()
        if not armed:
            raise RuntimeError(f"arming refused: {why}")
        w_h = math.sqrt(self.cfg.mass * G / (self.cfg.n_rotors * self.cfg.kf))
        self.motor.omega[:] = w_h
        self._v_prev2[:] = 0.0
