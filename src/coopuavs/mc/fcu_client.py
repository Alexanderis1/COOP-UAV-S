"""MC-side endpoint of the FCU coop-link (P4-2 stage-1 passthrough).

``FcuClient`` owns the wire protocol: it streams HEARTBEAT + VEL_SP up,
runs the autonomous arming flow off STATUS telemetry (STANDBY -> ARM,
ARMED -> OFFBOARD once a fresh setpoint is on the wire), and decodes
NAV/STATUS coming down. All u8 enum fields use the P3-R F10 registry
tables — never local literals.

``SitlBody`` wraps a client in the ``sim/physics.PointMass`` duck so the
legacy tactical agent (``InterceptorUav``) is untouched in stage 1:

- ``command_velocity(v)`` clips to ``max_speed`` (PointMass parity) and
  latches the setpoint; ``step(dt)`` — called once per agent update on
  every FSM path — ticks the client: drain telemetry, heartbeat, VEL_SP;
- ``.position`` / ``.velocity`` are the latest NAV **estimate** (f32
  wire precision), seeded with the home point until the first frame —
  the agent never reads truth (SIM-GT-001); truth lives in the
  ``sil.vehicle.FriendlyVehicle`` adapter on the world side.

Failsafe etiquette: while the FCU has a latched failsafe reason the
client keeps heartbeating and streaming setpoints but does not command
OFFBOARD back — RTL/LAND belong to the FCU until the operator (or P4-3
MC logic) clears the situation.

Timing: ``now`` is the world/macro clock injected by the host (the
scenario wires ``lambda: world.t``); the channel arithmetic is exact, so
MC frames sent at node time ``t`` reach the FCU inside the next macro
step's micro window after serialization + latency.
"""

from __future__ import annotations

import numpy as np

from coopuavs.coopfc.link.coop_link import (
    MODE_CODES,
    MODE_NAMES,
    MSG,
    STATE_NAMES,
    FrameDecoder,
    decode_msg,
    encode_msg,
)

HEARTBEAT_PERIOD_S = 0.1
ARM_RETRY_S = 1.0
MC_SOURCE = 1          # HEARTBEAT.source: 0 = FCU, 1 = MC


class FcuClient:
    """One vehicle's MC-side link endpoint (up = MC->FCU, down = FCU->MC)."""

    def __init__(self, up, down):
        self._up = up
        self._down = down
        self._dec = FrameDecoder()
        self.nav: dict | None = None        # latest NAV fields
        self.status: dict | None = None     # latest STATUS fields
        self._last_hb: float | None = None
        self._last_arm: float | None = None

    # -- decoded telemetry views ------------------------------------------------

    @property
    def state(self) -> str:
        return STATE_NAMES[self.status["state"]] if self.status else ""

    @property
    def mode(self) -> str:
        return MODE_NAMES[self.status["mode"]] if self.status else ""

    @property
    def failsafe_active(self) -> bool:
        return bool(self.status) and self.status["failsafe"] != 0

    # -- wire ----------------------------------------------------------------------

    def poll(self, now: float) -> None:
        """Drain everything fully arrived by ``now``."""
        for frame in self._down.recv(now):
            for mid, payload in self._dec.feed(frame):
                if mid not in MSG:
                    continue
                name, vals = decode_msg(mid, payload)
                if name == "NAV":
                    self.nav = vals
                elif name == "STATUS":
                    self.status = vals

    def tick(self, now: float, v_cmd, yaw_sp: float = 0.0) -> None:
        """One MC cycle: telemetry in, heartbeat + setpoint + arming out."""
        self.poll(now)
        if self._last_hb is None or now - self._last_hb >= HEARTBEAT_PERIOD_S - 1e-9:
            self._up.send(encode_msg("HEARTBEAT", now, MC_SOURCE), now)
            self._last_hb = now
        # Setpoint first: when SET_MODE(OFFBOARD) drains in the same FCU
        # link batch, the FIFO wire guarantees the fresh VEL_SP lands first.
        self._up.send(encode_msg("VEL_SP", now, float(v_cmd[0]),
                                 float(v_cmd[1]), float(v_cmd[2]),
                                 float(yaw_sp)), now)
        if self.state == "STANDBY":
            if self._last_arm is None or now - self._last_arm >= ARM_RETRY_S:
                self._up.send(encode_msg("ARM", now), now)
                self._last_arm = now
        elif (self.state == "ARMED" and self.mode != "OFFBOARD"
                and not self.failsafe_active):
            self._up.send(encode_msg("SET_MODE", now,
                                     MODE_CODES["OFFBOARD"]), now)


class SitlBody:
    """PointMass-duck flight interface backed by a remote FCU (stage 1)."""

    def __init__(self, client: FcuClient, home, max_speed: float, clock,
                 max_accel: float = 20.0):
        self._client = client
        self._clock = clock
        self.max_speed = float(max_speed)
        self.max_accel = float(max_accel)   # PointMass parity (unused here)
        self.position = np.asarray(home, dtype=float).copy()
        self.velocity = np.zeros(3)
        self.cmd_velocity = np.zeros(3)

    def command_velocity(self, v_cmd: np.ndarray) -> None:
        v_cmd = np.asarray(v_cmd, dtype=float)
        speed = float(np.linalg.norm(v_cmd))
        if speed > self.max_speed:
            v_cmd = v_cmd * (self.max_speed / speed)
        self.cmd_velocity = v_cmd

    def step(self, dt: float) -> None:
        now = self._clock()
        self._client.tick(now, self.cmd_velocity)
        nav = self._client.nav
        if nav is not None:
            self.position = np.array([nav["px"], nav["py"], nav["pz"]])
            self.velocity = np.array([nav["vx"], nav["vy"], nav["vz"]])
