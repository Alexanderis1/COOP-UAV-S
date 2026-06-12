"""CoopLink: byte-honest framing + deterministic latency/bandwidth channel.

The FCU <-> Mission Computer serial link (PHY-UAV-010 split). P4 wires
one ``Channel`` per direction between the VirtualMCU and the MC-side
tactical stack; P3-7 ships the protocol and the channel model.

Framing (MAVLink-shaped, deliberately simpler):

    SYNC0 SYNC1 | len u16 LE (payload bytes) | msg_id u8 | payload | crc32 u32 LE

CRC32 (zlib) over ``len|msg_id|payload``. The decoder is a streaming
byte parser: feed any chunking, get whole frames; a corrupted frame
costs exactly the bytes up to its bad CRC (resync scans for the next
SYNC pair) and is tallied in ``bad_frames`` — the P5 CBIT LINK monitor
consumes that.

Channel model (deterministic by construction — pure arithmetic, no RNG;
stochastic loss/jitter are sim-side comms concerns, P4):

- serialization: a frame occupies the wire for ``8 len / bandwidth_bps``
  seconds, frames queue FIFO behind the previous wire-end;
- latency: every byte additionally travels ``latency_s``;
- backpressure: at most ``queue_max_bytes`` may be in flight; a send
  that would exceed it is REFUSED (deterministic sender-side drop,
  tallied in ``dropped``) — refusing the newest frame keeps every
  accepted frame's timing independent of later traffic.

Message payloads are ``struct``-packed little-endian (registry MSG);
all multi-byte values LE. Heartbeats are ordinary HEARTBEAT messages —
the FCU side feeds ``Fcu.on_heartbeat`` from them (P4 wiring).
"""

from __future__ import annotations

import struct
import zlib
from collections import deque

SYNC0, SYNC1 = 0x55, 0xAA
_HDR = 4          # sync0 sync1 len_u16
_CRC = 4

# msg_id -> (name, struct format for the payload, field names)
MSG = {
    0: ("HEARTBEAT", "<dB", ("stamp", "source")),
    1: ("ARM", "<d", ("stamp",)),
    2: ("DISARM", "<d", ("stamp",)),
    3: ("SET_MODE", "<dB", ("stamp", "mode")),
    4: ("VEL_SP", "<dffff", ("stamp", "vx", "vy", "vz", "yaw")),
    5: ("SET_HOME", "<dfff", ("stamp", "x", "y", "z")),
    6: ("STATUS", "<dBBBBf", ("stamp", "state", "mode", "failsafe",
                              "batt", "sigma_pos_h")),
    7: ("NAV", "<dffffffffff", ("stamp", "qw", "qx", "qy", "qz",
                                "px", "py", "pz", "vx", "vy", "vz")),
}
_BY_NAME = {name: (mid, fmt) for mid, (name, fmt, _) in MSG.items()}


def encode_msg(name: str, *values) -> bytes:
    mid, fmt = _BY_NAME[name]
    return encode_frame(mid, struct.pack(fmt, *values))


def decode_msg(msg_id: int, payload: bytes):
    name, fmt, fields = MSG[msg_id]
    return name, dict(zip(fields, struct.unpack(fmt, payload)))


def encode_frame(msg_id: int, payload: bytes) -> bytes:
    if not 0 <= msg_id <= 255:
        raise ValueError(f"msg_id must fit u8, got {msg_id!r}")
    body = struct.pack("<HB", len(payload), msg_id) + payload
    crc = zlib.crc32(body)
    return bytes((SYNC0, SYNC1)) + body + struct.pack("<I", crc)


class FrameDecoder:
    """Streaming parser; feed() returns completed (msg_id, payload)."""

    def __init__(self):
        self._buf = bytearray()
        self.bad_frames = 0

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self._buf.extend(data)
        out = []
        while True:
            sync = self._buf.find(bytes((SYNC0, SYNC1)))
            if sync < 0:
                # keep a trailing lone SYNC0 (its partner may follow)
                del self._buf[:max(0, len(self._buf) - 1)]
                return out
            if sync:
                del self._buf[:sync]
            if len(self._buf) < _HDR + 1:
                return out
            (length,) = struct.unpack_from("<H", self._buf, 2)
            total = _HDR + 1 + length + _CRC
            if len(self._buf) < total:
                return out
            body = bytes(self._buf[2:_HDR + 1 + length])
            (crc,) = struct.unpack_from("<I", self._buf, _HDR + 1 + length)
            if zlib.crc32(body) == crc:
                out.append((self._buf[4], body[3:]))
                del self._buf[:total]
            else:
                self.bad_frames += 1
                del self._buf[:2]      # resync past this SYNC pair


class Channel:
    """One direction of the link: FIFO wire with serialization delay,
    fixed latency, and bounded in-flight bytes (see module docstring)."""

    def __init__(self, latency_s: float = 0.02,
                 bandwidth_bps: float = 57600.0,
                 queue_max_bytes: int = 4096):
        if latency_s < 0.0 or bandwidth_bps <= 0.0 or queue_max_bytes < 1:
            raise ValueError("bad channel parameters")
        self.latency_s = float(latency_s)
        self.bandwidth_bps = float(bandwidth_bps)
        self.queue_max_bytes = int(queue_max_bytes)
        self._wire: deque = deque()      # (arrival_time, frame_bytes)
        self._wire_free_at = 0.0         # when the wire finishes the last frame
        self._in_flight = 0
        self.sent = 0
        self.dropped = 0

    def send(self, frame: bytes, now: float) -> bool:
        """Queue a frame; False = refused (in-flight budget exceeded)."""
        n = len(frame)
        if self._in_flight + n > self.queue_max_bytes:
            self.dropped += 1
            return False
        tx_end = max(self._wire_free_at, now) + 8.0 * n / self.bandwidth_bps
        self._wire_free_at = tx_end
        self._wire.append((tx_end + self.latency_s, frame))
        self._in_flight += n
        self.sent += 1
        return True

    def recv(self, now: float) -> list[bytes]:
        """All frames fully arrived by `now`, in send order."""
        out = []
        while self._wire and self._wire[0][0] <= now + 1e-12:
            _, frame = self._wire.popleft()
            self._in_flight -= len(frame)
            out.append(frame)
        return out
