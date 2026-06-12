"""P3-7: coop_link framing, heartbeat messages, latency/bandwidth queue.

Determinism is the load-bearing property: the channel is pure
arithmetic (no RNG), so arrival times are exact closed-form values the
tests pin literally. Corruption costs exactly one frame; the stream
resynchronizes on the next sync pair.
"""

from __future__ import annotations

import struct

from coopuavs.coopfc.link import (
    Channel, FrameDecoder, MSG, decode_msg, encode_frame, encode_msg,
)


def test_frame_round_trip_every_message_type():
    dec = FrameDecoder()
    for mid, (name, fmt, fields) in MSG.items():
        values = tuple(float(i + 1) if c == "d" or c == "f" else (i + 1)
                       for i, c in enumerate(fmt.lstrip("<")))
        frame = encode_msg(name, *values)
        got = dec.feed(frame)
        assert len(got) == 1 and got[0][0] == mid
        gname, payload = decode_msg(*got[0])
        assert gname == name and tuple(payload) == fields
    assert dec.bad_frames == 0


def test_decoder_handles_arbitrary_chunking():
    frames = [encode_msg("HEARTBEAT", 1.25, 7),
              encode_msg("VEL_SP", 2.0, 1.0, -2.0, 0.5, 0.1),
              encode_msg("ARM", 3.0)]
    stream = b"".join(frames)
    dec = FrameDecoder()
    got = []
    for i in range(len(stream)):           # one byte at a time
        got.extend(dec.feed(stream[i:i + 1]))
    assert [g[0] for g in got] == [0, 4, 1]
    assert dec.bad_frames == 0


def test_corrupted_frame_rejected_and_stream_resyncs():
    good = encode_msg("ARM", 1.0)
    bad = bytearray(encode_msg("DISARM", 2.0))
    bad[7] ^= 0xFF                          # flip a payload byte
    dec = FrameDecoder()
    got = dec.feed(bytes(bad) + good)
    assert [g[0] for g in got] == [1]       # only the good ARM frame
    assert dec.bad_frames == 1
    name, payload = decode_msg(*got[0])
    assert name == "ARM" and payload["stamp"] == 1.0


def test_garbage_between_frames_is_skipped():
    dec = FrameDecoder()
    stream = b"\x00\x12\x55" + encode_msg("ARM", 1.0) + b"\xAA\x55" \
        + encode_msg("DISARM", 2.0)
    got = dec.feed(stream)
    assert [g[0] for g in got] == [1, 2]


def test_channel_latency_and_serialization_exact():
    ch = Channel(latency_s=0.05, bandwidth_bps=8000.0)  # 1 byte/ms
    frame = encode_msg("ARM", 1.0)          # 19 bytes -> 19 ms on the wire
    n = len(frame)
    assert ch.send(frame, now=1.0)
    t_arrive = 1.0 + n * 8.0 / 8000.0 + 0.05
    assert ch.recv(t_arrive - 1e-6) == []   # not a tick earlier
    assert ch.recv(t_arrive) == [frame]


def test_channel_bandwidth_queues_back_to_back():
    ch = Channel(latency_s=0.0, bandwidth_bps=8000.0)
    f = encode_msg("ARM", 1.0)
    n = len(f)
    ser = n * 8.0 / 8000.0
    for _ in range(3):
        assert ch.send(f, now=0.0)          # burst at t=0
    # frames serialize FIFO: arrivals at exactly ser, 2*ser, 3*ser
    assert len(ch.recv(ser)) == 1
    assert ch.recv(2.0 * ser - 1e-6) == []
    assert len(ch.recv(2.0 * ser)) == 1
    assert len(ch.recv(3.0 * ser)) == 1


def test_channel_backpressure_refuses_newest_deterministically():
    f = encode_msg("ARM", 1.0)
    ch = Channel(latency_s=0.0, bandwidth_bps=8000.0,
                 queue_max_bytes=2 * len(f))
    assert ch.send(f, now=0.0)
    assert ch.send(f, now=0.0)
    assert not ch.send(f, now=0.0)          # third refused
    assert ch.dropped == 1 and ch.sent == 2
    ch.recv(1.0)                            # drain frees the budget
    assert ch.send(f, now=1.0)


def test_channel_idle_wire_does_not_accumulate_history():
    # A frame sent long after the wire went idle starts serializing at
    # `now`, not at the historical wire-free time.
    ch = Channel(latency_s=0.0, bandwidth_bps=8000.0)
    f = encode_msg("ARM", 1.0)
    ser = len(f) * 8.0 / 8000.0
    ch.send(f, now=0.0)
    ch.recv(10.0)
    ch.send(f, now=10.0)
    assert ch.recv(10.0 + ser) == [f]


def test_channel_determinism_run_twice():
    def run():
        ch = Channel(latency_s=0.013, bandwidth_bps=57600.0)
        arrivals = []
        for k in range(50):
            ch.send(encode_msg("VEL_SP", k * 0.02, 1.0, 2.0, 3.0, 0.1),
                    now=k * 0.02)
            arrivals.extend((k, len(ch.recv(k * 0.02))) for _ in (0,))
        arrivals.append(len(ch.recv(100.0)))
        return arrivals

    assert run() == run()


def test_heartbeat_message_carries_stamp_and_source():
    frame = encode_msg("HEARTBEAT", 12.34, 2)
    dec = FrameDecoder()
    ((mid, payload),) = dec.feed(frame)
    name, fields = decode_msg(mid, payload)
    assert name == "HEARTBEAT"
    assert fields["stamp"] == 12.34 and fields["source"] == 2


def test_bad_msg_id_and_length_guard():
    try:
        encode_frame(300, b"")
        raised = False
    except ValueError:
        raised = True
    assert raised
    # length field honest: payload of 5 -> len field 5
    frame = encode_frame(9, b"hello")
    (length,) = struct.unpack_from("<H", frame, 2)
    assert length == 5
