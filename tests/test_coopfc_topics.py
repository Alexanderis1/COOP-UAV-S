"""P3-1: coopfc/core/topics.py — uORB-style latest-value topic store.

Semantics under test (the determinism contract for intra-FCU dataflow):
publish overwrites (no queue — a slow subscriber sees only the newest
sample), each subscription tracks its own updated flag, reads are polled
at the subscriber's own task rate (no callbacks, nothing runs in the
publisher's stack), and declared message types are enforced at publish.
"""

from __future__ import annotations

import pytest

from coopuavs.coopfc.core.topics import TopicStore


class Msg:
    def __init__(self, v):
        self.v = v


def test_publish_then_read_latest_value():
    store = TopicStore()
    pub = store.advertise("imu_raw", Msg)
    sub = store.subscribe("imu_raw")
    assert sub.updated is False
    assert sub.read() is None

    m = Msg(1)
    pub.publish(m)
    assert sub.updated is True
    assert sub.read() is m
    assert sub.updated is False  # read consumed the flag
    assert sub.read() is m  # latest value remains readable


def test_overwrite_keeps_only_newest():
    store = TopicStore()
    pub = store.advertise("setpoint", Msg)
    sub = store.subscribe("setpoint")
    pub.publish(Msg(1))
    pub.publish(Msg(2))
    last = Msg(3)
    pub.publish(last)
    assert sub.read() is last


def test_subscribers_track_updated_independently():
    store = TopicStore()
    pub = store.advertise("att", Msg)
    fast = store.subscribe("att")
    slow = store.subscribe("att")
    pub.publish(Msg(1))
    assert fast.read().v == 1
    assert fast.updated is False
    assert slow.updated is True  # slow has not read yet
    assert slow.read().v == 1


def test_subscribe_before_advertise():
    store = TopicStore()
    sub = store.subscribe("gps_fix")
    pub = store.advertise("gps_fix", Msg)
    pub.publish(Msg(7))
    assert sub.updated is True
    assert sub.read().v == 7


def test_declared_type_enforced_on_publish():
    store = TopicStore()
    pub = store.advertise("health", Msg)
    with pytest.raises(TypeError):
        pub.publish("not a Msg")


def test_conflicting_advertise_type_rejected():
    store = TopicStore()
    store.advertise("health", Msg)

    class Other:
        pass

    with pytest.raises(ValueError):
        store.advertise("health", Other)
    # Re-advertising with the same type is fine (multi-publisher).
    store.advertise("health", Msg)


def test_untyped_topic_accepts_anything():
    store = TopicStore()
    pub = store.advertise("debug")
    sub = store.subscribe("debug")
    pub.publish({"k": 1})
    assert sub.read() == {"k": 1}


def test_generation_counts_publishes():
    store = TopicStore()
    pub = store.advertise("imu_raw", Msg)
    sub = store.subscribe("imu_raw")
    for i in range(5):
        pub.publish(Msg(i))
    # uORB-style: the subscriber can see it missed samples.
    assert sub.missed() == 4
    sub.read()
    assert sub.missed() == 0


def test_independent_stores_do_not_share():
    a, b = TopicStore(), TopicStore()
    a.advertise("x", Msg).publish(Msg(1))
    assert b.subscribe("x").read() is None
