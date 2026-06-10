"""Support-role speed sourcing: the relay/herd decision keys on the
*shooter's* ability to win the tail chase, and cutoff posts are computed
with each blocker's own speed from shared telemetry."""

import numpy as np

from coopuavs.core.bus import MessageBus
from coopuavs.core.messages import (
    EngagementTask,
    Header,
    Track,
    TrackArray,
    UavMode,
    UavState,
)
from coopuavs.interceptors import cooperation
from coopuavs.interceptors.effectors import net_gun
from coopuavs.interceptors.uav import InterceptorUav


def support_uav(bus: MessageBus) -> InterceptorUav:
    uav = InterceptorUav("net-1", bus, home=np.zeros(3),
                         effector=net_gun(), max_speed=50.0)
    uav.body.position = np.array([0.0, -500.0, 300.0])
    return uav


def task() -> EngagementTask:
    return EngagementTask(header=Header(stamp=0.0), task_id=3, track_id=1,
                          shooter_id="hawk-1", support_ids=["net-1"],
                          desired_kill_box=np.array([0.0, 2000.0, 0.0]))


def track_array(speed: float) -> TrackArray:
    return TrackArray(header=Header(stamp=0.0), tracks=[Track(
        header=Header(stamp=0.0), track_id=1,
        position=np.array([0.0, 3000.0, 800.0]),
        velocity=np.array([0.0, -speed, 0.0]),
    )])


def test_relay_decision_uses_the_shooters_speed():
    bus = MessageBus()
    uav = support_uav(bus)
    bus.publish("uav/state", UavState(header=Header(stamp=0.0), uav_id="hawk-1",
                                      position=np.array([0.0, 500.0, 300.0]),
                                      max_speed=80.0))
    bus.publish("engagement/tasks", [task()])

    # 60 m/s outruns this 50 m/s net carrier but NOT the 80 m/s shooter:
    # the right support play is herding pressure, not a blocking post.
    bus.publish("tracks", track_array(60.0))
    uav.update(0.0, 0.1)
    assert uav.mode == UavMode.HERDING

    # 100 m/s outruns the shooter too — now the relay is on.
    bus.publish("tracks", track_array(100.0))
    uav.update(0.1, 0.1)
    assert uav.mode == UavMode.BLOCKING


def test_cutoff_points_use_each_blockers_own_speed():
    trk = Track(header=Header(stamp=0.0), track_id=1,
                position=np.array([0.0, 3000.0, 800.0]),
                velocity=np.array([0.0, -100.0, 0.0]))
    post_pos = np.array([0.0, 2000.0, 300.0])
    fast = cooperation.cutoff_points(trk, 1, [post_pos], [300.0])
    slow = cooperation.cutoff_points(trk, 1, [post_pos], [1.0])
    # The fast blocker claims an early corridor point; the slow one is
    # pushed down-corridor (here all the way to the fallback horizon post).
    assert (np.linalg.norm(fast[0] - trk.position)
            < np.linalg.norm(slow[0] - trk.position))


def test_cutoff_points_pair_each_blocker_with_its_own_speed():
    """A mixed-speed pair must zip position[i] with speed[i]: broadcasting
    speeds[0] to everyone or mispairing the zip claims posts the slow
    airframe cannot hold (or wastes the fast one's reach)."""
    trk = Track(header=Header(stamp=0.0), track_id=1,
                position=np.array([0.0, 3000.0, 800.0]),
                velocity=np.array([0.0, -100.0, 0.0]))
    pos = np.array([0.0, 2000.0, 300.0])
    mixed = cooperation.cutoff_points(trk, 2, [pos, pos.copy()], [1.0, 300.0])
    fast = cooperation.cutoff_points(trk, 2, [pos, pos.copy()], [300.0, 300.0])
    # The slow first blocker falls back to the horizon post...
    assert (np.linalg.norm(mixed[0] - trk.position)
            > np.linalg.norm(fast[0] - trk.position))
    # ...while the fast second blocker still claims the earliest reachable
    # slot — earlier than when it queues behind an equally fast peer.
    assert (np.linalg.norm(mixed[1] - trk.position)
            < np.linalg.norm(fast[1] - trk.position))
