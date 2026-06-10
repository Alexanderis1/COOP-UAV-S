"""Sensor-rate vs fusion-rate regressions.

The 10 Hz onboard seeker delivers two scans of the same target inside one
5 Hz fusion cycle. GNN's one-detection-per-track constraint only holds per
scan, so the second scan must update the track the first one fed — not
become a leftover that seeds a duplicate which then confirms and persists.
"""

import numpy as np

from coopuavs.core.bus import MessageBus
from coopuavs.core.messages import Detection, Header
from coopuavs.perception.fusion import FusionNode


def detection(t, pos, sensor="seeker-u1", sigma=3.0):
    return Detection(
        header=Header(stamp=t, frame_id="map"),
        sensor_id=sensor,
        position=np.asarray(pos, dtype=float),
        cov=np.eye(3) * sigma**2,
    )


def test_two_scans_per_cycle_yield_one_confirmed_track():
    bus = MessageBus()
    fusion = FusionNode(bus, rate_hz=5.0)
    out = []
    bus.subscribe("tracks", out.append)
    rng = np.random.default_rng(3)

    pos0 = np.array([400.0, 100.0, 80.0])
    vel = np.array([-30.0, 0.0, 0.0])
    for k in range(30):                        # 6 s at 5 Hz fusion
        t = 0.2 * k
        for ts in (t - 0.1, t):                # two 10 Hz scans in the buffer
            if ts < 0.0:
                continue
            bus.publish(
                "detections",
                detection(ts, pos0 + vel * ts + rng.normal(0.0, 3.0, 3)),
            )
        fusion.maybe_update(t, 0.2)

    tracks = out[-1].tracks
    assert len(tracks) == 1
    # Both scans of every cycle were absorbed by the single track.
    assert tracks[0].n_hits > 40
