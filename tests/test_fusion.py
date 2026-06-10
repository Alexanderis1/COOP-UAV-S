import numpy as np

from coopuavs.core.bus import MessageBus
from coopuavs.core.messages import Detection, Header, ThreatClass
from coopuavs.perception.fusion import FusionNode


def detection(t, pos, sensor="radar-1", sigma=20.0):
    return Detection(
        header=Header(stamp=t, frame_id="map"),
        sensor_id=sensor,
        position=np.asarray(pos, dtype=float),
        cov=np.eye(3) * sigma**2,
    )


def test_single_target_converges():
    bus = MessageBus()
    fusion = FusionNode(bus, rate_hz=5.0)
    out = []
    bus.subscribe("tracks", out.append)
    rng = np.random.default_rng(0)

    pos = np.array([1000.0, 2000.0, 500.0])
    vel = np.array([-40.0, -10.0, 0.0])
    t = 0.0
    for _ in range(50):
        bus.publish("detections", detection(t, pos + rng.normal(0, 20, 3)))
        fusion.maybe_update(t, 0.2)
        pos = pos + vel * 0.2
        t += 0.2

    tracks = out[-1].tracks
    assert len(tracks) == 1
    trk = tracks[0]
    assert np.linalg.norm(trk.position - pos) < 60.0
    assert np.linalg.norm(trk.velocity - vel) < 12.0


def test_two_sensors_one_object_one_track():
    """Same target seen by two sensors in the same cycle must not split."""
    bus = MessageBus()
    fusion = FusionNode(bus, rate_hz=5.0)
    out = []
    bus.subscribe("tracks", out.append)
    rng = np.random.default_rng(1)

    pos = np.array([500.0, 500.0, 300.0])
    t = 0.0
    for _ in range(30):
        bus.publish("detections", detection(t, pos + rng.normal(0, 15, 3), "radar-1"))
        bus.publish("detections", detection(t, pos + rng.normal(0, 30, 3), "eo-1", sigma=35.0))
        fusion.maybe_update(t, 0.2)
        pos = pos + np.array([-30.0, 0.0, 0.0]) * 0.2
        t += 0.2

    assert len(out[-1].tracks) == 1


def test_classification_belief_follows_evidence():
    bus = MessageBus()
    fusion = FusionNode(bus, rate_hz=5.0)
    out = []
    bus.subscribe("tracks", out.append)

    pos = np.array([200.0, 0.0, 300.0])
    t = 0.0
    for _ in range(20):
        det = detection(t, pos, "eo-1", sigma=10.0)
        det.class_likelihoods = {ThreatClass.DECOY: 0.7, ThreatClass.OWA_STRATEGIC: 0.2}
        bus.publish("detections", det)
        fusion.maybe_update(t, 0.2)
        t += 0.2

    trk = out[-1].tracks[0]
    assert trk.p_decoy > 0.5
