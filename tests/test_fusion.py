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


def rf_detection(t, pos, sensor_pos, sensor="rf-1"):
    """Bearing-only pseudo-position: tight across the line of sight,
    enormous along it (mirrors RfSensor's covariance construction)."""
    rel = np.asarray(pos, dtype=float) - np.asarray(sensor_pos, dtype=float)
    los = rel / np.linalg.norm(rel)
    cross, along = 330.0, 9000.0
    cov = np.eye(3) * cross**2 + np.outer(los, los) * (along**2 - cross**2)
    return Detection(
        header=Header(stamp=t, frame_id="map"),
        sensor_id=sensor,
        position=np.asarray(pos, dtype=float),
        cov=cov,
    )


def test_distant_rf_first_contacts_seed_separate_tracks():
    """Two simultaneous bearing-only contacts kilometres apart are distinct
    objects; the seed gate must scale with the tight cross-range sigma, not
    the ~9 km along-range sigma (which used to merge anything within
    ~12.7 km)."""
    fusion = FusionNode(MessageBus(), rate_hz=5.0)
    a = rf_detection(0.0, [8000.0, 0.0, 1000.0], [0.0, 0.0, 0.0])
    b = rf_detection(0.0, [8000.0, 4000.0, 1000.0], [0.0, 0.0, 0.0])
    assert len(fusion._seed_clusters([a, b])) == 2


def test_seed_clusters_merges_noise_level_duplicates():
    fusion = FusionNode(MessageBus(), rate_hz=5.0)
    a = detection(0.0, [1000.0, 0.0, 500.0], sigma=60.0)
    b = detection(0.0, [1050.0, 0.0, 500.0], sigma=60.0)
    assert len(fusion._seed_clusters([a, b])) == 1


def test_seeding_detection_classification_is_kept():
    """The first detection of an object often carries the best evidence
    (EO/IR structure ID, RF fingerprint) — seeding the track must not
    silently discard it."""
    bus = MessageBus()
    fusion = FusionNode(bus, rate_hz=5.0)

    det = detection(0.0, [200.0, 0.0, 300.0], "eo-1", sigma=10.0)
    det.class_likelihoods = {ThreatClass.FPV: 0.9}
    bus.publish("detections", det)
    fusion.maybe_update(0.0, 0.2)

    assert len(fusion.tracks) == 1
    belief = fusion.tracks[0].class_belief
    assert belief and belief[ThreatClass.FPV] > 0.5
