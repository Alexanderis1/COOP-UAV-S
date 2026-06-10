"""Central multi-sensor fusion node.

Consumes every :class:`Detection` on the ``detections`` topic (all sensors,
all modalities) and maintains the single system track picture published on
``tracks``. Association is global-nearest-neighbour over a Mahalanobis gate
solved with the Hungarian algorithm — the right starting point before
graduating to JPDA/LMB filters (see docs/RESEARCH.md §3).

Track lifecycle: tentative on first detection, confirmed after
``confirm_hits`` updates, dropped after ``max_coast`` seconds unseen.
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import linear_sum_assignment

from ..core.bus import MessageBus
from ..core.messages import Detection, Header, Track, TrackArray
from ..core.node import Node
from . import classification
from .tracking import KalmanTrack

TRACKS_TOPIC = "tracks"
GATE_MAHALANOBIS2 = 16.0   # ~chi2 0.999 for 3 dof


class FusionNode(Node):
    def __init__(
        self,
        bus: MessageBus,
        rate_hz: float = 5.0,
        confirm_hits: int = 3,
        max_coast: float = 5.0,
    ):
        super().__init__("fusion", bus, rate_hz=rate_hz)
        self.confirm_hits = confirm_hits
        self.max_coast = max_coast
        self.tracks: list[KalmanTrack] = []
        self._pending: list[Detection] = []
        self._pub = self.create_publisher(TRACKS_TOPIC)
        # Late-bound lambda: update() swaps _pending for a fresh list each cycle.
        self.create_subscription("detections", lambda det: self._pending.append(det))

    def update(self, t: float, dt: float) -> None:
        detections, self._pending = self._pending, []
        detections.sort(key=lambda d: d.header.stamp)

        for trk in self.tracks:
            trk.predict(t)

        self._associate_and_update(detections)

        self.tracks = [
            trk for trk in self.tracks if trk.time_since_update(t) < self.max_coast
        ]
        self._pub.publish(self._snapshot(t))

    # -- association --------------------------------------------------------------

    def _associate_and_update(self, detections: list[Detection]) -> None:
        """Associate per sensor scan: within one scan a target appears at
        most once, so GNN's one-detection-per-track constraint is correct
        scan-wise. A scan is one sensor at one timestamp — a sensor faster
        than the fusion rate (the 10 Hz seeker against this 5 Hz cycle)
        buffers several scans per cycle, which are absorbed sequentially
        so the later scan updates the track the earlier one fed instead of
        seeding a duplicate. Precise scans (radar) are processed first so
        they, not bearing-only pseudo-positions, seed new tracks; equal
        precision falls back to time order."""
        by_scan: dict[tuple[str, float], list[Detection]] = {}
        for det in detections:
            by_scan.setdefault((det.sensor_id, det.header.stamp), []).append(det)
        scans = sorted(
            by_scan.values(),
            key=lambda scan: (
                float(np.mean([np.trace(d.cov) for d in scan])),
                scan[0].header.stamp,
            ),
        )
        for scan in scans:
            for det in self._seed_clusters(self._associate_scan(scan)):
                trk = KalmanTrack(det)
                # The seeding detection is evidence like any other: its class
                # likelihoods and RF signature must enter the belief too.
                classification.update_track_classification(trk, det)
                self.tracks.append(trk)

    def _associate_scan(self, detections: list[Detection]) -> list[Detection]:
        """Update gated tracks, return the unassociated leftovers."""
        if not self.tracks:
            return detections

        cost = np.full((len(self.tracks), len(detections)), 1e6)
        for i, trk in enumerate(self.tracks):
            for j, det in enumerate(detections):
                d2 = trk.mahalanobis2(det)
                if d2 < GATE_MAHALANOBIS2:
                    cost[i, j] = d2

        rows, cols = linear_sum_assignment(cost)
        used = set()
        for i, j in zip(rows, cols):
            if cost[i, j] >= 1e6:
                continue
            self.tracks[i].update(detections[j])
            classification.update_track_classification(self.tracks[i], detections[j])
            used.add(j)
        return [d for j, d in enumerate(detections) if j not in used]

    def _seed_clusters(self, detections: list[Detection]) -> list[Detection]:
        """Collapse near-coincident leftovers so one object seeds one track.

        Distinguishability is set by the *tightest* measurement axis: two
        detections merge when their gap is within the sum of their smallest
        principal standard deviations. For bearing-only RF that is the
        cross-range sigma (hundreds of metres), not the ~9 km along-range
        pseudo-position sigma, so distinct simultaneous first contacts stay
        separate. Under-merging is self-healing — an unsupported duplicate
        track coasts out within ``max_coast`` seconds."""
        seeds: list[Detection] = []
        for det in detections:
            r_det = float(np.sqrt(np.linalg.eigvalsh(det.cov)[0]))
            for s in seeds:
                gap = float(np.linalg.norm(det.position - s.position))
                if gap < r_det + float(np.sqrt(np.linalg.eigvalsh(s.cov)[0])):
                    break
            else:
                seeds.append(det)
        return seeds

    # -- output ----------------------------------------------------------------------

    def _snapshot(self, t: float) -> TrackArray:
        out = []
        for trk in self.tracks:
            if trk.n_hits < self.confirm_hits:
                continue
            cb = classification.effective_belief(trk)
            out.append(
                Track(
                    header=Header(stamp=t),
                    track_id=trk.track_id,
                    position=trk.position.copy(),
                    velocity=trk.velocity.copy(),
                    cov=trk.P.copy(),
                    class_belief=dict(cb),
                    p_decoy=classification.p_decoy(trk),
                    n_hits=trk.n_hits,
                    age=t - trk.t_created,
                    time_since_update=trk.time_since_update(t),
                )
            )
        return TrackArray(header=Header(stamp=t), tracks=out)
