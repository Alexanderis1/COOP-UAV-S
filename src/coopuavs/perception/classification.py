"""Threat classification and decoy discrimination.

Maintains a Bayesian belief over :class:`ThreatClass` per track, fused from
whatever evidence arrives:

* sensor class likelihoods (EO/IR structure ID, acoustic engine signature);
* RF signature consistency — the Gerbera tactic means ``sig-owa-a`` is
  evidence for {OWA_STRATEGIC, DECOY} *jointly*, never between them;
* kinematic consistency — a track whose speed profile contradicts a class
  hypothesis loses belief in it.

The output that matters operationally is ``p_decoy``: the engagement layer
deprioritises (but does not ignore) likely decoys, because shooting decoys
is exactly the ammunition-exhaustion outcome the enemy wants.
"""

from __future__ import annotations

import numpy as np

from ..core.messages import Detection, ThreatClass
from ..threats.enemy_drone import THREAT_PROFILES
from .tracking import KalmanTrack

CONCRETE_CLASSES = [c for c in ThreatClass if c is not ThreatClass.UNKNOWN]

# What each observed RF signature says, as a likelihood per class.
_RF_EVIDENCE: dict[str, dict[ThreatClass, float]] = {
    "sig-owa-a": {ThreatClass.OWA_STRATEGIC: 0.45, ThreatClass.DECOY: 0.45},
    "sig-owa-jet": {ThreatClass.OWA_JET: 0.85},
    "sig-fpv": {ThreatClass.FPV: 0.85},
    "sig-loiter": {ThreatClass.LOITERING: 0.85},
}
_RF_FLOOR = 0.03


def uniform_belief() -> dict[ThreatClass, float]:
    p = 1.0 / len(CONCRETE_CLASSES)
    return {c: p for c in CONCRETE_CLASSES}


def _normalise(belief: dict[ThreatClass, float]) -> dict[ThreatClass, float]:
    total = sum(belief.values())
    if total <= 0.0:
        return uniform_belief()
    return {c: v / total for c, v in belief.items()}


def bayes_update(
    belief: dict[ThreatClass, float], likelihoods: dict[ThreatClass, float]
) -> dict[ThreatClass, float]:
    posterior = {c: belief.get(c, 0.0) * likelihoods.get(c, 1e-3) for c in CONCRETE_CLASSES}
    return _normalise(posterior)


def update_track_classification(track: KalmanTrack, det: Detection) -> None:
    if not track.class_belief:
        track.class_belief = uniform_belief()
    if det.class_likelihoods:
        track.class_belief = bayes_update(track.class_belief, det.class_likelihoods)
    if det.rf_signature is not None:
        evidence = _RF_EVIDENCE.get(det.rf_signature, {})
        lik = {c: evidence.get(c, _RF_FLOOR) for c in CONCRETE_CLASSES}
        track.class_belief = bayes_update(track.class_belief, lik)


def kinematic_likelihood(track: KalmanTrack) -> dict[ThreatClass, float]:
    """Consistency of the estimated speed with each class's nominal speed.

    This is *state*, not new evidence: it must be blended once per readout
    (see :func:`effective_belief`), never multiplied into the accumulated
    belief each cycle — that double-counts the same fact and saturates the
    posterior within seconds.
    """
    if track.n_hits < 5:
        return {c: 1.0 for c in CONCRETE_CLASSES}
    speed = float((track.velocity @ track.velocity) ** 0.5)
    lik = {}
    for c in CONCRETE_CLASSES:
        nominal = THREAT_PROFILES[c].speed
        ratio = (speed - nominal) / (0.35 * nominal + 5.0)
        lik[c] = max(1e-3, float(np.exp(-0.5 * ratio * ratio)))
    return lik


def effective_belief(track: KalmanTrack) -> dict[ThreatClass, float]:
    """Accumulated sensor belief blended with current kinematic consistency."""
    base = track.class_belief or uniform_belief()
    return bayes_update(base, kinematic_likelihood(track))


def p_decoy(track: KalmanTrack) -> float:
    return effective_belief(track).get(ThreatClass.DECOY, 0.0)
