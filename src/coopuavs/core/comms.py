"""Simulated network layer for the C2/UAV datalinks (SIM-COM-001/002/003).

Every routed topic (C2↔UAV and UAV↔UAV traffic) passes through this model
instead of being delivered synchronously on the bus: each message gets a
per-link latency (configurable mean + jitter), a loss roll whose probability
rises with the platform's range from the base-station radio head, and any
scenario-scripted jam events covering the platform (area + time window).
Delivered messages sit in a queue drained at the start of every world step,
so a clearance token experiences exactly the transport the real interlock
would (SIM-COM-003) — a lost or late token leaves the shooter holding fire.

The default configuration is a near-perfect link (tiny constant latency,
zero loss): the v0.1 reference scenario keeps its verified behaviour, and
no RNG draws are consumed unless jitter or loss are actually configured.

Per-platform link quality (0..1) is computed from the recent delivery ratio
and the instantaneous loss state (jamming shows immediately), pushed onto
the platform as ``link_quality`` so its telemetry (``UavState.link``,
PHY-UAV-043) and the recorder frames reflect it.

This is a sim-side component: like the sensor models it may read true
platform positions — it *is* the radio channel physics.
"""

from __future__ import annotations

import heapq
import itertools
from collections import defaultdict, deque
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Callable

import numpy as np

if TYPE_CHECKING:  # avoid a core -> sim import at runtime
    from ..sim.world import World

# Topics that ride the simulated radio links. Everything else (sensor feeds
# into the GCS, the adjudicator's physics topics, eval taps) is wired.
ROUTED_TOPICS = {
    "engagement/tasks",
    "engagement/fire_request",
    "engagement/clearance",
    "tracks",
    "uav/state",
    "uav/command",
}

# Sliding window over which the delivery ratio feeds link quality, sim s.
QUALITY_WINDOW_S = 5.0


@dataclass
class JamEvent:
    """Scriptable area jamming (SIM-COM-002): extra loss probability for
    any platform inside ``area_radius`` of ``area_center`` during the
    [t_start, t_end] window."""

    t_start: float
    t_end: float
    area_center: tuple[float, float] = (0.0, 0.0)
    area_radius: float = 1000.0
    loss: float = 1.0

    def affects(self, t: float, pos: np.ndarray) -> bool:
        if not (self.t_start <= t <= self.t_end):
            return False
        dx = float(pos[0]) - float(self.area_center[0])
        dy = float(pos[1]) - float(self.area_center[1])
        return (dx * dx + dy * dy) ** 0.5 <= self.area_radius


class CommsModel:
    """Latency/loss/jam transport for the routed bus topics."""

    def __init__(
        self,
        world: "World",
        base_pos: tuple[float, float, float] = (0.0, 0.0, 0.0),
        latency_s: float = 0.01,
        jitter_s: float = 0.0,
        base_loss: float = 0.0,
        loss_per_km: float = 0.0,
        jam: list | tuple = (),
    ):
        self.world = world
        self.base_pos = np.asarray(base_pos, dtype=float)
        self.latency_s = float(latency_s)
        self.jitter_s = float(jitter_s)
        self.base_loss = float(base_loss)
        self.loss_per_km = float(loss_per_km)
        self.jam = [j if isinstance(j, JamEvent) else JamEvent(**j) for j in (jam or ())]

        self._endpoints: dict[str, Any] = {}     # endpoint id -> platform
        self._queue: list[tuple[float, int, Callable, Any]] = []
        self._seq = itertools.count()
        self._stats: dict[str, deque] = defaultdict(deque)   # id -> (t, ok)
        # Own stream (DESIGN_REVIEW 5.1): loss/jitter rolls must not depend
        # on how many sensor draws happened earlier in the tick.
        self._rng = world.rng_registry.stream("comms")

        world.bus.router = self
        world.comms = self

    # -- wiring ----------------------------------------------------------------

    def register_endpoint(self, endpoint: str, platform: Any) -> None:
        """Attach a radio to a platform (must expose ``.position``; gets
        ``.link_quality`` written back every step)."""
        self._endpoints[endpoint] = platform
        platform.link_quality = 1.0

    # -- bus router interface ----------------------------------------------------

    def routes(self, topic: str, sender: str | None, receiver: str | None) -> bool:
        # A node's own subscription to a topic it publishes is process-local
        # (e.g. a UAV hears peers on ``uav/state``): self-delivery never
        # rides the radio, so it must not roll loss twice on the same link
        # or bias the delivery-ratio window with loopback samples.
        if sender is not None and sender == receiver:
            return False
        return topic in ROUTED_TOPICS and (sender is not None or receiver is not None)

    def send(self, topic: str, msg: Any, callback: Callable,
             sender: str | None, receiver: str | None) -> None:
        """One delivery attempt over the link(s) between two endpoints."""
        t = self.world.t
        for endpoint in (sender, receiver):
            if endpoint is None:
                continue
            loss = self.link_loss(endpoint, t)
            ok = True
            if loss > 0.0:
                ok = float(self._rng.random()) >= loss
            self._note(endpoint, t, ok)
            if not ok:
                return                                # packet lost on this hop
        latency = self.latency_s
        if self.jitter_s > 0.0:
            latency = max(0.0, float(self._rng.normal(self.latency_s, self.jitter_s)))
        heapq.heappush(self._queue, (t + latency, next(self._seq), callback, msg))

    # -- world integration ---------------------------------------------------------

    def step(self, t: float) -> None:
        """Drain due deliveries (called by the world at the start of every
        step) and refresh per-platform link quality telemetry."""
        while self._queue and self._queue[0][0] <= t + 1e-9:
            _, _, callback, msg = heapq.heappop(self._queue)
            callback(msg)
        for endpoint, platform in self._endpoints.items():
            platform.link_quality = self.link_quality(endpoint, t)

    # -- link model (SIM-COM-001/002) ------------------------------------------------

    def link_loss(self, endpoint: str, t: float) -> float:
        """Instantaneous packet-loss probability for one platform's link."""
        platform = self._endpoints.get(endpoint)
        pos = np.asarray(platform.position, dtype=float) if platform is not None \
            else self.base_pos
        range_km = float(np.hypot(pos[0] - self.base_pos[0], pos[1] - self.base_pos[1])) / 1000.0
        loss = self.base_loss + self.loss_per_km * range_km
        for jam in self.jam:
            if jam.affects(t, pos):
                loss += jam.loss
        return float(min(max(loss, 0.0), 1.0))

    def link_quality(self, endpoint: str, t: float) -> float:
        """0..1 datalink quality: recent delivery ratio capped by the
        instantaneous loss state (jamming shows immediately)."""
        quality = 1.0 - self.link_loss(endpoint, t)
        window = self._stats.get(endpoint)
        if window:
            while window and window[0][0] < t - QUALITY_WINDOW_S:
                window.popleft()
            if window:
                ratio = sum(ok for _, ok in window) / len(window)
                quality = min(quality, ratio)
        return round(quality, 3)

    def _note(self, endpoint: str, t: float, ok: bool) -> None:
        self._stats[endpoint].append((t, ok))
