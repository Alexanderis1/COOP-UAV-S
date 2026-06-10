"""Run controller — wall-clock execution of one scenario (SIM-RT-002/003).

The seam between the simulation and the serve layer (ICD_RUNTIME §5): the
server owns sockets and tick scheduling, the controller owns sim time.
``tick(wall_dt)`` converts elapsed wall time into whole fixed sim steps at
the current speed factor (carrying the fractional remainder), so for a
given seed the *sequence of sim states* is identical whatever the tick
pattern, pause history or speed schedule — mode switching does not alter
results (SIM-RT-002, verified by test).

Payload accessors return exactly the ICD data objects: ``frame()`` (§2.2),
``scene()`` (§2.1), ``truth()`` (§4), ``summary()`` (§2.4).
"""

from __future__ import annotations

import math

from .scenario import Scenario

MIN_SPEED = 0.1
MAX_SPEED = 10.0


class RunController:
    def __init__(self, scenario: Scenario):
        self.scenario = scenario
        self.world = scenario.world
        self.recorder = scenario.recorder
        self.speed = float(scenario.meta.get("speed", 1.0))
        self.posture = scenario.meta.get("posture", "human_confirm")
        self.status = "running"
        self._acc = 0.0                 # un-stepped sim time remainder
        self._frames_emitted = 0
        self._sync_run_info()

    # -- control (ICD §3) -----------------------------------------------------

    def pause(self) -> None:
        if self.status == "running":
            self.status = "paused"
            self._sync_run_info()

    def resume(self) -> None:
        if self.status == "paused":
            self.status = "running"
            self._sync_run_info()

    def set_speed(self, speed: float) -> None:
        """Set the time-scale factor, clamped to [MIN_SPEED, MAX_SPEED];
        raises ``ValueError`` on a non-finite value (nan would otherwise
        slip through the min/max clamp)."""
        speed = float(speed)
        if not math.isfinite(speed):
            raise ValueError(f"speed must be finite, got {speed}")
        self.speed = min(max(speed, MIN_SPEED), MAX_SPEED)
        self._sync_run_info()

    def set_posture(self, posture: str) -> None:
        """Switch the autonomy posture (ICD §3 ``set_posture``); raises
        ``ValueError`` on an unknown posture."""
        orch = getattr(self.scenario, "orchestrator", None)
        if orch is not None:
            orch.set_posture(posture)          # validates + resolves pendings
        elif posture not in ("human_confirm", "pre_authorized", "weapons_hold"):
            raise ValueError(f"unknown posture '{posture}'")
        self.posture = posture
        self._sync_run_info()

    def stop(self) -> None:
        if self.status != "done":
            self.status = "done"
            self._sync_run_info()

    # -- execution ----------------------------------------------------------------

    def tick(self, wall_dt: float) -> list[dict]:
        """Advance the sim by the wall time elapsed since the last tick and
        return the frame payloads produced (at the recorder cadence)."""
        if self.status != "running":
            return []
        self._sync_run_info()
        self._acc += wall_dt * self.speed
        dt = self.world.dt
        n_steps = int(self._acc / dt)
        self._acc -= n_steps * dt

        for _ in range(n_steps):
            self.world.step()
            if self._finished():
                self.status = "done"
                self._sync_run_info()
                break

        new = self.recorder.frames[self._frames_emitted:]
        self._frames_emitted = len(self.recorder.frames)
        return new

    def _finished(self) -> bool:
        w = self.world
        if w.t >= self.scenario.duration:
            return True
        return (
            not w._spawn_queue
            and bool(w.enemies)
            and not any(e.alive for e in w.enemies.values())
        )

    # -- payload accessors (ICD §2/§4) ------------------------------------------------

    def frame(self) -> dict:
        """Latest §2.2 frame data (for late joiners), with the *current*
        run block — status/speed/posture changes show even while paused."""
        if self.recorder.frames:
            frame = dict(self.recorder.frames[-1])
            frame["run"] = dict(self.recorder.run_info)
            return frame
        return self.recorder.snapshot(consume_events=False)

    def scene(self) -> dict:
        """§2.1 scene data."""
        return self.recorder.scene()

    def truth(self) -> dict:
        """§4 evaluation-channel data."""
        return self.scenario.eval_tracker.truth_payload()

    def summary(self) -> dict:
        """World summary + §4 metrics (the §2.4 ``summary`` data)."""
        out = self.world.summary()
        out["metrics"] = self.scenario.eval_tracker.metrics()
        return out

    # -- internals ------------------------------------------------------------------------

    def _sync_run_info(self) -> None:
        self.recorder.run_info = {
            "status": self.status,
            "speed": self.speed,
            "posture": self.posture,
        }
