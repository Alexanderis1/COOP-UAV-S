"""CBIT engine: dictionary-driven fault state (raise/debounce/latch).

One engine per processor (the FCU hosts one; the MC app hosts a small
one for MC-side faults, P5-4). Monitors call ``report(code, active,
now)`` at their own cadence — the engine is deliberately dumb: ALL
behavior comes from the dictionary row, so the fault matrix is the
single source of truth the P5 gate covers.

Plain Python on purpose (coopfc import fence; the fast monitors run at
50 Hz inside the flight-software tick — no numpy, no allocation beyond
the per-fault state dict created lazily on first report).
"""

from __future__ import annotations

from .dictionary import ACT_NONE, FAULTS, act_rank


class _FaultState:
    __slots__ = ("pending_since", "raised", "raised_at", "detail")

    def __init__(self):
        self.pending_since: float | None = None
        self.raised = False
        self.raised_at: float | None = None
        self.detail = ""


class CbitEngine:
    def __init__(self, faults: dict | None = None):
        self._faults = FAULTS if faults is None else faults
        self._state: dict[str, _FaultState] = {}
        # In bit order so faults()/word()/snapshot() iterate one stable
        # order everywhere (determinism contract for telemetry).
        self._codes = sorted(self._faults, key=lambda c: self._faults[c].bit)

    def _st(self, code: str) -> _FaultState:
        if code not in self._faults:
            raise KeyError(f"unknown fault code {code!r}")
        st = self._state.get(code)
        if st is None:
            st = self._state[code] = _FaultState()
        return st

    # ------------------------------------------------------------ reporting

    def report(self, code: str, active: bool, now: float,
               detail: str = "") -> None:
        """One monitor observation. Debounce on raise: the condition
        must hold continuously for the row's ``debounce_s`` of monitor
        time. Non-latching faults clear on the first healthy report;
        latching faults hold until ``clear()`` (ground op)."""
        spec = self._faults[code] if code in self._faults else None
        if spec is None:
            raise KeyError(f"unknown fault code {code!r}")
        st = self._st(code)
        if active:
            if detail:
                st.detail = detail
            if st.raised:
                return
            if st.pending_since is None:
                st.pending_since = now
            if now - st.pending_since >= spec.debounce_s:
                st.raised = True
                st.raised_at = now
        else:
            st.pending_since = None
            if st.raised and not spec.latching:
                st.raised = False
                st.raised_at = None
                st.detail = ""

    def clear(self, code: str) -> None:
        """Ground/maintenance clear (pack swap, repair): drops the fault
        regardless of latching. No-op when not raised."""
        if code not in self._faults:
            raise KeyError(f"unknown fault code {code!r}")
        st = self._state.get(code)
        if st is not None:
            st.pending_since = None
            st.raised = False
            st.raised_at = None
            st.detail = ""

    # ----------------------------------------------------------- aggregates

    def raised(self, code: str) -> bool:
        if code not in self._faults:
            raise KeyError(f"unknown fault code {code!r}")
        st = self._state.get(code)
        return st is not None and st.raised

    def faults(self) -> list[str]:
        """Raised codes in bit order."""
        return [c for c in self._codes if self.raised(c)]

    def word(self) -> int:
        """u32 fault bitmask (the HEALTH wire field, P5-1c)."""
        w = 0
        for code in self._codes:
            if self.raised(code):
                w |= 1 << self._faults[code].bit
        return w

    @property
    def inhibit_arming(self) -> bool:
        return any(self._faults[c].inhibit_arming for c in self._codes
                   if self.raised(c))

    @property
    def inhibit_fire(self) -> bool:
        return any(self._faults[c].inhibit_fire for c in self._codes
                   if self.raised(c))

    def degraded(self) -> tuple[str, str]:
        """Highest-priority CBIT response among raised faults, with the
        fault code that commands it (the failsafe reason the FCU
        latches). Mirror rows are excluded — the legacy failsafe chain
        owns their response (dictionary docstring; bit-identity
        contract). Ties go to the lowest bit (stable)."""
        best, cause = ACT_NONE, ""
        for code in self._codes:
            spec = self._faults[code]
            if spec.mirror or not self.raised(code):
                continue
            if act_rank(spec.degraded_mode) > act_rank(best):
                best, cause = spec.degraded_mode, code
        return best, cause

    def degraded_mode(self) -> str:
        return self.degraded()[0]

    def arming_inhibitors(self) -> list[str]:
        """Raised inhibit_arming faults, bit order (cmd_arm refusal text)."""
        return [c for c in self._codes
                if self.raised(c) and self._faults[c].inhibit_arming]

    # ------------------------------------------------------------- snapshot

    def snapshot(self) -> dict:
        """Raised faults -> {since, latched, detail} (UavHealth payload
        shape, P5-4). ``since`` is condition onset, not raise time —
        detection latency stays visible to the consumer."""
        out = {}
        for code in self._codes:
            st = self._state.get(code)
            if st is None or not st.raised:
                continue
            out[code] = {
                "since": st.pending_since,
                "latched": self._faults[code].latching,
                "detail": st.detail,
            }
        return out
