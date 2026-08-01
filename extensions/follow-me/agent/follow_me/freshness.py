"""A monotonic freshness gate for one telemetry input.

A behaviour that steers the aircraft from cached telemetry needs the same
question answered about every input it caches: *did this arrive recently
enough to still be true?* A boolean "have we ever seen one" latch cannot
answer it. Once set, a latch reports the input healthy forever, so a
subscription that silently stops delivering leaves the behaviour computing
on frozen values while every status surface still reads normal.

The gate is the smallest thing that answers it: the monotonic time of the
last arrival, plus the maximum age at which that arrival is still usable.
It is pure and I/O-free — mark it from the message handler, ask it from the
control loop:

    self._attitude = FreshnessGate(0.5)
    ...
    def _on_attitude(self, msg):        # handler
        ...
        self._attitude.mark(time.monotonic())
    ...
    if not self._attitude.is_fresh(now):  # loop
        return                            # hold, do not command

One gate per INPUT, never one shared across several. Inputs that arrive on
different messages stall independently, and a shared mark is refreshed by
whichever message is still flowing — which hides exactly the partial stall
the gate exists to catch.

This is a sibling of the ``LockedTargetTracker`` coast window: that gate
ages the CAMERA input, this one ages every other input a locked-target
behaviour projects through. It lives here for now, but it is deliberately
free of any Follow-Me coupling so it can move into the shared agent SDK
beside ``LockedTargetTracker`` and be inherited rather than re-derived.
"""

from __future__ import annotations

__all__ = ["FreshnessGate"]


class FreshnessGate:
    """Tracks whether one telemetry input arrived recently enough to use.

    ``max_age_s`` is the oldest an arrival may be and still count as fresh.
    Size it from the cadence the input is actually expected at, with enough
    margin for a dropped message and link jitter — not from a single window
    shared across inputs that arrive at different rates.
    """

    __slots__ = ("_max_age_s", "_last_s")

    def __init__(self, max_age_s: float) -> None:
        # A negative window must not flip the fail-safe direction, and here
        # the safe direction is "stale": clamping to 0 makes the gate report
        # stale (hold, do not command) rather than fresh-forever. Mirrors the
        # clamp on the locked-target coast window.
        self._max_age_s = max(0.0, float(max_age_s))
        self._last_s: float | None = None

    @property
    def max_age_s(self) -> float:
        return self._max_age_s

    @property
    def has_reported(self) -> bool:
        """True once any arrival has been marked, regardless of its age.

        Distinguishes "never arrived" from "stopped arriving" for a caller
        that reports the two differently. It is NOT a usability check —
        :meth:`is_fresh` is.
        """
        return self._last_s is not None

    def mark(self, now_monotonic_s: float) -> None:
        """Record an arrival. ``now_monotonic_s`` MUST come from a monotonic
        clock (``time.monotonic()``); a wall clock can step backwards and
        would make a fresh input read as stale, or worse."""
        self._last_s = float(now_monotonic_s)

    def age_s(self, now_monotonic_s: float) -> float | None:
        """Seconds since the last arrival, or ``None`` if none ever arrived."""
        if self._last_s is None:
            return None
        return now_monotonic_s - self._last_s

    def is_fresh(self, now_monotonic_s: float) -> bool:
        """Whether the last arrival is still within ``max_age_s``.

        False when nothing has ever arrived, so a caller that has never
        received the input holds rather than commanding on initial values.
        """
        if self._last_s is None:
            return False
        return (now_monotonic_s - self._last_s) <= self._max_age_s

    def reset(self) -> None:
        """Forget the last arrival, returning the gate to never-reported."""
        self._last_s = None
