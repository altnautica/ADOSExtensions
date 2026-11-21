"""Whether the subject was standing when the follow last saw it.

A search after a lost lock must not carry a standing subject's estimate off
at the velocity its last fixes left. Range noise on a standing subject reads
as a slow walk: a recorded forest follow lost a subject that had stood for
almost a minute with the estimate moving at 0.45 m/s, the search ran that
on, and the vantage points it asked for, a standoff from the moving
estimate, came round onto the subject itself.

**The rule.** The subject was still at its last fix when, over the
STILL_WINDOW_S of fused estimates up to it (at least STILL_MIN_FIXES of
them, the oldest at least STILL_MIN_SPAN_S before the newest):

* the median of the estimate's speed was at most STILL_SPEED_MPS (noise
  swings it, a walk keeps it up);
* the estimate moved on net at no more than STILL_SPEED_MPS (noise comes
  back, a walk does not); and
* the speed at the newest fix was at most STILL_RESIDUAL_MPS (a subject that
  set off just before the loss is moving, whatever it did before).

Its still centre is the median of those estimates: the newest ones, which
the residual velocity has already dragged, do not move it.
"""

from __future__ import annotations

import math
from collections import deque

STILL_WINDOW_S = 6.0
STILL_MIN_SPAN_S = 5.0
STILL_MIN_FIXES = 6
STILL_SPEED_MPS = 0.3
# The standing subject detector's own entry speed
# (:data:`follow_me.standoff.STILL_ENTER_SPEED_MPS`).
STILL_RESIDUAL_MPS = 0.6


def _median(values: list[float]) -> float:
    s = sorted(values)
    m = len(s) // 2
    return s[m] if len(s) % 2 else 0.5 * (s[m - 1] + s[m])


class SpeedHistory:
    """The estimates fused over the last STILL_WINDOW_S: ``(t, n, e,
    speed)``, oldest first."""

    def __init__(self) -> None:
        self._fixes: deque[tuple[float, float, float, float]] = deque()

    def clear(self) -> None:
        self._fixes.clear()

    def shift(self, dn: float, de: float) -> None:
        """Move with a jump of the local frame."""
        self._fixes = deque((t, n + dn, e + de, s) for t, n, e, s in self._fixes)

    def add(self, t: float, north: float, east: float, speed_mps: float) -> None:
        fixes = self._fixes
        if fixes and t < fixes[-1][0]:
            # Fixes arrive in order; one out of order says nothing new.
            return
        fixes.append((t, north, east, speed_mps))
        while fixes and t - fixes[0][0] > STILL_WINDOW_S:
            fixes.popleft()

    def still_centre(self) -> tuple[float, float] | None:
        """Where the subject stood at its newest fix (see the module's
        rule), or ``None`` when it was not standing still."""
        fixes = self._fixes
        if len(fixes) < STILL_MIN_FIXES:
            return None
        t0, n0, e0, _s0 = fixes[0]
        t1, n1, e1, s1 = fixes[-1]
        span = t1 - t0
        if span < STILL_MIN_SPAN_S or s1 > STILL_RESIDUAL_MPS:
            return None
        if _median([f[3] for f in fixes]) > STILL_SPEED_MPS:
            return None
        if math.hypot(n1 - n0, e1 - e0) > STILL_SPEED_MPS * span:
            return None
        return (_median([f[1] for f in fixes]), _median([f[2] for f in fixes]))


__all__ = [
    "STILL_MIN_FIXES",
    "STILL_MIN_SPAN_S",
    "STILL_RESIDUAL_MPS",
    "STILL_SPEED_MPS",
    "STILL_WINDOW_S",
    "SpeedHistory",
]
