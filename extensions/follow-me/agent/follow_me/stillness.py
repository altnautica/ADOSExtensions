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
  back, a walk does not);
* the speed at the newest fix was at most STILL_RESIDUAL_MPS (a subject that
  set off just before the loss is moving, whatever it did before); and,
  given where the vehicle was at the loss,
* over the last STILL_RECENT_S the estimate came no more than
  STILL_APPROACH_M nearer the vehicle, and at the newest fix it was coming
  toward it at no more than STILL_SPEED_MPS.

The last check looks only toward the vehicle. A walk that starts a second or
two before the loss reads just like range noise dragging a standing
subject's estimate; a subject taken for still while it walked away is
searched for behind it, the vehicle on the far side, but one taken for
still while it walked toward the vehicle walks into the vantage points
round where it stood.

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
STILL_RECENT_S = 2.0
STILL_APPROACH_M = 0.3
# The walk up a held standing subject is taken to be making: the newest
# velocity toward the vehicle between these. In simulation a 0.5 m/s walk
# up reads 0.37-0.63 m/s, while range noise on a standing subject reads up
# to about 0.33 m/s with a box-height range, and on a camera's noisier range
# a run of outliers up to 0.8 m/s with a jump of more than a metre (the
# standing detector ends standing on a real walk faster than that).
WALK_UP_MIN_MPS = 0.35
WALK_UP_MAX_MPS = 0.7


def _median(values: list[float]) -> float:
    s = sorted(values)
    m = len(s) // 2
    return s[m] if len(s) % 2 else 0.5 * (s[m - 1] + s[m])


class SpeedHistory:
    """The estimates fused over the last STILL_WINDOW_S: ``(t, n, e, vn,
    ve)``, oldest first."""

    def __init__(self) -> None:
        self._fixes: deque[tuple[float, float, float, float, float]] = deque()

    def clear(self) -> None:
        self._fixes.clear()

    def shift(self, dn: float, de: float) -> None:
        """Move with a jump of the local frame."""
        self._fixes = deque((t, n + dn, e + de, vn, ve) for t, n, e, vn, ve in self._fixes)

    def add(self, t: float, north: float, east: float, vn: float, ve: float) -> None:
        fixes = self._fixes
        if fixes and t < fixes[-1][0]:
            # Fixes arrive in order; one out of order says nothing new.
            return
        fixes.append((t, north, east, vn, ve))
        while fixes and t - fixes[0][0] > STILL_WINDOW_S:
            fixes.popleft()

    def still_centre(self, vehicle_ne: tuple[float, float] | None = None) -> tuple[float, float] | None:
        """Where the subject stood at its newest fix (see the module's
        rule), or ``None`` when it was not standing still. ``vehicle_ne``:
        where the vehicle was at the loss (``None`` skips the approach
        check)."""
        fixes = self._fixes
        if len(fixes) < STILL_MIN_FIXES:
            return None
        t0, n0, e0, _vn0, _ve0 = fixes[0]
        t1, n1, e1, vn1, ve1 = fixes[-1]
        span = t1 - t0
        if span < STILL_MIN_SPAN_S or math.hypot(vn1, ve1) > STILL_RESIDUAL_MPS:
            return None
        if _median([math.hypot(f[3], f[4]) for f in fixes]) > STILL_SPEED_MPS:
            return None
        if math.hypot(n1 - n0, e1 - e0) > STILL_SPEED_MPS * span:
            return None
        if vehicle_ne is not None and self.approached(vehicle_ne):
            return None
        return (_median([f[1] for f in fixes]), _median([f[2] for f in fixes]))

    def approached(self, vehicle_ne: tuple[float, float], *, both: bool = False) -> bool:
        """Whether the last STILL_RECENT_S brought the estimate toward
        ``vehicle_ne`` (see the module's rule): its newest velocity or its
        displacement over them. With ``both``, for a subject the follow
        holds as standing, the two together over a full STILL_RECENT_S of
        fixes, the velocity between WALK_UP_MIN_MPS and WALK_UP_MAX_MPS: a
        slow walk up shows all of that, while range noise gives one now and
        then, and a run of agreeing outliers a start over or a jump far
        faster than the fixes either side. ``False`` with no fix."""
        if not self._fixes:
            return False
        t1, n1, e1, vn1, ve1 = self._fixes[-1]
        if both and self._fixes[0][0] > t1 - STILL_RECENT_S:
            return False
        dn, de = vehicle_ne[0] - n1, vehicle_ne[1] - e1
        d = math.hypot(dn, de)
        if d <= 1e-6:
            return True
        un, ue = dn / d, de / d
        coming = vn1 * un + ve1 * ue > STILL_SPEED_MPS
        then = next(f for f in self._fixes if f[0] >= t1 - STILL_RECENT_S)
        came = (n1 - then[1]) * un + (e1 - then[2]) * ue > STILL_APPROACH_M
        if both:
            return came and WALK_UP_MIN_MPS < vn1 * un + ve1 * ue <= WALK_UP_MAX_MPS
        return coming or came


__all__ = [
    "STILL_APPROACH_M",
    "STILL_MIN_FIXES",
    "STILL_MIN_SPAN_S",
    "STILL_RECENT_S",
    "STILL_RESIDUAL_MPS",
    "STILL_SPEED_MPS",
    "STILL_WINDOW_S",
    "WALK_UP_MAX_MPS",
    "WALK_UP_MIN_MPS",
    "SpeedHistory",
]
