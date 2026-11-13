"""Is the subject faster than the follow can fly?

The limit in force is the lower of the follow's ``max_speed_mps`` and the
pilot's cap without GNSS (2 m/s). The pilot's own cap is not forwarded to
plugins, so it is rebuilt here from feeds a plugin can read, failing closed
the way the pilot does: GNSS only with a fresh 3-D fix from GPS_RAW_INT, and
never while a fresh ``nav.state`` from the vision-nav extension reports its
external-navigation mode (that source may be the EKF's active one, and
without the EKF source parameters it cannot be told apart).

The subject's speed is judged on measured fixes, not the subject filter:
the filter caps a person at 3 m/s, so it can never show a sprinter outrunning
a 6 m/s follow. Over the last 2 s of fixes, either the subject's
displacement rate is above the limit, or the range keeps opening while the
vehicle already flies at the limit.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Any

# The vision-nav extension's navigation state event.
NAV_STATE_TOPIC = "nav.state"
# The pilot's horizontal speed cap while the position source is not GNSS.
GNSS_DENIED_MAX_SPEED_MPS = 2.0
# A GPS fix or nav.state older than this says nothing (the pilot's rule).
NAV_FEED_STALE_S = 3.0
_FIX_3D = 3
_EXTERNAL_NAV = "external_nav"

# The window of fixes the subject's speed is measured over, the shortest
# span that gives a rate, and how old the newest fix may be.
_WINDOW_S = 2.0
_MIN_SPAN_S = 1.0
_MAX_FIX_AGE_S = 1.0
# The range counts as opening above this rate while the vehicle flies at
# this share of the limit or more.
OPENING_MPS = 0.3
AT_LIMIT_SHARE = 0.8


class NavFeed:
    """The latest GPS fix type and vision-nav mode, with when they arrived."""

    __slots__ = ("gps_fix", "gps_at", "nav_mode", "nav_at")

    def __init__(self) -> None:
        self.gps_fix: int | None = None
        self.gps_at = -math.inf
        self.nav_mode: str | None = None
        self.nav_at = -math.inf

    def on_gps_fix(self, fix_type: int, now: float) -> None:
        self.gps_fix = fix_type
        self.gps_at = now

    def on_nav_state(self, payload: Any, now: float) -> None:
        mode = payload.get("mode") if isinstance(payload, dict) else None
        self.nav_mode = str(mode) if isinstance(mode, str) else None
        self.nav_at = now

    def gnss_denied(self, now: float) -> bool:
        if now - self.nav_at <= NAV_FEED_STALE_S and self.nav_mode == _EXTERNAL_NAV:
            return True
        fresh_fix = now - self.gps_at <= NAV_FEED_STALE_S
        return not (fresh_fix and self.gps_fix is not None and self.gps_fix >= _FIX_3D)


def effective_limit(max_speed_mps: float, feed: NavFeed, now: float) -> float:
    """The horizontal speed the follow can actually fly at ``now``."""
    if feed.gnss_denied(now):
        return min(max_speed_mps, GNSS_DENIED_MAX_SPEED_MPS)
    return max_speed_mps


class SubjectMotion:
    """The subject's measured positions and ranges over the last 2 s."""

    __slots__ = ("_fixes",)

    def __init__(self) -> None:
        self._fixes: deque[tuple[float, float, float, float]] = deque()

    def clear(self) -> None:
        self._fixes.clear()

    def record(self, t: float, north: float, east: float, range_m: float) -> None:
        self._fixes.append((t, north, east, range_m))
        while self._fixes and t - self._fixes[0][0] > _WINDOW_S:
            self._fixes.popleft()

    def rates(self, now: float) -> tuple[float, float] | None:
        """``(speed, opening)`` over the window, m/s, or ``None`` without a
        recent window long enough to measure."""
        if not self._fixes:
            return None
        t0, n0, e0, r0 = self._fixes[0]
        t1, n1, e1, r1 = self._fixes[-1]
        span = t1 - t0
        if span < _MIN_SPAN_S or now - t1 > _MAX_FIX_AGE_S:
            return None
        return math.hypot(n1 - n0, e1 - e0) / span, (r1 - r0) / span


def faster_than(rates: tuple[float, float] | None, limit: float, vehicle_speed: float) -> bool:
    """Whether the measured motion shows the subject pulling away."""
    if rates is None:
        return False
    speed, opening = rates
    if speed > limit:
        return True
    return opening > OPENING_MPS and vehicle_speed >= AT_LIMIT_SHARE * limit


__all__ = [
    "AT_LIMIT_SHARE",
    "GNSS_DENIED_MAX_SPEED_MPS",
    "NAV_FEED_STALE_S",
    "NAV_STATE_TOPIC",
    "OPENING_MPS",
    "NavFeed",
    "SubjectMotion",
    "effective_limit",
    "faster_than",
]
