"""Follow height under a canopy.

A planar lidar sees a tree crown's underside only when the copter leans and
the scan tilts into it; the pilot keeps those returns as overhead hits, not
walls, and reports the lowest one ahead along the subject's path as the
highest height the copter can fly under it (``flight.overhead``,
``fly_under_agl_m``: its swept band, the airframe's top plus 0.3 m, under
the leaves). The follow then flies the highest height in
``[MIN_FOLLOW_HEIGHT_M, follow_height_m]`` that clears them, and says so
(``height_reason`` ``canopy``). When even the floor does not clear them it
holds the floor and says that (``canopy_floor``): it never flies under the
floor, which keeps it over the subject's head, and the pilot's brake keeps
it out of leaves that reach the band.

A report counts for ``HOLD_S`` after it arrives, so a canopy the level
scans pass under (and so stop reporting) does not lift the copter back into
it between leans. A null report after leaves were reported is not clear
air: the pilot forgets returns after a while in level flight, still under
the crowns. The lowered height then holds until the copter has flown,
horizontally, ``PAST_FACTOR`` times the last report's ``ahead_m`` (at least
``PAST_MIN_M``) beyond the last overhead point; nothing else climbs it back
into the leaves.
"""

from __future__ import annotations

import math
from dataclasses import replace
from typing import Any

from .state import MIN_FOLLOW_HEIGHT_M, FollowConfig

# Bus topic carrying the pilot's overhead report to the lease holder.
OVERHEAD_TOPIC = "flight.overhead"
# How long a report of leaves ahead keeps the follow under them, s.
HOLD_S = 5.0
# After a null report, the lowered height holds until the copter is this
# many times the last ``ahead_m`` past the last overhead point, ...
PAST_FACTOR = 2.0
# ... and at least this far past it, m.
PAST_MIN_M = 6.0

# ``height_reason`` words on the wire.
HEIGHT_CANOPY = "canopy"
HEIGHT_CANOPY_FLOOR = "canopy_floor"

NE = tuple[float, float]


class CanopyHeight:
    """The highest follow height under the leaves the pilot reports ahead."""

    def __init__(self) -> None:
        self.reset()

    def on_overhead(
        self, payload: dict[str, Any], now: float, at_ne: NE | None = None
    ) -> None:
        """One ``flight.overhead`` report: ``fly_under_agl_m`` is null when
        nothing overhead lies ahead. ``at_ne`` is the vehicle's horizontal
        position (local north, east, m) when the report arrived, if known."""
        value = payload.get("fly_under_agl_m")
        try:
            fly_under = None if value is None else float(value)
        except (TypeError, ValueError):
            return
        if fly_under is not None and not math.isfinite(fly_under):
            return
        if fly_under is None:
            # Not clear air: the leaves last reported (and still in force)
            # are still overhead until the copter has flown past them.
            if self._fly_under_m is not None and (
                self._past or now - self._at_s <= HOLD_S
            ):
                self._past = True
            return
        if self._fly_under_m is None or (now - self._at_s > HOLD_S and not self._past):
            self._fly_under_m = fly_under
        else:
            # Within the hold the lowest report stands until it lapses.
            self._fly_under_m = min(self._fly_under_m, fly_under)
        self._at_s = now
        self._past = False
        ahead = payload.get("ahead_m")
        try:
            self._ahead_m = max(0.0, float(ahead)) if ahead is not None else 0.0
        except (TypeError, ValueError):
            self._ahead_m = 0.0
        if not math.isfinite(self._ahead_m):
            self._ahead_m = 0.0
        self._at_ne = at_ne

    def reset(self) -> None:
        self._fly_under_m: float | None = None
        self._at_s: float = -math.inf
        self._past = False
        self._ahead_m = 0.0
        self._at_ne: NE | None = None

    def ceiling_m(self, now: float, at_ne: NE | None = None) -> float | None:
        """The fly-under height in force, or ``None``. ``at_ne`` is the
        vehicle's horizontal position now, if known."""
        if self._fly_under_m is None:
            return None
        if self._past:
            if at_ne is None or self._at_ne is None:
                return self._fly_under_m
            flown = math.dist(at_ne, self._at_ne)
            past = self._ahead_m + max(PAST_FACTOR * self._ahead_m, PAST_MIN_M)
            if flown < past:
                return self._fly_under_m
            self.reset()
            return None
        if now - self._at_s > HOLD_S:
            return None
        return self._fly_under_m

    def apply_to(
        self, cfg: FollowConfig, now: float, at_ne: NE | None = None
    ) -> FollowConfig:
        """``cfg`` with the follow height lowered under the canopy, never
        under ``MIN_FOLLOW_HEIGHT_M``, and why."""
        ceiling = self.ceiling_m(now, at_ne)
        if ceiling is None or ceiling >= cfg.follow_height_m:
            return replace(cfg, follow_height_lowered=False, follow_height_reason=None)
        if ceiling >= MIN_FOLLOW_HEIGHT_M:
            return replace(
                cfg,
                follow_height_m=ceiling,
                follow_height_lowered=True,
                follow_height_reason=HEIGHT_CANOPY,
            )
        return replace(
            cfg,
            follow_height_m=MIN_FOLLOW_HEIGHT_M,
            follow_height_lowered=cfg.follow_height_m > MIN_FOLLOW_HEIGHT_M,
            follow_height_reason=HEIGHT_CANOPY_FLOOR,
        )


__all__ = [
    "HEIGHT_CANOPY",
    "HEIGHT_CANOPY_FLOOR",
    "HOLD_S",
    "OVERHEAD_TOPIC",
    "PAST_FACTOR",
    "PAST_MIN_M",
    "CanopyHeight",
]
