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
it between leans; a report of clear air ahead ends it at once.
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

# ``height_reason`` words on the wire.
HEIGHT_CANOPY = "canopy"
HEIGHT_CANOPY_FLOOR = "canopy_floor"


class CanopyHeight:
    """The highest follow height under the leaves the pilot reports ahead."""

    def __init__(self) -> None:
        self._fly_under_m: float | None = None
        self._at_s: float = -math.inf

    def on_overhead(self, payload: dict[str, Any], now: float) -> None:
        """One ``flight.overhead`` report: ``fly_under_agl_m`` is null when
        nothing overhead lies ahead."""
        value = payload.get("fly_under_agl_m")
        try:
            fly_under = None if value is None else float(value)
        except (TypeError, ValueError):
            return
        if fly_under is not None and not math.isfinite(fly_under):
            return
        if fly_under is None:
            self._fly_under_m = None
            self._at_s = -math.inf
            return
        if self._fly_under_m is None or now - self._at_s > HOLD_S:
            self._fly_under_m = fly_under
        else:
            # Within the hold the lowest report stands until it lapses.
            self._fly_under_m = min(self._fly_under_m, fly_under)
        self._at_s = now

    def reset(self) -> None:
        self._fly_under_m = None
        self._at_s = -math.inf

    def ceiling_m(self, now: float) -> float | None:
        """The fly-under height in force, or ``None``."""
        if self._fly_under_m is None or now - self._at_s > HOLD_S:
            return None
        return self._fly_under_m

    def apply_to(self, cfg: FollowConfig, now: float) -> FollowConfig:
        """``cfg`` with the follow height lowered under the canopy, never
        under ``MIN_FOLLOW_HEIGHT_M``, and why."""
        ceiling = self.ceiling_m(now)
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
    "CanopyHeight",
]
