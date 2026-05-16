"""Altitude / depth scale sources for rangefinder-free optical flow.

The rangefinder-free OF estimator scales angular flow to metric
velocity using whichever altitude source is healthy. The ladder is:

* primary — ``GLOBAL_POSITION_INT.relative_alt`` (baro-EKF altitude)
* secondary — ``VFR_HUD.alt`` minus the captured take-off altitude
* tertiary — ``GPS_RAW_INT.alt`` minus take-off, outdoor-flag gated
* last resort — a static fallback (default 1.5 m), marked critical

The ladder runs as a :class:`BaseScaleSource` so the estimator stays
agnostic to where the altitude actually came from. A future depth
sensor or VIO-derived scale plugs in behind the same contract.
"""

from __future__ import annotations

from altnautica_vision_nav.scale.base import (
    BaseScaleSource,
    NullScaleSource,
    ScalePick,
    ScaleSourceLabel,
)
from altnautica_vision_nav.scale.mavlink_ladder import MavlinkScaleLadder

__all__ = [
    "BaseScaleSource",
    "MavlinkScaleLadder",
    "NullScaleSource",
    "ScalePick",
    "ScaleSourceLabel",
]
