"""Scale-source contract.

Every scale source returns a :class:`ScalePick` describing the chosen
distance, the rung it came from, and a quality multiplier applied to
the optical-flow quality before MAVLink emission. Each scale source
keeps its own freshness logic; the estimator does not need to know
how the value was derived, only whether it is currently usable.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Literal, Optional

ScaleSourceLabel = Literal["rangefinder", "baro", "gps", "vision", "static", "none"]


@dataclass(frozen=True)
class ScalePick:
    """One ladder evaluation.

    ``distance_m`` is the chosen altitude or depth value in metres,
    clamped to the source's own valid range. ``source`` names the rung
    that produced it; the GCS sensors card surfaces this verbatim.
    ``quality_multiplier`` is applied to the OF tracker's raw quality
    (0..255) before EKF emission so the firmware automatically
    de-weights degraded rungs.
    """

    distance_m: float
    source: ScaleSourceLabel
    quality_multiplier: float


class BaseScaleSource(ABC):
    """Common contract for every scale provider.

    The estimator calls :meth:`pick` on every frame. The source
    decides which rung is healthy now and returns it, or ``None`` if
    no rung is currently safe to feed the EKF (the estimator then
    marks itself ``degraded`` or refuses to emit).
    """

    @abstractmethod
    def start(self) -> None:
        """Begin subscribing to whatever messages feed the ladder."""

    @abstractmethod
    def stop(self) -> None:
        """Stop receiving updates. Idempotent."""

    @abstractmethod
    def pick(self) -> Optional[ScalePick]:
        """Evaluate the ladder and return the current pick, or ``None``."""


class NullScaleSource(BaseScaleSource):
    """Always returns ``None``; useful in tests and the off mode.

    The estimator interprets ``None`` from :meth:`pick` as "no scale
    source available right now" and either emits OPTICAL_FLOW_RAD with
    ``distance = 0`` (which ArduPilot ignores anyway, per the
    upstream EKF source code) or skips emission entirely depending on
    the mode policy.
    """

    def start(self) -> None:
        return None

    def stop(self) -> None:
        return None

    def pick(self) -> Optional[ScalePick]:
        return None
