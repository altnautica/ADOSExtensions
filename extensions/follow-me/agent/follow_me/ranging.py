"""Range cues: the planar-scan lookup, inverse-variance fusion and the
range/bearing covariance. Pure math, no IPC; used by :mod:`follow_me.projection`."""

from __future__ import annotations

import math
from dataclasses import dataclass

# Bearing window around the subject the scan is sampled in.
_LIDAR_HALF_WIDTH_RAD = math.radians(2.0)


@dataclass(frozen=True)
class RangeCue:
    """One horizontal-range estimate to the subject with its 1-sigma error."""

    kind: str
    range_m: float
    sigma_m: float

    def to_dict(self) -> dict[str, object]:
        return {
            "kind": self.kind,
            "range_m": round(self.range_m, 3),
            "sigma_m": round(self.sigma_m, 3),
        }


@dataclass(frozen=True)
class Scan:
    """One planar lidar revolution in the body FRD frame. ``ranges_m[i]`` is
    at ``angle_start_rad + i * angle_increment_rad``, measured clockwise from
    the nose; ``0`` is no return."""

    angle_start_rad: float
    angle_increment_rad: float
    ranges_m: tuple[float, ...]


def lidar_range_at(scan: Scan, bearing_body_rad: float) -> float | None:
    """The nearest return within +/-2 degrees of a body-frame bearing
    (clockwise from the nose), or ``None`` when no bin there has a return."""
    inc = scan.angle_increment_rad
    if inc <= 0.0 or not scan.ranges_m:
        return None
    best: float | None = None
    two_pi = 2.0 * math.pi
    for i, r in enumerate(scan.ranges_m):
        if not (r > 0.0) or not math.isfinite(r):
            continue
        angle = scan.angle_start_rad + i * inc
        diff = (angle - bearing_body_rad + math.pi) % two_pi - math.pi
        # Inclusive edge; the 1e-9 absorbs the wrap arithmetic's rounding so
        # a bin exactly 2 degrees off still counts.
        if abs(diff) <= _LIDAR_HALF_WIDTH_RAD + 1e-9 and (best is None or r < best):
            best = r
    return best


def fuse(cues: list[RangeCue]) -> tuple[float, float] | None:
    """Inverse-variance fusion: ``(range, sigma)`` or ``None`` with no cue."""
    weight = 0.0
    acc = 0.0
    for c in cues:
        w = 1.0 / max(c.sigma_m, 1e-3) ** 2
        weight += w
        acc += w * c.range_m
    if weight <= 0.0:
        return None
    return (acc / weight, math.sqrt(1.0 / weight))


def range_bearing_covariance(
    bearing_rad: float, sigma_range_m: float, sigma_cross_m: float
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Rotate a diagonal (along, across) covariance into north/east."""
    c, s = math.cos(bearing_rad), math.sin(bearing_rad)
    a = sigma_range_m * sigma_range_m
    b = sigma_cross_m * sigma_cross_m
    return (
        (c * c * a + s * s * b, c * s * (a - b)),
        (c * s * (a - b), s * s * a + c * c * b),
    )


__all__ = ["RangeCue", "Scan", "fuse", "lidar_range_at", "range_bearing_covariance"]
