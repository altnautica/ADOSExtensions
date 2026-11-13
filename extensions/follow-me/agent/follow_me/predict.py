"""Where the lost subject is predicted to be: the filter's mean, its spread as
an ellipse, and a short predicted path for the operator's display."""

from __future__ import annotations

import math
from dataclasses import dataclass

from follow_me.target_filter import TargetFilter

MAX_PATH_POINTS = 10


@dataclass(frozen=True)
class SearchEllipse:
    center_ne: tuple[float, float]
    semi_major_m: float
    semi_minor_m: float
    # Bearing of the major axis from north, in [0, 180).
    heading_deg: float


@dataclass(frozen=True)
class PredictedTarget:
    mean_ne: tuple[float, float]
    vel_ne: tuple[float, float]
    # Position covariance: (nn, ne, ee).
    cov_ne: tuple[float, float, float]
    t_s: float


def predict_target(filt: TargetFilter, t: float) -> PredictedTarget | None:
    pred = filt.search_predicted(t)
    if pred is None:
        return None
    x, p = pred
    return PredictedTarget((x[0], x[1]), (x[2], x[3]), (p[0][0], p[0][1], p[1][1]), t)


def predicted_path(
    filt: TargetFilter, now: float, *, horizon_s: float = 5.0, step_s: float = 0.5
) -> list[tuple[float, float]]:
    n = min(MAX_PATH_POINTS, int(horizon_s / step_s))
    out: list[tuple[float, float]] = []
    for k in range(n):
        p = predict_target(filt, now + k * step_s)
        if p is None:
            break
        out.append((round(p.mean_ne[0], 2), round(p.mean_ne[1], 2)))
    return out


def search_ellipse(p: PredictedTarget, k: float) -> SearchEllipse:
    a, b, c = p.cov_ne
    tr, det = a + c, a * c - b * b
    disc = math.sqrt(max(0.0, tr * tr / 4.0 - det))
    l1, l2 = max(0.0, tr / 2.0 + disc), max(0.0, tr / 2.0 - disc)
    # Major-axis direction in the north/east plane, as a bearing from north.
    heading = 0.5 * math.degrees(math.atan2(2.0 * b, a - c)) % 180.0
    return SearchEllipse(p.mean_ne, k * math.sqrt(l1), k * math.sqrt(l2), heading)


__all__ = [
    "MAX_PATH_POINTS",
    "PredictedTarget",
    "SearchEllipse",
    "predict_target",
    "predicted_path",
    "search_ellipse",
]
