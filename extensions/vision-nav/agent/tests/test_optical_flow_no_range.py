"""Tests for the rangefinder-free optical-flow estimator.

Pins the integration with :class:`MavlinkScaleLadder` and the quality
multiplier application, without requiring opencv to be installed.
"""

from __future__ import annotations

from typing import Any, List, Optional

import pytest

from altnautica_vision_nav.estimators.optical_flow_no_range import (
    OpticalFlowNoRangeEstimator,
)
from altnautica_vision_nav.scale.base import BaseScaleSource, ScalePick


class _StubResult:
    def __init__(self, quality: int) -> None:
        self.quality = quality
        self.integration_time_us = 33_333
        self.flow_rate_x = 0.1
        self.flow_rate_y = -0.05
        self.flow_rate_z = 0.0
        self.flow_x_dpi = 0.0
        self.flow_y_dpi = 0.0
        self.flow_comp_m_x = 0.0
        self.flow_comp_m_y = 0.0


class _StubProcessor:
    """OpticalFlowLk stand-in. Tests drive ``next_quality`` per call."""

    def __init__(self) -> None:
        self._next_quality = 0
        self.last_distance_m: Optional[float] = None

    def queue(self, quality: int) -> None:
        self._next_quality = quality

    def process(
        self,
        _prev,
        _curr,
        _dt,
        gyro=None,  # noqa: ARG002
        distance_m=None,
    ) -> _StubResult:
        self.last_distance_m = distance_m
        return _StubResult(self._next_quality)


class _StubScale(BaseScaleSource):
    """Always returns a configured ScalePick (or None on demand)."""

    def __init__(self, picks: List[Optional[ScalePick]]) -> None:
        self._picks = list(picks)
        self.started = False

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.started = False

    def pick(self) -> Optional[ScalePick]:
        if not self._picks:
            return None
        next_pick = self._picks.pop(0)
        # Keep the last pick alive after we've drained the queue so
        # tests that step multiple times don't fall off the end.
        if not self._picks:
            self._picks.append(next_pick)
        return next_pick


def _make_pick(distance_m: float, source: str, qm: float) -> ScalePick:
    return ScalePick(
        distance_m=distance_m, source=source, quality_multiplier=qm
    )


def test_baro_rung_applies_quality_multiplier(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Raw OF quality 100 + baro multiplier 0.7 -> reported quality 70."""

    proc = _StubProcessor()
    scale = _StubScale([_make_pick(2.0, "baro", 0.7)])
    est = OpticalFlowNoRangeEstimator(
        processor=proc, scale_source=scale, quality_gate=50
    )

    proc.queue(100)
    out = est.step(prev_frame=object(), curr_frame=object(), dt_seconds=0.033)
    assert out is not None
    assert out.flow_distance_m == pytest.approx(2.0)
    assert out.flow_scale_source == "baro"
    assert out.flow_quality == 70
    assert out.state == "converged"
    # Inner processor sees the picked distance via the synthetic range.
    assert proc.last_distance_m == pytest.approx(2.0)


def test_static_rung_marks_degraded_even_at_high_raw_quality() -> None:
    """Sitting on the static rung is an operationally degraded signal.

    Even when the OF tracker reports excellent raw quality, the static
    rung implies no real altitude source is healthy. The state machine
    refuses to call this ``converged`` so the GCS surfaces the red
    banner; meanwhile the EKF still receives an emission (at very low
    quality) so it can fuse the velocity opportunistically.
    """

    proc = _StubProcessor()
    scale = _StubScale([_make_pick(1.5, "static", 0.2)])
    est = OpticalFlowNoRangeEstimator(processor=proc, scale_source=scale)

    proc.queue(255)
    out = est.step(prev_frame=object(), curr_frame=object(), dt_seconds=0.033)
    assert out is not None
    assert out.flow_quality == 51  # round(255 * 0.2)
    assert out.state == "degraded"


def test_gps_rung_label_maps_through() -> None:
    """``"gps"`` scale label passes through to the estimator output."""

    proc = _StubProcessor()
    scale = _StubScale([_make_pick(5.0, "gps", 0.4)])
    est = OpticalFlowNoRangeEstimator(processor=proc, scale_source=scale)

    proc.queue(180)
    out = est.step(prev_frame=object(), curr_frame=object(), dt_seconds=0.033)
    assert out is not None
    assert out.flow_scale_source == "gps"
    assert out.flow_quality == 72  # round(180 * 0.4)


def test_no_scale_pick_falls_through_at_critical_quality() -> None:
    """A None pick produces a degraded emit, not a refusal."""

    proc = _StubProcessor()
    scale = _StubScale([None])
    est = OpticalFlowNoRangeEstimator(processor=proc, scale_source=scale)

    proc.queue(200)
    out = est.step(prev_frame=object(), curr_frame=object(), dt_seconds=0.033)
    assert out is not None
    assert out.flow_distance_m is None
    assert out.flow_scale_source is None
    # 200 * 0.2 critical-quality multiplier => 40, below the gate.
    assert out.flow_quality == 40


def test_registered_in_registry() -> None:
    """``optical_flow_degraded`` is wired into the global registry."""

    from altnautica_vision_nav.estimators import ESTIMATOR_REGISTRY

    assert "optical_flow_degraded" in ESTIMATOR_REGISTRY
    assert (
        ESTIMATOR_REGISTRY["optical_flow_degraded"].__name__
        == "OpticalFlowNoRangeEstimator"
    )
