"""Tests for the hybrid OF + VIO estimator.

Pins the state-combination rules and the dual-emission path through
the component router. Uses stub child estimators so the test does not
need cv2 or a real vendor binary.
"""

from __future__ import annotations

from typing import Optional

import pytest

from altnautica_vision_nav.estimators.base import (
    BaseEstimator,
    EstimatorOutput,
    EstimatorState,
)
from altnautica_vision_nav.estimators.hybrid import HybridEstimator


class _StubEstimator(BaseEstimator):
    """Returns a programmed EstimatorOutput on every step()."""

    estimator_id = "stub"
    output_mode = "none"

    def __init__(
        self,
        *,
        result: Optional[EstimatorOutput] = None,
        output_mode: str = "none",
    ) -> None:
        self._result = result
        self.output_mode = output_mode  # type: ignore[assignment]
        self.configure_called = False
        self.shutdown_called = False
        self.last_frame_args: Optional[tuple] = None

    def configure(self, _config: object) -> None:
        self.configure_called = True

    def shutdown(self) -> None:
        self.shutdown_called = True

    def step(
        self,
        *,
        prev_frame: Optional[object] = None,
        curr_frame: Optional[object] = None,
        dt_seconds: float = 0.0,
        imu_batch: Optional[object] = None,
        range_reading: Optional[object] = None,
    ) -> Optional[EstimatorOutput]:
        self.last_frame_args = (prev_frame, curr_frame)
        return self._result


def _of_output(state: EstimatorState = "converged") -> EstimatorOutput:
    return EstimatorOutput(
        timestamp_us=1_000_000,
        output_mode="optical_flow",
        state=state,
        flow_rate_x=0.1,
        flow_rate_y=-0.05,
        flow_rate_z=0.0,
        flow_quality=180,
        flow_distance_m=1.5,
        flow_scale_source="rangefinder",
        integration_time_us=33_333,
    )


def _vio_output(state: EstimatorState = "converged") -> EstimatorOutput:
    return EstimatorOutput(
        timestamp_us=1_000_000,
        output_mode="vio",
        state=state,
        pose=(1.0, 2.0, 3.0, 0.0, 0.0, 0.0),
        velocity=(0.1, 0.2, 0.3),
        covariance=tuple([0.0] * 21),
        feature_count=42,
        reset_counter=0,
    )


def test_hybrid_estimator_id_matches_registry_key() -> None:
    """Registry-key contract holds for the hybrid entry."""

    assert HybridEstimator.estimator_id == "hybrid_of_plus_vio"


def test_configure_and_shutdown_forward_to_children() -> None:
    of_child = _StubEstimator(output_mode="optical_flow")
    vio_child = _StubEstimator(output_mode="vio")
    hybrid = HybridEstimator(of_estimator=of_child, vio_estimator=vio_child)

    hybrid.configure({"mode": "hybrid_of_plus_vio"})
    assert of_child.configure_called is True
    assert vio_child.configure_called is True

    hybrid.shutdown()
    assert of_child.shutdown_called is True
    assert vio_child.shutdown_called is True


def test_primary_output_is_vio_with_of_in_extras() -> None:
    """When both children produce, the primary carries VIO + OF in extras."""

    of_child = _StubEstimator(result=_of_output(), output_mode="optical_flow")
    vio_child = _StubEstimator(result=_vio_output(), output_mode="vio")
    hybrid = HybridEstimator(of_estimator=of_child, vio_estimator=vio_child)

    primary = hybrid.step(
        prev_frame=object(), curr_frame=object(), dt_seconds=0.033
    )
    assert primary is not None
    assert primary.output_mode == "vio"
    assert primary.pose == (1.0, 2.0, 3.0, 0.0, 0.0, 0.0)
    assert "of_output" in primary.extras
    of_extra = primary.extras["of_output"]
    assert of_extra is not None
    assert of_extra.output_mode == "optical_flow"
    assert of_extra.flow_quality == 180


def test_only_of_output_when_vio_missing() -> None:
    """When VIO has no sample yet, the primary is the OF output."""

    of_child = _StubEstimator(result=_of_output(), output_mode="optical_flow")
    vio_child = _StubEstimator(result=None, output_mode="vio")
    hybrid = HybridEstimator(of_estimator=of_child, vio_estimator=vio_child)

    primary = hybrid.step(
        prev_frame=object(), curr_frame=object(), dt_seconds=0.033
    )
    assert primary is not None
    assert primary.output_mode == "optical_flow"
    assert primary.flow_quality == 180


def test_no_output_when_both_children_silent() -> None:
    """Both children returning None means the tick is silent."""

    of_child = _StubEstimator(result=None, output_mode="optical_flow")
    vio_child = _StubEstimator(result=None, output_mode="vio")
    hybrid = HybridEstimator(of_estimator=of_child, vio_estimator=vio_child)
    assert hybrid.step(prev_frame=object(), curr_frame=object()) is None


def test_combined_state_takes_worse_of_two() -> None:
    """The combined state is the worse of the two child states.

    Hybrid is only as healthy as its weaker child. The operator sees
    the pre-arm card's per-child check rows for the underlying cause.
    """

    of_child = _StubEstimator(
        result=_of_output(state="degraded"), output_mode="optical_flow"
    )
    vio_child = _StubEstimator(
        result=_vio_output(state="converged"), output_mode="vio"
    )
    hybrid = HybridEstimator(of_estimator=of_child, vio_estimator=vio_child)
    primary = hybrid.step(prev_frame=object(), curr_frame=object())
    assert primary is not None
    assert primary.state == "degraded"


def test_failed_child_pulls_combined_state_to_failed() -> None:
    of_child = _StubEstimator(
        result=_of_output(state="converged"), output_mode="optical_flow"
    )
    vio_child = _StubEstimator(
        result=_vio_output(state="failed"), output_mode="vio"
    )
    hybrid = HybridEstimator(of_estimator=of_child, vio_estimator=vio_child)
    primary = hybrid.step(prev_frame=object(), curr_frame=object())
    assert primary is not None
    assert primary.state == "failed"


def test_per_camera_frame_routing() -> None:
    """Inputs carrying ``of_frame`` + ``vio_frame`` route correctly."""

    class _Inputs:
        def __init__(self, of_frame: object, vio_frame: object) -> None:
            self.of_frame = of_frame
            self.vio_frame = vio_frame

    of_child = _StubEstimator(result=_of_output(), output_mode="optical_flow")
    vio_child = _StubEstimator(result=_vio_output(), output_mode="vio")
    hybrid = HybridEstimator(of_estimator=of_child, vio_estimator=vio_child)

    of_frame_curr = object()
    vio_frame_curr = object()
    inputs_curr = _Inputs(of_frame_curr, vio_frame_curr)
    of_frame_prev = object()
    vio_frame_prev = object()
    inputs_prev = _Inputs(of_frame_prev, vio_frame_prev)

    hybrid.step(prev_frame=inputs_prev, curr_frame=inputs_curr)

    assert of_child.last_frame_args == (of_frame_prev, of_frame_curr)
    assert vio_child.last_frame_args == (vio_frame_prev, vio_frame_curr)


def test_registered_in_global_registry() -> None:
    from altnautica_vision_nav.estimators import ESTIMATOR_REGISTRY

    assert "hybrid_of_plus_vio" in ESTIMATOR_REGISTRY
    assert ESTIMATOR_REGISTRY["hybrid_of_plus_vio"] is HybridEstimator
