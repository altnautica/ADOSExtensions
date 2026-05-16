"""Tests for the estimator scaffold.

The scaffold lands the estimator ABCs + registry + adapter alongside
the existing pipeline without rewiring it. These tests pin the
scaffold's public surface so a future refactor that wires the new
pipeline does not regress the contract.
"""

from __future__ import annotations

import pytest

from altnautica_vision_nav.estimators import (
    ESTIMATOR_REGISTRY,
    BaseEstimator,
    EstimatorOutput,
    NullEstimator,
    OpticalFlowEstimator,
    available_estimators,
)


def test_registry_contains_off_and_optical_flow() -> None:
    """The scaffold wires exactly two estimators today; the registry
    accepts additional entries without contract changes."""

    keys = available_estimators()
    assert "off" in keys
    assert "optical_flow" in keys
    assert ESTIMATOR_REGISTRY["off"] is NullEstimator
    assert ESTIMATOR_REGISTRY["optical_flow"] is OpticalFlowEstimator


def test_registry_keys_match_estimator_id_attributes() -> None:
    """Registry key and ``estimator_id`` attribute must agree.

    A drift between them would have the mode picker selecting a class
    whose ``estimator_id`` does not match the user's request, which
    breaks the round-trip contract surfaced on the heartbeat.
    """

    for key, cls in ESTIMATOR_REGISTRY.items():
        assert cls.estimator_id == key, (
            f"registry key {key!r} != estimator_id {cls.estimator_id!r}"
        )


def test_base_estimator_is_abstract() -> None:
    """``BaseEstimator`` should refuse direct instantiation.

    The contract is enforced by :class:`abc.ABC` so anyone subclassing
    without implementing :meth:`step` gets a clear error at construction
    time rather than a runtime AttributeError.
    """

    with pytest.raises(TypeError):
        BaseEstimator()  # type: ignore[abstract]


def test_null_estimator_emits_off_state() -> None:
    """Null estimator returns ``state="off"`` and ``output_mode="none"``."""

    est = NullEstimator()
    out = est.step()
    assert out is not None
    assert out.output_mode == "none"
    assert out.state == "off"
    assert out.flow_rate_x is None
    assert out.pose is None


def test_estimator_output_default_factory() -> None:
    """``EstimatorOutput.extras`` defaults to an empty dict per-instance.

    A shared default would let one estimator's extras bleed into
    another's snapshot. The dataclass field uses ``default_factory``
    so each instance gets its own dict; this test guards against a
    future regression that introduces a class-level default.
    """

    a = EstimatorOutput(timestamp_us=0, output_mode="none")
    b = EstimatorOutput(timestamp_us=0, output_mode="none")
    a.extras["seen"] = True
    assert "seen" not in b.extras


def test_optical_flow_estimator_state_progression(monkeypatch) -> None:
    """OF estimator climbs init -> converged -> degraded as quality moves.

    Uses a stub ``OpticalFlowLk`` so the test never imports cv2 or
    numpy. The state machine in :class:`OpticalFlowEstimator` is the
    surface under test; the underlying tracker is a pure delegate.
    """

    class _StubResult:
        def __init__(self, quality: int) -> None:
            self.quality = quality
            self.integration_time_us = 33_333
            self.flow_rate_x = 0.0
            self.flow_rate_y = 0.0
            self.flow_rate_z = 0.0
            self.flow_x_dpi = 0.0
            self.flow_y_dpi = 0.0
            self.flow_comp_m_x = 0.0
            self.flow_comp_m_y = 0.0

    class _StubProcessor:
        def __init__(self) -> None:
            self._next_quality = 0

        def queue(self, quality: int) -> None:
            self._next_quality = quality

        def process(
            self,
            _prev,
            _curr,
            _dt,
            gyro=None,  # noqa: ARG002
            distance_m=None,  # noqa: ARG002
        ) -> _StubResult:
            return _StubResult(self._next_quality)

    proc = _StubProcessor()
    est = OpticalFlowEstimator(processor=proc, quality_gate=50)  # type: ignore[arg-type]

    # First step at low quality: still in init (never converged).
    proc.queue(10)
    out = est.step(prev_frame=object(), curr_frame=object(), dt_seconds=0.033)
    assert out is not None
    assert out.state == "init"
    assert out.flow_quality == 10

    # Quality crosses the gate: converged.
    proc.queue(120)
    out = est.step(prev_frame=object(), curr_frame=object(), dt_seconds=0.033)
    assert out is not None
    assert out.state == "converged"
    assert out.flow_quality == 120

    # Quality drops below the gate after converging: degraded.
    proc.queue(20)
    out = est.step(prev_frame=object(), curr_frame=object(), dt_seconds=0.033)
    assert out is not None
    assert out.state == "degraded"
    assert out.flow_quality == 20


def test_optical_flow_estimator_returns_none_without_frames() -> None:
    """Step with no frame pair returns ``None`` (pipeline skips emit)."""

    class _StubProcessor:
        def process(self, *_, **__) -> object:  # pragma: no cover - never called
            raise AssertionError("process should not be invoked without frames")

    est = OpticalFlowEstimator(processor=_StubProcessor())  # type: ignore[arg-type]
    assert est.step(prev_frame=None, curr_frame=None, dt_seconds=0.033) is None
