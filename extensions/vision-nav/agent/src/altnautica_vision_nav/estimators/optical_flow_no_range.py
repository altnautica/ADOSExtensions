"""Rangefinder-free optical-flow estimator.

Same Lucas-Kanade tracker as :class:`OpticalFlowEstimator`, but the
scale comes from a :class:`BaseScaleSource` (baro / GPS / static)
instead of a dedicated rangefinder driver. Quality is multiplied by
the rung's quality factor so the EKF auto-de-weights degraded scale
sources.

When the ladder returns ``None``, the estimator falls back to the
static rung internally (the live ladder always returns the static
rung when nothing else is healthy) so the estimator's emit decision
stays simple. The estimator's :attr:`state` reflects how trustworthy
the current emission is:

* ``"init"`` — no converged sample yet
* ``"converged"`` — quality (after multiplier) is above the gate
* ``"degraded"`` — quality dropped below the gate after a converged
  sample, OR the scale source is on the static rung
"""

from __future__ import annotations

from typing import Optional

from altnautica_vision_nav.estimators.base import (
    BaseEstimator,
    EstimatorOutput,
    EstimatorState,
    ScaleSource,
)
from altnautica_vision_nav.estimators.optical_flow import OpticalFlowEstimator
from altnautica_vision_nav.processors.optical_flow_lk import (
    OpticalFlowLk,
    OpticalFlowResult,
)
from altnautica_vision_nav.scale.base import (
    BaseScaleSource,
    NullScaleSource,
    ScalePick,
)

DEFAULT_QUALITY_GATE = 50


def _scale_label_to_estimator_label(label: str) -> Optional[ScaleSource]:
    """Translate the ladder's source label into the estimator's enum.

    The ladder uses ``"baro"`` / ``"gps"`` / ``"static"`` / ``"none"``
    while the estimator output uses ``"rangefinder"`` / ``"baro"`` /
    ``"gps"`` / ``"vision"``. The static rung is reported as
    ``"baro"`` because that's the physical fallback expectation
    (operator-set FLOW_HGT_OVR-style constant); the GCS reads the
    quality multiplier separately to know it is degraded.
    """

    if label == "baro":
        return "baro"
    if label == "gps":
        return "gps"
    if label == "vision":
        return "vision"
    if label == "static":
        return "baro"
    return None


class OpticalFlowNoRangeEstimator(BaseEstimator):
    """OF estimator that scales without a rangefinder.

    Wraps :class:`OpticalFlowLk`, applies the chosen scale rung, and
    multiplies the raw quality before reporting it. Selecting this
    estimator is the operator's explicit "I do not have a rangefinder
    on this airframe" path; the GCS labels it as a degraded mode and
    warns about the practical risks.
    """

    estimator_id = "optical_flow_degraded"
    output_mode = "optical_flow"

    def __init__(
        self,
        *,
        processor: Optional[OpticalFlowLk] = None,
        scale_source: Optional[BaseScaleSource] = None,
        quality_gate: int = DEFAULT_QUALITY_GATE,
    ) -> None:
        self._of = OpticalFlowEstimator(
            processor=processor or OpticalFlowLk(),
            quality_gate=quality_gate,
        )
        self._scale = scale_source or NullScaleSource()
        self._quality_gate = int(quality_gate)
        self._state: EstimatorState = "init"
        self._seen_converged_once: bool = False

    @property
    def state(self) -> EstimatorState:
        return self._state

    def configure(self, config: object) -> None:
        # Defer to the inner OF estimator; the scale source is wired
        # by whoever constructs this estimator.
        self._of.configure(config)

    def step(
        self,
        *,
        prev_frame: Optional[object] = None,
        curr_frame: Optional[object] = None,
        dt_seconds: float = 0.0,
        imu_batch: Optional[object] = None,
        range_reading: Optional[object] = None,
    ) -> Optional[EstimatorOutput]:
        # Resolve scale before stepping the inner OF so the wrapped
        # estimator receives a synthetic ``range_reading`` carrying
        # the chosen rung's distance. Quality multiplier is applied
        # after the OF result lands.
        pick: Optional[ScalePick] = self._scale.pick()
        synthetic_range = _SyntheticRange(pick.distance_m) if pick else None

        inner = self._of.step(
            prev_frame=prev_frame,
            curr_frame=curr_frame,
            dt_seconds=dt_seconds,
            imu_batch=imu_batch,
            range_reading=synthetic_range,
        )
        if inner is None:
            return None

        if pick is None:
            # The estimator returned a sample but no scale rung was
            # healthy. Treat as degraded with a zero-distance emit;
            # ArduPilot ignores distance anyway. Quality penalty
            # matches the static-fallback rung so the EKF de-weights.
            quality_mult = 0.2
            scale_label: Optional[ScaleSource] = None
        else:
            quality_mult = float(pick.quality_multiplier)
            scale_label = _scale_label_to_estimator_label(pick.source)

        scaled_quality = (
            None
            if inner.flow_quality is None
            else max(0, min(255, int(round(inner.flow_quality * quality_mult))))
        )

        # Recompute the state machine from the scaled quality so the
        # operator's view of "are we converged?" reflects the actual
        # signal-to-noise the EKF will see, not the raw OF quality.
        # Sitting on the static rung is treated as degraded regardless
        # of raw quality: there is no real altitude source under the
        # scale and the EKF should de-weight accordingly.
        on_static = pick is not None and pick.source == "static"
        if scaled_quality is not None and scaled_quality >= self._quality_gate:
            if on_static:
                self._state = "degraded"
            else:
                self._state = "converged"
                self._seen_converged_once = True
        elif self._seen_converged_once:
            self._state = "degraded"
        elif on_static:
            self._state = "degraded"
        else:
            self._state = "init"

        return EstimatorOutput(
            timestamp_us=inner.timestamp_us,
            output_mode="optical_flow",
            state=self._state,
            flow_rate_x=inner.flow_rate_x,
            flow_rate_y=inner.flow_rate_y,
            flow_rate_z=inner.flow_rate_z,
            flow_quality=scaled_quality,
            flow_distance_m=(pick.distance_m if pick else None),
            flow_scale_source=scale_label,
            integration_time_us=inner.integration_time_us,
        )


class _SyntheticRange:
    """Range-reading shim so the inner OF estimator stays unchanged.

    Mirrors the duck-typed ``.distance_m`` attribute the OF estimator
    reads off real rangefinder readings. No quality field — the outer
    estimator owns the quality multiplier.
    """

    __slots__ = ("distance_m",)

    def __init__(self, distance_m: float) -> None:
        self.distance_m = float(distance_m)


def register_in_registry() -> dict[str, type[BaseEstimator]]:
    """Return the registry entry to be merged by ``estimators.__init__``.

    Kept as a function so the registry consumer can defer import (the
    estimator depends on the scale package which subscribes to MAVLink,
    a heavier import path than the bare OF estimator).
    """

    return {"optical_flow_degraded": OpticalFlowNoRangeEstimator}
