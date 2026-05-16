"""Optical-flow estimator implementing :class:`BaseEstimator`.

Thin adapter around the existing :class:`OpticalFlowLk` processor. The
processor stays the home of the actual Lucas-Kanade tracker so its
tests, behaviour, and tuning surface remain untouched in this phase.
This adapter restates the output as :class:`EstimatorOutput` so a
future estimator-agnostic pipeline can swap in any other estimator
without changing the call site.

State mapping rules:

* ``state="init"`` until the first non-``None`` result with positive
  quality (the processor needs at least one frame pair to home in)
* ``state="converged"`` once a sample with quality ≥ the configured
  gate has been seen
* ``state="degraded"`` after a sample below the gate when the
  estimator had previously converged
* ``state="failed"`` is not raised by this estimator; the pipeline
  signals it externally when the capture source dies

Scale-source mapping mirrors the rangefinder topology of the active
config: ``"rangefinder"`` when distance is supplied, ``"none"``
otherwise. The rangefinder-free OF mode adds ``"baro"`` / ``"gps"`` /
``"vision"`` values in a later phase.
"""

from __future__ import annotations

from typing import Optional

from altnautica_vision_nav.estimators.base import (
    BaseEstimator,
    EstimatorOutput,
    EstimatorState,
    ScaleSource,
)
from altnautica_vision_nav.processors.optical_flow_lk import (
    OpticalFlowLk,
    OpticalFlowResult,
)

DEFAULT_QUALITY_GATE = 50


class OpticalFlowEstimator(BaseEstimator):
    """Wrap :class:`OpticalFlowLk` behind the estimator contract."""

    estimator_id = "optical_flow"
    output_mode = "optical_flow"

    def __init__(
        self,
        *,
        processor: Optional[OpticalFlowLk] = None,
        quality_gate: int = DEFAULT_QUALITY_GATE,
    ) -> None:
        self._processor = processor or OpticalFlowLk()
        self._quality_gate = int(quality_gate)
        self._state: EstimatorState = "init"
        self._seen_converged_once: bool = False

    @property
    def state(self) -> EstimatorState:
        """Current internal state. Useful in tests and the health snap."""

        return self._state

    def configure(self, _config: object) -> None:
        # No-op for the adapter: the processor's tunables (FOV, focal
        # length, max features, pyramid levels) are wired up by the
        # caller that constructs the OpticalFlowLk instance, not here.
        return None

    def step(
        self,
        *,
        prev_frame: Optional[object] = None,
        curr_frame: Optional[object] = None,
        dt_seconds: float = 0.0,
        imu_batch: Optional[object] = None,
        range_reading: Optional[object] = None,
    ) -> Optional[EstimatorOutput]:
        if prev_frame is None or curr_frame is None:
            return None

        gyro = self._extract_gyro(imu_batch)
        distance_m = self._extract_distance(range_reading)
        scale_source: ScaleSource = (
            "rangefinder" if distance_m is not None else "none"
        )

        result: OpticalFlowResult = self._processor.process(
            prev_frame,
            curr_frame,
            dt_seconds,
            gyro=gyro,
            distance_m=distance_m,
        )

        quality = int(result.quality)
        if quality >= self._quality_gate:
            self._state = "converged"
            self._seen_converged_once = True
        elif self._seen_converged_once:
            self._state = "degraded"
        else:
            self._state = "init"

        return EstimatorOutput(
            timestamp_us=int(result.integration_time_us),
            output_mode="optical_flow",
            state=self._state,
            flow_rate_x=float(result.flow_rate_x),
            flow_rate_y=float(result.flow_rate_y),
            flow_rate_z=float(result.flow_rate_z),
            flow_quality=quality,
            flow_distance_m=distance_m,
            flow_scale_source=scale_source,
            integration_time_us=int(result.integration_time_us),
        )

    @staticmethod
    def _extract_gyro(imu_batch: Optional[object]) -> Optional[object]:
        """Pull a gyro sample from whatever the pipeline hands us.

        Today's callers pass a raw ``GyroReading`` directly. A later
        wiring upgrade introduces a richer ``ImuBatch`` shape with
        attribute access. This helper accepts both so the wiring change
        can land without breaking the OF estimator.
        """

        if imu_batch is None:
            return None
        latest = getattr(imu_batch, "latest_gyro", None)
        if callable(latest):
            try:
                return latest()
            except Exception:  # noqa: BLE001
                return None
        # Already a GyroReading-shaped object.
        return imu_batch

    @staticmethod
    def _extract_distance(range_reading: Optional[object]) -> Optional[float]:
        if range_reading is None:
            return None
        distance = getattr(range_reading, "distance_m", None)
        if distance is None:
            return None
        try:
            return float(distance)
        except (TypeError, ValueError):
            return None
