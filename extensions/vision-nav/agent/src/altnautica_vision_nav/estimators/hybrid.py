"""Hybrid optical-flow + VIO estimator.

Runs an OF estimator and a VIO estimator side-by-side on independent
cameras. The downward camera feeds the OF tracker; the forward camera
feeds the VIO engine. The pipeline emits the OF output on MAVLink
component 198 and the VIO output on component 197 in parallel; the
ArduPilot EKF3 fuses both inputs when the source set is configured
for ExternalNav.

The primary :class:`EstimatorOutput` returned by :meth:`step` carries
the VIO pose (richer state). The OF output rides in
``primary.extras["of_output"]`` so the
:class:`MavlinkComponentRouter` can emit it alongside the VIO pose
without losing the OF half of the fused state.

Hybrid mode requires both estimators to be wired with their own
cameras and IMU paths. The pre-arm gate runs both check sets; the
mode is armable only when each child estimator passes its own
checks.
"""

from __future__ import annotations

from typing import Optional

from altnautica_vision_nav.estimators.base import (
    BaseEstimator,
    EstimatorOutput,
    EstimatorState,
)


class HybridEstimator(BaseEstimator):
    """Compose an OF estimator + a VIO estimator into one mode.

    Both child estimators are passed in at construction time; the
    hybrid step() forwards the same ``imu_batch`` to both and expects
    the caller to provide distinct ``prev_frame`` / ``curr_frame``
    arguments per child (the pipeline holds two camera streams and
    zips them by timestamp).
    """

    estimator_id = "hybrid_of_plus_vio"
    output_mode = "vio"

    def __init__(
        self,
        *,
        of_estimator: BaseEstimator,
        vio_estimator: BaseEstimator,
    ) -> None:
        self._of = of_estimator
        self._vio = vio_estimator
        self._state: EstimatorState = "off"

    @property
    def state(self) -> EstimatorState:
        return self._state

    @property
    def of_estimator(self) -> BaseEstimator:
        return self._of

    @property
    def vio_estimator(self) -> BaseEstimator:
        return self._vio

    def configure(self, config: object) -> None:
        self._of.configure(config)
        self._vio.configure(config)

    def shutdown(self) -> None:
        self._of.shutdown()
        self._vio.shutdown()

    def step(
        self,
        *,
        prev_frame: Optional[object] = None,
        curr_frame: Optional[object] = None,
        dt_seconds: float = 0.0,
        imu_batch: Optional[object] = None,
        range_reading: Optional[object] = None,
    ) -> Optional[EstimatorOutput]:
        # The hybrid pipeline holds two camera streams. The frame
        # arguments here carry the forward camera (VIO input); the
        # OF child reads its frames from a per-camera attribute on
        # the inputs object when present, else falls back to the
        # primary frames (degraded behaviour the operator should not
        # be in normally).
        of_prev = _maybe_attr(prev_frame, "of_frame", prev_frame)
        of_curr = _maybe_attr(curr_frame, "of_frame", curr_frame)
        vio_prev = _maybe_attr(prev_frame, "vio_frame", prev_frame)
        vio_curr = _maybe_attr(curr_frame, "vio_frame", curr_frame)

        of_out = self._of.step(
            prev_frame=of_prev,
            curr_frame=of_curr,
            dt_seconds=dt_seconds,
            imu_batch=imu_batch,
            range_reading=range_reading,
        )
        vio_out = self._vio.step(
            prev_frame=vio_prev,
            curr_frame=vio_curr,
            dt_seconds=dt_seconds,
            imu_batch=imu_batch,
            range_reading=None,  # VIO ignores rangefinder by design
        )

        # Aggregate state: the operator's view of "is the hybrid mode
        # ready" is "are both children healthy?". The combined state
        # is the worse of the two: degraded > converging > converged.
        self._state = _combine_states(
            of_out.state if of_out else "off",
            vio_out.state if vio_out else "off",
        )

        # Pick the primary output. VIO wins because its richer state
        # is what the EKF fuses for position; the OF output rides in
        # extras for the router to also emit on comp 198.
        if vio_out is not None:
            vio_out.extras["of_output"] = of_out
            primary = EstimatorOutput(
                timestamp_us=vio_out.timestamp_us,
                output_mode=vio_out.output_mode,
                state=self._state,
                pose=vio_out.pose,
                velocity=vio_out.velocity,
                covariance=vio_out.covariance,
                feature_count=vio_out.feature_count,
                reset_counter=vio_out.reset_counter,
                drift_estimate_m=vio_out.drift_estimate_m,
                sync_offset_ms=vio_out.sync_offset_ms,
                extras={"of_output": of_out} if of_out is not None else {},
            )
            return primary
        if of_out is not None:
            # No VIO sample yet but OF produced one. Emit only the OF
            # half this tick; the router handles output_mode=="optical_flow".
            return EstimatorOutput(
                timestamp_us=of_out.timestamp_us,
                output_mode="optical_flow",
                state=self._state,
                flow_rate_x=of_out.flow_rate_x,
                flow_rate_y=of_out.flow_rate_y,
                flow_rate_z=of_out.flow_rate_z,
                flow_quality=of_out.flow_quality,
                flow_distance_m=of_out.flow_distance_m,
                flow_scale_source=of_out.flow_scale_source,
                integration_time_us=of_out.integration_time_us,
            )
        return None


def _maybe_attr(obj: object, name: str, fallback: object) -> object:
    """Return ``obj.<name>`` when present, else ``fallback``.

    The hybrid pipeline passes a custom inputs object that carries
    both ``of_frame`` and ``vio_frame`` attributes. Tests and the
    degraded fallback pass plain frame objects without those
    attributes, in which case both children read the same frame
    (works for synthetic tests; not flight-safe in production).
    """

    if obj is None:
        return None
    return getattr(obj, name, fallback)


# State priorities for combining child results. Lower number == worse.
_STATE_PRIORITY = {
    "failed": 0,
    "degraded": 1,
    "off": 2,
    "init": 3,
    "converging": 4,
    "converged": 5,
}


def _combine_states(a: str, b: str) -> EstimatorState:
    """Return the worse of two child estimator states.

    The hybrid mode is only as healthy as its weaker child; an OF
    sub-estimator on a static scale rung pulls the combined state
    down to ``degraded`` even when the VIO sub-estimator is
    converged. The operator's pre-arm card surfaces both child check
    sets so the underlying cause is visible.
    """

    pa = _STATE_PRIORITY.get(a, 0)
    pb = _STATE_PRIORITY.get(b, 0)
    return a if pa <= pb else b  # type: ignore[return-value]
