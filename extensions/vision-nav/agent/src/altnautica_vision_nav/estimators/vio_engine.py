"""VIO estimator that delegates to a vendor-binary engine.

``VioEstimator`` implements :class:`BaseEstimator` and is the bridge
between the plugin's frame + IMU stream and an :class:`BaseVioEngine`
that owns the vendor binary. Pose returns from the engine become
:class:`EstimatorOutput` records on the ``vio`` path: position +
velocity + covariance + feature count + reset counter.

The estimator owns the watchdog: each :meth:`step` polls the engine
for new messages, drains the pose queue, and ticks the heartbeat
watcher. A missed heartbeat triggers an engine restart through the
configured callback (the supervisor passes a ``process.terminate +
spawn`` closure on construction).
"""

from __future__ import annotations

from typing import Optional

from altnautica_vision_nav.estimators.base import (
    BaseEstimator,
    EstimatorOutput,
    EstimatorState,
    OutputMode,
)
from altnautica_vision_nav.shim.engine import (
    BaseVioEngine,
    EngineConfig,
    PoseMessage,
)
from altnautica_vision_nav.shim.heartbeat import HeartbeatWatcher

DEFAULT_PIXEL_FORMAT = 0  # FRAME_FORMAT_GRAY8


class VioEstimator(BaseEstimator):
    """Estimator that owns one vendor VIO engine.

    The estimator is intentionally engine-agnostic: an
    :class:`OpenVinsEngine` and a :class:`VinsFusionEngine` are
    interchangeable from the pipeline's point of view. Differences
    (binary basename, engine_id, internal tuning) live in the engine
    subclass.

    Two thin subclasses below pin the registry-facing ``estimator_id``
    to ``"vio_openvins"`` and ``"vio_vins_fusion"`` respectively so
    the registry-key contract (key == class id) holds for them too.
    Both subclasses behave identically; the engine is what differs.

    The ``output_mode`` is fixed at ``"vio"`` so the pipeline routes
    emissions to MAVLink component 197 (the
    :class:`MavlinkComponentRouter` reads ``output_mode`` per sample
    to decide which component to emit on).
    """

    estimator_id = "vio"
    output_mode: OutputMode = "vio"

    def __init__(
        self,
        *,
        engine: BaseVioEngine,
        engine_config: EngineConfig,
        heartbeat: Optional[HeartbeatWatcher] = None,
    ) -> None:
        self._engine = engine
        self._engine_config = engine_config
        self._heartbeat = heartbeat
        self._latest_pose: Optional[PoseMessage] = None
        self._state: EstimatorState = "off"

    @property
    def state(self) -> EstimatorState:
        return self._state

    @property
    def engine(self) -> BaseVioEngine:
        """Expose the engine for tests + the supervisor restart path."""

        return self._engine

    def configure(self, _config: object) -> None:
        """Start the engine on configure; safe to call repeatedly."""

        if self._engine.started:
            return
        self._engine.start(config=self._engine_config)
        self._state = "init"

    def shutdown(self) -> None:
        self._engine.stop()
        self._state = "off"

    def step(
        self,
        *,
        prev_frame: Optional[object] = None,
        curr_frame: Optional[object] = None,
        dt_seconds: float = 0.0,
        imu_batch: Optional[object] = None,
        range_reading: Optional[object] = None,
    ) -> Optional[EstimatorOutput]:
        if not self._engine.started:
            return None

        self._forward_imu(imu_batch)
        self._forward_frame(curr_frame)
        self._engine.poll()
        if self._heartbeat is not None:
            if self._heartbeat.check():
                # Watchdog restarted the engine; this tick has no pose.
                self._state = "init"
                return None

        poses = self._engine.drain_poses()
        if not poses:
            return None
        # Use the latest pose; the pipeline emits at the engine's rate
        # rather than the camera rate when more than one arrived.
        pose = poses[-1]
        self._latest_pose = pose
        self._state = _state_from_pose(pose.state)
        return self._pose_to_output(pose)

    # ------------------------------------------------------------------
    # IPC forwarding
    # ------------------------------------------------------------------

    def _forward_imu(self, imu_batch: Optional[object]) -> None:
        """Push pending IMU samples to the engine.

        Accepts a sequence of samples (the time-aligner returns a
        single sample paired with the current frame, but tests may
        pass a list directly), a single sample, or ``None``.
        """

        if imu_batch is None:
            return
        samples = self._extract_imu_samples(imu_batch)
        for sample in samples:
            ts_us = int(sample.ts_ns / 1000)
            self._engine.send_imu(
                ts_us=ts_us,
                gyro=(sample.xgyro, sample.ygyro, sample.zgyro),
                accel=(sample.xacc, sample.yacc, sample.zacc),
            )

    @staticmethod
    def _extract_imu_samples(imu_batch: object) -> list:
        # Single AlignedSample with .imu_sample attribute (TimeAligner path)
        aligned_sample = getattr(imu_batch, "imu_sample", None)
        if aligned_sample is not None and getattr(
            aligned_sample, "ts_ns", None
        ) is not None:
            return [aligned_sample]
        # Direct ImuSample
        if getattr(imu_batch, "ts_ns", None) is not None:
            return [imu_batch]
        # Iterable of ImuSample
        try:
            return [s for s in imu_batch if getattr(s, "ts_ns", None) is not None]
        except TypeError:
            return []

    def _forward_frame(self, curr_frame: Optional[object]) -> None:
        if curr_frame is None:
            return
        ts_us = getattr(curr_frame, "ts_us", None)
        pixels = getattr(curr_frame, "pixels", None)
        width = getattr(curr_frame, "width", None)
        height = getattr(curr_frame, "height", None)
        if pixels is None or ts_us is None or width is None or height is None:
            return
        stride = getattr(curr_frame, "stride", width)
        pixel_format = getattr(curr_frame, "pixel_format", DEFAULT_PIXEL_FORMAT)
        self._engine.send_frame(
            ts_us=int(ts_us),
            width=int(width),
            height=int(height),
            stride=int(stride),
            pixel_format=int(pixel_format),
            pixels=bytes(pixels),
        )

    # ------------------------------------------------------------------
    # Output translation
    # ------------------------------------------------------------------

    def _pose_to_output(self, pose: PoseMessage) -> EstimatorOutput:
        return EstimatorOutput(
            timestamp_us=pose.ts_us,
            output_mode="vio",
            state=self._state,
            pose=tuple(pose.position + _quat_to_euler(pose.orientation_quat)),
            velocity=pose.velocity,
            covariance=pose.covariance,
            feature_count=pose.feature_count,
            reset_counter=pose.reset_counter,
        )


class VioOpenvinsEstimator(VioEstimator):
    """OpenVINS-flavoured VIO estimator.

    Identical to :class:`VioEstimator` apart from the registry key.
    The caller still constructs an :class:`OpenVinsEngine` and passes
    it in; the subclass exists only so the registry-key contract
    (``ESTIMATOR_REGISTRY["vio_openvins"].estimator_id ==
    "vio_openvins"``) holds.
    """

    estimator_id = "vio_openvins"


class VioVinsFusionEstimator(VioEstimator):
    """VINS-Fusion-flavoured VIO estimator. See :class:`VioOpenvinsEstimator`."""

    estimator_id = "vio_vins_fusion"


def _state_from_pose(pose_state: str) -> EstimatorState:
    """Translate vendor-side state to the BaseEstimator vocabulary.

    The vendor emits ``"init"``, ``"converging"``, ``"converged"``,
    ``"degraded"``, or ``"failed"`` — same vocabulary as the estimator
    framework so the mapping is identity for the documented values.
    Any unknown string falls back to ``"degraded"`` so the GCS
    surfaces a warning rather than silently treating it as healthy.
    """

    if pose_state in (
        "off",
        "init",
        "converging",
        "converged",
        "degraded",
        "failed",
    ):
        return pose_state  # type: ignore[return-value]
    return "degraded"


def _quat_to_euler(quat: tuple[float, float, float, float]) -> tuple[float, float, float]:
    """Convert (w, x, y, z) -> (roll, pitch, yaw) in radians.

    Standard aerospace ZYX intrinsic convention. Used to fill the
    ``pose`` six-tuple ``(x, y, z, roll, pitch, yaw)`` consumed by the
    estimator output and the MAVLink emitter.
    """

    import math

    w, x, y, z = quat
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (roll, pitch, yaw)
