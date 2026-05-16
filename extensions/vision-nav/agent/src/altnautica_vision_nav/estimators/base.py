"""Estimator base class and output contract.

Every estimator answers ``step(frame, imu_batch, range_reading, dt)``
with an :class:`EstimatorOutput`. Optical-flow estimators fill the
``flow_*`` fields and leave the VIO fields ``None``. VIO estimators
fill ``pose``, ``velocity``, ``covariance`` and leave the flow fields
``None``. The pipeline reads ``output_mode`` to decide which MAVLink
component and message family to emit on.

State machine ``EstimatorState`` is shared across kinds:

* ``off`` — estimator is paused or disabled
* ``init`` — first frames arriving, processor warming up
* ``converging`` — feature tracking acquired, output not yet trustworthy
* ``converged`` — output is steady-state and may be fed to the FC
* ``degraded`` — output quality dropped; recovery may still be possible
* ``failed`` — terminal; pipeline should switch modes or restart
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Literal, Optional, Sequence

OutputMode = Literal["optical_flow", "vio", "none"]
EstimatorState = Literal[
    "off",
    "init",
    "converging",
    "converged",
    "degraded",
    "failed",
]
ScaleSource = Literal["rangefinder", "baro", "gps", "vision", "none"]


@dataclass
class EstimatorOutput:
    """One sample produced by an estimator.

    ``timestamp_us`` is the time-of-frame the output corresponds to,
    in the FC-aligned monotonic clock when ``clock_align`` is available
    and the local monotonic clock otherwise. The pipeline stamps the
    MAVLink emission with this value.

    ``output_mode`` decides which fields the consumer reads. The two
    halves of the shape never carry data simultaneously: an OF sample
    sets ``flow_*`` and leaves ``pose``/``velocity``/``covariance``
    ``None``; a VIO sample does the opposite. ``"none"`` is the
    explicit empty-shape value emitted by :class:`NullEstimator`.
    """

    timestamp_us: int
    output_mode: OutputMode
    state: EstimatorState = "off"
    # OF path
    flow_rate_x: Optional[float] = None
    flow_rate_y: Optional[float] = None
    flow_rate_z: Optional[float] = None
    flow_quality: Optional[int] = None
    flow_distance_m: Optional[float] = None
    flow_scale_source: Optional[ScaleSource] = None
    integration_time_us: Optional[int] = None
    # VIO path (future phases populate these)
    pose: Optional[Sequence[float]] = None  # (x, y, z, roll, pitch, yaw)
    velocity: Optional[Sequence[float]] = None  # (vx, vy, vz) in local frame
    covariance: Optional[Sequence[float]] = None  # 21-element upper-triangular
    feature_count: Optional[int] = None
    reset_counter: Optional[int] = None
    # Cross-mode diagnostics
    drift_estimate_m: Optional[float] = None
    sync_offset_ms: Optional[float] = None
    extras: dict = field(default_factory=dict)


class BaseEstimator(ABC):
    """Estimator contract consumed by the estimator-agnostic pipeline.

    Concrete subclasses declare their ``estimator_id`` (matches the
    registry key) and ``output_mode``. The pipeline calls
    :meth:`configure` once with the per-drone config block, then loops
    on :meth:`step` for every frame pair, and finally calls
    :meth:`shutdown` on plugin teardown.

    The contract does not require subclasses to implement
    :meth:`configure` or :meth:`shutdown` — both have safe no-op
    defaults — so a trivial estimator can be a single ``step`` method.
    """

    estimator_id: str = "base"
    output_mode: OutputMode = "none"

    def configure(self, _config: object) -> None:
        """Receive the live per-drone config block.

        Default is a no-op. Estimators that need to inspect camera FOV,
        rangefinder topology, or scale-source preference override this.
        """

    @abstractmethod
    def step(
        self,
        *,
        prev_frame: Optional[object] = None,
        curr_frame: Optional[object] = None,
        dt_seconds: float = 0.0,
        imu_batch: Optional[object] = None,
        range_reading: Optional[object] = None,
    ) -> Optional[EstimatorOutput]:
        """Process one frame pair (and optional IMU + range) into a sample.

        Returns ``None`` when the inputs are insufficient (e.g. the
        first frame after start, or a frame pair with no trackable
        features). Returning ``None`` is a normal signal to the
        pipeline: it skips emission and keeps the streak counter
        running.
        """

    def shutdown(self) -> None:
        """Release any held resources. Default is a no-op."""
