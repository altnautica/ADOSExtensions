"""Off-mode estimator.

Returns no telemetry and never emits MAVLink. Selecting this estimator
from config is the supported way to keep the plugin loaded with all
sensors discovered but the EKF feed silent. Useful for hardware
diagnostics, calibration runs, and demo-mode dry-runs where the
mode-picker UI should show the rest of the plugin operational.
"""

from __future__ import annotations

import time
from typing import Optional

from altnautica_vision_nav.estimators.base import (
    BaseEstimator,
    EstimatorOutput,
)


class NullEstimator(BaseEstimator):
    """Always returns ``None``; reports ``state="off"`` to the pipeline."""

    estimator_id = "off"
    output_mode = "none"

    def step(
        self,
        *,
        prev_frame: Optional[object] = None,
        curr_frame: Optional[object] = None,
        dt_seconds: float = 0.0,
        imu_batch: Optional[object] = None,
        range_reading: Optional[object] = None,
    ) -> Optional[EstimatorOutput]:
        # Returning a typed EstimatorOutput rather than None lets the
        # health publisher report ``estimatorState="off"`` cleanly. The
        # pipeline treats ``output_mode="none"`` as a signal to skip
        # MAVLink emission.
        return EstimatorOutput(
            timestamp_us=int(time.monotonic_ns() // 1000),
            output_mode="none",
            state="off",
        )
