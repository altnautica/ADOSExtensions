"""IMU sources and frame-IMU time alignment.

Estimators consume IMU samples through the :class:`BaseImuSource`
contract. Optical-flow estimators only read the gyro axes (for
derotation). VIO estimators read both gyro and accel. A future direct
DroneCAN or I2C source plugs in behind the same ABC without rewiring
the estimator.

The :class:`TimeAligner` pairs each camera frame with the closest IMU
sample, applies the calibration ``timeshift_cam_imu`` offset, and
watches the residual drift so the GCS can surface a yellow / red
sync-offset warning before a VIO arm gate.
"""

from __future__ import annotations

from altnautica_vision_nav.imu.base import BaseImuSource, ImuSample
from altnautica_vision_nav.imu.mavlink_raw_imu import MavlinkRawImu
from altnautica_vision_nav.imu.mavlink_scaled_imu2 import MavlinkScaledImu2
from altnautica_vision_nav.imu.time_aligner import (
    DRIFT_BAND_GREEN_MS,
    DRIFT_BAND_RED_MS,
    DRIFT_BAND_YELLOW_MS,
    DriftBand,
    TimeAligner,
)

__all__ = [
    "BaseImuSource",
    "DriftBand",
    "DRIFT_BAND_GREEN_MS",
    "DRIFT_BAND_YELLOW_MS",
    "DRIFT_BAND_RED_MS",
    "ImuSample",
    "MavlinkRawImu",
    "MavlinkScaledImu2",
    "TimeAligner",
]
