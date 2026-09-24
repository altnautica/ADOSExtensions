"""Camera intrinsics and camera-IMU extrinsics loaders.

Both loaders accept Kalibr-compatible YAML so an operator can drop a
``camchain.yaml`` produced by ``kalibr_calibrate_imu_camera`` straight
into the plugin's config directory. The schemas accept v1 monocular
pinhole only; stereo and fisheye intrinsics models are deferred.

The two loader entry points are :func:`load_intrinsics` and
:func:`load_extrinsics`. Both raise an operator-readable error on any
validation failure, so a file can be checked before it is copied onto a
vehicle.
"""

from __future__ import annotations

from altnautica_vision_nav_calib.intrinsics import (
    CameraIntrinsics,
    CameraIntrinsicsError,
    DistortionModel,
    load_intrinsics,
)
from altnautica_vision_nav_calib.extrinsics import (
    CameraImuExtrinsics,
    CameraImuExtrinsicsError,
    load_extrinsics,
)


class CalibrationError(Exception):
    """Umbrella exception for any calibration parse / validate failure."""


__all__ = [
    "CalibrationError",
    "CameraImuExtrinsics",
    "CameraImuExtrinsicsError",
    "CameraIntrinsics",
    "CameraIntrinsicsError",
    "DistortionModel",
    "load_extrinsics",
    "load_intrinsics",
]
