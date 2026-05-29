"""Camera-IMU extrinsics + timeshift loader.

The same Kalibr ``cam0`` block carries ``T_cam_imu`` (4x4 transform
from IMU to camera frame) and ``timeshift_cam_imu`` (scalar, seconds).
The sign convention is the Kalibr canonical one:

    t_imu = t_cam + timeshift_cam_imu

A positive offset means the IMU clock is ahead of the camera clock;
a negative offset (the common case on USB UVC where the frame
timestamp is set after USB transfer completes) means the camera clock
is ahead.

The loader validates SE(3) (orthonormal rotation block, det = +1,
bottom row ``[0,0,0,1]``) and sanity-bounds the translation
magnitude. Drone cam-IMU lever-arms are millimetres to centimetres;
any translation > 1 m is almost always a unit error (metres vs
millimetres) and is rejected.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Optional

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

TIMESHIFT_WARN_LIMIT_S = 0.2
TIMESHIFT_ERROR_LIMIT_S = 0.5
TRANSLATION_ERROR_LIMIT_M = 1.0


class CameraImuExtrinsicsError(ValueError):
    """Raised when an extrinsics file fails to parse or validate."""


class CameraImuExtrinsics(BaseModel):
    """``cam0`` extrinsics: SE(3) transform + scalar timeshift.

    The 4x4 transform follows the Kalibr convention: it maps points
    expressed in the IMU body frame to points expressed in the camera
    frame. ``timeshift_cam_imu`` is in seconds and the sign follows
    Kalibr's ``t_imu = t_cam + timeshift_cam_imu``.
    """

    model_config = ConfigDict(extra="ignore")

    T_cam_imu: list[list[float]] = Field(min_length=4, max_length=4)
    timeshift_cam_imu: float = 0.0

    @property
    def rotation(self) -> list[list[float]]:
        return [row[:3] for row in self.T_cam_imu[:3]]

    @property
    def translation(self) -> list[float]:
        return [row[3] for row in self.T_cam_imu[:3]]

    @property
    def has_suspicious_timeshift(self) -> bool:
        """Surfaced as a warning, not an error.

        A 200 ms offset between an IMU and a USB camera is plausible
        on a software-timestamped stack but worth flagging; the
        calibration was probably run at a different camera mode than
        the one that's running now.
        """

        return abs(self.timeshift_cam_imu) > TIMESHIFT_WARN_LIMIT_S

    @model_validator(mode="after")
    def _validate(self) -> "CameraImuExtrinsics":
        # Bottom row of an SE(3) is canonically [0, 0, 0, 1].
        bottom = self.T_cam_imu[3]
        if (
            abs(bottom[0]) > 1e-6
            or abs(bottom[1]) > 1e-6
            or abs(bottom[2]) > 1e-6
            or abs(bottom[3] - 1.0) > 1e-6
        ):
            raise CameraImuExtrinsicsError(
                f"T_cam_imu bottom row must be [0,0,0,1], got {bottom}"
            )

        # Top-left 3x3 must be orthonormal with determinant +1. Reject
        # reflections (det = -1) outright; those are not valid SE(3).
        rot = self.rotation
        gram = _matmul_3x3(rot, _transpose_3x3(rot))
        if not _is_close_to_identity(gram, atol=1e-4):
            raise CameraImuExtrinsicsError(
                "T_cam_imu rotation block is not orthonormal"
            )

        det = _det_3x3(rot)
        if abs(det - 1.0) > 1e-4:
            raise CameraImuExtrinsicsError(
                f"T_cam_imu rotation determinant {det:.4f} must be +1"
            )

        # Translation sanity. Lever-arm units mistakes are by far the
        # most common operator error here; a 1 m lever arm means
        # someone wrote metres-as-millimetres or vice versa.
        t = self.translation
        norm = (t[0] ** 2 + t[1] ** 2 + t[2] ** 2) ** 0.5
        if norm > TRANSLATION_ERROR_LIMIT_M:
            raise CameraImuExtrinsicsError(
                f"T_cam_imu translation magnitude {norm:.3f} m exceeds 1.0 m; "
                "check that the calibration is in metres, not millimetres"
            )

        if abs(self.timeshift_cam_imu) > TIMESHIFT_ERROR_LIMIT_S:
            raise CameraImuExtrinsicsError(
                f"timeshift_cam_imu {self.timeshift_cam_imu:.3f}s exceeds "
                f"±{TIMESHIFT_ERROR_LIMIT_S:.1f}s sanity bound"
            )

        return self


def load_extrinsics(path: str | Path) -> CameraImuExtrinsics:
    """Load and validate the extrinsics block from a Kalibr YAML file.

    Accepts the same layouts as the intrinsics loader: ``{cam0: {...}}``
    or a bare block. Returns the validated :class:`CameraImuExtrinsics`.
    """

    path = Path(path)
    if not path.exists():
        raise CameraImuExtrinsicsError(f"calibration file not found: {path}")

    try:
        text = path.read_text(encoding="utf-8")
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise CameraImuExtrinsicsError(f"YAML parse failed: {exc}") from exc

    block = _select_cam_block(data)
    if not isinstance(block, dict):
        raise CameraImuExtrinsicsError(
            "expected a mapping at the top level (or under 'cam0')"
        )

    try:
        return CameraImuExtrinsics.model_validate(block)
    except CameraImuExtrinsicsError:
        raise
    except Exception as exc:  # noqa: BLE001 - Pydantic raises a different type
        raise CameraImuExtrinsicsError(str(exc)) from exc


def _select_cam_block(data: Any) -> Any:
    if data is None:
        raise CameraImuExtrinsicsError("calibration file is empty")
    if isinstance(data, dict) and "cam0" in data:
        return data["cam0"]
    return data


# Tiny linear-algebra helpers so the loader stays numpy-free. The
# heavy Lucas-Kanade path already pulls numpy in; this loader is run
# once at start-up and the values are tiny, so a few hand-rolled
# helpers are cheaper than another import.


def _matmul_3x3(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [
            a[r][0] * b[0][c] + a[r][1] * b[1][c] + a[r][2] * b[2][c]
            for c in range(3)
        ]
        for r in range(3)
    ]


def _transpose_3x3(m: list[list[float]]) -> list[list[float]]:
    return [[m[c][r] for c in range(3)] for r in range(3)]


def _det_3x3(m: list[list[float]]) -> float:
    return (
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    )


def _is_close_to_identity(m: list[list[float]], *, atol: float) -> bool:
    for r in range(3):
        for c in range(3):
            target = 1.0 if r == c else 0.0
            if abs(m[r][c] - target) > atol:
                return False
    return True
