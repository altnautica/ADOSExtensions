"""Camera capture backends and the MAVLink gyro tap."""

from altnautica_vision_nav.capture.gyro_tap import GyroReading, GyroTap
from altnautica_vision_nav.capture.v4l2_source import V4l2Source

__all__ = [
    "GyroReading",
    "GyroTap",
    "V4l2Source",
    "LibcameraSource",
]


def __getattr__(name: str):
    # LibcameraSource imports picamera2, which is not available on dev
    # hosts or CI. Defer the import so the package stays importable.
    if name == "LibcameraSource":
        from altnautica_vision_nav.capture.libcamera_source import LibcameraSource

        return LibcameraSource
    raise AttributeError(f"module 'altnautica_vision_nav.capture' has no attribute {name!r}")
