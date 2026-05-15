"""Lucas-Kanade pyramid optical-flow processor.

Inputs are two consecutive grayscale frames and a ``dt`` interval.
Outputs are flow rates in image-plane units (dpi), angular rates
(rad/s), and an optional metric-velocity estimate when a
height-above-ground distance is supplied.

The result shape matches the MAVLink ``OPTICAL_FLOW`` /
``OPTICAL_FLOW_RAD`` messages closely so the emission layer can copy
fields without further math.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Optional, Tuple

try:  # pragma: no cover
    import cv2  # type: ignore[import-untyped]
    import numpy as np  # type: ignore[import-untyped]
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore[assignment]
    np = None  # type: ignore[assignment]

from altnautica_vision_nav.capture.gyro_tap import GyroReading

log = logging.getLogger(__name__)

# Pixel displacement larger than this is treated as a tracking outlier
# and dropped from the median estimator. Real downward-pointing flow
# on a 640x480 frame at typical descent rates stays well below this.
MAX_PIXEL_DELTA = 30.0

DEFAULT_MAX_FEATURES = 200
DEFAULT_PYRAMID_LEVELS = 3
DEFAULT_WIN_SIZE: Tuple[int, int] = (15, 15)
DEFAULT_FOV_HORIZONTAL_DEG = 60.0
DEFAULT_FX_PIXELS = 500.0

# OPTICAL_FLOW.flow_x / flow_y are reported in tenths of a pixel per
# the MAVLink message comment, but the wider ecosystem (px4flow,
# downstream EKF tuners) treats the legacy scale as 8x. This codepath
# follows the 8x scaling so the value lines up with what firmware
# expects on the wire.
DPI_SCALE = 8


@dataclass(frozen=True)
class OpticalFlowResult:
    """One sample of optical-flow output ready for MAVLink emission."""

    flow_x_dpi: float
    flow_y_dpi: float
    flow_comp_m_x: float
    flow_comp_m_y: float
    flow_rate_x: float
    flow_rate_y: float
    flow_rate_z: float
    quality: int
    integration_time_us: int


class OpticalFlowLk:
    """Stateful pyramidal Lucas-Kanade flow estimator.

    Construct once per pipeline, then call :meth:`process` on every
    consecutive frame pair. The estimator builds a per-call set of
    corner features via ``goodFeaturesToTrack`` so it does not need to
    persist tracking state between calls; the first frame stays in
    memory only to the extent the caller passes it back in.

    Parameters
    ----------
    max_features:
        Upper bound on the corners fed into the pyramid tracker.
    pyramid_levels:
        Number of pyramid levels (0 means a single-resolution pass).
    win_size:
        Search window for each pyramid level.
    fov_horizontal_deg:
        Camera horizontal field of view in degrees. Used for gyro
        derotation magnitude.
    fx_pixels:
        Focal length in pixels. Used to convert image-plane displacement
        to angular rate.
    """

    def __init__(
        self,
        max_features: int = DEFAULT_MAX_FEATURES,
        pyramid_levels: int = DEFAULT_PYRAMID_LEVELS,
        win_size: Tuple[int, int] = DEFAULT_WIN_SIZE,
        fov_horizontal_deg: float = DEFAULT_FOV_HORIZONTAL_DEG,
        fx_pixels: float = DEFAULT_FX_PIXELS,
    ) -> None:
        if cv2 is None:  # pragma: no cover - import guard.
            raise RuntimeError("opencv is required for OpticalFlowLk")
        self.max_features = int(max_features)
        self.pyramid_levels = int(pyramid_levels)
        self.win_size = (int(win_size[0]), int(win_size[1]))
        self.fov_horizontal_rad = math.radians(float(fov_horizontal_deg))
        self.fx_pixels = float(fx_pixels)
        # The vertical FOV depends on the live frame aspect, so it is
        # set on the first ``process()`` call once the frame is in hand.
        self.fov_vertical_rad: float = 0.0
        # CLAHE is built once and reused per call; the OpenCV object is
        # cheap to keep around and constructing it each call would burn
        # an unnecessary allocation.
        self._clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def process(
        self,
        prev_gray: "np.ndarray",  # type: ignore[name-defined]
        curr_gray: "np.ndarray",  # type: ignore[name-defined]
        dt_seconds: float,
        gyro: Optional[GyroReading] = None,
        distance_m: Optional[float] = None,
    ) -> OpticalFlowResult:
        """Compute one flow sample from a pair of consecutive frames.

        Parameters
        ----------
        prev_gray, curr_gray:
            Single-channel or BGR frames. BGR inputs are converted to
            gray internally so callers can pass whatever the source
            delivers.
        dt_seconds:
            Interval between the two frames in seconds. Must be
            positive; a clamp guards divide-by-zero on bad timestamps.
        gyro:
            Optional latest gyro reading from :class:`GyroTap`. When
            present, the rotational component is subtracted from the
            visual flow rate.
        distance_m:
            Optional height-above-ground in metres. When present, a
            metric translational velocity is reported.
        """

        prev = self._to_gray(prev_gray)
        curr = self._to_gray(curr_gray)
        h, w = curr.shape[:2]
        aspect = (h / float(w)) if w > 0 else 0.75
        self.fov_vertical_rad = self.fov_horizontal_rad * aspect

        prev_eq = self._clahe.apply(prev)
        curr_eq = self._clahe.apply(curr)

        dt = max(float(dt_seconds), 1e-6)
        integration_time_us = int(round(dt * 1_000_000))

        corners = cv2.goodFeaturesToTrack(
            prev_eq,
            maxCorners=self.max_features,
            qualityLevel=0.01,
            minDistance=8,
            blockSize=7,
        )
        if corners is None or len(corners) == 0:
            return _empty_result(
                integration_time_us=integration_time_us,
                gyro=gyro,
            )

        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            prev_eq,
            curr_eq,
            corners,
            None,
            winSize=self.win_size,
            maxLevel=self.pyramid_levels,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03),
        )
        if next_pts is None or status is None:
            return _empty_result(
                integration_time_us=integration_time_us,
                gyro=gyro,
            )

        flat_status = status.reshape(-1)
        deltas = (next_pts - corners).reshape(-1, 2)
        tracked = flat_status == 1
        deltas = deltas[tracked]
        if deltas.size == 0:
            return _empty_result(
                integration_time_us=integration_time_us,
                gyro=gyro,
            )

        magnitudes = np.linalg.norm(deltas, axis=1)
        inlier_mask = magnitudes <= MAX_PIXEL_DELTA
        deltas = deltas[inlier_mask]
        if deltas.size == 0:
            return _empty_result(
                integration_time_us=integration_time_us,
                gyro=gyro,
            )

        dx_pixels = float(np.median(deltas[:, 0]))
        dy_pixels = float(np.median(deltas[:, 1]))
        num_tracked = int(deltas.shape[0])

        flow_x_dpi = dx_pixels * DPI_SCALE
        flow_y_dpi = dy_pixels * DPI_SCALE

        flow_rate_x = math.atan(dx_pixels / self.fx_pixels) / dt
        flow_rate_y = math.atan(dy_pixels / self.fx_pixels) / dt
        flow_rate_z = 0.0

        if gyro is not None:
            flow_rate_x -= gyro.xgyro * self.fov_vertical_rad / 2.0
            flow_rate_y -= gyro.ygyro * self.fov_horizontal_rad / 2.0
            flow_rate_z = gyro.zgyro

        if distance_m is not None and distance_m > 0:
            flow_comp_m_x = flow_rate_x * float(distance_m)
            flow_comp_m_y = flow_rate_y * float(distance_m)
        else:
            flow_comp_m_x = 0.0
            flow_comp_m_y = 0.0

        quality = max(0, min(255, int(num_tracked * 5)))

        return OpticalFlowResult(
            flow_x_dpi=flow_x_dpi,
            flow_y_dpi=flow_y_dpi,
            flow_comp_m_x=flow_comp_m_x,
            flow_comp_m_y=flow_comp_m_y,
            flow_rate_x=flow_rate_x,
            flow_rate_y=flow_rate_y,
            flow_rate_z=flow_rate_z,
            quality=quality,
            integration_time_us=integration_time_us,
        )

    def _to_gray(self, frame: "np.ndarray") -> "np.ndarray":  # type: ignore[name-defined]
        """Return a single-channel uint8 view of ``frame``."""

        if frame.ndim == 2:
            if frame.dtype != np.uint8:
                return frame.astype(np.uint8)
            return frame
        if frame.ndim == 3 and frame.shape[2] == 3:
            return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if frame.ndim == 3 and frame.shape[2] == 4:
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)
        raise ValueError(f"unsupported frame shape for optical flow: {frame.shape}")


def _empty_result(
    integration_time_us: int, gyro: Optional[GyroReading]
) -> OpticalFlowResult:
    """Return a zero-flow result, optionally carrying gyro Z."""

    flow_rate_z = gyro.zgyro if gyro is not None else 0.0
    return OpticalFlowResult(
        flow_x_dpi=0.0,
        flow_y_dpi=0.0,
        flow_comp_m_x=0.0,
        flow_comp_m_y=0.0,
        flow_rate_x=0.0,
        flow_rate_y=0.0,
        flow_rate_z=flow_rate_z,
        quality=0,
        integration_time_us=integration_time_us,
    )
