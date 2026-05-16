"""Frame-IMU time alignment.

Estimators do their best work when each frame is paired with the IMU
sample taken at the same instant. Without a hardware sync trigger, the
camera and IMU sit on the agent's monotonic clock with a static offset
(``timeshift_cam_imu`` from Kalibr calibration) plus per-sample USB or
scheduler jitter on top.

The aligner does two things:

1. Given a camera frame timestamp, returns the closest IMU sample
   (linearly interpolated when both bracketing samples are present).
2. Tracks the residual drift over a sliding window and reports a
   :class:`DriftBand` (green / yellow / red) the GCS can surface.

Banding rules (revised per real-world measurement on USB UVC + Pi 4B):

* green: residual ≤ 10 ms
* yellow: 10 ms < residual ≤ 30 ms
* red: residual > 30 ms (refuse VIO arm)

Residual is computed as ``|frame_ts + timeshift_cam_imu - nearest_imu_ts|``
in milliseconds, averaged over the last ``window_size`` lookups.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, Literal, Optional, Tuple

from altnautica_vision_nav.imu.base import BaseImuSource, ImuSample

DriftBand = Literal["green", "yellow", "red"]

DRIFT_BAND_GREEN_MS = 10.0
DRIFT_BAND_YELLOW_MS = 30.0
# Red is anything above yellow; named here for symmetry in the public
# API surface.
DRIFT_BAND_RED_MS = float("inf")


@dataclass(frozen=True)
class AlignedSample:
    """An IMU sample paired with a camera frame timestamp.

    ``residual_ms`` is the absolute residual after applying the
    configured ``timeshift_cam_imu``. Useful in the estimator state
    machine as a per-step diagnostic.
    """

    frame_ts_ns: int
    imu_sample: ImuSample
    residual_ms: float


class TimeAligner:
    """Pair frame timestamps with IMU samples, watch drift over time."""

    def __init__(
        self,
        source: BaseImuSource,
        *,
        timeshift_cam_imu_s: float = 0.0,
        window_size: int = 60,
    ) -> None:
        self._source = source
        self._timeshift_ns = int(timeshift_cam_imu_s * 1e9)
        self._residuals: Deque[float] = deque(maxlen=int(window_size))

    @property
    def timeshift_s(self) -> float:
        """Currently-applied static offset in seconds."""

        return self._timeshift_ns / 1e9

    def set_timeshift(self, timeshift_cam_imu_s: float) -> None:
        """Replace the static offset (Kalibr calibration result)."""

        self._timeshift_ns = int(timeshift_cam_imu_s * 1e9)
        # Old residuals were computed against the previous offset and
        # are now meaningless; clear so the band reflects the new
        # calibration on the next lookup cycle.
        self._residuals.clear()

    def lookup(self, frame_ts_ns: int) -> Optional[AlignedSample]:
        """Find the IMU sample best matching ``frame_ts_ns``.

        Applies the static offset to the frame timestamp before
        searching. Interpolates linearly between the two bracketing
        samples when both are present; falls back to the nearest
        sample otherwise. Returns ``None`` when the IMU buffer is
        empty.
        """

        target_ns = frame_ts_ns + self._timeshift_ns
        samples = list(self._source.recent())
        if not samples:
            return None

        # Find bracket. samples are oldest-first; binary search would
        # be tidier but n is small (~400 max) and Python's tight loop
        # cost dominates anything algorithmic here.
        before: Optional[ImuSample] = None
        after: Optional[ImuSample] = None
        for sample in samples:
            if sample.ts_ns <= target_ns:
                before = sample
            else:
                after = sample
                break

        if before is not None and after is not None:
            picked, residual_ns = _interpolate(before, after, target_ns)
        elif before is not None:
            picked, residual_ns = before, abs(target_ns - before.ts_ns)
        elif after is not None:
            picked, residual_ns = after, abs(target_ns - after.ts_ns)
        else:
            return None  # unreachable given the empty check above

        residual_ms = residual_ns / 1e6
        self._residuals.append(residual_ms)
        return AlignedSample(
            frame_ts_ns=frame_ts_ns,
            imu_sample=picked,
            residual_ms=residual_ms,
        )

    def mean_residual_ms(self) -> Optional[float]:
        """Rolling average residual in milliseconds, or ``None`` if empty."""

        if not self._residuals:
            return None
        return sum(self._residuals) / len(self._residuals)

    def drift_band(self) -> DriftBand:
        """Categorise the current rolling residual into a band.

        Returns ``"green"`` when no samples have been paired yet; the
        GCS reads this and treats it as "calibration not yet running"
        rather than a sync failure.
        """

        mean = self.mean_residual_ms()
        if mean is None:
            return "green"
        if mean <= DRIFT_BAND_GREEN_MS:
            return "green"
        if mean <= DRIFT_BAND_YELLOW_MS:
            return "yellow"
        return "red"


def _interpolate(
    before: ImuSample,
    after: ImuSample,
    target_ns: int,
) -> Tuple[ImuSample, int]:
    """Linearly interpolate gyro and accel between two bracketing samples.

    Returns the interpolated sample stamped at ``target_ns`` and the
    residual distance to the nearest of the two real samples.
    """

    span = max(after.ts_ns - before.ts_ns, 1)
    weight = (target_ns - before.ts_ns) / span
    weight = max(0.0, min(1.0, weight))

    def lerp(a: float, b: float) -> float:
        return a + (b - a) * weight

    interpolated = ImuSample(
        ts_ns=target_ns,
        xgyro=lerp(before.xgyro, after.xgyro),
        ygyro=lerp(before.ygyro, after.ygyro),
        zgyro=lerp(before.zgyro, after.zgyro),
        xacc=lerp(before.xacc, after.xacc),
        yacc=lerp(before.yacc, after.yacc),
        zacc=lerp(before.zacc, after.zacc),
    )
    residual_ns = min(
        abs(target_ns - before.ts_ns),
        abs(target_ns - after.ts_ns),
    )
    return interpolated, residual_ns
