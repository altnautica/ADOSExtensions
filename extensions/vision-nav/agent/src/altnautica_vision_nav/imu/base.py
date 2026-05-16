"""IMU source contract.

Every IMU subscriber returns the same :class:`ImuSample` shape. SI
units throughout: gyro in rad/s, accel in m/s². The source records the
monotonic timestamp at which the sample was received on the agent, so
the time aligner can pair frames with IMU samples on a single clock
without cross-clock drift.

The MAVLink-flavoured sources (``RAW_IMU``, ``SCALED_IMU2``) implement
this contract. A future ``DirectI2cImu`` / ``DroneCanImu`` plugs in
the same way; estimators do not need to know which source they hold.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass
from typing import Deque, Iterable, Optional


@dataclass(frozen=True)
class ImuSample:
    """One IMU sample reduced to SI units.

    ``ts_ns`` is the agent monotonic timestamp captured at ingestion.
    Gyro is rad/s in the body frame (x forward, y right, z down in
    ArduPilot's NED-on-the-airframe convention; PX4 matches). Accel is
    m/s² in the same frame.
    """

    ts_ns: int
    xgyro: float
    ygyro: float
    zgyro: float
    xacc: float
    yacc: float
    zacc: float


class BaseImuSource(ABC):
    """Common contract for every IMU subscriber.

    Concrete sources keep a fixed-capacity ring buffer of recent
    samples so the :class:`TimeAligner` can interpolate (or pick the
    closest) for any given frame timestamp. The default buffer holds
    2 s at 200 Hz, plenty for the camera-IMU pairing the estimators
    do today and for the higher rates a future direct source brings.
    """

    source_id: str = "base"

    def __init__(self, *, buffer_capacity: int = 400) -> None:
        self._buffer: Deque[ImuSample] = deque(maxlen=int(buffer_capacity))
        self._last_rate_hz: Optional[float] = None
        self._last_seen_ns: Optional[int] = None

    @abstractmethod
    def start(self) -> None:
        """Begin receiving samples. Idempotent."""

    @abstractmethod
    def stop(self) -> None:
        """Stop receiving samples. Idempotent."""

    def latest(self) -> Optional[ImuSample]:
        """Return the most recent sample or ``None`` if no data yet."""

        if not self._buffer:
            return None
        return self._buffer[-1]

    def recent(self) -> Iterable[ImuSample]:
        """Iterate the in-buffer samples oldest-first.

        Returned in chronological order. The time aligner walks this
        sequence when pairing a frame timestamp with the closest IMU
        sample. The buffer is fixed-capacity so memory stays bounded.
        """

        return list(self._buffer)

    def rate_hz(self) -> Optional[float]:
        """Smoothed estimate of the IMU sample rate in Hz.

        ``None`` until at least two samples have arrived. The estimate
        uses an EMA over inter-sample intervals so a single jitter
        sample does not whip the value around.
        """

        return self._last_rate_hz

    def _record(self, sample: ImuSample) -> None:
        """Append ``sample`` to the buffer and refresh the rate estimate.

        Subclasses call this from their MAVLink handler. The base class
        owns the buffer + rate-EMA state so concrete sources stay
        focused on the wire format they translate.
        """

        if self._last_seen_ns is not None:
            dt_s = max((sample.ts_ns - self._last_seen_ns) / 1e9, 1e-6)
            instantaneous_hz = 1.0 / dt_s
            if self._last_rate_hz is None:
                self._last_rate_hz = instantaneous_hz
            else:
                # EMA with alpha = 0.1; converges in ~30 samples and
                # rejects a single 10x jitter on the wire.
                self._last_rate_hz = (
                    0.9 * self._last_rate_hz + 0.1 * instantaneous_hz
                )
        self._last_seen_ns = sample.ts_ns
        self._buffer.append(sample)
