"""Base interface every rangefinder driver implements."""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional


@dataclass
class RangeReading:
    """One distance measurement from a rangefinder driver."""

    distance_m: float
    """Measured distance in meters."""

    quality: int
    """Confidence in the reading, 0..100. 0 means invalid."""

    timestamp_monotonic_ns: int
    """Monotonic timestamp captured at the moment of the read."""

    raw_status: Optional[int] = None
    """Driver-specific status code, when the underlying sensor exposes one."""


class RangefinderDriver(ABC):
    """Common contract for downward-facing distance sensors."""

    @abstractmethod
    async def open(self) -> None:
        """Open the underlying transport (serial port, I2C bus, MAVLink sub)."""

    @abstractmethod
    async def close(self) -> None:
        """Release the underlying transport."""

    @abstractmethod
    async def read(self) -> Optional[RangeReading]:
        """Return one reading, or None when no fresh data is available."""

    @property
    @abstractmethod
    def min_range_m(self) -> float:
        """Lower bound of the sensor's valid measurement window, in meters."""

    @property
    @abstractmethod
    def max_range_m(self) -> float:
        """Upper bound of the sensor's valid measurement window, in meters."""

    @property
    @abstractmethod
    def name(self) -> str:
        """Short stable identifier for logs and telemetry."""
