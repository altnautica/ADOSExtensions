"""UVC backend abstraction for the thermal camera driver.

The PureThermal 2 dongle exposes the FLIR Lepton 3.5 as a USB UVC
video device. A UVC backend reaches the dongle through a native
libuvc binding. No such binding ships in this module: this module
defines only the abstraction the driver programs against.

This module ships:

* :class:`LibUvcBackend` Protocol describing the surface the driver
  needs from any UVC backend.

* :class:`UvcDeviceInfo` and :class:`UvcFrame` data classes that the
  Protocol's methods exchange.

A concrete backend satisfying :class:`LibUvcBackend` is injected into
:class:`~altnautica_thermal_camera.plugin.ThermalUsbPlugin` through its
``backend_factory`` argument. Without one the plugin has no device and
reports an explicit unavailable state rather than synthesising data, so
no synthetic or stand-in backend lives on this import path. The unit
suite's synthetic backend lives in ``agent/tests/mock_backend.py`` and
is never importable from the production package.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterator, Protocol, runtime_checkable

DEFAULT_WIDTH = 160
DEFAULT_HEIGHT = 120
DEFAULT_FPS = 8.7


@dataclass(frozen=True)
class UvcDeviceInfo:
    """Summary of a UVC device discovered on the bus.

    The driver builds a :class:`ados.sdk.drivers.camera.CameraCandidate`
    from this and lets the peripheral manager arbitrate.
    """

    serial: str
    vid: int
    pid: int
    bus_path: str
    firmware_version: str
    width: int = DEFAULT_WIDTH
    height: int = DEFAULT_HEIGHT
    fps: float = DEFAULT_FPS
    radiometric: bool = True
    itar_restricted: bool = False


@dataclass(frozen=True)
class UvcFrame:
    """One Y16 frame yielded by the backend.

    ``y16`` is a flat ``tuple[int, ...]`` of length ``width * height`` so
    consumers can iterate without forcing a numpy dependency. The
    driver wraps this into the SDK's ``FrameBuffer`` shape.
    """

    timestamp_ns: int
    sequence: int
    width: int
    height: int
    y16: tuple[int, ...]
    metadata: dict[str, object] = field(default_factory=dict)


@runtime_checkable
class LibUvcBackend(Protocol):
    """The slice of libuvc the driver depends on.

    A concrete backend must implement ``enumerate``, ``open``,
    ``close``, ``frames``, ``set_radiometry``, ``set_tlinear_resolution``,
    ``trigger_ffc``, and ``firmware_version``. Implementations may be
    synchronous; the driver wraps them in ``asyncio.to_thread`` calls
    when the host runs the iterator from an async loop.
    """

    def enumerate(self) -> list[UvcDeviceInfo]: ...

    def open(self, device: UvcDeviceInfo) -> None: ...

    def close(self, device: UvcDeviceInfo) -> None: ...

    def frames(self, device: UvcDeviceInfo) -> Iterator[UvcFrame]: ...

    def set_radiometry(self, device: UvcDeviceInfo, enabled: bool) -> None: ...

    def set_tlinear_resolution(
        self, device: UvcDeviceInfo, k_per_count: float
    ) -> None:
        """Lock the camera's TLinear resolution to a known value.

        The Lepton 3.5 supports two resolutions: ``0.01`` K per count
        (the default) and ``0.1`` K per count. The driver MUST call
        this on open so frames carry temperatures in a known unit
        regardless of any sticky setting from a prior session.
        """

    def trigger_ffc(self, device: UvcDeviceInfo) -> None: ...

    def firmware_version(self, device: UvcDeviceInfo) -> str: ...
