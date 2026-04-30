"""LeptonUvcDriver: concrete CameraDriver for FLIR Lepton over USB UVC.

The driver subclasses :class:`ados.sdk.drivers.camera.CameraDriver`. It
delegates physical I/O to a :class:`LibUvcBackend` so the same code
runs against the in-tree :class:`MockUvcBackend` in tests and against
a real libuvc binding once that drops in.

The driver does NOT own the colorize step or the H.264 encode. Those
live in the agent's video pipeline and consume the radiometric frames
the driver pushes onto the event bus. Keeping the driver narrow makes
the abstraction stable for forks (Workswell, Boson, Vue Pro) that
share UVC plumbing but ship different colorize and encode stacks.
"""

from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass
from typing import Any, AsyncIterator

from altnautica_thermal_camera.uvc_backend import (
    LibUvcBackend,
    UvcDeviceInfo,
)

try:  # pragma: no cover - SDK is provided by the agent runtime
    from ados.sdk.drivers.camera import (
        CameraCandidate,
        CameraCapabilities,
        CameraDriver,
        CameraSession,
        FrameBuffer,
    )
    from ados.sdk.drivers.errors import (
        DriverDeviceNotFound,
        DriverError,
    )
except ImportError:  # pragma: no cover - exercised when running unit tests
    # Standalone test fallback: the extension's tests should not require the
    # full agent runtime. The SDK ABCs are re-declared here as plain stubs so
    # ``pytest`` can import the driver and verify its behaviour. The real
    # subclass relationship is restored as soon as the agent SDK is on the
    # import path.
    from abc import ABC, abstractmethod
    from dataclasses import dataclass as _dataclass
    from typing import AsyncIterator as _AsyncIter

    @_dataclass(frozen=True)
    class CameraCandidate:  # type: ignore[no-redef]
        driver_id: str
        device_id: str
        label: str
        bus: str
        vid_pid: tuple[int, int] | None = None
        metadata: dict[str, Any] | None = None

    @_dataclass(frozen=True)
    class CameraCapabilities:  # type: ignore[no-redef]
        radiometric: bool
        bit_depth: int
        width: int
        height: int
        fps: float
        pixel_format: str
        streaming_protocol: str
        color_spaces: list[str]
        has_audio: bool = False

    @_dataclass(frozen=True)
    class FrameBuffer:  # type: ignore[no-redef]
        timestamp_ns: int
        sequence: int
        width: int
        height: int
        pixel_format: str
        data: memoryview
        radiometric_k: memoryview | None = None
        metadata: dict[str, Any] | None = None

    class CameraSession:  # type: ignore[no-redef]
        pass

    class CameraDriver(ABC):  # type: ignore[no-redef]
        @abstractmethod
        async def discover(self) -> list[CameraCandidate]: ...

        @abstractmethod
        async def open(
            self, candidate: CameraCandidate, config: dict[str, Any]
        ) -> CameraSession: ...

        @abstractmethod
        async def close(self, session: CameraSession) -> None: ...

        @abstractmethod
        def capabilities(self, session: CameraSession) -> CameraCapabilities: ...

        @abstractmethod
        async def frame_iterator(
            self, session: CameraSession
        ) -> _AsyncIter[FrameBuffer]: ...

        @abstractmethod
        async def set_param(
            self, session: CameraSession, param: str, value: Any
        ) -> None: ...

    class DriverError(Exception):  # type: ignore[no-redef]
        pass

    class DriverDeviceNotFound(DriverError):  # type: ignore[no-redef]
        pass


DRIVER_ID = "altnautica.thermal-flir-lepton-usb"
PIXEL_FORMAT_Y16 = "Y16"
STREAMING_PROTOCOL = "uvc"
LEPTON_BIT_DEPTH = 14
MIN_FIRMWARE_VERSION = (1, 2, 2)
ITAR_CAPPED_FPS = 8.0


@dataclass
class LeptonUvcSession(CameraSession):
    """Per-open driver state.

    The session carries the device record, a numpy view onto the
    in-flight frame, the active palette name, and a hot copy of the
    config block. The peripheral manager treats this opaquely.
    """

    device: UvcDeviceInfo
    palette: str
    radiometric: bool
    fps: float
    open_timestamp_ns: int


class LeptonUvcDriver(CameraDriver):
    """CameraDriver subclass for FLIR Lepton 3.5 over PureThermal 2.

    The driver constructs against a backend that satisfies
    :class:`LibUvcBackend`. Tests pass a :class:`MockUvcBackend`. The
    real native binding lands once procurement closes.

    The backend's ``frames`` is a synchronous iterator. The driver
    bridges it into the SDK's async ``frame_iterator`` via
    :func:`asyncio.to_thread`, keeping the agent's event loop free.
    """

    driver_id = DRIVER_ID

    def __init__(self, backend: LibUvcBackend) -> None:
        self._backend = backend

    async def discover(self) -> list[CameraCandidate]:
        devices = await asyncio.to_thread(self._backend.enumerate)
        return [
            CameraCandidate(
                driver_id=self.driver_id,
                device_id=dev.serial,
                label=f"FLIR Lepton 3.5 (PureThermal 2, sn={dev.serial})",
                bus="usb",
                vid_pid=(dev.vid, dev.pid),
                metadata={
                    "firmware_version": dev.firmware_version,
                    "bus_path": dev.bus_path,
                    "radiometric": dev.radiometric,
                    "itar_restricted": dev.itar_restricted,
                },
            )
            for dev in devices
        ]

    async def open(
        self,
        candidate: CameraCandidate,
        config: dict[str, Any],
    ) -> LeptonUvcSession:
        device = await self._resolve_device(candidate)
        if not _firmware_meets(device.firmware_version, MIN_FIRMWARE_VERSION):
            raise DriverError(
                f"PureThermal firmware {device.firmware_version} is below the "
                f"required {'.'.join(str(p) for p in MIN_FIRMWARE_VERSION)}."
            )

        await asyncio.to_thread(self._backend.open, device)
        try:
            await asyncio.to_thread(
                self._backend.set_radiometry, device, True
            )
        except Exception:
            await asyncio.to_thread(self._backend.close, device)
            raise

        palette = str(config.get("palette", "ironbow"))
        capped_fps = ITAR_CAPPED_FPS if device.itar_restricted else device.fps
        return LeptonUvcSession(
            device=device,
            palette=palette,
            radiometric=device.radiometric,
            fps=capped_fps,
            open_timestamp_ns=time.monotonic_ns(),
        )

    async def close(self, session: CameraSession) -> None:
        if not isinstance(session, LeptonUvcSession):
            raise DriverError("session is not a LeptonUvcSession")
        await asyncio.to_thread(self._backend.close, session.device)

    def capabilities(self, session: CameraSession) -> CameraCapabilities:
        if not isinstance(session, LeptonUvcSession):
            raise DriverError("session is not a LeptonUvcSession")
        return CameraCapabilities(
            radiometric=session.radiometric,
            bit_depth=LEPTON_BIT_DEPTH,
            width=session.device.width,
            height=session.device.height,
            fps=session.fps,
            pixel_format=PIXEL_FORMAT_Y16,
            streaming_protocol=STREAMING_PROTOCOL,
            color_spaces=["Y16"],
            has_audio=False,
        )

    async def frame_iterator(
        self, session: CameraSession
    ) -> AsyncIterator[FrameBuffer]:
        if not isinstance(session, LeptonUvcSession):
            raise DriverError("session is not a LeptonUvcSession")
        device = session.device
        return _async_frames(self._backend, device)

    async def set_param(
        self, session: CameraSession, param: str, value: Any
    ) -> None:
        if not isinstance(session, LeptonUvcSession):
            raise DriverError("session is not a LeptonUvcSession")
        if param == "palette":
            from altnautica_thermal_camera.palettes import list_palettes

            valid = list_palettes()
            if value not in valid:
                raise ValueError(
                    f"unknown palette {value!r}; valid: {', '.join(valid)}"
                )
            session.palette = value
            return
        if param == "ffc":
            await asyncio.to_thread(self._backend.trigger_ffc, session.device)
            return
        if param == "radiometry":
            enabled = bool(value)
            await asyncio.to_thread(
                self._backend.set_radiometry, session.device, enabled
            )
            session.radiometric = enabled
            return
        raise ValueError(f"unknown parameter {param!r}")

    async def _resolve_device(self, candidate: CameraCandidate) -> UvcDeviceInfo:
        devices = await asyncio.to_thread(self._backend.enumerate)
        for dev in devices:
            if dev.serial == candidate.device_id:
                return dev
        raise DriverDeviceNotFound(
            f"no UVC device matched candidate {candidate.device_id!r}"
        )


async def _async_frames(
    backend: LibUvcBackend, device: UvcDeviceInfo
) -> AsyncIterator[FrameBuffer]:
    """Bridge a synchronous backend iterator into the SDK's async surface."""

    sync_iter = await asyncio.to_thread(backend.frames, device)
    while True:
        try:
            uvc_frame = await asyncio.to_thread(_pull_one, sync_iter)
        except StopIteration:
            return
        if uvc_frame is None:
            return
        # The Y16 tuple becomes a contiguous bytes buffer the SDK can
        # hand to consumers as a memoryview without copying again.
        raw = bytes()
        b = bytearray(len(uvc_frame.y16) * 2)
        for i, v in enumerate(uvc_frame.y16):
            b[i * 2] = v & 0xFF
            b[i * 2 + 1] = (v >> 8) & 0xFF
        raw = bytes(b)
        yield FrameBuffer(
            timestamp_ns=uvc_frame.timestamp_ns,
            sequence=uvc_frame.sequence,
            width=uvc_frame.width,
            height=uvc_frame.height,
            pixel_format=PIXEL_FORMAT_Y16,
            data=memoryview(raw),
            radiometric_k=None,
            metadata=dict(uvc_frame.metadata),
        )


def _pull_one(iterator: Any) -> Any:
    return next(iterator)


def _firmware_meets(
    version_str: str, minimum: tuple[int, int, int]
) -> bool:
    """Compare a "1.2.3" style version string against a minimum tuple."""

    try:
        parts = tuple(int(p) for p in version_str.strip().split("."))
    except ValueError:
        return False
    padded = parts + (0,) * (3 - len(parts))
    return padded[:3] >= minimum
