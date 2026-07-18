"""Stub the agent SDK surfaces when ADOSDroneAgent is not on the import path.

The plugin imports ``ados.sdk.vision`` (detection wire shapes), the driver ABCs
``ados.sdk.drivers.gimbal`` / ``ados.sdk.drivers.camera``, and
``ados.sdk.tracking`` (the shared locked-target gate). In an isolated checkout
those are absent, so this provides minimal compatible namespaces so the unit
tests run anywhere. When the real host package is importable each stub steps
aside and the tests exercise the real contract.
"""

from __future__ import annotations

import importlib
import sys
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from types import ModuleType
from typing import Any, AsyncIterator


def _module(name: str) -> ModuleType:
    mod = sys.modules.get(name)
    if mod is None:
        mod = ModuleType(name)
        mod.__path__ = []  # type: ignore[attr-defined]
        sys.modules[name] = mod
    return mod


def _ensure_vision_stub() -> None:
    try:
        importlib.import_module("ados.sdk.vision")
        return
    except ModuleNotFoundError:
        pass
    _module("ados")
    _module("ados.sdk")
    vision = ModuleType("ados.sdk.vision")

    @dataclass(frozen=True)
    class BoundingBox:
        x: float
        y: float
        width: float
        height: float

    @dataclass(frozen=True)
    class Detection:
        bbox: BoundingBox
        class_label: str
        confidence: float
        track_id: int | None = None
        assoc_confidence: float | None = None
        lock_state: str | None = None
        attributes: dict[str, Any] | None = None

    @dataclass(frozen=True)
    class DetectionBatch:
        model_id: str
        camera_id: str
        frame_id: int
        ts_ms: int
        detections: list[Detection] = field(default_factory=list)
        v: int = 1

    vision.BoundingBox = BoundingBox
    vision.Detection = Detection
    vision.DetectionBatch = DetectionBatch
    sys.modules["ados.sdk"].vision = vision  # type: ignore[attr-defined]
    sys.modules["ados.sdk.vision"] = vision


def _ensure_driver_stubs() -> None:
    _module("ados")
    _module("ados.sdk")
    _module("ados.sdk.drivers")

    try:
        importlib.import_module("ados.sdk.drivers.gimbal")
    except ModuleNotFoundError:
        gimbal = ModuleType("ados.sdk.drivers.gimbal")

        @dataclass(frozen=True)
        class GimbalCandidate:
            driver_id: str
            device_id: str
            label: str
            bus: str

        @dataclass(frozen=True)
        class GimbalCapabilities:
            has_pitch: bool
            has_yaw: bool
            has_roll: bool
            pitch_min_deg: float
            pitch_max_deg: float
            yaw_min_deg: float
            yaw_max_deg: float
            roll_min_deg: float
            roll_max_deg: float
            max_rate_dps: float | None = None
            supports_follow_mode: bool = False
            supports_lock_mode: bool = False

        @dataclass(frozen=True)
        class GimbalState:
            timestamp_ns: int
            pitch_deg: float
            yaw_deg: float
            roll_deg: float
            mode: str = "neutral"

        class GimbalSession:  # noqa: D401
            """Opaque per-open state."""

        class GimbalDriver(ABC):
            @abstractmethod
            async def discover(self) -> list: ...
            @abstractmethod
            async def open(self, candidate, config) -> Any: ...
            @abstractmethod
            async def close(self, session) -> None: ...
            @abstractmethod
            def capabilities(self, session) -> Any: ...
            @abstractmethod
            async def command_attitude(
                self, session, pitch_deg, yaw_deg, roll_deg=0.0
            ) -> None: ...
            @abstractmethod
            async def command_rate(
                self, session, pitch_rate_dps, yaw_rate_dps, roll_rate_dps=0.0
            ) -> None: ...
            @abstractmethod
            def get_state(self, session) -> Any: ...
            @abstractmethod
            async def state_iterator(self, session) -> AsyncIterator: ...

        for name, obj in [
            ("GimbalCandidate", GimbalCandidate),
            ("GimbalCapabilities", GimbalCapabilities),
            ("GimbalState", GimbalState),
            ("GimbalSession", GimbalSession),
            ("GimbalDriver", GimbalDriver),
        ]:
            setattr(gimbal, name, obj)
        sys.modules["ados.sdk.drivers.gimbal"] = gimbal

    try:
        importlib.import_module("ados.sdk.drivers.camera")
    except ModuleNotFoundError:
        camera = ModuleType("ados.sdk.drivers.camera")

        @dataclass(frozen=True)
        class CameraCandidate:
            driver_id: str
            device_id: str
            label: str
            bus: str

        @dataclass(frozen=True)
        class CameraCapabilities:
            width: int
            height: int
            fps: float
            radiometric: bool = False

        class CameraSession:  # noqa: D401
            """Opaque per-open state."""

        class CameraDriver(ABC):
            @abstractmethod
            async def discover(self) -> list: ...
            @abstractmethod
            async def open(self, candidate, config) -> Any: ...
            @abstractmethod
            async def close(self, session) -> None: ...
            @abstractmethod
            def capabilities(self, session) -> Any: ...
            @abstractmethod
            async def frame_iterator(self, session) -> AsyncIterator: ...
            @abstractmethod
            async def set_param(self, session, param, value) -> None: ...

        for name, obj in [
            ("CameraCandidate", CameraCandidate),
            ("CameraCapabilities", CameraCapabilities),
            ("CameraSession", CameraSession),
            ("CameraDriver", CameraDriver),
        ]:
            setattr(camera, name, obj)
        sys.modules["ados.sdk.drivers.camera"] = camera


_ensure_vision_stub()
_ensure_driver_stubs()
