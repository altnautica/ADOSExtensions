"""Test setup: stub the agent SDK gimbal ABC if the real package is
not on the import path.

The plugin imports ``ados.sdk.drivers.gimbal``. When the extension is
tested in isolation (no ADOSDroneAgent on PYTHONPATH), we provide a
minimal compatible namespace so unit tests can run anywhere.
"""

from __future__ import annotations

import importlib
import sys
from abc import ABC, abstractmethod
from dataclasses import dataclass
from types import ModuleType
from typing import Any, AsyncIterator


def _ensure_sdk_stub() -> None:
    try:
        importlib.import_module("ados.sdk.drivers.gimbal")
        return
    except ModuleNotFoundError:
        pass

    pkg = ModuleType("ados")
    pkg.__path__ = []  # type: ignore[attr-defined]
    sdk = ModuleType("ados.sdk")
    sdk.__path__ = []  # type: ignore[attr-defined]
    drivers = ModuleType("ados.sdk.drivers")
    drivers.__path__ = []  # type: ignore[attr-defined]
    gimbal = ModuleType("ados.sdk.drivers.gimbal")

    @dataclass(frozen=True)
    class GimbalCandidate:
        driver_id: str
        device_id: str
        label: str
        bus: str
        vid_pid: tuple[int, int] | None = None
        metadata: dict[str, Any] | None = None

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
        pitch_rate_dps: float = 0.0
        yaw_rate_dps: float = 0.0
        roll_rate_dps: float = 0.0
        mode: str = "neutral"
        metadata: dict[str, Any] | None = None

    class GimbalSession:  # noqa: D401 - matches real surface
        """Opaque per-open state."""

    class GimbalDriver(ABC):
        @abstractmethod
        async def discover(self) -> list[GimbalCandidate]: ...
        @abstractmethod
        async def open(
            self, candidate: GimbalCandidate, config: dict[str, Any]
        ) -> GimbalSession: ...
        @abstractmethod
        async def close(self, session: GimbalSession) -> None: ...
        @abstractmethod
        def capabilities(self, session: GimbalSession) -> GimbalCapabilities: ...
        @abstractmethod
        async def command_attitude(
            self,
            session: GimbalSession,
            pitch_deg: float,
            yaw_deg: float,
            roll_deg: float = 0.0,
        ) -> None: ...
        @abstractmethod
        async def command_rate(
            self,
            session: GimbalSession,
            pitch_rate_dps: float,
            yaw_rate_dps: float,
            roll_rate_dps: float = 0.0,
        ) -> None: ...
        @abstractmethod
        def get_state(self, session: GimbalSession) -> GimbalState: ...
        @abstractmethod
        async def state_iterator(
            self, session: GimbalSession
        ) -> AsyncIterator[GimbalState]: ...

    gimbal.GimbalCandidate = GimbalCandidate
    gimbal.GimbalCapabilities = GimbalCapabilities
    gimbal.GimbalState = GimbalState
    gimbal.GimbalSession = GimbalSession
    gimbal.GimbalDriver = GimbalDriver

    sys.modules["ados"] = pkg
    sys.modules["ados.sdk"] = sdk
    sys.modules["ados.sdk.drivers"] = drivers
    sys.modules["ados.sdk.drivers.gimbal"] = gimbal


_ensure_sdk_stub()
