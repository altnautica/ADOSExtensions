"""Storm32 NT serial driver (stub).

The Storm32 NT board is a MAVLink-Gimbal-v2-native target whenever the
firmware is connected to the FC's MAVLink bus directly. This serial
driver covers the alternative deployment where the Storm32 hangs off
the companion computer's UART and the agent has to translate.

The class exists today so the supervisor can register a discoverable
``GimbalDriver`` slot. ``open()`` raises ``NotImplementedError`` until
the serial parser lands.
"""

from __future__ import annotations

from typing import Any, AsyncIterator

from ados.sdk.drivers.gimbal import (
    GimbalCandidate,
    GimbalCapabilities,
    GimbalDriver,
    GimbalSession,
    GimbalState,
)


class Storm32NtGimbalDriver(GimbalDriver):
    """Storm32 NT direct-serial driver (work in progress)."""

    driver_id = "storm32-uart"

    async def discover(self) -> list[GimbalCandidate]:
        return []

    async def open(
        self, candidate: GimbalCandidate, config: dict[str, Any]
    ) -> GimbalSession:
        raise NotImplementedError(
            "Storm32 NT direct-serial driver is not implemented yet. "
            "Connect the Storm32 to the FC's MAVLink network and use "
            "the MAVLink driver instead."
        )

    async def close(self, session: GimbalSession) -> None:  # pragma: no cover
        raise NotImplementedError

    def capabilities(self, session: GimbalSession) -> GimbalCapabilities:  # pragma: no cover
        raise NotImplementedError

    async def command_attitude(
        self,
        session: GimbalSession,
        pitch_deg: float,
        yaw_deg: float,
        roll_deg: float = 0.0,
    ) -> None:  # pragma: no cover
        raise NotImplementedError

    async def command_rate(
        self,
        session: GimbalSession,
        pitch_rate_dps: float,
        yaw_rate_dps: float,
        roll_rate_dps: float = 0.0,
    ) -> None:  # pragma: no cover
        raise NotImplementedError

    def get_state(self, session: GimbalSession) -> GimbalState:  # pragma: no cover
        raise NotImplementedError

    async def state_iterator(
        self, session: GimbalSession
    ) -> AsyncIterator[GimbalState]:  # pragma: no cover
        raise NotImplementedError
