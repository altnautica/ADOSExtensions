"""Gremsy direct-serial driver (stub).

Gremsy gimbals (T3, T7, S1) ship MAVLink Gimbal v2 in their latest
firmware, and the recommended path is the MAVLink driver. This serial
driver is reserved for installs that route the gimbal through the
companion computer's UART rather than the FC bus.

The class is concrete so the supervisor can list it; ``open()`` raises
``NotImplementedError`` until a parser lands.
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


class GremsyGimbalDriver(GimbalDriver):
    """Gremsy direct-serial driver (work in progress)."""

    driver_id = "gremsy-uart"

    async def discover(self) -> list[GimbalCandidate]:
        return []

    async def open(
        self, candidate: GimbalCandidate, config: dict[str, Any]
    ) -> GimbalSession:
        raise NotImplementedError(
            "Gremsy direct-serial driver is not implemented yet. "
            "Connect the gimbal to the FC's MAVLink network and use "
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
