"""SimpleBGC native protocol driver (stub).

Concrete subclass of :class:`GimbalDriver` so the supervisor can
register it and so vendors forking the extension see the pattern. The
SBGC native protocol speaks ``0x3E <cmd> <length> <payload> <crc>``
frames over UART. A real implementation translates ``SBGC_CMD_CONTROL``
(0x43) and ``SBGC_CMD_REALTIME_DATA_3`` (0x17) frames against the
gimbal driver surface.

Until the parser lands, ``open()`` raises :class:`NotImplementedError`
so attempts to use this transport surface as configuration errors
rather than silent stalls.
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


class SimpleBgcGimbalDriver(GimbalDriver):
    """SBGC32 native UART driver (work in progress)."""

    driver_id = "simplebgc-uart"

    async def discover(self) -> list[GimbalCandidate]:
        """Return an empty candidate list until the bridge is wired.

        Returning ``[]`` keeps the supervisor's discovery loop quiet.
        Once the parser lands we will scan ``/dev/serial/by-id`` for
        SBGC vendor strings and report each match here.
        """

        return []

    async def open(
        self, candidate: GimbalCandidate, config: dict[str, Any]
    ) -> GimbalSession:
        raise NotImplementedError(
            "SimpleBGC bridge is not implemented yet. Use the MAVLink "
            "driver against an SBGC running the MAVLink-bridge firmware, "
            "or wait for the next release."
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
