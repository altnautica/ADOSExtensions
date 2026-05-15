"""Comp-198 HEARTBEAT state machine.

The plugin owns a peripheral MAVLink component (id 198, sub-id 1) that
publishes a 1 Hz HEARTBEAT so the flight stack and Mission Control can
see the optical-flow companion node on the bus. The ``system_status``
field maps to the plugin's runtime state, so a degraded or terminating
companion is visible immediately without inspecting message rates.

HEARTBEAT wire layout (MAVLink common.xml, message #0):

    custom_mode      uint32
    type             uint8
    autopilot        uint8
    base_mode        uint8
    system_status    uint8
    mavlink_version  uint8

CRC_EXTRA = 50 (dialect fingerprint, identical across common dialects).
"""

from __future__ import annotations

import asyncio
import enum
import itertools
import logging
import struct
from typing import Final

from ._framing import pack_v2

log = logging.getLogger(__name__)


MSG_ID_HEARTBEAT: Final[int] = 0
CRC_HEARTBEAT: Final[int] = 50

# MAVLink type / autopilot / status enums used here. Values match
# common.xml. Kept inline so the plugin does not import the host's
# enum module.
MAV_TYPE_GENERIC: Final[int] = 0
MAV_AUTOPILOT_INVALID: Final[int] = 8
MAVLINK_VERSION: Final[int] = 3

MAV_STATE_STANDBY: Final[int] = 3
MAV_STATE_ACTIVE: Final[int] = 4
MAV_STATE_CRITICAL: Final[int] = 6
MAV_STATE_FLIGHT_TERMINATION: Final[int] = 8

DEFAULT_SYS_ID: Final[int] = 1
TICK_INTERVAL_S: Final[float] = 1.0

_PACK_HEARTBEAT: Final[struct.Struct] = struct.Struct("<IBBBBB")


class CompanionState(enum.Enum):
    """High-level state of the optical-flow companion node."""

    INACTIVE = "inactive"
    ACTIVE = "active"
    CRITICAL = "critical"
    TERMINATING = "terminating"


_STATE_TO_SYSTEM_STATUS: Final[dict[CompanionState, int]] = {
    CompanionState.INACTIVE: MAV_STATE_STANDBY,
    CompanionState.ACTIVE: MAV_STATE_ACTIVE,
    CompanionState.CRITICAL: MAV_STATE_CRITICAL,
    CompanionState.TERMINATING: MAV_STATE_FLIGHT_TERMINATION,
}


def build_heartbeat(
    *,
    sys_id: int,
    comp_id: int,
    seq: int,
    system_status: int,
    type_: int = MAV_TYPE_GENERIC,
    autopilot: int = MAV_AUTOPILOT_INVALID,
    base_mode: int = 0,
    custom_mode: int = 0,
    mavlink_version: int = MAVLINK_VERSION,
) -> bytes:
    """Pack a HEARTBEAT (#0) v2 frame."""
    payload = _PACK_HEARTBEAT.pack(
        int(custom_mode) & 0xFFFFFFFF,
        int(type_) & 0xFF,
        int(autopilot) & 0xFF,
        int(base_mode) & 0xFF,
        int(system_status) & 0xFF,
        int(mavlink_version) & 0xFF,
    )
    return pack_v2(
        msg_id=MSG_ID_HEARTBEAT,
        crc_extra=CRC_HEARTBEAT,
        payload=payload,
        sys_id=sys_id,
        comp_id=comp_id,
        seq=seq,
    )


class CompanionHeartbeat:
    """1 Hz HEARTBEAT emitter with a state-driven ``system_status``.

    The state transitions are explicit calls; the tick task picks up the
    current state each iteration so transitions surface on the next
    HEARTBEAT (at most ``TICK_INTERVAL_S`` later). Calling ``stop()``
    emits a final HEARTBEAT in :class:`CompanionState.TERMINATING` so
    receivers see the shutdown rather than a stale ACTIVE state.
    """

    def __init__(
        self,
        *,
        component_id: int,
        sys_id: int = DEFAULT_SYS_ID,
    ) -> None:
        self._component_id = int(component_id)
        self._sys_id = int(sys_id)
        self._state: CompanionState = CompanionState.INACTIVE
        self._task: asyncio.Task[None] | None = None
        self._ctx: "object | None" = None
        self._seq_it = itertools.count()

    @property
    def state(self) -> CompanionState:
        return self._state

    @property
    def component_id(self) -> int:
        return self._component_id

    def transition(self, new_state: CompanionState) -> None:
        """Move to a new companion state and log the change.

        The next HEARTBEAT tick picks up the new ``system_status``;
        callers do not need to wait for the tick to complete.
        """
        if new_state is self._state:
            return
        old_state = self._state
        self._state = new_state
        self._log_transition(old_state, new_state)

    async def start(self, ctx: "object") -> None:
        """Begin the 1 Hz HEARTBEAT tick. Idempotent."""
        if self._task is not None and not self._task.done():
            return
        self._ctx = ctx
        self._task = asyncio.create_task(self._tick_loop())

    async def stop(self) -> None:
        """Cancel the tick and emit a final TERMINATING HEARTBEAT."""
        prior_state = self._state
        self._state = CompanionState.TERMINATING
        if prior_state is not CompanionState.TERMINATING:
            self._log_transition(prior_state, CompanionState.TERMINATING)
        await self._emit_once()
        task = self._task
        self._task = None
        if task is not None:
            task.cancel()
            try:
                await task
            except (asyncio.CancelledError, Exception):  # noqa: BLE001
                pass

    async def _tick_loop(self) -> None:
        try:
            while True:
                await self._emit_once()
                await asyncio.sleep(TICK_INTERVAL_S)
        except asyncio.CancelledError:
            return
        except Exception as exc:  # noqa: BLE001
            log.warning("heartbeat tick loop failed: %s", exc)

    async def _emit_once(self) -> None:
        ctx = self._ctx
        if ctx is None:
            return
        frame = build_heartbeat(
            sys_id=self._sys_id,
            comp_id=self._component_id,
            seq=next(self._seq_it) & 0xFF,
            system_status=_STATE_TO_SYSTEM_STATUS[self._state],
        )
        mavlink = getattr(ctx, "mavlink", None)
        if mavlink is None:
            return
        send = getattr(mavlink, "send", None)
        if send is None:
            return
        try:
            result = send(frame, component_id=self._component_id)
            if asyncio.iscoroutine(result):
                await result
        except Exception as exc:  # noqa: BLE001
            log.warning("heartbeat emit failed: %s", exc)

    def _log_transition(
        self, old_state: CompanionState, new_state: CompanionState
    ) -> None:
        ctx = self._ctx
        message = "companion heartbeat state: %s -> %s"
        if ctx is not None:
            plugin_log = getattr(ctx, "log", None)
            if plugin_log is not None:
                info = getattr(plugin_log, "info", None)
                if callable(info):
                    try:
                        info(
                            "companion_heartbeat_transition",
                            old=old_state.value,
                            new=new_state.value,
                        )
                        return
                    except TypeError:
                        # Plain stdlib logger does not accept kwargs;
                        # fall through to a positional call.
                        try:
                            info(message, old_state.value, new_state.value)
                            return
                        except Exception:  # noqa: BLE001
                            pass
                    except Exception:  # noqa: BLE001
                        pass
        log.info(message, old_state.value, new_state.value)
