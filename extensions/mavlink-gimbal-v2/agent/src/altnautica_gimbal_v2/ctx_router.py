"""Adapter that lets the MAVLink driver reach ``ctx.mavlink.send``.

The ``MavlinkGimbalDriver`` was written against a ``RouterHandle`` that
exposes a synchronous ``send_command(cmd) -> bool``, where ``cmd`` is a
``CommandLong`` / ``CommandInt`` value it produced. The agent's plugin
runtime no longer hands a hand-injected router into the plugin; instead
it exposes ``ctx.mavlink.send(frame_bytes, component_id=...)``, which
takes a fully-packed MAVLink v2 frame.

``_CtxRouter`` bridges the two: it satisfies the driver's synchronous
``send_command`` surface, packs the ``CommandLong`` / ``CommandInt`` into
a real wire frame with pymavlink (so the message layout and CRC-extra are
exactly right -- never hand-rolled), and fire-and-forgets the frame onto
``ctx.mavlink.send`` via the running event loop. The driver does not block
on an ACK; the gimbal-manager state machine handles re-emit on timeout, so
a scheduled send that fails is logged, never raised back into the driver.

The frame is sent under the vehicle's own system id with the onboard
computer component id (191), matching the Follow-Me convention, so the
autopilot / gimbal manager treats the command as an authorized on-board
command on the same vehicle.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Any

from pymavlink.dialects.v20 import common as mavlink2

from altnautica_gimbal_v2.mavlink_messages import CommandInt, CommandLong

log = logging.getLogger(__name__)

# Component id the companion sends as. MAV_COMP_ID_ONBOARD_COMPUTER (191):
# the autopilot treats a command from the onboard computer on the vehicle's
# own system id as an authorized command. Matches the Follow-Me plugin.
ONBOARD_COMPUTER_COMP_ID = 191


class _CtxRouter:
    """A ``RouterHandle``-compatible bridge onto ``ctx.mavlink.send``.

    Built inside ``on_start`` where a running event loop exists; the loop
    is captured then so ``send_command`` (called synchronously from the
    driver's async command methods, i.e. always on the loop thread) can
    schedule the send onto it.
    """

    def __init__(
        self,
        *,
        ctx_mavlink: Any,
        src_system: int,
        loop: asyncio.AbstractEventLoop,
        src_component: int = ONBOARD_COMPUTER_COMP_ID,
    ) -> None:
        self._ctx_mavlink = ctx_mavlink
        self._src_component = int(src_component)
        self._loop = loop
        # A stateless encoder bound to the source (system, component). No
        # file object is attached; it is used only to pack messages into
        # bytes. The host router owns the outbound link sequence.
        self._mav = mavlink2.MAVLink(
            None, srcSystem=int(src_system), srcComponent=int(src_component)
        )
        # Strong references to in-flight send tasks. asyncio only holds a
        # weak reference to a task, so without this the fire-and-forget
        # send could be garbage collected mid-await; the done-callback
        # discards each task when it finishes.
        self._pending: set[asyncio.Task[Any]] = set()

    def send_command(self, cmd: CommandLong | CommandInt) -> bool:
        """Pack ``cmd`` to a wire frame and fire it onto ``ctx.mavlink.send``.

        Synchronous and non-raising: any failure to pack or schedule the
        send is logged and reported as ``False`` so the driver never sees
        an exception from its send path.
        """

        try:
            if isinstance(cmd, CommandLong):
                msg = self._mav.command_long_encode(
                    cmd.target_system,
                    cmd.target_component,
                    cmd.command,
                    0,  # confirmation
                    cmd.param1,
                    cmd.param2,
                    cmd.param3,
                    cmd.param4,
                    cmd.param5,
                    cmd.param6,
                    cmd.param7,
                )
            elif isinstance(cmd, CommandInt):
                msg = self._mav.command_int_encode(
                    cmd.target_system,
                    cmd.target_component,
                    cmd.frame,
                    cmd.command,
                    0,  # current
                    0,  # autocontinue
                    cmd.param1,
                    cmd.param2,
                    cmd.param3,
                    cmd.param4,
                    cmd.x,
                    cmd.y,
                    cmd.z,
                )
            else:
                log.warning(
                    "gimbal ctx-router received an unsupported command type: %s",
                    type(cmd).__name__,
                )
                return False
            frame = msg.pack(self._mav)
        except Exception:  # noqa: BLE001
            log.warning("gimbal ctx-router failed to pack command", exc_info=True)
            return False

        loop = self._loop
        if loop is None:  # pragma: no cover - guarded by construction
            log.warning("gimbal ctx-router has no event loop; dropping frame")
            return False

        try:
            task = loop.create_task(
                self._ctx_mavlink.send(frame, component_id=self._src_component)
            )
            self._pending.add(task)
            task.add_done_callback(self._on_send_done)
        except Exception:  # noqa: BLE001
            log.warning("gimbal ctx-router failed to schedule send", exc_info=True)
            return False
        return True

    def _on_send_done(self, task: asyncio.Task[Any]) -> None:
        """Drop the completed task's strong reference and surface (not
        swallow-silently) any send failure. Retrieving the exception here
        also prevents asyncio's 'task exception was never retrieved'
        warning for a fire-and-forget send."""
        self._pending.discard(task)
        if task.cancelled():
            return
        exc = task.exception()
        if exc is not None:
            log.warning("gimbal ctx-router send failed: %s", exc)


__all__ = ["ONBOARD_COMPUTER_COMP_ID", "_CtxRouter"]
