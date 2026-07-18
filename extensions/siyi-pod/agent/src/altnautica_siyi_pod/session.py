"""The single owner of the SIYI transport session.

A SIYI optical pod is one device that a smart-camera plugin drives from several
places at once: a gimbal loop, a telemetry poller, a laser/geolocation path, an
AI-track republisher, and the operator's config-driven control loop. They must
not each poke the socket. :class:`SiyiSession` owns the one transport and is the
only writer:

* every request goes out under a lock (a serialized command queue) with a fresh
  sequence number, and its reply is correlated back by that sequence number with
  a bounded timeout and re-emit;
* unsolicited frames the pod pushes (streamed attitude, a laser result, an
  AI-track box) fan out to any number of subscribers by command id;
* a Rule-37-style liveness signal (the count of frames received) lets a watchdog
  tell a live-but-silent link from a working one.

This is the composite one-session / many-consumers pattern: consumers hold a
handle to the session, never to the socket.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Awaitable, Callable

from altnautica_siyi_pod.commands import Command
from altnautica_siyi_pod.framing import Frame, build_frame, iter_frames

log = logging.getLogger(__name__)

PushHandler = Callable[[Frame], None] | Callable[[Frame], Awaitable[None]]

_DEFAULT_TIMEOUT_S = 0.5
_DEFAULT_RETRIES = 2


class SiyiTimeout(TimeoutError):
    """A request went unanswered within the timeout and retry budget."""


class SiyiSession:
    """Owns one transport; serializes commands and demuxes replies."""

    def __init__(
        self,
        transport,
        *,
        timeout_s: float = _DEFAULT_TIMEOUT_S,
        retries: int = _DEFAULT_RETRIES,
    ) -> None:
        self._transport = transport
        self._timeout_s = timeout_s
        self._retries = retries
        self._buffer = b""
        self._seq = 0
        self._pending: dict[int, asyncio.Future[Frame]] = {}
        self._subscribers: list[tuple[int | None, PushHandler]] = []
        self._send_lock = asyncio.Lock()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._open = False
        # Liveness: total frames received. A watchdog compares this across a
        # window to catch a live-but-silent link (Rule 37).
        self.frames_received = 0

    # -- lifecycle --------------------------------------------------------
    async def start(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._transport.set_on_bytes(self._on_bytes)
        await self._transport.open()
        self._open = True

    async def stop(self) -> None:
        self._open = False
        for fut in self._pending.values():
            if not fut.done():
                fut.cancel()
        self._pending.clear()
        try:
            await self._transport.close()
        except Exception:  # noqa: BLE001
            log.warning("siyi transport close failed", exc_info=True)

    # -- inbound ----------------------------------------------------------
    def _on_bytes(self, chunk: bytes) -> None:
        self._buffer += chunk
        frames, self._buffer = iter_frames(self._buffer)
        for frame in frames:
            self.frames_received += 1
            fut = self._pending.pop(frame.seq, None)
            if fut is not None and not fut.done():
                fut.set_result(frame)
                # A correlated reply is not also fanned out as a push.
                continue
            self._fanout(frame)

    def _fanout(self, frame: Frame) -> None:
        for want_cmd, handler in list(self._subscribers):
            if want_cmd is not None and want_cmd != frame.cmd_id:
                continue
            try:
                result = handler(frame)
                if asyncio.iscoroutine(result) and self._loop is not None:
                    self._loop.create_task(result)
            except Exception:  # noqa: BLE001
                log.warning("siyi push handler failed", exc_info=True)

    # -- subscriptions ----------------------------------------------------
    def subscribe(
        self, handler: PushHandler, *, cmd_id: int | None = None
    ) -> Callable[[], None]:
        """Register a handler for unsolicited frames (optionally one cmd id)."""
        entry = (cmd_id, handler)
        self._subscribers.append(entry)

        def _unsub() -> None:
            try:
                self._subscribers.remove(entry)
            except ValueError:
                pass

        return _unsub

    # -- outbound ---------------------------------------------------------
    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFFFF
        if self._seq == 0:
            self._seq = 1
        return self._seq

    async def request(self, command: Command) -> Frame:
        """Send a command and await the pod's correlated reply."""
        if not self._open:
            raise RuntimeError("session is not open")
        async with self._send_lock:
            last_exc: Exception | None = None
            for _attempt in range(self._retries + 1):
                seq = self._next_seq()
                fut: asyncio.Future[Frame] = (
                    self._loop.create_future()
                    if self._loop is not None
                    else asyncio.get_running_loop().create_future()
                )
                self._pending[seq] = fut
                frame = build_frame(
                    command.cmd_id, command.data, seq=seq, need_ack=True
                )
                try:
                    await self._transport.send(frame)
                    return await asyncio.wait_for(fut, timeout=self._timeout_s)
                except (asyncio.TimeoutError, TimeoutError) as exc:
                    last_exc = exc
                    self._pending.pop(seq, None)
                    continue
            raise SiyiTimeout(
                f"no reply to cmd {command.cmd_id:#04x} after "
                f"{self._retries + 1} attempts"
            ) from last_exc

    async def command(self, command: Command) -> None:
        """Fire-and-forget a command (no reply awaited)."""
        if not self._open:
            raise RuntimeError("session is not open")
        async with self._send_lock:
            seq = self._next_seq()
            frame = build_frame(
                command.cmd_id, command.data, seq=seq, need_ack=command.need_ack
            )
            await self._transport.send(frame)
