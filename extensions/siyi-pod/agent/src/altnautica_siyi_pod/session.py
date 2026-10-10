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
* a liveness signal (the count of frames received) lets a watchdog tell a
  live-but-silent link from a working one;
* a supervisor keeps the link up: when the transport fails to open, or reports
  that its read side ended (TCP EOF, a serial device that went away), the
  session closes it, waits a fixed interval, and reopens it, with no attempt
  cap. While the link is down every request raises :class:`SiyiTimeout`, and
  each reopen after the first notifies the reconnect listeners so the pod is
  re-identified.

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
# Fixed wait between a lost or failed link and the next open attempt.
_RECONNECT_S = 2.0


class SiyiTimeout(TimeoutError):
    """A request went unanswered within the timeout and retry budget, or the
    link to the pod is down."""


class SiyiSession:
    """Owns one transport; serializes commands and demuxes replies."""

    def __init__(
        self,
        transport,
        *,
        timeout_s: float = _DEFAULT_TIMEOUT_S,
        retries: int = _DEFAULT_RETRIES,
        reconnect_s: float = _RECONNECT_S,
    ) -> None:
        self._transport = transport
        self._timeout_s = timeout_s
        self._retries = retries
        self._reconnect_s = reconnect_s
        self._buffer = b""
        self._seq = 0
        self._pending: dict[int, asyncio.Future[Frame]] = {}
        self._subscribers: list[tuple[int | None, PushHandler]] = []
        self._reconnect_listeners: list[Callable[[], None]] = []
        self._send_lock = asyncio.Lock()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._running = False
        self._up = False
        self._lost = asyncio.Event()
        self._supervisor: asyncio.Task | None = None
        # Number of successful transport opens over the session's life.
        self.connects = 0
        # Liveness: total frames received. A watchdog compares this across a
        # rolling window to catch a transport that is up but has gone silent —
        # an open socket is not evidence that the pod is answering.
        self.frames_received = 0

    # -- lifecycle --------------------------------------------------------
    async def start(self) -> None:
        """Open the link and start supervising it.

        Never raises on an absent pod: a failed first open is logged and the
        supervisor keeps retrying.
        """
        self._loop = asyncio.get_running_loop()
        self._transport.set_on_bytes(self._on_bytes)
        self._transport.set_on_lost(self._on_transport_lost)
        self._running = True
        await self._try_open()
        self._supervisor = self._loop.create_task(self._supervise())

    async def stop(self) -> None:
        self._running = False
        if self._supervisor is not None:
            self._supervisor.cancel()
            try:
                await self._supervisor
            except asyncio.CancelledError:
                pass
            self._supervisor = None
        self._mark_down()
        await self._close_transport()

    @property
    def link_up(self) -> bool:
        """Whether the transport is currently open."""
        return self._up

    def add_reconnect_listener(self, callback: Callable[[], None]) -> None:
        """Call ``callback`` each time the link comes back after being lost."""
        self._reconnect_listeners.append(callback)

    # -- supervision ------------------------------------------------------
    async def _try_open(self) -> bool:
        self._lost = asyncio.Event()
        try:
            await self._transport.open()
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001
            log.warning("siyi transport open failed: %s", exc)
            await self._close_transport()
            return False
        self._buffer = b""
        self._up = True
        self.connects += 1
        if self.connects > 1:
            log.info("siyi transport reconnected")
            for callback in list(self._reconnect_listeners):
                try:
                    callback()
                except Exception:  # noqa: BLE001
                    log.warning("siyi reconnect listener failed", exc_info=True)
        return True

    async def _supervise(self) -> None:
        while self._running:
            if self._up:
                await self._lost.wait()
                log.warning("siyi transport lost; reopening")
                self._mark_down()
                await self._close_transport()
            await asyncio.sleep(self._reconnect_s)
            if self._running:
                await self._try_open()

    def _on_transport_lost(self) -> None:
        """Called by the transport when its read side has ended."""
        self._lost.set()

    def _mark_down(self) -> None:
        self._up = False
        for fut in self._pending.values():
            if not fut.done():
                fut.set_exception(SiyiTimeout("SIYI link went down"))
        self._pending.clear()

    async def _close_transport(self) -> None:
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

    def _require_up(self) -> None:
        if not self._up:
            raise SiyiTimeout("SIYI link is down")

    async def request(self, command: Command) -> Frame:
        """Send a command and await the pod's correlated reply."""
        self._require_up()
        async with self._send_lock:
            last_exc: Exception | None = None
            for _attempt in range(self._retries + 1):
                self._require_up()
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
        self._require_up()
        async with self._send_lock:
            seq = self._next_seq()
            frame = build_frame(
                command.cmd_id, command.data, seq=seq, need_ack=command.need_ack
            )
            await self._transport.send(frame)
