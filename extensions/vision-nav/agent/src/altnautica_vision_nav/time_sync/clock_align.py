"""Bidirectional TIMESYNC handler.

The plugin and the flight controller swap TIMESYNC messages once a
second. The exchange follows the standard MAVLink protocol:

* The plugin emits ``TIMESYNC(tc1=0, ts1=monotonic_ns)``.
* The FC responds with ``TIMESYNC(tc1=fc_time_ns, ts1=plugin_ts1)``.
* The plugin computes ``offset = tc1 - ts1`` and folds it into an EMA.

The estimated offset converts the plugin's monotonic clock into FC time
space so the OPTICAL_FLOW / OPTICAL_FLOW_RAD / ODOMETRY ``time_usec``
fields line up with the FC's notion of "now". A sudden drift larger
than ``DRIFT_BREACH_NS`` bumps a reset counter that downstream pose
emitters propagate into the ``reset_counter`` field of ODOMETRY /
VISION_POSITION_ESTIMATE so the FC's estimator knows the alignment
discontinuity is real.
"""

from __future__ import annotations

import asyncio
import itertools
import logging
import random
import struct
import time
from collections.abc import Awaitable, Callable
from typing import Final

from ..mavlink._framing import pack_v2

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Message id + CRC_EXTRA
# ---------------------------------------------------------------------------

MSG_ID_TIMESYNC: Final[int] = 111
CRC_TIMESYNC: Final[int] = 34

DEFAULT_SYS_ID: Final[int] = 1
TICK_INTERVAL_S: Final[float] = 1.0

# Drift breach threshold: 50 ms in nanoseconds. A larger gap between
# consecutive estimates is treated as a real discontinuity (clock step
# on the FC, link disruption, scheduler stall), not noise.
DRIFT_BREACH_NS: Final[int] = 50_000_000

# EMA smoothing factor applied after the first sample. The first sample
# adopts the raw estimate verbatim; subsequent samples blend at 0.1.
EMA_ALPHA: Final[float] = 0.1

# How long an outgoing ts1 stays "fresh" enough to accept the matching
# response. A 5 second window is comfortably wider than the 1 Hz tick
# and protects against a stale loopback echo from earlier rounds.
TS1_FRESH_WINDOW_NS: Final[int] = 5_000_000_000

# Cap on the in-flight ts1 set. Each entry is just an int and they
# expire after the fresh window, so the bound is mostly a defensive
# guard against pathological churn.
MAX_INFLIGHT_TS1: Final[int] = 64


_PACK_TIMESYNC: Final[struct.Struct] = struct.Struct("<qq")


def build_timesync(
    *,
    sys_id: int,
    comp_id: int,
    seq: int,
    tc1: int,
    ts1: int,
) -> bytes:
    """Pack a TIMESYNC (#111) v2 frame."""
    payload = _PACK_TIMESYNC.pack(int(tc1), int(ts1))
    return pack_v2(
        msg_id=MSG_ID_TIMESYNC,
        crc_extra=CRC_TIMESYNC,
        payload=payload,
        sys_id=sys_id,
        comp_id=comp_id,
        seq=seq,
    )


class ClockAlign:
    """Maintain a running estimate of ``fc_ns - monotonic_ns``.

    Use :meth:`convert_to_fc_clock` to translate a plugin-side
    ``time.monotonic_ns()`` reading into FC clock space. The estimate
    is zero before the first response lands, which means timestamps
    fall back to raw monotonic during boot. That is acceptable: the
    optical-flow emitter throttles itself behind the first ACTIVE
    HEARTBEAT, so the first ``time_usec`` it sends has already passed
    one or more TIMESYNC rounds.
    """

    def __init__(
        self,
        *,
        component_id: int,
        sys_id: int = DEFAULT_SYS_ID,
    ) -> None:
        self._component_id = int(component_id)
        self._sys_id = int(sys_id)
        self.offset_ns: int = 0
        self.last_sample_ns: int = 0
        self.reset_counter: int = 0
        self._has_sample: bool = False
        self._inflight: dict[int, int] = {}
        self._ctx: "object | None" = None
        self._tick_task: asyncio.Task[None] | None = None
        self._seq_it = itertools.count()

    # ----- public api -------------------------------------------------

    def convert_to_fc_clock(self, monotonic_ns: int) -> int:
        """Return the FC-clock timestamp for a monotonic reading."""
        return int(monotonic_ns) + int(self.offset_ns)

    async def start(self, ctx: "object") -> None:
        """Subscribe to TIMESYNC and begin emitting 1 Hz queries."""
        if self._tick_task is not None and not self._tick_task.done():
            return
        self._ctx = ctx
        mavlink = getattr(ctx, "mavlink", None)
        if mavlink is not None:
            subscribe = getattr(mavlink, "subscribe", None)
            if subscribe is not None:
                try:
                    result = subscribe("TIMESYNC", self._on_timesync)
                    if asyncio.iscoroutine(result):
                        await result
                except Exception as exc:  # noqa: BLE001
                    log.warning("TIMESYNC subscribe failed: %s", exc)
        self._tick_task = asyncio.create_task(self._tick_loop())

    async def stop(self) -> None:
        """Cancel the emit loop. The host releases the subscription."""
        task = self._tick_task
        self._tick_task = None
        if task is not None:
            task.cancel()
            try:
                await task
            except (asyncio.CancelledError, Exception):  # noqa: BLE001
                pass
        self._inflight.clear()
        self._ctx = None

    # ----- internals --------------------------------------------------

    async def _tick_loop(self) -> None:
        try:
            while True:
                await self._emit_outgoing()
                await asyncio.sleep(TICK_INTERVAL_S)
        except asyncio.CancelledError:
            return
        except Exception as exc:  # noqa: BLE001
            log.warning("TIMESYNC tick loop failed: %s", exc)

    async def _emit_outgoing(self) -> None:
        ctx = self._ctx
        if ctx is None:
            return
        ts1 = self._fresh_ts1()
        # Track this ts1 so the matching response is accepted.
        self._inflight[ts1] = ts1
        self._gc_inflight(ts1)
        frame = build_timesync(
            sys_id=self._sys_id,
            comp_id=self._component_id,
            seq=next(self._seq_it) & 0xFF,
            tc1=0,
            ts1=ts1,
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
            log.warning("TIMESYNC emit failed: %s", exc)

    async def _on_timesync(self, message: dict) -> None:
        """Handle an incoming TIMESYNC payload from the host.

        The host delivers MAVLink subscriptions as a dict with at least
        ``msg_name`` and a payload frame. Some host configurations
        forward the decoded fields directly; we accept both shapes.
        """
        tc1, ts1 = _extract_tc1_ts1(message)
        if tc1 is None or ts1 is None:
            return
        if tc1 == 0:
            # Echo of our own outgoing query, or an FC also asking the
            # network for time. Either way, not a response we can act on.
            return
        if ts1 not in self._inflight:
            return
        self._inflight.pop(ts1, None)
        self.handle_timesync_response(tc1=tc1, ts1=ts1)

    def handle_timesync_response(self, *, tc1: int, ts1: int) -> None:
        """Fold a TIMESYNC response into the offset estimate.

        Exposed publicly so unit tests can drive the state machine
        without going through the IPC bridge. Bumps ``reset_counter``
        when the new estimate diverges from the running offset by more
        than :data:`DRIFT_BREACH_NS`.
        """
        estimate = int(tc1) - int(ts1)
        now = time.monotonic_ns()
        if not self._has_sample:
            self.offset_ns = estimate
            self._has_sample = True
        else:
            drift = abs(estimate - self.offset_ns)
            if drift > DRIFT_BREACH_NS:
                self.reset_counter = (self.reset_counter + 1) & 0xFFFFFFFF
            # EMA: offset = (1 - alpha) * offset + alpha * estimate.
            self.offset_ns = int(
                round((1.0 - EMA_ALPHA) * self.offset_ns + EMA_ALPHA * estimate)
            )
        self.last_sample_ns = now

    def _fresh_ts1(self) -> int:
        """Return a monotonic-ns ts1 that has not been queued before.

        Collisions are vanishingly rare at 1 Hz but cheap to guard
        against. A small random nudge breaks any tie with a previous
        emission that landed in the same nanosecond bucket.
        """
        ts1 = time.monotonic_ns()
        while ts1 in self._inflight:
            ts1 = time.monotonic_ns() + random.randint(1, 4096)
        return ts1

    def _gc_inflight(self, now_ns: int) -> None:
        """Drop ts1 entries older than the fresh window."""
        if len(self._inflight) <= MAX_INFLIGHT_TS1:
            stale = [
                key
                for key in self._inflight
                if now_ns - key > TS1_FRESH_WINDOW_NS
            ]
        else:
            # Over the cap: also drop oldest until we are back in budget.
            stale = sorted(self._inflight.keys())[
                : len(self._inflight) - MAX_INFLIGHT_TS1
            ]
        for key in stale:
            self._inflight.pop(key, None)


def _extract_tc1_ts1(message: dict) -> tuple[int | None, int | None]:
    """Pull ``tc1`` and ``ts1`` out of a host TIMESYNC delivery.

    The host forwards either a decoded payload dict (preferred) or a
    raw frame the plugin parses itself. Both paths land here.
    """
    if not isinstance(message, dict):
        return None, None

    # Decoded shape: dict has tc1 + ts1 either at the top level or
    # nested under a "fields" or "payload" key.
    for source in (message, message.get("fields"), message.get("payload")):
        if isinstance(source, dict):
            tc1 = source.get("tc1")
            ts1 = source.get("ts1")
            if isinstance(tc1, int) and isinstance(ts1, int):
                return tc1, ts1

    # Raw-frame shape: parse the v2 payload bytes.
    frame = message.get("frame")
    if isinstance(frame, (bytes, bytearray)) and len(frame) >= 12:
        payload = _payload_from_v2_frame(bytes(frame))
        if payload is not None and len(payload) >= _PACK_TIMESYNC.size:
            tc1, ts1 = _PACK_TIMESYNC.unpack_from(payload, 0)
            return int(tc1), int(ts1)
        if payload is not None and 0 < len(payload) < _PACK_TIMESYNC.size:
            # Empty-byte truncation; pad and re-decode.
            padded = payload + b"\x00" * (_PACK_TIMESYNC.size - len(payload))
            tc1, ts1 = _PACK_TIMESYNC.unpack_from(padded, 0)
            return int(tc1), int(ts1)
    return None, None


def _payload_from_v2_frame(frame: bytes) -> bytes | None:
    """Best-effort v2 frame payload extraction (without CRC validation).

    Used as a fallback when the host forwards undecoded TIMESYNC frames.
    The host's normal path is to decode and forward fields, so this is
    rarely hit; we keep it for parity with hosts that route raw bytes.
    """
    if not frame or frame[0] != 0xFD:
        return None
    if len(frame) < 12:
        return None
    payload_len = frame[1]
    payload_end = 10 + payload_len
    if len(frame) < payload_end + 2:
        return None
    return frame[10:payload_end]


# Type alias for the callback signature, exposed for tests that want
# to feed handcrafted dicts through the on-message path.
TimesyncCallback = Callable[[dict], Awaitable[None] | None]
