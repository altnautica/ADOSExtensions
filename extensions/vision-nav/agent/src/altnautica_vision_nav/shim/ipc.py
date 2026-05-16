"""IPC primitives for the VIO shim.

Two abstractions live here:

* :class:`ShmFrameRing` — a ring of fixed-size slots backed by
  ``multiprocessing.shared_memory`` for camera frames. The Python
  side writes frames into a slot and notifies the vendor binary
  through the control channel; the binary mmaps the same name and
  reads zero-copy.
* :class:`MsgpackChannel` — a length-prefixed msgpack codec over a
  ``socket.socket``. Supports both ``SOCK_SEQPACKET`` (preferred,
  preserves message boundaries) and ``SOCK_STREAM`` (universal
  fallback with a 4-byte big-endian length prefix).

Both classes accept an injected transport so tests can drive them
without spawning real processes or touching /dev/shm.
"""

from __future__ import annotations

import io
import socket
import struct
import threading
from dataclasses import dataclass
from typing import Optional

import msgpack

LENGTH_PREFIX_FMT = ">I"
LENGTH_PREFIX_BYTES = 4
MAX_MESSAGE_BYTES = 4 * 1024 * 1024  # 4 MB hard cap; matches plugin RPC

# Frame format tags shared with the C++ side. Adding a value here must
# be matched by an update to the vendor wrapper.
FRAME_FORMAT_GRAY8 = 0
FRAME_FORMAT_NV12 = 1
FRAME_FORMAT_RGB8 = 2


@dataclass(frozen=True)
class FrameSlot:
    """Pointer + metadata for one frame in the SHM ring.

    The Python side fills the slot and then sends a ``frame_ready``
    control message carrying ``ts_us``, ``seq``, and ``slot_index``.
    The vendor binary mmaps the SHM region, reads the slot header,
    and consumes ``pixel_bytes`` zero-copy.
    """

    slot_index: int
    ts_us: int
    seq: int
    width: int
    height: int
    stride: int
    pixel_format: int


class ShmFrameRing:
    """Fixed-capacity SHM ring buffer for camera frames.

    The ring has ``slot_count`` slots, each ``slot_size`` bytes. The
    writer (Python) advances ``write_idx`` after copying pixels into a
    slot; the reader (vendor) advances ``read_idx`` after consuming.
    Both indices are modulo ``slot_count``.

    The class supports a ``buffer`` injection so tests can use a
    ``bytearray`` instead of a real shared-memory region. In
    production the buffer comes from
    ``multiprocessing.shared_memory.SharedMemory(create=True, size=...)``.
    """

    def __init__(
        self,
        *,
        slot_count: int = 4,
        slot_size: int = 1280 * 800 * 3,
        buffer: Optional[bytearray] = None,
    ) -> None:
        if slot_count < 2:
            raise ValueError("slot_count must be >= 2")
        self._slot_count = int(slot_count)
        self._slot_size = int(slot_size)
        if buffer is None:
            self._buffer: bytearray = bytearray(self._slot_count * self._slot_size)
        else:
            if len(buffer) < self._slot_count * self._slot_size:
                raise ValueError("buffer too small for slot_count * slot_size")
            self._buffer = buffer
        self._lock = threading.Lock()
        self._write_idx: int = 0
        self._seq: int = 0

    @property
    def slot_count(self) -> int:
        return self._slot_count

    @property
    def slot_size(self) -> int:
        return self._slot_size

    def write(
        self,
        *,
        ts_us: int,
        width: int,
        height: int,
        stride: int,
        pixel_format: int,
        pixels: bytes,
    ) -> FrameSlot:
        """Copy ``pixels`` into the next slot and return its descriptor.

        Raises ``ValueError`` when the frame exceeds ``slot_size``. The
        ring overwrites the oldest slot on wrap; readers that fall
        behind by more than ``slot_count`` frames will see the loss in
        the seq gap and can re-sync on the next ``frame_ready``.
        """

        if len(pixels) > self._slot_size:
            raise ValueError(
                f"frame size {len(pixels)} exceeds slot size {self._slot_size}"
            )
        with self._lock:
            slot_index = self._write_idx % self._slot_count
            offset = slot_index * self._slot_size
            self._buffer[offset : offset + len(pixels)] = pixels
            slot = FrameSlot(
                slot_index=slot_index,
                ts_us=int(ts_us),
                seq=int(self._seq),
                width=int(width),
                height=int(height),
                stride=int(stride),
                pixel_format=int(pixel_format),
            )
            self._write_idx += 1
            self._seq += 1
            return slot

    def read(self, slot_index: int) -> bytes:
        """Return a copy of the pixels in ``slot_index``.

        Used by tests. The real vendor binary mmaps the shared memory
        region and reads slot bytes directly; the Python writer never
        reads back from the ring in production.
        """

        if not 0 <= slot_index < self._slot_count:
            raise IndexError("slot_index out of range")
        offset = slot_index * self._slot_size
        return bytes(self._buffer[offset : offset + self._slot_size])


class MsgpackChannel:
    """Length-prefixed msgpack codec over a socket-like transport.

    Production uses a ``socket.socket`` opened against the vendor
    binary's UDS endpoint. Tests pass an in-memory ``BytesIO`` pair to
    exercise the codec end-to-end without a real socket. The transport
    object only needs to implement ``send`` / ``recv`` (or a
    file-like ``write`` / ``read`` shim).
    """

    def __init__(self, transport: object) -> None:
        self._transport = transport
        self._send_lock = threading.Lock()
        self._recv_buffer = bytearray()

    def send(self, message: dict) -> None:
        """Encode ``message`` as msgpack and write the framed bytes."""

        body = msgpack.packb(message, use_bin_type=True)
        if len(body) > MAX_MESSAGE_BYTES:
            raise ValueError(
                f"message size {len(body)} exceeds {MAX_MESSAGE_BYTES} bytes"
            )
        header = struct.pack(LENGTH_PREFIX_FMT, len(body))
        with self._send_lock:
            self._write_all(header + body)

    def recv(self) -> Optional[dict]:
        """Read one framed msgpack message; ``None`` on clean EOF.

        Used in a polling loop in production. The codec accumulates
        partial reads into ``_recv_buffer`` so a slow socket that
        delivers the length prefix in one read and the body in another
        does not corrupt the stream.
        """

        header = self._read_exact(LENGTH_PREFIX_BYTES)
        if header is None:
            return None
        (length,) = struct.unpack(LENGTH_PREFIX_FMT, header)
        if length == 0:
            return {}
        if length > MAX_MESSAGE_BYTES:
            raise ValueError(
                f"declared message size {length} exceeds {MAX_MESSAGE_BYTES}"
            )
        body = self._read_exact(length)
        if body is None:
            return None
        return msgpack.unpackb(body, raw=False)

    def close(self) -> None:
        close = getattr(self._transport, "close", None)
        if callable(close):
            try:
                close()
            except Exception:  # noqa: BLE001 - teardown must not raise
                pass

    # ------------------------------------------------------------------
    # Transport glue
    # ------------------------------------------------------------------

    def _write_all(self, data: bytes) -> None:
        send = getattr(self._transport, "send", None)
        if callable(send):
            offset = 0
            while offset < len(data):
                sent = send(data[offset:])
                if sent == 0:
                    raise ConnectionError("transport closed during send")
                offset += sent
            return
        write = getattr(self._transport, "write", None)
        if callable(write):
            write(data)
            return
        raise TypeError("transport supports neither send() nor write()")

    def _read_exact(self, count: int) -> Optional[bytes]:
        # Drain anything available on the transport; return ``None``
        # when the buffer is short of ``count`` bytes. The polling
        # caller re-invokes ``recv`` on the next tick once more bytes
        # arrive. A genuine half-closed connection eventually surfaces
        # as a heartbeat miss in the watchdog rather than a torn-frame
        # exception here.
        while len(self._recv_buffer) < count:
            chunk = self._recv_chunk()
            if not chunk:
                return None
            self._recv_buffer.extend(chunk)
        out = bytes(self._recv_buffer[:count])
        del self._recv_buffer[:count]
        return out

    def _recv_chunk(self) -> bytes:
        recv = getattr(self._transport, "recv", None)
        if callable(recv):
            return recv(MAX_MESSAGE_BYTES)
        read = getattr(self._transport, "read", None)
        if callable(read):
            return read(MAX_MESSAGE_BYTES)
        raise TypeError("transport supports neither recv() nor read()")


def open_uds_seqpacket(path: str) -> socket.socket:
    """Open a SOCK_SEQPACKET UDS to ``path``.

    Falls back to SOCK_STREAM when the kernel refuses SEQPACKET (rare;
    Linux supports it). Used in production to connect the Python shim
    to the spawned vendor binary's listener.
    """

    try:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    except OSError:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(path)
    return sock


def _make_paired_byte_pipes() -> tuple[io.BytesIO, io.BytesIO]:
    """Returns (read_side, write_side) for in-memory tests.

    Not used in production. Tests construct a pair of ``BytesIO``
    objects and a thin adapter that bridges them; see
    ``tests/test_vio_shim.py`` for the fixture pattern.
    """

    return io.BytesIO(), io.BytesIO()
