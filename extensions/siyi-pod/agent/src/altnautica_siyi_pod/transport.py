"""Transport backends for the SIYI SDK link.

The pod speaks the same frames over UDP, TCP, or a TTL serial port. A backend is
anything that satisfies :class:`SiyiTransport`: open the link, send a framed
byte string, hand every received byte chunk to a callback, and report when its
read side has ended so the session can reopen it. The session (``session.py``)
owns exactly one transport and does all framing, so the backends stay dumb byte
pipes.

:class:`MockTransport` is a deterministic in-memory pod used by every unit test
so the whole driver runs with no hardware attached: it answers the firmware and
hardware-id queries for a configurable model, echoes gimbal attitude, and
synthesises a laser range and an AI-track box. The real backends
(:class:`UdpTransport`, :class:`TcpTransport`, :class:`UartTransport`) use the
standard library, plus ``pyserial`` for the serial port.
"""

from __future__ import annotations

import asyncio
import os
import struct
from typing import Callable, Protocol, runtime_checkable

from altnautica_siyi_pod import commands as C
from altnautica_siyi_pod.capability_profile import HW_ZT30
from altnautica_siyi_pod.framing import CTRL_NEED_ACK, build_frame, parse_frame

DEFAULT_HOST = "192.168.144.25"
DEFAULT_PORT = 37260
DEFAULT_BAUD = 115200

OnBytes = Callable[[bytes], None]
OnLost = Callable[[], None]


@runtime_checkable
class SiyiTransport(Protocol):
    """The byte-pipe surface the session depends on."""

    def set_on_bytes(self, callback: OnBytes) -> None: ...

    def set_on_lost(self, callback: OnLost) -> None: ...

    async def open(self) -> None: ...

    async def close(self) -> None: ...

    async def send(self, frame: bytes) -> None: ...


class MockTransport:
    """In-memory SIYI pod for hardware-free tests.

    ``model`` is a hardware-id code (see ``capability_profile``); the mock
    answers the hardware-id query with it so capability negotiation resolves the
    matching profile. Constructor knobs seed the synthetic telemetry the tests
    assert against.
    """

    def __init__(
        self,
        *,
        model: int = HW_ZT30,
        laser_range_m: float = 42.0,
        firmware: str = "v1.2.3",
        answer_identity: bool = True,
        track_box: tuple | None = None,
        fail_opens: int = 0,
    ) -> None:
        self.model = model
        self.laser_range_m = laser_range_m
        self.firmware = firmware
        # The AI-track box the mock reports (track_id, x, y, w, h, locked), or
        # None for no active track; tests mutate it to drive the republish path.
        self.track_box = track_box
        # When False the mock stays silent on the firmware / hardware-id queries,
        # simulating a pod that is not yet reachable (drives the retry path).
        self.answer_identity = answer_identity
        # How many upcoming open() calls fail (a pod that is not yet powered,
        # or a serial device that has not enumerated); drives the reconnect path.
        self.fail_opens = fail_opens
        self.opens = 0
        self._on_bytes: OnBytes | None = None
        self._on_lost: OnLost | None = None
        self.opened = False
        self.sent: list[bytes] = []
        # Internal pod state the mock mutates in response to commands.
        self.yaw_deg = 0.0
        self.pitch_deg = 0.0
        self.roll_deg = 0.0
        self.zoom = 1.0
        self.recording = False
        self.gimbal_mode = "follow"
        self.palette = 0
        self.photos_taken = 0
        # Image-source assignment the mock records (stream_id -> source_id) plus
        # the on-pod split/PiP toggle, so tests assert the source routing.
        self.image_sources: dict[int, int] = {}
        self.split_mode = False

    def set_on_bytes(self, callback: OnBytes) -> None:
        self._on_bytes = callback

    def set_on_lost(self, callback: OnLost) -> None:
        self._on_lost = callback

    async def open(self) -> None:
        self.opens += 1
        if self.fail_opens > 0:
            self.fail_opens -= 1
            raise ConnectionRefusedError("mock pod is not reachable")
        self.opened = True

    def drop(self) -> None:
        """End the read side, as a TCP EOF or an unplugged serial device does."""
        self.opened = False
        if self._on_lost is not None:
            self._on_lost()

    async def close(self) -> None:
        self.opened = False

    async def send(self, frame: bytes) -> None:
        self.sent.append(frame)
        reply = self._reply_for(parse_frame(frame))
        if reply is not None and self._on_bytes is not None:
            self._on_bytes(reply)

    # -- the synthetic pod behaviour --------------------------------------
    def _reply_for(self, f) -> bytes | None:
        cmd = f.cmd_id
        seq = f.seq
        if cmd == C.CMD_HARDWARE_ID:
            if not self.answer_identity:
                return None
            return build_frame(C.CMD_HARDWARE_ID, bytes([self.model, 0x00]), seq=seq)
        if cmd == C.CMD_FIRMWARE_VERSION:
            if not self.answer_identity:
                return None
            # Synthetic firmware triple (camera / gimbal / zoom board).
            return build_frame(
                C.CMD_FIRMWARE_VERSION, b"\x01\x02\x03\x00\x00\x00", seq=seq
            )
        if cmd == C.CMD_GIMBAL_ATTITUDE:
            return build_frame(
                C.CMD_GIMBAL_ATTITUDE, self._attitude_payload(), seq=seq
            )
        if cmd == C.CMD_SET_GIMBAL_ATTITUDE:
            yaw, pitch = struct.unpack_from("<hh", f.data, 0)
            self.yaw_deg = yaw / 10.0
            self.pitch_deg = pitch / 10.0
            return self._ack(cmd, seq)
        if cmd == C.CMD_GIMBAL_SPEED:
            return self._ack(cmd, seq)
        if cmd == C.CMD_CENTER:
            self.yaw_deg = 0.0
            self.pitch_deg = 0.0
            return self._ack(cmd, seq)
        if cmd == C.CMD_ABSOLUTE_ZOOM:
            self.zoom = f.data[0] + (f.data[1] % 10) / 10.0 if len(f.data) >= 2 else 1.0
            return self._ack(cmd, seq)
        if cmd == C.CMD_CURRENT_ZOOM:
            integer = int(self.zoom)
            frac = int(round((self.zoom - integer) * 10)) % 10
            return build_frame(C.CMD_CURRENT_ZOOM, bytes([integer, frac]), seq=seq)
        if cmd == C.CMD_LASER_RANGE:
            decimetres = int(round(self.laser_range_m * 10))
            return build_frame(
                C.CMD_LASER_RANGE, struct.pack("<H", decimetres & 0xFFFF), seq=seq
            )
        if cmd == C.CMD_FUNCTION_FEEDBACK and f.data:
            func = f.data[0]
            if func == C.FUNC_TAKE_PHOTO:
                self.photos_taken += 1
            elif func == C.FUNC_RECORD_TOGGLE:
                self.recording = not self.recording
            elif func == C.FUNC_MOTION_LOCK:
                self.gimbal_mode = "lock"
            elif func == C.FUNC_MOTION_FOLLOW:
                self.gimbal_mode = "follow"
            elif func == C.FUNC_MOTION_FPV:
                self.gimbal_mode = "fpv"
            return self._ack(cmd, seq)
        if cmd == C.CMD_THERMAL_PALETTE and f.data:
            self.palette = f.data[0]
            return self._ack(cmd, seq)
        if cmd == C.CMD_SET_IMAGE_SOURCE and len(f.data) >= 2:
            self.image_sources[f.data[0]] = f.data[1]
            return self._ack(cmd, seq)
        if cmd == C.CMD_SET_SPLIT_MODE and f.data:
            self.split_mode = bool(f.data[0])
            return self._ack(cmd, seq)
        if cmd == C.CMD_AI_TRACK:
            if f.data and f.data[0] == C.AI_TRACK_QUERY_FLAG:
                return build_frame(C.CMD_AI_TRACK, self._track_payload(), seq=seq)
            return self._ack(cmd, seq)
        # Every other need-ack command gets a generic ack so the session's
        # request future resolves.
        if f.ctrl & CTRL_NEED_ACK:
            return self._ack(cmd, seq)
        return None

    def _attitude_payload(self) -> bytes:
        return struct.pack(
            "<hhhhhh",
            int(round(self.yaw_deg * 10)),
            int(round(self.pitch_deg * 10)),
            int(round(self.roll_deg * 10)),
            0,
            0,
            0,
        )

    def _track_payload(self) -> bytes:
        if self.track_box is None:
            return bytes([0x00])  # status 0 = no active track
        tid, x, y, w, h, locked = self.track_box
        status = 0x01 if locked else 0x02  # non-zero = tracking; bit 0 = locked
        return struct.pack("<BBHHHH", status, tid, x, y, w, h)

    def push_attitude(self) -> None:
        """Deliver an unsolicited attitude frame (simulates a data stream)."""
        if self._on_bytes is not None:
            self._on_bytes(build_frame(C.CMD_GIMBAL_ATTITUDE, self._attitude_payload()))

    @staticmethod
    def _ack(cmd_id: int, seq: int) -> bytes:
        return build_frame(cmd_id, b"\x01", seq=seq)


class _UdpProtocol(asyncio.DatagramProtocol):
    def __init__(self, on_bytes: OnBytes, on_lost: OnLost | None) -> None:
        self._on_bytes = on_bytes
        self.on_lost = on_lost

    def datagram_received(self, data: bytes, _addr) -> None:
        self._on_bytes(data)

    def connection_lost(self, _exc: Exception | None) -> None:
        if self.on_lost is not None:
            self.on_lost()


class UdpTransport:
    """UDP link to the pod (the default Ethernet path)."""

    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT) -> None:
        self._host = host
        self._port = port
        self._on_bytes: OnBytes | None = None
        self._on_lost: OnLost | None = None
        self._transport: asyncio.DatagramTransport | None = None
        self._protocol: _UdpProtocol | None = None

    def set_on_bytes(self, callback: OnBytes) -> None:
        self._on_bytes = callback

    def set_on_lost(self, callback: OnLost) -> None:
        self._on_lost = callback

    async def open(self) -> None:
        loop = asyncio.get_running_loop()
        cb = self._on_bytes or (lambda _b: None)
        self._transport, self._protocol = await loop.create_datagram_endpoint(
            lambda: _UdpProtocol(cb, self._on_lost),
            remote_addr=(self._host, self._port),
        )

    async def close(self) -> None:
        # Detach first: a close the session asked for is not a lost link.
        if self._protocol is not None:
            self._protocol.on_lost = None
            self._protocol = None
        if self._transport is not None:
            self._transport.close()
            self._transport = None

    async def send(self, frame: bytes) -> None:
        if self._transport is None:
            raise RuntimeError("UDP transport is not open")
        self._transport.sendto(frame)


class TcpTransport:
    """TCP link to the pod."""

    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT) -> None:
        self._host = host
        self._port = port
        self._on_bytes: OnBytes | None = None
        self._on_lost: OnLost | None = None
        self._reader: asyncio.StreamReader | None = None
        self._writer: asyncio.StreamWriter | None = None
        self._read_task: asyncio.Task | None = None

    def set_on_bytes(self, callback: OnBytes) -> None:
        self._on_bytes = callback

    def set_on_lost(self, callback: OnLost) -> None:
        self._on_lost = callback

    async def open(self) -> None:
        self._reader, self._writer = await asyncio.open_connection(
            self._host, self._port
        )
        self._read_task = asyncio.create_task(self._read_loop())

    async def _read_loop(self) -> None:
        assert self._reader is not None
        try:
            while True:
                chunk = await self._reader.read(4096)
                if not chunk:
                    break
                if self._on_bytes is not None:
                    self._on_bytes(chunk)
        except OSError:
            pass
        if self._on_lost is not None:
            self._on_lost()

    async def close(self) -> None:
        if self._read_task is not None:
            self._read_task.cancel()
            self._read_task = None
        if self._writer is not None:
            self._writer.close()
            self._writer = None
        self._reader = None

    async def send(self, frame: bytes) -> None:
        if self._writer is None:
            raise RuntimeError("TCP transport is not open")
        self._writer.write(frame)
        await self._writer.drain()


class UartTransport:
    """TTL serial link to the pod (115200 8N1) over ``pyserial``.

    The port is opened non-blocking and its file descriptor is watched by the
    event loop: the reader callback drains whatever arrived into the session,
    and writes go straight to the descriptor. A read error or end-of-file (the
    adapter was unplugged) reports the link lost so the session reopens it.
    Needs an event loop with ``add_reader`` support (any POSIX selector loop).
    """

    def __init__(self, port: str, baud: int = DEFAULT_BAUD) -> None:
        self._port = port
        self._baud = baud
        self._on_bytes: OnBytes | None = None
        self._on_lost: OnLost | None = None
        self._serial = None
        self._fd: int | None = None
        self._loop: asyncio.AbstractEventLoop | None = None

    def set_on_bytes(self, callback: OnBytes) -> None:
        self._on_bytes = callback

    def set_on_lost(self, callback: OnLost) -> None:
        self._on_lost = callback

    async def open(self) -> None:
        import serial  # pyserial

        loop = asyncio.get_running_loop()
        # timeout=0 / write_timeout=0: pyserial opens the descriptor
        # O_NONBLOCK and never blocks the loop on a read or a write.
        ser = serial.Serial(self._port, self._baud, timeout=0, write_timeout=0)
        fd = ser.fileno()
        try:
            loop.add_reader(fd, self._on_readable)
        except BaseException:
            ser.close()
            raise
        self._serial = ser
        self._fd = fd
        self._loop = loop

    def _on_readable(self) -> None:
        fd = self._fd
        if fd is None:
            return
        try:
            chunk = os.read(fd, 4096)
        except (BlockingIOError, InterruptedError):
            return
        except OSError:
            chunk = b""
        if not chunk:
            self._lose()
            return
        if self._on_bytes is not None:
            self._on_bytes(chunk)

    def _lose(self) -> None:
        # Stop watching the dead descriptor; the session closes and reopens.
        if self._loop is not None and self._fd is not None:
            self._loop.remove_reader(self._fd)
        self._fd = None
        if self._on_lost is not None:
            self._on_lost()

    async def close(self) -> None:
        if self._loop is not None and self._fd is not None:
            self._loop.remove_reader(self._fd)
        self._fd = None
        self._loop = None
        if self._serial is not None:
            ser, self._serial = self._serial, None
            ser.close()

    async def send(self, frame: bytes) -> None:
        fd = self._fd
        loop = self._loop
        if fd is None or loop is None:
            raise RuntimeError("UART transport is not open")
        view = memoryview(frame)
        while view:
            try:
                written = os.write(fd, view)
            except (BlockingIOError, InterruptedError):
                await self._writable(loop, fd)
                continue
            view = view[written:]

    @staticmethod
    async def _writable(loop: asyncio.AbstractEventLoop, fd: int) -> None:
        ready = loop.create_future()

        def _wake() -> None:
            if not ready.done():
                ready.set_result(None)

        loop.add_writer(fd, _wake)
        try:
            await ready
        finally:
            loop.remove_writer(fd)
