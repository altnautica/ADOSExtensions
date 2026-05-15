"""TF-Luna LiDAR rangefinder driver (Benewake) over UART.

Frame format (9 bytes per packet):
    byte 0:  0x59 (header)
    byte 1:  0x59 (header)
    byte 2:  distance low byte
    byte 3:  distance high byte           -> distance_cm = u16 little-endian
    byte 4:  signal strength low byte
    byte 5:  signal strength high byte    -> strength    = u16 little-endian
    byte 6:  temperature low byte
    byte 7:  temperature high byte        -> temp_c      = (u16 / 8) - 256
    byte 8:  checksum = sum(bytes 0..7) & 0xFF
"""
from __future__ import annotations

import asyncio
import time
from typing import Optional

import serial  # pyserial

from .base import RangefinderDriver, RangeReading

FRAME_LEN = 9
HEADER = 0x59
DEFAULT_BAUD = 115200
READ_WINDOW_S = 0.1
MAX_BUFFER_BYTES = 256
TOP_QUALITY_SIGNAL = 20000


def parse_frame(frame: bytes) -> Optional[tuple[float, int, int]]:
    """Parse a single 9-byte TF-Luna packet.

    Returns (distance_m, quality_0_100, raw_status) on a valid frame, or None
    if the header or checksum is bad.

    `raw_status` is the signal-strength field, exposed as a driver-specific
    diagnostic value.
    """
    if len(frame) != FRAME_LEN:
        return None
    if frame[0] != HEADER or frame[1] != HEADER:
        return None
    checksum = sum(frame[0:8]) & 0xFF
    if checksum != frame[8]:
        return None
    distance_cm = frame[2] | (frame[3] << 8)
    signal_strength = frame[4] | (frame[5] << 8)
    distance_m = distance_cm / 100.0
    quality = min(100, int(signal_strength / (TOP_QUALITY_SIGNAL / 100)))
    if quality < 0:
        quality = 0
    return distance_m, quality, signal_strength


def find_frame(buffer: bytes) -> tuple[Optional[bytes], bytes]:
    """Scan for the next valid 9-byte frame in `buffer`.

    Returns (frame_or_none, remaining_buffer). When a frame is found, the
    remaining buffer starts after the parsed frame so the next call can pick
    up where this one left off. Invalid frames (bad checksum) are skipped one
    byte at a time so we re-sync on the next 0x59 0x59 pair.
    """
    i = 0
    n = len(buffer)
    while i + FRAME_LEN <= n:
        if buffer[i] == HEADER and buffer[i + 1] == HEADER:
            candidate = bytes(buffer[i : i + FRAME_LEN])
            if parse_frame(candidate) is not None:
                return candidate, bytes(buffer[i + FRAME_LEN :])
        i += 1
    # No frame; preserve a small tail so a header straddling the read boundary
    # can be matched on the next call.
    tail_start = max(0, n - (FRAME_LEN - 1))
    return None, bytes(buffer[tail_start:])


class TfLunaUart(RangefinderDriver):
    """Benewake TF-Luna driver over a UART link."""

    def __init__(self, device: str, baud: int = DEFAULT_BAUD) -> None:
        self._device = device
        self._baud = baud
        self._serial: Optional[serial.Serial] = None
        self._buffer: bytes = b""

    @property
    def name(self) -> str:
        return "tfluna_uart"

    @property
    def min_range_m(self) -> float:
        return 0.2

    @property
    def max_range_m(self) -> float:
        return 8.0

    async def open(self) -> None:
        def _open() -> serial.Serial:
            return serial.Serial(
                port=self._device,
                baudrate=self._baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,
            )

        self._serial = await asyncio.to_thread(_open)

    async def close(self) -> None:
        ser = self._serial
        self._serial = None
        self._buffer = b""
        if ser is not None:
            await asyncio.to_thread(ser.close)

    async def read(self) -> Optional[RangeReading]:
        ser = self._serial
        if ser is None:
            return None

        def _drain() -> bytes:
            chunks: list[bytes] = []
            deadline = time.monotonic() + READ_WINDOW_S
            while time.monotonic() < deadline:
                waiting = ser.in_waiting if ser is not None else 0
                if waiting:
                    chunks.append(ser.read(waiting))
                else:
                    # Block briefly to give the sensor a chance to emit a
                    # frame without busy-spinning. pyserial timeout=0 means
                    # this returns immediately when no bytes are present.
                    chunks.append(ser.read(FRAME_LEN))
                    if not chunks[-1]:
                        time.sleep(0.005)
            return b"".join(chunks)

        new_bytes = await asyncio.to_thread(_drain)
        self._buffer = (self._buffer + new_bytes)[-MAX_BUFFER_BYTES:]

        # Consume frames; keep only the last one (freshest reading).
        latest: Optional[tuple[float, int, int]] = None
        while True:
            frame, remaining = find_frame(self._buffer)
            self._buffer = remaining
            if frame is None:
                break
            parsed = parse_frame(frame)
            if parsed is not None:
                latest = parsed

        if latest is None:
            return None

        distance_m, quality, raw_status = latest
        return RangeReading(
            distance_m=distance_m,
            quality=quality,
            timestamp_monotonic_ns=time.monotonic_ns(),
            raw_status=raw_status,
        )
