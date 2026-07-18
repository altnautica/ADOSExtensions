"""SIYI SDK frame codec.

Every SIYI optical pod speaks the same on-the-wire frame regardless of the
transport (UDP, TCP, or TTL serial):

    0x55 0x66 | CTRL(1) | DATA_LEN(2 LE) | SEQ(2 LE) | CMD_ID(1) | DATA(N) | CRC16(2 LE)

All multi-byte fields are little-endian. The CRC is CRC-16/XMODEM (polynomial
0x1021, initial value 0x0000, no reflection) computed over every byte from the
0x55 start marker through the last DATA byte. The heartbeat frame documented in
the SIYI SDK,

    55 66 01 01 00 00 00 00 00 59 8B

is the canonical golden vector the unit tests check the codec against.

This module is pure (no I/O) so it is fully unit-testable and reused by both
the mock and the real transports.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

STX = b"\x55\x66"

# Control byte flags. Bit 0 requests an acknowledgement from the pod; the pod
# echoes the same CMD_ID back with the ACK bit set. We request an ack on the
# commands whose response we correlate by sequence number.
CTRL_NEED_ACK = 0x01
CTRL_NO_ACK = 0x00

_HEADER_LEN = 2 + 1 + 2 + 2 + 1  # STX + CTRL + LEN + SEQ + CMD
_CRC_LEN = 2
_MIN_FRAME_LEN = _HEADER_LEN + _CRC_LEN


class FramingError(ValueError):
    """A byte buffer did not parse as a valid SIYI frame."""


@dataclass(frozen=True)
class Frame:
    """A decoded SIYI frame."""

    cmd_id: int
    data: bytes
    seq: int
    ctrl: int

    @property
    def is_ack(self) -> bool:
        """True when the pod set the ack bit (a reply to a need-ack send)."""
        return bool(self.ctrl & CTRL_NEED_ACK)


def crc16_xmodem(data: bytes) -> int:
    """CRC-16/XMODEM over ``data`` (poly 0x1021, init 0x0000, MSB-first)."""
    crc = 0x0000
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


def build_frame(
    cmd_id: int,
    data: bytes = b"",
    *,
    seq: int = 0,
    need_ack: bool = True,
) -> bytes:
    """Encode one SIYI frame ready to write to the transport."""
    if not 0 <= cmd_id <= 0xFF:
        raise FramingError(f"cmd_id out of range: {cmd_id}")
    if len(data) > 0xFFFF:
        raise FramingError(f"data too long: {len(data)} bytes")
    ctrl = CTRL_NEED_ACK if need_ack else CTRL_NO_ACK
    head = (
        STX
        + bytes([ctrl])
        + struct.pack("<H", len(data))
        + struct.pack("<H", seq & 0xFFFF)
        + bytes([cmd_id])
        + data
    )
    return head + struct.pack("<H", crc16_xmodem(head))


def parse_frame(buffer: bytes) -> Frame:
    """Decode exactly one framed message, validating length and CRC."""
    if len(buffer) < _MIN_FRAME_LEN:
        raise FramingError(f"buffer too short: {len(buffer)} bytes")
    if buffer[:2] != STX:
        raise FramingError("bad start marker")
    ctrl = buffer[2]
    data_len = struct.unpack_from("<H", buffer, 3)[0]
    seq = struct.unpack_from("<H", buffer, 5)[0]
    cmd_id = buffer[7]
    expected = _HEADER_LEN + data_len + _CRC_LEN
    if len(buffer) != expected:
        raise FramingError(
            f"length mismatch: header claims {data_len} data bytes "
            f"(frame {expected}) but got {len(buffer)}"
        )
    data = buffer[_HEADER_LEN : _HEADER_LEN + data_len]
    got_crc = struct.unpack_from("<H", buffer, _HEADER_LEN + data_len)[0]
    want_crc = crc16_xmodem(buffer[: _HEADER_LEN + data_len])
    if got_crc != want_crc:
        raise FramingError(f"crc mismatch: got {got_crc:#06x}, want {want_crc:#06x}")
    return Frame(cmd_id=cmd_id, data=data, seq=seq, ctrl=ctrl)


def iter_frames(buffer: bytes) -> tuple[list[Frame], bytes]:
    """Split a byte stream into whole frames, returning any trailing partial.

    A stream transport (TCP, serial) can deliver several frames coalesced or a
    frame split across reads. This resyncs on the 0x55 0x66 marker, yields every
    complete valid frame, and hands back the leftover bytes for the next read.
    Corrupt bytes before a valid marker are discarded.
    """
    frames: list[Frame] = []
    view = buffer
    while True:
        start = view.find(STX)
        if start == -1:
            # No marker at all: keep only a trailing partial marker byte.
            return frames, view[-1:] if view[-1:] == b"\x55" else b""
        view = view[start:]
        if len(view) < _HEADER_LEN + _CRC_LEN:
            return frames, view
        data_len = struct.unpack_from("<H", view, 3)[0]
        frame_len = _HEADER_LEN + data_len + _CRC_LEN
        if len(view) < frame_len:
            return frames, view
        candidate = view[:frame_len]
        try:
            frames.append(parse_frame(candidate))
            view = view[frame_len:]
        except FramingError:
            # Bad frame at this marker: skip one byte and resync.
            view = view[1:]
