"""SIYI frame codec tests, anchored on the SDK heartbeat golden vector."""

from __future__ import annotations

import pytest

from altnautica_siyi_pod.framing import (
    FramingError,
    build_frame,
    crc16_xmodem,
    iter_frames,
    parse_frame,
)

# The exact heartbeat frame documented in the SIYI Gimbal Camera External SDK.
HEARTBEAT_HEX = "556601010000000000598b"


def test_heartbeat_golden_vector():
    frame = build_frame(0x00, b"\x00", seq=0, need_ack=True)
    assert frame.hex() == HEARTBEAT_HEX


def test_crc_matches_heartbeat_body():
    body = bytes.fromhex("556601010000000000")
    assert crc16_xmodem(body) == 0x8B59


@pytest.mark.parametrize(
    "cmd_id,data,seq",
    [
        (0x0E, b"\x10\x27\xd8\xff", 7),
        (0x02, b"", 1),
        (0x15, b"\x9b\x00", 65535),
        (0x0D, bytes(range(12)), 300),
    ],
)
def test_round_trip(cmd_id, data, seq):
    frame = build_frame(cmd_id, data, seq=seq)
    parsed = parse_frame(frame)
    assert parsed.cmd_id == cmd_id
    assert parsed.data == data
    assert parsed.seq == seq


def test_crc_rejected_on_corruption():
    frame = bytearray(build_frame(0x0D, b"\x01\x02", seq=5))
    frame[-1] ^= 0xFF
    with pytest.raises(FramingError):
        parse_frame(bytes(frame))


def test_bad_start_marker():
    with pytest.raises(FramingError):
        parse_frame(b"\x00\x00\x01\x00\x00\x00\x00\x00\x00\x00")


def test_length_mismatch():
    good = build_frame(0x0D, b"\x01\x02", seq=1)
    with pytest.raises(FramingError):
        parse_frame(good + b"\x00")


def test_iter_frames_coalesced():
    a = build_frame(0x0D, b"\x01\x02", seq=1)
    b = build_frame(0x15, b"\x9b\x00", seq=2)
    frames, rest = iter_frames(a + b)
    assert [f.cmd_id for f in frames] == [0x0D, 0x15]
    assert rest == b""


def test_iter_frames_partial_tail():
    a = build_frame(0x0D, b"\x01\x02", seq=1)
    b = build_frame(0x15, b"\x9b\x00", seq=2)
    stream = a + b
    frames, rest = iter_frames(stream[:-1])  # last CRC byte withheld
    assert [f.cmd_id for f in frames] == [0x0D]
    # The leftover is the partial second frame, completed on the next read.
    frames2, rest2 = iter_frames(rest + stream[-1:])
    assert [f.cmd_id for f in frames2] == [0x15]
    assert rest2 == b""


def test_iter_frames_resyncs_past_garbage():
    a = build_frame(0x0D, b"\x01\x02", seq=1)
    frames, _rest = iter_frames(b"\xde\xad\xbe\xef" + a)
    assert [f.cmd_id for f in frames] == [0x0D]
