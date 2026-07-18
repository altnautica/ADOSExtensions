"""SIYI command encoder/decoder tests."""

from __future__ import annotations

import struct

import pytest

from altnautica_siyi_pod import commands as C
from altnautica_siyi_pod.framing import build_frame, parse_frame


def _round_trip(command: C.Command) -> None:
    frame = build_frame(command.cmd_id, command.data, seq=1)
    parsed = parse_frame(frame)
    assert parsed.cmd_id == command.cmd_id
    assert parsed.data == command.data


@pytest.mark.parametrize(
    "command",
    [
        C.request_firmware(),
        C.request_hardware_id(),
        C.request_gimbal_attitude(),
        C.request_laser_range(),
        C.autofocus(),
        C.gimbal_speed(50, -30),
        C.set_gimbal_attitude(12.5, -7.0),
        C.center(),
        C.set_gimbal_mode("lock"),
        C.manual_zoom(1),
        C.absolute_zoom(4.5),
        C.take_photo(),
        C.record_toggle(),
        C.set_thermal_palette(3),
        C.set_thermal_gain(True),
        C.set_image_source("main", "eo_zoom"),
        C.set_image_source("sub", "ir"),
        C.set_split_mode(True),
        C.request_track_box(),
    ],
)
def test_command_frames_round_trip(command):
    _round_trip(command)


def test_decode_track_box():
    # PLACEHOLDER wire layout (status, track_id, x, y, w, h) — exercised so the
    # republish path is covered; the real layout is bench-resolved (Rule 44).
    payload = struct.pack("<BBHHHH", 0x01, 7, 100, 120, 40, 60)
    box = C.decode_track_box(payload)
    assert box is not None
    assert box.track_id == 7
    assert box.locked is True
    assert (box.x, box.y, box.width, box.height) == (100.0, 120.0, 40.0, 60.0)
    # A non-zero status with bit 0 clear = tracking but not locked.
    unlocked = struct.pack("<BBHHHH", 0x02, 1, 0, 0, 0, 0)
    assert C.decode_track_box(unlocked).locked is False
    # Status 0 and truncated payloads mean no active track.
    assert C.decode_track_box(bytes([0x00])) is None
    assert C.decode_track_box(b"\x01\x02") is None
    assert C.decode_track_box(b"") is None


def test_set_image_source_payload():
    # PLACEHOLDER wire layout (stream id, source id) — exercised so the control
    # path is covered; the real opcode/layout is bench-resolved (Rule 44).
    assert C.set_image_source("main", "eo_zoom").data == bytes(
        [C.STREAM_MAIN, C.IMG_SOURCE_EO_ZOOM]
    )
    assert C.set_image_source("sub", "ir").data == bytes(
        [C.STREAM_SUB, C.IMG_SOURCE_IR]
    )
    assert C.set_image_source("sub", "split").data == bytes(
        [C.STREAM_SUB, C.IMG_SOURCE_SPLIT]
    )


def test_set_split_mode_payload():
    assert C.set_split_mode(True).data == bytes([0x01])
    assert C.set_split_mode(False).data == bytes([0x00])


def test_set_gimbal_attitude_payload():
    cmd = C.set_gimbal_attitude(12.5, -7.0)
    yaw, pitch = struct.unpack("<hh", cmd.data)
    assert yaw == 125
    assert pitch == -70


def test_absolute_zoom_payload():
    assert C.absolute_zoom(4.5).data == bytes([4, 5])
    assert C.absolute_zoom(1.0).data == bytes([1, 0])
    # Below 1.0 is clamped up to 1.0.
    assert C.absolute_zoom(0.2).data == bytes([1, 0])


def test_gimbal_speed_clamps():
    assert struct.unpack("<bb", C.gimbal_speed(999, -999).data) == (100, -100)


def test_gimbal_mode_rejects_unknown():
    with pytest.raises(ValueError):
        C.set_gimbal_mode("orbit")


def test_decode_gimbal_attitude_round_trip():
    payload = struct.pack("<hhhhhh", 125, -70, 3, 0, 0, 0)
    att = C.decode_gimbal_attitude(payload)
    assert att.yaw_deg == 12.5
    assert att.pitch_deg == -7.0
    assert att.roll_deg == 0.3


def test_decode_laser_range():
    assert C.decode_laser_range(struct.pack("<H", 155)) == 15.5
    assert C.decode_laser_range(struct.pack("<H", 12000)) == 1200.0


def test_decode_current_zoom():
    assert C.decode_current_zoom(bytes([4, 5])) == 4.5
    assert C.decode_current_zoom(bytes([1, 0])) == 1.0


def test_decoders_reject_short_payloads():
    with pytest.raises(ValueError):
        C.decode_gimbal_attitude(b"\x00")
    with pytest.raises(ValueError):
        C.decode_laser_range(b"\x00")
