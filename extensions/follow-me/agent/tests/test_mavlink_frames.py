"""MAVLink frame builder tests: valid v2 frames + decode round-trip."""

from __future__ import annotations

import math

from pymavlink.dialects.v20 import common as mavlink2

from follow_me import mavlink_frames


def _decode(frame: bytes):
    mav = mavlink2.MAVLink(None)
    mav.robust_parsing = True
    msgs = []
    for byte in frame:
        m = mav.parse_char(bytes([byte]))
        if m is not None:
            msgs.append(m)
    return msgs[-1] if msgs else None


def _local(**overrides: float) -> dict:
    args = dict(
        north_m=12.5,
        east_m=-3.25,
        down_m=-4.0,
        vn_mps=1.4,
        ve_mps=-0.2,
        vd_mps=0.0,
        yaw_rad=1.2,
    )
    args.update(overrides)
    return mavlink_frames.build_local_setpoint(**args)


def test_local_setpoint_carries_position_velocity_and_yaw() -> None:
    # The setpoint is built as ctx.flight.guided_setpoint kwargs; the agent's
    # scoped flight sender encodes it as SET_POSITION_TARGET_LOCAL_NED (84).
    sp = _local()
    assert sp["kind"] == "local_ned"
    assert sp["coordinate_frame"] == mavlink2.MAV_FRAME_LOCAL_NED == 1
    assert (sp["x"], sp["y"], sp["z"]) == (12.5, -3.25, -4.0)
    assert (sp["vx"], sp["vy"], sp["vz"]) == (1.4, -0.2, 0.0)
    assert sp["yaw"] == 1.2


def test_local_setpoint_mask_is_2496() -> None:
    mask = _local()["type_mask"]
    assert mask == 2496
    # Acceleration (6,7,8) and yaw-rate (11) ignored.
    for bit in (6, 7, 8, 11):
        assert mask & (1 << bit), f"bit {bit} should be set (ignored)"
    # Position (0-2), velocity (3-5) and yaw (10) used.
    for bit in (0, 1, 2, 3, 4, 5, 10):
        assert not (mask & (1 << bit)), f"bit {bit} should be clear (used)"


def test_gimbal_pitchyaw_is_valid_command_long() -> None:
    frame = mavlink_frames.build_gimbal_pitchyaw(pitch_deg=-30.0, yaw_deg=15.0)
    assert frame[0] == 0xFD
    msg = _decode(frame)
    assert msg is not None
    assert msg.get_type() == "COMMAND_LONG"
    assert msg.command == mavlink2.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
    assert abs(msg.param1 - (-30.0)) < 1e-4
    assert abs(msg.param2 - 15.0) < 1e-4
    assert math.isnan(msg.param3)
    assert math.isnan(msg.param4)


def test_gimbal_angles_point_down_at_subject() -> None:
    # Subject directly ahead, vehicle 10 m up, slant 14.14 (45deg), bearing
    # north, vehicle facing north => pitch -45, yaw 0.
    pitch, yaw = mavlink_frames.gimbal_angles_for_target(
        slant_range_m=math.sqrt(200.0),
        agl_m=10.0,
        bearing_rad=0.0,
        vehicle_yaw_rad=0.0,
    )
    assert abs(pitch - (-45.0)) < 0.5
    assert abs(yaw) < 1e-6


def test_gimbal_yaw_is_relative_to_vehicle_heading_and_wrapped() -> None:
    # Subject bearing east (90deg), vehicle facing north => gimbal yaw 90.
    _, yaw = mavlink_frames.gimbal_angles_for_target(
        slant_range_m=20.0,
        agl_m=10.0,
        bearing_rad=math.pi / 2.0,
        vehicle_yaw_rad=0.0,
    )
    assert abs(yaw - 90.0) < 1e-3
    # Subject bearing west, vehicle facing east: 270 wraps to -180..180.
    _, yaw2 = mavlink_frames.gimbal_angles_for_target(
        slant_range_m=20.0,
        agl_m=10.0,
        bearing_rad=-math.pi / 2.0,
        vehicle_yaw_rad=math.pi / 2.0,
    )
    assert -180.0 < yaw2 <= 180.0
    assert abs(abs(yaw2) - 180.0) < 1e-3
