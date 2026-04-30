"""Encoder helper tests.

The encoders return both a typed object and the wire payload bytes.
We assert the command id, the param1..7 floats, and a deterministic
sample of the byte layout so accidental struct shifts get caught.
"""

from __future__ import annotations

import struct

from altnautica_gimbal_v2.mavlink_messages import (
    MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
    MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
    MAV_CMD_DO_SET_ROI_LOCATION,
    MAV_CMD_DO_SET_ROI_NONE,
    encode_gimbal_manager_configure,
    encode_gimbal_manager_pitchyaw,
    encode_set_roi_location,
    encode_set_roi_none,
)


def test_pitchyaw_basic_command_id_and_params() -> None:
    cmd = encode_gimbal_manager_pitchyaw(pitch_deg=-30.0, yaw_deg=45.0)
    assert cmd.command == MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
    assert cmd.command == 1000
    assert cmd.param1 == -30.0
    assert cmd.param2 == 45.0
    assert cmd.param3 == 0.0
    assert cmd.param4 == 0.0
    assert cmd.param5 == 0.0  # default flags
    assert cmd.param6 == 0.0  # reserved
    assert cmd.param7 == 0.0  # default gimbal device id
    assert cmd.target_component == 1


def test_pitchyaw_payload_param_slots_match_spec() -> None:
    cmd = encode_gimbal_manager_pitchyaw(
        pitch_deg=-30.0,
        yaw_deg=45.0,
        pitch_rate_dps=1.5,
        yaw_rate_dps=-2.5,
        flags=7,
        gimbal_device_id=2,
        target_system=1,
        target_component=1,
    )
    layout = "<fffffffHBBB"
    fields = struct.unpack(layout, cmd.payload)
    assert fields[0] == -30.0  # p1 pitch
    assert fields[1] == 45.0  # p2 yaw
    assert fields[2] == 1.5  # p3 pitch rate
    assert fields[3] == -2.5  # p4 yaw rate
    assert fields[4] == 7.0  # p5 flags
    assert fields[5] == 0.0  # p6 reserved
    assert fields[6] == 2.0  # p7 gimbal device id
    assert fields[7] == 1000  # command id
    assert fields[8] == 1  # target system
    assert fields[9] == 1  # target component


def test_pitchyaw_emits_command_long_byte_length() -> None:
    cmd = encode_gimbal_manager_pitchyaw(pitch_deg=0.0, yaw_deg=0.0)
    assert len(cmd.payload) == struct.calcsize("<fffffffHBBB")


def test_configure_param_slots_match_spec() -> None:
    cmd = encode_gimbal_manager_configure(
        primary_sysid=1,
        primary_compid=190,
        gimbal_device_id=2,
    )
    assert cmd.command == MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
    assert cmd.command == 1001
    assert cmd.param1 == 1.0
    assert cmd.param2 == 190.0
    assert cmd.param3 == 0.0
    assert cmd.param4 == 0.0
    assert cmd.param5 == 0.0  # reserved
    assert cmd.param6 == 0.0  # reserved
    assert cmd.param7 == 2.0  # gimbal device id


def test_set_roi_location_uses_command_int_with_scaled_lat_lon() -> None:
    cmd = encode_set_roi_location(lat_deg=12.971, lon_deg=77.594, alt_m=50.0)
    assert cmd.command == MAV_CMD_DO_SET_ROI_LOCATION
    assert cmd.command == 195
    assert cmd.x == 129710000
    assert cmd.y == 775940000
    assert cmd.z == 50.0
    assert cmd.frame == 6  # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    assert cmd.target_component == 1


def test_set_roi_location_payload_carries_int32_lat_lon() -> None:
    cmd = encode_set_roi_location(lat_deg=-33.8688, lon_deg=151.2093, alt_m=80.0)
    layout = "<ffffiifHBBBBB"
    fields = struct.unpack(layout, cmd.payload)
    # x and y are at indices 4 and 5 (four floats then two int32s).
    assert fields[4] == -338688000
    assert fields[5] == 1512093000
    assert fields[6] == 80.0
    assert fields[7] == 195  # command id


def test_set_roi_none_takes_no_parameters() -> None:
    cmd = encode_set_roi_none()
    assert cmd.command == MAV_CMD_DO_SET_ROI_NONE
    assert cmd.command == 197
    assert cmd.param1 == 0.0
    assert cmd.param2 == 0.0
    assert cmd.param3 == 0.0
    assert cmd.param4 == 0.0
    assert cmd.param5 == 0.0
    assert cmd.param6 == 0.0
    assert cmd.param7 == 0.0


def test_default_target_component_is_autopilot_manager() -> None:
    cmd = encode_gimbal_manager_pitchyaw(pitch_deg=0, yaw_deg=0)
    assert cmd.target_component == 1
