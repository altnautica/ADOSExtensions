"""MAVLink command payload encoders for the gimbal manager.

Pure-Python encoders for the four commands the driver issues. The
agent's MAVLink router handles framing, sequence numbers, signing, and
the system + component routing fields. This module returns the raw
seven-argument payload bytes a router needs to wrap in either a
``COMMAND_LONG`` frame or a ``COMMAND_INT`` frame.

We avoid importing the full MAVLink dialect to keep the plugin import
cost low; the four commands are simple enough that a struct.pack call
each suffices.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Final

# MAVLink command identifiers as published in the MAVLink common.xml
# message set. Values are stable across dialects.
MAV_CMD_DO_SET_ROI_LOCATION: Final[int] = 195
MAV_CMD_DO_SET_ROI_NONE: Final[int] = 197
MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW: Final[int] = 1000
MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE: Final[int] = 1001

# COMMAND_LONG payload: seven floats followed by command and two bytes
# of confirmation/target metadata. We encode the seven floats plus the
# u16 command id; the router prepends target sys/comp.
_COMMAND_LONG_PAYLOAD: Final[str] = "<fffffffHBBB"

# COMMAND_INT payload: four float params (1..4), then param5 and param6
# as int32 (lat/lon scaled by 1e7), then param7 as float for altitude.
# Followed by command id (uint16), target system (uint8), target
# component (uint8), frame (uint8), current (uint8), and autocontinue
# (uint8). Total 32 bytes.
_COMMAND_INT_PAYLOAD: Final[str] = "<ffffiifHBBBBB"


@dataclass(frozen=True)
class CommandLong:
    """A decoded ``COMMAND_LONG`` payload ready for the router.

    The router takes ``payload`` plus ``target_system`` and
    ``target_component`` to compose the on-the-wire frame. Confirmation
    is always 0 for fresh commands; the router increments it for retries.
    """

    command: int
    target_system: int
    target_component: int
    param1: float
    param2: float
    param3: float
    param4: float
    param5: float
    param6: float
    param7: float
    payload: bytes


@dataclass(frozen=True)
class CommandInt:
    """A decoded ``COMMAND_INT`` payload ready for the router.

    Lat and lon are signed int32 in the MAVLink spec, scaled by 1e7. The
    helpers below scale floating-point degrees automatically so plugin
    code works with familiar units.
    """

    command: int
    target_system: int
    target_component: int
    frame: int
    param1: float
    param2: float
    param3: float
    param4: float
    x: int
    y: int
    z: float
    payload: bytes


def encode_gimbal_manager_pitchyaw(
    pitch_deg: float,
    yaw_deg: float,
    pitch_rate_dps: float = 0.0,
    yaw_rate_dps: float = 0.0,
    flags: int = 0,
    gimbal_device_id: int = 0,
    target_system: int = 1,
    target_component: int = 1,
) -> CommandLong:
    """Compose ``MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW`` (1000).

    Pitch and yaw are degrees in the gimbal manager's reference frame.
    By default the manager treats yaw as absolute (earth-frame, north
    referenced); ``flags`` toggles body-relative or pitch/yaw-lock
    behaviour. Rates are degrees per second and are zero for
    pure-position commands. Gimbal device id 0 means "the primary
    gimbal". ``target_component`` defaults to 1 (the autopilot's
    built-in gimbal manager); override after discovering an alternate
    manager via ``GIMBAL_MANAGER_INFORMATION``.

    Param slots (per MAVLink common.xml): p1 pitch, p2 yaw, p3 pitch
    rate, p4 yaw rate, p5 flags, p6 reserved (0), p7 gimbal_device_id.
    """

    return _build_long(
        MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
        target_system,
        target_component,
        pitch_deg,
        yaw_deg,
        pitch_rate_dps,
        yaw_rate_dps,
        float(flags),
        0.0,
        float(gimbal_device_id),
    )


def encode_gimbal_manager_configure(
    primary_sysid: int,
    primary_compid: int,
    secondary_sysid: int = 0,
    secondary_compid: int = 0,
    gimbal_device_id: int = 0,
    target_system: int = 1,
    target_component: int = 1,
) -> CommandLong:
    """Compose ``MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE`` (1001).

    Assigns which component holds primary or secondary control of the
    gimbal. The default arguments leave secondary unassigned.
    ``target_component`` defaults to 1 (the autopilot's built-in
    gimbal manager).

    Param slots (per MAVLink common.xml): p1 sysid_primary, p2
    compid_primary, p3 sysid_secondary, p4 compid_secondary, p5
    reserved (0), p6 reserved (0), p7 gimbal_device_id.
    """

    return _build_long(
        MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
        target_system,
        target_component,
        float(primary_sysid),
        float(primary_compid),
        float(secondary_sysid),
        float(secondary_compid),
        0.0,
        0.0,
        float(gimbal_device_id),
    )


def encode_set_roi_location(
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    target_system: int = 1,
    target_component: int = 1,
    frame: int = 6,
    gimbal_device_id: int = 0,
) -> CommandInt:
    """Compose ``MAV_CMD_DO_SET_ROI_LOCATION`` (195) as ``COMMAND_INT``.

    ``lat_deg`` and ``lon_deg`` are converted from degrees to the
    int32 1e7-scaled form on the wire. ``alt_m`` is metres. The frame
    defaults to ``MAV_FRAME_GLOBAL_RELATIVE_ALT`` (3); operators with
    AGL targets do not need to override it.
    """

    lat_scaled = int(round(lat_deg * 1e7))
    lon_scaled = int(round(lon_deg * 1e7))
    payload = struct.pack(
        _COMMAND_INT_PAYLOAD,
        float(gimbal_device_id),
        0.0,
        0.0,
        0.0,
        lat_scaled,
        lon_scaled,
        float(alt_m),
        MAV_CMD_DO_SET_ROI_LOCATION,
        target_system,
        target_component,
        frame,
        0,
        0,
    )
    return CommandInt(
        command=MAV_CMD_DO_SET_ROI_LOCATION,
        target_system=target_system,
        target_component=target_component,
        frame=frame,
        param1=float(gimbal_device_id),
        param2=0.0,
        param3=0.0,
        param4=0.0,
        x=lat_scaled,
        y=lon_scaled,
        z=float(alt_m),
        payload=payload,
    )


def encode_set_roi_none(
    target_system: int = 1,
    target_component: int = 1,
) -> CommandLong:
    """Compose ``MAV_CMD_DO_SET_ROI_NONE`` (197).

    A bare release. Per MAVLink common.xml the command takes no
    parameters; all seven payload slots are zero.
    """

    return _build_long(
        MAV_CMD_DO_SET_ROI_NONE,
        target_system,
        target_component,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def _build_long(
    command: int,
    target_system: int,
    target_component: int,
    p1: float,
    p2: float,
    p3: float,
    p4: float,
    p5: float,
    p6: float,
    p7: float,
) -> CommandLong:
    payload = struct.pack(
        _COMMAND_LONG_PAYLOAD,
        p1,
        p2,
        p3,
        p4,
        p5,
        p6,
        p7,
        command,
        target_system,
        target_component,
        0,
    )
    return CommandLong(
        command=command,
        target_system=target_system,
        target_component=target_component,
        param1=p1,
        param2=p2,
        param3=p3,
        param4=p4,
        param5=p5,
        param6=p6,
        param7=p7,
        payload=payload,
    )
