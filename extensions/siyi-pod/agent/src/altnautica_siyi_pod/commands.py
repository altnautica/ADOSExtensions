"""SIYI SDK command set.

Each helper returns a :class:`Command` (a command id plus its DATA payload); the
session frames it with the next sequence number. Decoders take the DATA bytes of
a reply frame.

Every command id and DATA layout in this module is one published in the vendor
SDK document. Nothing here is a guess: a control whose wire format is not known
is not implemented at all, because an unrecognised opcode on embedded gimbal
firmware is not reliably a no-op — it can trip a fault state or collide with a
real vendor command on a pod carried by a flying aircraft.
"""

from __future__ import annotations

import struct
from typing import NamedTuple

# --- command ids ------------------------------------------------------------
CMD_FIRMWARE_VERSION = 0x01
CMD_HARDWARE_ID = 0x02
CMD_AUTOFOCUS = 0x04
CMD_MANUAL_ZOOM = 0x05
CMD_MANUAL_FOCUS = 0x06
CMD_GIMBAL_SPEED = 0x07
CMD_CENTER = 0x08
CMD_GIMBAL_INFO = 0x0A
CMD_FUNCTION_FEEDBACK = 0x0C  # photo / record / gimbal motion mode
CMD_GIMBAL_ATTITUDE = 0x0D
CMD_SET_GIMBAL_ATTITUDE = 0x0E
CMD_ABSOLUTE_ZOOM = 0x0F
CMD_LASER_RANGE = 0x15
CMD_CURRENT_ZOOM = 0x18
CMD_THERMAL_GAIN = 0x1A  # gain high / low (thermal models)
CMD_THERMAL_PALETTE = 0x1B  # palette select (thermal models)
CMD_ENCODING_INFO = 0x20
CMD_SET_ENCODING = 0x21
CMD_DATA_STREAM = 0x25

# Function-feedback (0x0C) sub-function codes.
FUNC_TAKE_PHOTO = 0x00
FUNC_RECORD_TOGGLE = 0x02
FUNC_MOTION_LOCK = 0x03
FUNC_MOTION_FOLLOW = 0x04
FUNC_MOTION_FPV = 0x05

# Manual-zoom directions.
ZOOM_OUT = -1
ZOOM_STOP = 0
ZOOM_IN = 1


_GIMBAL_MODE_FUNC = {
    "lock": FUNC_MOTION_LOCK,
    "follow": FUNC_MOTION_FOLLOW,
    "fpv": FUNC_MOTION_FPV,
}


class Command(NamedTuple):
    """A command id plus its DATA payload, ready for the session to frame."""

    cmd_id: int
    data: bytes = b""
    need_ack: bool = True


# --- requests (no payload) --------------------------------------------------
def request_firmware() -> Command:
    return Command(CMD_FIRMWARE_VERSION)


def request_hardware_id() -> Command:
    return Command(CMD_HARDWARE_ID)


def request_gimbal_attitude() -> Command:
    return Command(CMD_GIMBAL_ATTITUDE)


def request_gimbal_info() -> Command:
    return Command(CMD_GIMBAL_INFO)


def request_current_zoom() -> Command:
    return Command(CMD_CURRENT_ZOOM)


def request_laser_range() -> Command:
    return Command(CMD_LASER_RANGE)


def autofocus() -> Command:
    # DATA per SDK: 1 = trigger autofocus.
    return Command(CMD_AUTOFOCUS, b"\x01")


# --- gimbal control ---------------------------------------------------------
def gimbal_speed(yaw: int, pitch: int) -> Command:
    """Rate command, both axes in -100..100 (percent of max rate)."""
    yaw = max(-100, min(100, int(yaw)))
    pitch = max(-100, min(100, int(pitch)))
    return Command(CMD_GIMBAL_SPEED, struct.pack("<bb", yaw, pitch))


def set_gimbal_attitude(yaw_deg: float, pitch_deg: float) -> Command:
    """Absolute-angle command. Angles are transmitted x10 as int16."""
    yaw = int(round(yaw_deg * 10))
    pitch = int(round(pitch_deg * 10))
    return Command(CMD_SET_GIMBAL_ATTITUDE, struct.pack("<hh", yaw, pitch))


def center() -> Command:
    return Command(CMD_CENTER, b"\x01")


def set_gimbal_mode(mode: str) -> Command:
    """Motion mode: lock | follow | fpv (via the 0x0C function feedback)."""
    func = _GIMBAL_MODE_FUNC.get(mode)
    if func is None:
        raise ValueError(f"unknown gimbal mode: {mode!r}")
    return Command(CMD_FUNCTION_FEEDBACK, bytes([func]))


# --- camera control ---------------------------------------------------------
def manual_zoom(direction: int) -> Command:
    """Rocker zoom: -1 out, 0 stop, +1 in."""
    return Command(CMD_MANUAL_ZOOM, struct.pack("<b", int(direction)))


def absolute_zoom(zoom: float) -> Command:
    """Absolute zoom as integer + tenths (e.g. 4.5x -> int 4, frac 5)."""
    zoom = max(1.0, float(zoom))
    integer = int(zoom)
    frac = int(round((zoom - integer) * 10)) % 10
    return Command(CMD_ABSOLUTE_ZOOM, bytes([integer, frac]))


def take_photo() -> Command:
    return Command(CMD_FUNCTION_FEEDBACK, bytes([FUNC_TAKE_PHOTO]))


def record_toggle() -> Command:
    return Command(CMD_FUNCTION_FEEDBACK, bytes([FUNC_RECORD_TOGGLE]))


# --- thermal (thermal models only; gated by the capability profile) ---------
# Palette codes the pod accepts, 0 .. THERMAL_PALETTE_COUNT - 1.
THERMAL_PALETTE_COUNT = 8


def set_thermal_palette(palette_code: int) -> Command:
    """Select a thermal colour palette by code.

    The code is range-checked rather than masked into a byte: masking turns an
    out-of-range value into a different but structurally valid command that the
    pod executes, so a bad write silently selects the wrong palette instead of
    being refused.
    """
    code = int(palette_code)
    if not 0 <= code < THERMAL_PALETTE_COUNT:
        raise ValueError(
            f"thermal palette code must be 0..{THERMAL_PALETTE_COUNT - 1}, "
            f"got {palette_code!r}"
        )
    return Command(CMD_THERMAL_PALETTE, bytes([code]))


def set_thermal_gain(high: bool) -> Command:
    return Command(CMD_THERMAL_GAIN, bytes([0x01 if high else 0x00]))


# --- push streams -----------------------------------------------------------
def set_data_stream(stream_type: int, frequency_hz: int) -> Command:
    """Enable a push stream (e.g. attitude, laser) at a fixed rate."""
    kind = int(stream_type)
    hz = int(frequency_hz)
    if not 0 <= kind <= 0xFF:
        raise ValueError(f"stream type must be 0..255, got {stream_type!r}")
    if not 0 <= hz <= 0xFF:
        raise ValueError(f"stream frequency must be 0..255 Hz, got {frequency_hz!r}")
    return Command(CMD_DATA_STREAM, bytes([kind, hz]))


# --- decoders ---------------------------------------------------------------
class GimbalAttitude(NamedTuple):
    yaw_deg: float
    pitch_deg: float
    roll_deg: float
    yaw_rate: float
    pitch_rate: float
    roll_rate: float


def decode_gimbal_attitude(data: bytes) -> GimbalAttitude:
    """Decode a 0x0D reply: yaw/pitch/roll then their rates, each int16 x10."""
    if len(data) < 12:
        raise ValueError(f"gimbal attitude reply too short: {len(data)} bytes")
    yaw, pitch, roll, yv, pv, rv = struct.unpack_from("<hhhhhh", data, 0)
    return GimbalAttitude(
        yaw_deg=yaw / 10.0,
        pitch_deg=pitch / 10.0,
        roll_deg=roll / 10.0,
        yaw_rate=yv / 10.0,
        pitch_rate=pv / 10.0,
        roll_rate=rv / 10.0,
    )


def decode_laser_range(data: bytes) -> float:
    """Decode a 0x15 reply: uint16 range in decimetres -> metres."""
    if len(data) < 2:
        raise ValueError(f"laser range reply too short: {len(data)} bytes")
    (decimetres,) = struct.unpack_from("<H", data, 0)
    return decimetres / 10.0


def decode_current_zoom(data: bytes) -> float:
    """Decode a 0x18 reply: integer + tenths -> float zoom factor."""
    if len(data) < 2:
        raise ValueError(f"zoom reply too short: {len(data)} bytes")
    return data[0] + (data[1] % 10) / 10.0

