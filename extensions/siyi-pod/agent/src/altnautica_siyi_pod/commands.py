"""SIYI SDK command set.

Each helper returns a :class:`Command` (a command id plus its DATA payload); the
session frames it with the next sequence number. Decoders take the DATA bytes of
a reply frame. The command ids are stable across the whole SIYI optical-pod line;
where a DATA layout is firmware-specific it is noted so it can be confirmed
against the SIYI SDK document and on a bench pod (Rule 25).
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
CMD_ENCODING_INFO = 0x20
CMD_SET_ENCODING = 0x21
CMD_DATA_STREAM = 0x25

# Image-source assignment (which sensor feeds the main/sub RTSP stream) and the
# on-pod split / picture-in-picture composite toggle.
#
# PLACEHOLDER opcodes + DATA layouts: the exact SIYI opcodes that assign a sensor
# to a stream and that toggle the on-pod split/PiP composite are SIYI SDK-PDF
# values we do NOT have. These placeholders make the command STRUCTURE and the
# control path exist and CI-covered via MockTransport; the real opcodes and DATA
# layout are resolved on the ZT30 bench against the SIYI SDK document before the
# on-rig gate (Rule 44 — never an invented wire value shipped as truth).
CMD_SET_IMAGE_SOURCE = 0xE1  # PLACEHOLDER — SDK PDF, resolve on the ZT30 bench
CMD_SET_SPLIT_MODE = 0xE2  # PLACEHOLDER — SDK PDF, resolve on the ZT30 bench
CMD_THERMAL_PALETTE = 0x1B  # palette select (thermal models); confirm on-rig
CMD_THERMAL_GAIN = 0x1A  # gain high/low (thermal models); confirm on-rig
CMD_THERMAL_TEMP_POINT = 0x14  # point temperature (thermal models)
CMD_AI_TRACK = 0x71  # AI-track designate / mode / output; confirm on-rig
CMD_FORMAT_SD = 0x48
CMD_REBOOT = 0x80

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

# Physical stream ids and image-source codes for CMD_SET_IMAGE_SOURCE. PLACEHOLDER
# values (see the CMD_SET_IMAGE_SOURCE note) — the real DATA layout is SDK-PDF,
# resolved on the ZT30 bench.
STREAM_MAIN = 0x00
STREAM_SUB = 0x01
IMG_SOURCE_EO_ZOOM = 0x00
IMG_SOURCE_EO_WIDE = 0x01
IMG_SOURCE_IR = 0x02
IMG_SOURCE_SPLIT = 0x03

_STREAM_CODES = {"main": STREAM_MAIN, "sub": STREAM_SUB}
_IMG_SOURCE_CODES = {
    "eo_zoom": IMG_SOURCE_EO_ZOOM,
    "eo_wide": IMG_SOURCE_EO_WIDE,
    "ir": IMG_SOURCE_IR,
    "split": IMG_SOURCE_SPLIT,
}

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


# --- image source / split composite (multi-sensor pods) ---------------------
def set_image_source(stream: str, source: str) -> Command:
    """Assign which sensor feeds a physical RTSP stream.

    ``stream`` is 'main' | 'sub'; ``source`` is eo_zoom | eo_wide | ir | split.
    PLACEHOLDER opcode + DATA layout (see the CMD_SET_IMAGE_SOURCE note) —
    resolve on the ZT30 bench.
    """
    stream_id = _STREAM_CODES[stream]
    source_id = _IMG_SOURCE_CODES[source]
    return Command(CMD_SET_IMAGE_SOURCE, bytes([stream_id, source_id]))


def set_split_mode(enabled: bool) -> Command:
    """Enable/disable the pod's on-pod split / PiP composite (two sensors in one
    stream). PLACEHOLDER opcode (see the CMD_SET_SPLIT_MODE note) — resolve on
    the ZT30 bench."""
    return Command(CMD_SET_SPLIT_MODE, bytes([0x01 if enabled else 0x00]))


# --- thermal (thermal models only; gated by the capability profile) ---------
def set_thermal_palette(palette_code: int) -> Command:
    return Command(CMD_THERMAL_PALETTE, bytes([palette_code & 0xFF]))


def set_thermal_gain(high: bool) -> Command:
    return Command(CMD_THERMAL_GAIN, bytes([0x01 if high else 0x00]))


# --- laser rangefinder (laser models only) ----------------------------------
def set_data_stream(stream_type: int, frequency_hz: int) -> Command:
    """Enable a push stream (e.g. attitude, laser) at a fixed rate."""
    return Command(CMD_DATA_STREAM, bytes([stream_type & 0xFF, frequency_hz & 0xFF]))


# --- AI tracking (ai-track models only) -------------------------------------
AI_TRACK_STOP_FLAG = 0x00
AI_TRACK_START_FLAG = 0x01


def ai_track_designate(x: int, y: int, width: int, height: int) -> Command:
    """Hand the pod a box (pod-frame pixels) to lock its on-pod tracker onto.

    The pod then tracks the subject and self-slews the gimbal. The exact
    coordinate convention (pixel box vs normalised centre) is confirmed on a
    bench pod against the SDK document (Rule 25).
    """
    return Command(
        CMD_AI_TRACK,
        struct.pack(
            "<BHHHH",
            AI_TRACK_START_FLAG,
            int(x) & 0xFFFF,
            int(y) & 0xFFFF,
            int(width) & 0xFFFF,
            int(height) & 0xFFFF,
        ),
    )


def ai_track_stop() -> Command:
    """Release the pod's AI tracker."""
    return Command(CMD_AI_TRACK, bytes([AI_TRACK_STOP_FLAG]))


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
