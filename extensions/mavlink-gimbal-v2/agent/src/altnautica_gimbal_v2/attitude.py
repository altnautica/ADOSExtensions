"""The gimbal attitude read-back: decode the gimbal device's status message and
shape the ``gimbal`` telemetry channel the GCS panel renders.

``ctx.mavlink.subscribe`` delivers ``{msg_name, frame, timestamp_ms}`` with
``frame`` the raw wire bytes, so every delivery is decoded here before a field
is read. ``GIMBAL_DEVICE_ATTITUDE_STATUS`` carries the attitude as a quaternion
and the angular velocity in rad/s; the panel wants degrees.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from pymavlink.dialects.v20 import common as mavlink2

ATTITUDE_STATUS_MSG = "GIMBAL_DEVICE_ATTITUDE_STATUS"

# Decoder for inbound deliveries. It holds no signing key, so a signed frame
# decodes without a signature check (the router already verified the link).
_DECODER = mavlink2.MAVLink(None)


def decode_fields(delivery: Mapping[str, Any]) -> dict[str, Any] | None:
    """The fields of the message in one ``ctx.mavlink.subscribe`` delivery.

    ``None`` when the frame is missing, does not decode (truncated, bad CRC,
    unknown id), or is not the message the delivery names.
    """
    frame = delivery.get("frame")
    if not isinstance(frame, (bytes, bytearray)) or not frame:
        return None
    try:
        msg = _DECODER.decode(bytearray(frame))
    except Exception:  # noqa: BLE001 - pymavlink raises MAVError and struct errors
        return None
    name = delivery.get("msg_name")
    if msg is None or (isinstance(name, str) and msg.get_type() != name):
        return None
    return msg.to_dict()


@dataclass(frozen=True)
class GimbalAttitude:
    """One decoded gimbal attitude, in degrees and degrees per second."""

    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    roll_rate_dps: float
    pitch_rate_dps: float
    yaw_rate_dps: float


def attitude_from_status(fields: Mapping[str, Any]) -> GimbalAttitude | None:
    """Euler angles from a decoded ``GIMBAL_DEVICE_ATTITUDE_STATUS``.

    ``None`` when the quaternion is not a finite, non-zero rotation. An unknown
    angular velocity is NaN on the wire and reads as 0.
    """
    q = fields.get("q")
    if not isinstance(q, (list, tuple)) or len(q) != 4:
        return None
    w, x, y, z = (float(v) for v in q)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isfinite(norm) or norm < 1e-6:
        return None
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def rate(key: str) -> float:
        value = float(fields.get(key, 0.0))
        return math.degrees(value) if math.isfinite(value) else 0.0

    return GimbalAttitude(
        roll_deg=math.degrees(roll),
        pitch_deg=math.degrees(pitch),
        yaw_deg=math.degrees(yaw),
        roll_rate_dps=rate("angular_velocity_x"),
        pitch_rate_dps=rate("angular_velocity_y"),
        yaw_rate_dps=rate("angular_velocity_z"),
    )


def telemetry_payload(att: GimbalAttitude, mode: str, timestamp_ms: int) -> dict[str, Any]:
    """The ``gimbal`` telemetry channel entry (the GCS panel's GimbalState)."""
    return {
        "timestampMs": timestamp_ms,
        "pitchDeg": att.pitch_deg,
        "yawDeg": att.yaw_deg,
        "rollDeg": att.roll_deg,
        "pitchRateDps": att.pitch_rate_dps,
        "yawRateDps": att.yaw_rate_dps,
        "rollRateDps": att.roll_rate_dps,
        "mode": mode,
    }
