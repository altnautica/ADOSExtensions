"""MAVLink v2 frame builders and the subscription decoder for the follow loop.

The agent's MAVLink router accepts a fully-packed v2 frame on
``ctx.mavlink.send`` and re-stamps the link-level sequence and (when
enabled) signing on the outbound link. We build the frames with
pymavlink so the message layout and CRC-extra are exactly right, then
hand the packed bytes to the router.

One message is produced: ``COMMAND_LONG`` with
``MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW`` (1000) to point a gimbal at the
subject when one is present, sent with the onboard-computer component id.
The vehicle itself is flown by the agent's pilot (see ``follow_me.motion``).

Inbound, ``ctx.mavlink.subscribe`` delivers ``{msg_name, frame,
timestamp_ms}`` with ``frame`` the raw wire bytes; :func:`decode_fields`
turns one delivery into the message's fields.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any

from pymavlink.dialects.v20 import common as mavlink2

# Component id the companion sends as. MAV_COMP_ID_ONBOARD_COMPUTER (191):
# the autopilot treats a command from the onboard computer as an authorized
# command on the same vehicle system id.
ONBOARD_COMPUTER_COMP_ID = 191

# Gimbal-manager flags: retain the gimbal lock so a pitch/yaw command holds
# the boresight rather than following the body. 0 leaves the flags at the
# manager default, which is sufficient for point-at-target.
_GIMBAL_FLAGS_DEFAULT = 0

def _mav(system_id: int, component_id: int) -> mavlink2.MAVLink:
    """A stateless MAVLink encoder bound to a (system, component) source.

    No serial/socket file object is attached; the encoder is used purely
    to pack messages into bytes. The router owns the live link sequence,
    so the encoder's own sequence stays at the message default.
    """
    return mavlink2.MAVLink(None, srcSystem=system_id, srcComponent=component_id)


# Decoder for inbound deliveries. It holds no signing key, so a signed frame
# decodes without a signature check (the router already verified the link).
_DECODER = mavlink2.MAVLink(None)


def decode_fields(delivery: Mapping[str, Any]) -> dict[str, Any] | None:
    """The fields of the message in one ``ctx.mavlink.subscribe`` delivery.

    ``None`` when the frame is missing, does not decode (truncated, bad CRC,
    unknown id), or is not the message the delivery names, so a handler never
    acts on a frame it could not read.
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


def build_gimbal_pitchyaw(
    *,
    pitch_deg: float,
    yaw_deg: float,
    system_id: int = 1,
    component_id: int = ONBOARD_COMPUTER_COMP_ID,
    target_system: int = 1,
    target_component: int = 1,
) -> bytes:
    """Pack a COMMAND_LONG / DO_GIMBAL_MANAGER_PITCHYAW frame.

    ``pitch_deg`` is the gimbal pitch toward the subject (negative is
    downward in the gimbal convention). ``yaw_deg`` is the gimbal yaw
    relative to the vehicle heading. The rate params are sent as NaN so
    the manager treats the command as an absolute angle, not a rate.
    """
    mav = _mav(system_id, component_id)
    msg = mav.command_long_encode(
        target_system,
        target_component,
        mavlink2.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
        0,  # confirmation
        float(pitch_deg),
        float(yaw_deg),
        float("nan"),  # pitch rate (absolute angle, no rate)
        float("nan"),  # yaw rate (absolute angle, no rate)
        float(_GIMBAL_FLAGS_DEFAULT),
        0.0,
        0.0,  # gimbal device id 0 = manager default
    )
    return msg.pack(mav)


def gimbal_angles_for_target(
    *,
    slant_range_m: float,
    agl_m: float,
    bearing_rad: float,
    vehicle_yaw_rad: float,
) -> tuple[float, float]:
    """Pitch/yaw degrees that point a gimbal at the ground subject.

    Pitch is the depression angle from horizontal down to the subject
    (negative downward, matching the gimbal convention). Yaw is the
    difference between the line-of-sight ground bearing and the vehicle
    heading, wrapped to (-180, 180].
    """
    agl = max(0.0, min(agl_m, slant_range_m))
    # asin of (height / slant) is the depression below horizontal.
    ratio = 0.0 if slant_range_m <= 1e-6 else agl / slant_range_m
    depression = math.degrees(math.asin(max(0.0, min(1.0, ratio))))
    pitch_deg = -depression
    yaw_rel = bearing_rad - vehicle_yaw_rad
    # Wrap to the half-open interval (-pi, pi]. Python's modulo returns a
    # non-negative remainder, so ``yaw_rel % 2pi`` lands in [0, 2pi); a
    # value above pi folds down by 2pi, leaving exactly +pi (not -pi) at
    # the half-turn so the interval stays open at -pi.
    yaw_rel = yaw_rel % (2.0 * math.pi)
    if yaw_rel > math.pi:
        yaw_rel -= 2.0 * math.pi
    return (pitch_deg, math.degrees(yaw_rel))


__all__ = [
    "ONBOARD_COMPUTER_COMP_ID",
    "build_gimbal_pitchyaw",
    "decode_fields",
    "gimbal_angles_for_target",
]
