"""MAVLink v2 frame builders for the follow loop.

The agent's MAVLink router accepts a fully-packed v2 frame on
``ctx.mavlink.send`` and re-stamps the link-level sequence and (when
enabled) signing on the outbound link. We build the frames with
pymavlink so the message layout and CRC-extra are exactly right, then
hand the packed bytes to the router.

Two messages are produced:

* ``SET_POSITION_TARGET_GLOBAL_INT`` (id 86) for the guided follow
  setpoint. The type mask keeps position and yaw and ignores velocity,
  acceleration, and yaw-rate so the autopilot flies to the lat/lon/alt
  and faces the subject.
* ``COMMAND_LONG`` with ``MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW`` (1000) to
  point a gimbal at the subject when one is present.

The companion sends under the autopilot's own system id with the
peripheral component id so the autopilot accepts the guided setpoint as
an onboard command.
"""

from __future__ import annotations

import math

from pymavlink.dialects.v20 import common as mavlink2

# Component id the companion sends as. MAV_COMP_ID_ONBOARD_COMPUTER (191):
# the autopilot treats a guided setpoint from the onboard computer as an
# authorized command on the same vehicle system id.
ONBOARD_COMPUTER_COMP_ID = 191

# Gimbal-manager flags: retain the gimbal lock so a pitch/yaw command holds
# the boresight rather than following the body. 0 leaves the flags at the
# manager default, which is sufficient for point-at-target.
_GIMBAL_FLAGS_DEFAULT = 0

# Type mask for SET_POSITION_TARGET_GLOBAL_INT. Bits set = field IGNORED.
# We keep position (lat/lon/alt) and yaw; ignore the three velocity, the
# three acceleration, the force flag, and the yaw-rate bits.
_IGNORE_VX = 1 << 3
_IGNORE_VY = 1 << 4
_IGNORE_VZ = 1 << 5
_IGNORE_AFX = 1 << 6
_IGNORE_AFY = 1 << 7
_IGNORE_AFZ = 1 << 8
_IGNORE_YAW_RATE = 1 << 11
POSITION_YAW_TYPE_MASK = (
    _IGNORE_VX
    | _IGNORE_VY
    | _IGNORE_VZ
    | _IGNORE_AFX
    | _IGNORE_AFY
    | _IGNORE_AFZ
    | _IGNORE_YAW_RATE
)


def _mav(system_id: int, component_id: int) -> mavlink2.MAVLink:
    """A stateless MAVLink encoder bound to a (system, component) source.

    No serial/socket file object is attached; the encoder is used purely
    to pack messages into bytes. The router owns the live link sequence,
    so the encoder's own sequence stays at the message default.
    """
    return mavlink2.MAVLink(None, srcSystem=system_id, srcComponent=component_id)


def build_position_target(
    *,
    lat_deg: float,
    lon_deg: float,
    alt_rel_m: float,
    yaw_rad: float,
    system_id: int = 1,
    component_id: int = ONBOARD_COMPUTER_COMP_ID,
    target_system: int = 1,
    target_component: int = 1,
) -> bytes:
    """Pack a SET_POSITION_TARGET_GLOBAL_INT guided follow setpoint.

    ``lat_deg`` / ``lon_deg`` are scaled to the 1e7 integer
    representation the message carries. ``alt_rel_m`` is metres above the
    arming/home altitude (the frame is GLOBAL_RELATIVE_ALT_INT). ``yaw``
    faces the vehicle at the subject.
    """
    lat_int = int(round(lat_deg * 1e7))
    lon_int = int(round(lon_deg * 1e7))
    mav = _mav(system_id, component_id)
    msg = mav.set_position_target_global_int_encode(
        0,  # time_boot_ms (0 = unused, the FC timestamps on receipt)
        target_system,
        target_component,
        mavlink2.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        POSITION_YAW_TYPE_MASK,
        lat_int,
        lon_int,
        float(alt_rel_m),
        0.0,  # vx (ignored)
        0.0,  # vy (ignored)
        0.0,  # vz (ignored)
        0.0,  # afx (ignored)
        0.0,  # afy (ignored)
        0.0,  # afz (ignored)
        float(yaw_rad),
        0.0,  # yaw_rate (ignored)
    )
    return msg.pack(mav)


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
    "POSITION_YAW_TYPE_MASK",
    "build_position_target",
    "build_gimbal_pitchyaw",
    "gimbal_angles_for_target",
]
