"""Mirror the pod up to the flight controller over MAVLink (the interop path).

The plugin's own GCS half is the primary control surface, but registering the
pod as standard MAVLink components lets the built-in Mission Control gimbal panel,
the autopilot, and any third-party MAVLink ground station see it for free. This
module builds the frames the pod mirrors upward:

* ``GIMBAL_DEVICE_ATTITUDE_STATUS`` (284) under the gimbal component (154) so the
  gimbal panel shows live attitude;
* ``DISTANCE_SENSOR`` (132) under the same component so the laser range can reach
  the flight controller.

Frames are real MAVLink v2 built with pymavlink (provided by the host venv) and
sent through ``ctx.mavlink.send`` under a component the plugin has registered.
"""

from __future__ import annotations

import logging
import math
import time

log = logging.getLogger(__name__)

# MAVLink component ids (MAVLink common.xml, stable across dialects).
COMP_GIMBAL = 154
COMP_CAMERA = 100

# DISTANCE_SENSOR type + orientation enums.
_MAV_DISTANCE_SENSOR_LASER = 3
_MAV_SENSOR_ROTATION_PITCH_270 = 25  # facing straight down (nadir default)


def euler_to_quaternion(
    roll_deg: float, pitch_deg: float, yaw_deg: float
) -> list[float]:
    """Aircraft-convention euler (deg) to a ``[w, x, y, z]`` quaternion."""
    roll = math.radians(roll_deg) / 2.0
    pitch = math.radians(pitch_deg) / 2.0
    yaw = math.radians(yaw_deg) / 2.0
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


class SiyiMavlinkBridge:
    """Builds and sends the pod's MAVLink mirror frames."""

    def __init__(self, ctx, *, system_id: int = 1) -> None:
        self._ctx = ctx
        self._system_id = system_id
        self._boot = time.monotonic()
        # One MAVLink packer per source component (srcComponent stamps the frame).
        from pymavlink.dialects.v20 import common as mavlink2

        self._mav = mavlink2
        self._gimbal_mav = mavlink2.MAVLink(
            None, srcSystem=system_id, srcComponent=COMP_GIMBAL
        )
        self._gimbal_mav.robust_parsing = True

    def _time_boot_ms(self) -> int:
        return int((time.monotonic() - self._boot) * 1000) & 0xFFFFFFFF

    def build_attitude_frame(
        self, yaw_deg: float, pitch_deg: float, roll_deg: float
    ) -> bytes:
        q = euler_to_quaternion(roll_deg, pitch_deg, yaw_deg)
        msg = self._gimbal_mav.gimbal_device_attitude_status_encode(
            0,  # target_system
            0,  # target_component
            self._time_boot_ms(),
            0,  # flags
            q,
            0.0,  # angular_velocity_x
            0.0,  # angular_velocity_y
            0.0,  # angular_velocity_z
            0,  # failure_flags
        )
        return msg.pack(self._gimbal_mav)

    def build_distance_frame(
        self, range_m: float, *, orientation: int = _MAV_SENSOR_ROTATION_PITCH_270
    ) -> bytes:
        # DISTANCE_SENSOR distances are uint16 centimetres, so the field
        # saturates at 655.35 m. A pod range beyond that clamps here; the true
        # metric range always rides the pod telemetry (siyi.pod.state), which is
        # not field-width bound.
        cm = max(0, int(round(range_m * 100)))
        msg = self._gimbal_mav.distance_sensor_encode(
            self._time_boot_ms(),
            50,  # min_distance, 0.5 m rangefinder floor
            0xFFFF,  # max_distance, the field ceiling (655.35 m)
            min(cm, 0xFFFF),  # current_distance (cm), clamped to the field
            _MAV_DISTANCE_SENSOR_LASER,
            0,  # id
            orientation,
            0,  # covariance
        )
        return msg.pack(self._gimbal_mav)

    async def send_attitude(
        self, yaw_deg: float, pitch_deg: float, roll_deg: float
    ) -> None:
        try:
            frame = self.build_attitude_frame(yaw_deg, pitch_deg, roll_deg)
            await self._ctx.mavlink.send(frame, component_id=COMP_GIMBAL)
        except Exception:  # noqa: BLE001
            log.debug("gimbal attitude mirror failed", exc_info=True)

    async def send_distance(self, range_m: float) -> None:
        try:
            frame = self.build_distance_frame(range_m)
            await self._ctx.mavlink.send(frame, component_id=COMP_GIMBAL)
        except Exception:  # noqa: BLE001
            log.debug("distance sensor mirror failed", exc_info=True)
