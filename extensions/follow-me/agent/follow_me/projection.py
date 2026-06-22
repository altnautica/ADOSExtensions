"""Image-to-world projection and follow-setpoint geometry.

Pure math, no IPC. Given the pixel center of the tracked subject's
bounding box, the camera intrinsics (derived from a horizontal field of
view), the vehicle attitude, an optional gimbal pointing, and the
above-ground-level height, this resolves the subject's ground position
and then the standoff follow setpoint the vehicle should fly to.

The pipeline:

1. A pinhole model turns the bbox center pixel into a unit ray in the
   camera optical frame.
2. The ray is rotated by the camera mount, then the gimbal (if the
   camera is gimbal-pointed), then the vehicle body-to-NED attitude,
   landing in the local NED frame.
3. The NED ray is intersected with the flat ground plane a height
   ``agl_m`` below the vehicle, yielding the north/east offset of the
   subject from the vehicle.
4. That offset is converted to a subject latitude/longitude using the
   equirectangular small-offset approximation.
5. The follow setpoint is a point ``follow_distance_m`` short of the
   subject along the vehicle->subject bearing, at ``follow_height_m`` AGL.

All angles are radians internally; the public ``project_follow_setpoint``
accepts attitude in radians (matching the MAVLink ATTITUDE message) and
gimbal angles in degrees (matching the gimbal command convention).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# WGS-84-ish metres-per-degree of latitude. Good to a fraction of a
# percent for the short standoff offsets a follow loop ever produces.
_M_PER_DEG_LAT = 111_320.0


@dataclass(frozen=True)
class CameraIntrinsics:
    """Focal lengths in pixels for a frame, derived from a field of view."""

    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_fov(
        cls,
        frame_width: int,
        frame_height: int,
        horizontal_fov_deg: float,
    ) -> "CameraIntrinsics":
        """Build intrinsics from the horizontal FOV and frame size.

        ``fx`` follows directly from the horizontal FOV. ``fy`` is set so
        the vertical FOV matches the same angular pixel density (square
        pixels), which is the correct assumption for a UVC sensor whose
        only published spec is the horizontal angle of view.
        """
        w = float(max(1, frame_width))
        h = float(max(1, frame_height))
        hfov = math.radians(_clamp(horizontal_fov_deg, 1.0, 179.0))
        fx = (w / 2.0) / math.tan(hfov / 2.0)
        # Square pixels: the same focal length in px applies vertically.
        fy = fx
        return cls(fx=fx, fy=fy, cx=w / 2.0, cy=h / 2.0)


@dataclass(frozen=True)
class GroundTarget:
    """The resolved subject position on the ground."""

    lat_deg: float
    lon_deg: float
    offset_north_m: float
    offset_east_m: float
    slant_range_m: float
    ground_range_m: float
    bearing_rad: float


@dataclass(frozen=True)
class FollowSetpoint:
    """The position the vehicle should fly to follow the subject."""

    lat_deg: float
    lon_deg: float
    alt_rel_m: float
    yaw_rad: float
    target: GroundTarget


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def _normalize3(v: tuple[float, float, float]) -> tuple[float, float, float]:
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if n <= 1e-12:
        return (0.0, 0.0, 1.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def _rot_x(a: float) -> tuple[tuple[float, float, float], ...]:
    c, s = math.cos(a), math.sin(a)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def _rot_y(a: float) -> tuple[tuple[float, float, float], ...]:
    c, s = math.cos(a), math.sin(a)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _rot_z(a: float) -> tuple[tuple[float, float, float], ...]:
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _matvec(
    m: tuple[tuple[float, float, float], ...],
    v: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def bbox_center(
    bbox_x: float, bbox_y: float, bbox_w: float, bbox_h: float
) -> tuple[float, float]:
    """Pixel center of a top-left-origin bounding box."""
    return (bbox_x + bbox_w / 2.0, bbox_y + bbox_h / 2.0)


def pixel_to_camera_ray(
    px: float, py: float, intr: CameraIntrinsics
) -> tuple[float, float, float]:
    """Unit ray in the camera optical frame for a pixel.

    Optical-frame convention: +x right, +y down, +z forward (out of the
    lens). The ray is normalized so the ground-plane intersection scales
    cleanly with the AGL height.
    """
    x = (px - intr.cx) / intr.fx
    y = (py - intr.cy) / intr.fy
    return _normalize3((x, y, 1.0))


def camera_ray_to_ned(
    ray_cam: tuple[float, float, float],
    *,
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
    mount_pitch_rad: float,
    gimbal_pitch_rad: float,
    gimbal_yaw_rad: float,
) -> tuple[float, float, float]:
    """Rotate a camera-optical-frame ray into local NED.

    Steps, applied to the ray in order:

    * Camera-optical -> body-FRD: the optical frame (x-right, y-down,
      z-forward) maps to the aircraft forward-right-down body frame by
      sending optical +z (forward) to body +x (forward), optical +x
      (right) to body +y (right), optical +y (down) to body +z (down).
    * Gimbal pointing (when the camera is gimbal-stabilized) and the
      fixed mount pitch are applied in the body frame: a downward mount
      pitch tilts the boresight below the nose.
    * Body-FRD -> NED via the aircraft yaw/pitch/roll (ZYX) attitude.
    """
    # Optical (x-right, y-down, z-forward) -> body FRD (x-fwd, y-right,
    # z-down).
    cx, cy, cz = ray_cam
    body_ray = (cz, cx, cy)

    # Apply gimbal + mount pitch/yaw in the body frame. A positive pitch
    # tilts the boresight downward (toward +z body), a positive yaw turns
    # it to the right (toward +y body). Mount pitch is a fixed offset.
    #
    # ``_rot_y(a)`` sends body-forward (+x) to ``(cos a, 0, -sin a)``; a
    # downward tilt must send +x toward body-down (+z), which needs a
    # negative rotation angle, so the pitch offset is negated here.
    pitch_off = mount_pitch_rad + gimbal_pitch_rad
    if pitch_off != 0.0:
        body_ray = _matvec(_rot_y(-pitch_off), body_ray)
    if gimbal_yaw_rad != 0.0:
        body_ray = _matvec(_rot_z(gimbal_yaw_rad), body_ray)

    # Body FRD -> NED: R = Rz(yaw) Ry(pitch) Rx(roll).
    r = _matmul(
        _rot_z(yaw_rad), _matmul(_rot_y(pitch_rad), _rot_x(roll_rad))
    )
    return _matvec(r, body_ray)


def _matmul(
    a: tuple[tuple[float, float, float], ...],
    b: tuple[tuple[float, float, float], ...],
) -> tuple[tuple[float, float, float], ...]:
    return tuple(
        tuple(
            a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
            for j in range(3)
        )
        for i in range(3)
    )


def ground_intersection(
    ray_ned: tuple[float, float, float],
    *,
    vehicle_lat_deg: float,
    vehicle_lon_deg: float,
    agl_m: float,
) -> GroundTarget | None:
    """Intersect a NED ray from the vehicle with the flat ground plane.

    The ground plane sits ``agl_m`` below the vehicle (NED down is +z).
    Returns ``None`` when the ray does not point downward (the subject is
    at or above the horizon, so no ground hit exists) or the AGL is not
    positive.
    """
    down = ray_ned[2]
    if agl_m <= 0.0 or down <= 1e-4:
        return None
    t = agl_m / down
    offset_n = t * ray_ned[0]
    offset_e = t * ray_ned[1]
    slant = t  # ray is unit length, so the parameter t is the slant range
    ground_range = math.hypot(offset_n, offset_e)
    bearing = math.atan2(offset_e, offset_n)

    lat = vehicle_lat_deg + offset_n / _M_PER_DEG_LAT
    lon = vehicle_lon_deg + offset_e / (
        _M_PER_DEG_LAT * max(1e-6, math.cos(math.radians(vehicle_lat_deg)))
    )
    return GroundTarget(
        lat_deg=lat,
        lon_deg=lon,
        offset_north_m=offset_n,
        offset_east_m=offset_e,
        slant_range_m=slant,
        ground_range_m=ground_range,
        bearing_rad=bearing,
    )


def follow_setpoint_from_target(
    target: GroundTarget,
    *,
    vehicle_lat_deg: float,
    vehicle_lon_deg: float,
    follow_distance_m: float,
    follow_height_m: float,
) -> FollowSetpoint:
    """Standoff follow point short of the subject along the line of sight.

    The setpoint sits ``follow_distance_m`` back from the subject along
    the vehicle->subject ground bearing, so the vehicle trails the
    subject at a fixed distance. When the subject is already inside the
    standoff distance the setpoint collapses onto the subject (the
    vehicle holds rather than backing up through it). Yaw points the nose
    at the subject so a forward-mounted camera keeps it framed.
    """
    bearing = target.bearing_rad
    standoff = min(follow_distance_m, target.ground_range_m)
    set_n = target.offset_north_m - standoff * math.cos(bearing)
    set_e = target.offset_east_m - standoff * math.sin(bearing)

    lat = vehicle_lat_deg + set_n / _M_PER_DEG_LAT
    lon = vehicle_lon_deg + set_e / (
        _M_PER_DEG_LAT * max(1e-6, math.cos(math.radians(vehicle_lat_deg)))
    )
    return FollowSetpoint(
        lat_deg=lat,
        lon_deg=lon,
        alt_rel_m=max(0.0, follow_height_m),
        yaw_rad=bearing,
        target=target,
    )


def project_follow_setpoint(
    *,
    bbox_x: float,
    bbox_y: float,
    bbox_w: float,
    bbox_h: float,
    frame_width: int,
    frame_height: int,
    horizontal_fov_deg: float,
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
    vehicle_lat_deg: float,
    vehicle_lon_deg: float,
    agl_m: float,
    follow_distance_m: float,
    follow_height_m: float,
    mount_pitch_deg: float = 0.0,
    gimbal_pitch_deg: float = 0.0,
    gimbal_yaw_deg: float = 0.0,
) -> FollowSetpoint | None:
    """Full pixel-to-follow-setpoint pipeline. ``None`` if no ground hit.

    ``mount_pitch_deg`` is the fixed downward tilt of a non-gimbal camera
    (e.g. a forward camera angled 20 deg below the horizon). When the
    camera is gimbal-pointed, ``gimbal_pitch_deg`` / ``gimbal_yaw_deg``
    carry the live gimbal angles instead.
    """
    intr = CameraIntrinsics.from_fov(
        frame_width, frame_height, horizontal_fov_deg
    )
    px, py = bbox_center(bbox_x, bbox_y, bbox_w, bbox_h)
    ray_cam = pixel_to_camera_ray(px, py, intr)
    ray_ned = camera_ray_to_ned(
        ray_cam,
        roll_rad=roll_rad,
        pitch_rad=pitch_rad,
        yaw_rad=yaw_rad,
        mount_pitch_rad=math.radians(mount_pitch_deg),
        gimbal_pitch_rad=math.radians(gimbal_pitch_deg),
        gimbal_yaw_rad=math.radians(gimbal_yaw_deg),
    )
    target = ground_intersection(
        ray_ned,
        vehicle_lat_deg=vehicle_lat_deg,
        vehicle_lon_deg=vehicle_lon_deg,
        agl_m=agl_m,
    )
    if target is None:
        return None
    return follow_setpoint_from_target(
        target,
        vehicle_lat_deg=vehicle_lat_deg,
        vehicle_lon_deg=vehicle_lon_deg,
        follow_distance_m=follow_distance_m,
        follow_height_m=follow_height_m,
    )


__all__ = [
    "CameraIntrinsics",
    "GroundTarget",
    "FollowSetpoint",
    "bbox_center",
    "pixel_to_camera_ray",
    "camera_ray_to_ned",
    "ground_intersection",
    "follow_setpoint_from_target",
    "project_follow_setpoint",
]
