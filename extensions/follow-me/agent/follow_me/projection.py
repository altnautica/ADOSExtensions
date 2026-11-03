"""Image-to-world projection and range estimation for the follow loop.

Pure math, no IPC. Given the tracked subject's bounding box, the camera
intrinsics (derived from a horizontal field of view), the vehicle attitude,
the camera mount or gimbal angles and the height above ground, this resolves
where the subject stands relative to the vehicle in the flight controller's
local NED frame, with an honest range uncertainty.

The pipeline:

1. A pinhole model turns the box **bottom-centre** (the subject's feet, the
   point that actually touches the ground) into a unit ray in the camera
   optical frame.
2. The ray is rotated by the camera mount, then the gimbal (if the camera is
   gimbal-pointed), then the vehicle body-to-NED attitude.
3. Up to three independent range cues are formed:

   * ``ground``: the ray intersected with flat ground ``agl_m`` below the
     vehicle. Its error grows with the height error over the tangent of the
     depression angle, so a rangefinder height is trusted far more than a
     barometric one.
   * ``box_height``: an adult subject is about 1.70 m tall, so the slant
     range is ``f_y * 1.70 / h_px``. Only used when the box touches no frame
     edge (a clipped box is shorter than the subject).
   * ``lidar``: the planar scan return at the subject's bearing, accepted only
     when it agrees with the other cues within 25 % (a planar scanner at
     follow height usually sees the scenery behind the subject, not the
     subject).

4. The cues are fused by inverse variance. The fused horizontal range and the
   feet-ray bearing give the subject's north/east offset from the vehicle.

All angles are radians internally; attitude is radians (as in the MAVLink
ATTITUDE message) and mount/gimbal angles are degrees at the public edges.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from follow_me.ranging import (
    RangeCue,
    Scan,
    fuse,
    lidar_range_at,
    range_bearing_covariance,
)

# Nominal adult standing height, metres. The box-height range cue and the
# predicted box drawn while coasting both use it.
SUBJECT_HEIGHT_M = 1.70
# Width-to-height ratio of a standing person's box, for the predicted box.
SUBJECT_ASPECT = 0.41

# The rangefinder is the AGL source only while the vehicle tilt is below this:
# past it the beam no longer measures height straight down.
_MAX_RANGEFINDER_TILT_RAD = math.radians(30.0)
# Height-error sigma of each AGL source, metres. A downward ToF rangefinder is
# good to a few centimetres; the barometric height drifts and does not follow
# the terrain at all.
_AGL_SIGMA_RNG_M = 0.3
_AGL_SIGMA_BARO_M = 1.5
# Proportional floor on the ground cue: pixel quantisation, lens distortion
# and attitude error grow with range.
_GROUND_SIGMA_FRAC = 0.05
_BOX_SIGMA_FRAC = 0.15
_LIDAR_SIGMA_M = 0.05
# A lidar return is accepted only within this fraction of the other cues.
_LIDAR_GATE_FRAC = 0.25
# Pixels a box edge must clear to count as not touching the frame edge.
_EDGE_MARGIN_PX = 2.0
# Angular error of the bearing beyond pixel quantisation: attitude and mount
# calibration.
_BEARING_SIGMA_RAD = 0.02
# Cross-range floor, metres, so a subject almost under the vehicle still gets
# a usable measurement covariance.
_MIN_CROSS_SIGMA_M = 0.1

AGL_RNG = "rng"
AGL_BARO = "baro"

CUE_GROUND = "ground"
CUE_BOX_HEIGHT = "box_height"
CUE_LIDAR = "lidar"


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
        return cls(fx=fx, fy=fx, cx=w / 2.0, cy=h / 2.0)


@dataclass(frozen=True)
class CameraPose:
    """Everything that orients the camera relative to local NED."""

    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    mount_pitch_deg: float = 0.0
    gimbal_pitch_deg: float = 0.0
    gimbal_yaw_deg: float = 0.0


@dataclass(frozen=True)
class TargetFix:
    """The subject's position relative to the vehicle, from one box."""

    offset_north_m: float
    offset_east_m: float
    range_m: float
    range_sigma_m: float
    bearing_rad: float
    depression_rad: float
    cues: tuple[RangeCue, ...]
    agl_m: float
    agl_source: str
    cross_sigma_m: float

    def covariance(self) -> tuple[tuple[float, float], tuple[float, float]]:
        """2x2 north/east measurement covariance: the range sigma along the
        line of sight, the bearing sigma across it."""
        return range_bearing_covariance(
            self.bearing_rad, self.range_sigma_m, self.cross_sigma_m
        )


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def _normalize3(v: tuple[float, float, float]) -> tuple[float, float, float]:
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if n <= 1e-12:
        return (0.0, 0.0, 1.0)
    return (v[0] / n, v[1] / n, v[2] / n)


Mat3 = tuple[tuple[float, float, float], ...]


def _rot_x(a: float) -> Mat3:
    c, s = math.cos(a), math.sin(a)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def _rot_y(a: float) -> Mat3:
    c, s = math.cos(a), math.sin(a)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _rot_z(a: float) -> Mat3:
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _transpose(m: Mat3) -> Mat3:
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))  # type: ignore[return-value]


def _matvec(m: Mat3, v: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def _matmul(a: Mat3, b: Mat3) -> Mat3:
    return tuple(  # type: ignore[return-value]
        tuple(
            a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
            for j in range(3)
        )
        for i in range(3)
    )


def bbox_feet(
    bbox_x: float, bbox_y: float, bbox_w: float, bbox_h: float
) -> tuple[float, float]:
    """Pixel of the box bottom-centre: where a standing subject meets the
    ground. The box centre sits about 0.85 m above the ground and projects
    long, past the subject, by ``0.85 / tan(depression)``."""
    return (bbox_x + bbox_w / 2.0, bbox_y + bbox_h)


def pixel_to_camera_ray(
    px: float, py: float, intr: CameraIntrinsics
) -> tuple[float, float, float]:
    """Unit ray in the camera optical frame for a pixel.

    Optical-frame convention: +x right, +y down, +z forward (out of the
    lens).
    """
    x = (px - intr.cx) / intr.fx
    y = (py - intr.cy) / intr.fy
    return _normalize3((x, y, 1.0))


def _body_to_ned(cam: CameraPose) -> Mat3:
    # R = Rz(yaw) Ry(pitch) Rx(roll), the ZYX aerospace convention.
    return _matmul(
        _rot_z(cam.yaw_rad), _matmul(_rot_y(cam.pitch_rad), _rot_x(cam.roll_rad))
    )


def camera_ray_to_ned(
    ray_cam: tuple[float, float, float], cam: CameraPose
) -> tuple[float, float, float]:
    """Rotate a camera-optical-frame ray into local NED.

    * Optical (x-right, y-down, z-forward) -> body FRD (x-fwd, y-right,
      z-down).
    * Mount + gimbal pitch (positive = boresight below the nose) and gimbal
      yaw (positive = right of the nose) in the body frame. ``_rot_y(a)``
      sends body +x to ``(cos a, 0, -sin a)``; a downward tilt must send +x
      toward body-down (+z), so the pitch offset is negated.
    * Body FRD -> NED via the vehicle attitude.
    """
    cx, cy, cz = ray_cam
    body = (cz, cx, cy)
    pitch_off = math.radians(cam.mount_pitch_deg + cam.gimbal_pitch_deg)
    if pitch_off != 0.0:
        body = _matvec(_rot_y(-pitch_off), body)
    if cam.gimbal_yaw_deg != 0.0:
        body = _matvec(_rot_z(math.radians(cam.gimbal_yaw_deg)), body)
    return _matvec(_body_to_ned(cam), body)


def ned_to_pixel(
    vec_ned: tuple[float, float, float],
    cam: CameraPose,
    intr: CameraIntrinsics,
) -> tuple[float, float] | None:
    """The inverse of :func:`camera_ray_to_ned` followed by the pinhole:
    the pixel a NED offset from the camera lands on, or ``None`` when the
    point is behind the lens."""
    body = _matvec(_transpose(_body_to_ned(cam)), vec_ned)
    if cam.gimbal_yaw_deg != 0.0:
        body = _matvec(_rot_z(-math.radians(cam.gimbal_yaw_deg)), body)
    pitch_off = math.radians(cam.mount_pitch_deg + cam.gimbal_pitch_deg)
    if pitch_off != 0.0:
        body = _matvec(_rot_y(pitch_off), body)
    ox, oy, oz = body[1], body[2], body[0]
    if oz <= 1e-6:
        return None
    return (intr.cx + intr.fx * ox / oz, intr.cy + intr.fy * oy / oz)


def select_agl(
    *,
    rangefinder_usable: bool,
    rangefinder_m: float,
    roll_rad: float,
    pitch_rad: float,
    baro_agl_m: float,
) -> tuple[float, str]:
    """The height above ground the ground cue projects through, and where it
    came from.

    The downward rangefinder wins while it is current, inside its own valid
    band (``rangefinder_usable``) and the vehicle is tilted less than 30
    degrees; the beam then measures the slant to the ground, so it is
    corrected by the tilt. Otherwise the barometric height is used.
    """
    tilt = math.acos(_clamp(math.cos(roll_rad) * math.cos(pitch_rad), -1.0, 1.0))
    if rangefinder_usable and tilt < _MAX_RANGEFINDER_TILT_RAD:
        return (rangefinder_m * math.cos(tilt), AGL_RNG)
    return (baro_agl_m, AGL_BARO)


def ground_cue(
    ray_ned: tuple[float, float, float], agl_m: float, agl_source: str
) -> RangeCue | None:
    """Horizontal range where the feet ray meets flat ground ``agl_m`` below.

    ``None`` when the ray does not point down or there is no height. The
    error is ``0.05 r`` plus the AGL error over ``tan(depression)``.
    """
    down = ray_ned[2]
    horiz = math.hypot(ray_ned[0], ray_ned[1])
    if agl_m <= 0.0 or down <= 1e-4:
        return None
    r = agl_m * horiz / down
    tan_dep = down / max(horiz, 1e-6)
    agl_sigma = _AGL_SIGMA_RNG_M if agl_source == AGL_RNG else _AGL_SIGMA_BARO_M
    return RangeCue(CUE_GROUND, r, _GROUND_SIGMA_FRAC * r + agl_sigma / tan_dep)


def box_height_cue(
    *,
    bbox_x: float,
    bbox_y: float,
    bbox_w: float,
    bbox_h: float,
    frame_width: int,
    frame_height: int,
    intr: CameraIntrinsics,
    depression_rad: float,
) -> RangeCue | None:
    """Horizontal range from the box height of a ~1.70 m subject, or ``None``
    when the box is clipped by any frame edge or degenerate."""
    if bbox_h <= 1.0:
        return None
    if (
        bbox_x <= _EDGE_MARGIN_PX
        or bbox_y <= _EDGE_MARGIN_PX
        or bbox_x + bbox_w >= frame_width - _EDGE_MARGIN_PX
        or bbox_y + bbox_h >= frame_height - _EDGE_MARGIN_PX
    ):
        return None
    slant = intr.fy * SUBJECT_HEIGHT_M / bbox_h
    r = slant * math.cos(depression_rad)
    return RangeCue(CUE_BOX_HEIGHT, r, _BOX_SIGMA_FRAC * r)


def measure_target(
    *,
    bbox_x: float,
    bbox_y: float,
    bbox_w: float,
    bbox_h: float,
    frame_width: int,
    frame_height: int,
    horizontal_fov_deg: float,
    cam: CameraPose,
    agl_m: float,
    agl_source: str,
    scan: Scan | None = None,
) -> TargetFix | None:
    """Resolve one subject box into a north/east offset from the vehicle.

    ``None`` only when every range cue is unavailable.
    """
    intr = CameraIntrinsics.from_fov(frame_width, frame_height, horizontal_fov_deg)
    px, py = bbox_feet(bbox_x, bbox_y, bbox_w, bbox_h)
    ray = camera_ray_to_ned(pixel_to_camera_ray(px, py, intr), cam)
    horiz = math.hypot(ray[0], ray[1])
    depression = math.atan2(ray[2], horiz)
    bearing = math.atan2(ray[1], ray[0])

    cues: list[RangeCue] = []
    g = ground_cue(ray, agl_m, agl_source)
    if g is not None:
        cues.append(g)
    b = box_height_cue(
        bbox_x=bbox_x,
        bbox_y=bbox_y,
        bbox_w=bbox_w,
        bbox_h=bbox_h,
        frame_width=frame_width,
        frame_height=frame_height,
        intr=intr,
        depression_rad=depression,
    )
    if b is not None:
        cues.append(b)
    others = fuse(cues)
    if others is None:
        return None
    if scan is not None:
        body_bearing = bearing - cam.yaw_rad
        lr = lidar_range_at(scan, body_bearing)
        if lr is not None and abs(lr - others[0]) <= _LIDAR_GATE_FRAC * others[0]:
            cues.append(RangeCue(CUE_LIDAR, lr, _LIDAR_SIGMA_M))
    fused = fuse(cues)
    assert fused is not None  # cues is non-empty here
    r, sigma = fused
    bearing_sigma = math.hypot(1.0 / intr.fx, _BEARING_SIGMA_RAD)
    return TargetFix(
        offset_north_m=r * math.cos(bearing),
        offset_east_m=r * math.sin(bearing),
        range_m=r,
        range_sigma_m=sigma,
        bearing_rad=bearing,
        depression_rad=depression,
        cues=tuple(cues),
        agl_m=agl_m,
        agl_source=agl_source,
        cross_sigma_m=max(r * bearing_sigma, _MIN_CROSS_SIGMA_M),
    )


def predict_bbox(
    *,
    offset_north_m: float,
    offset_east_m: float,
    agl_m: float,
    cam: CameraPose,
    frame_width: int,
    frame_height: int,
    horizontal_fov_deg: float,
) -> tuple[float, float, float, float] | None:
    """The box ``(x, y, w, h)`` a standing subject at a ground offset would
    occupy in the frame, or ``None`` when its feet fall outside the frame or
    behind the lens. Used to draw the predicted box while coasting."""
    intr = CameraIntrinsics.from_fov(frame_width, frame_height, horizontal_fov_deg)
    feet = ned_to_pixel((offset_north_m, offset_east_m, agl_m), cam, intr)
    head = ned_to_pixel(
        (offset_north_m, offset_east_m, agl_m - SUBJECT_HEIGHT_M), cam, intr
    )
    if feet is None or head is None:
        return None
    fx, fy = feet
    if not (0.0 <= fx <= frame_width and 0.0 <= fy <= frame_height):
        return None
    h = fy - head[1]
    if h <= 1.0:
        return None
    w = SUBJECT_ASPECT * h
    return (fx - w / 2.0, head[1], w, h)


__all__ = [
    "AGL_BARO",
    "AGL_RNG",
    "CUE_BOX_HEIGHT",
    "CUE_GROUND",
    "CUE_LIDAR",
    "SUBJECT_HEIGHT_M",
    "CameraIntrinsics",
    "CameraPose",
    "TargetFix",
    "bbox_feet",
    "box_height_cue",
    "camera_ray_to_ned",
    "ground_cue",
    "measure_target",
    "ned_to_pixel",
    "pixel_to_camera_ray",
    "predict_bbox",
    "select_agl",
]
