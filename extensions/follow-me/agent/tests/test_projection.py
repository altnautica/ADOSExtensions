"""Projection tests: pinhole ray, feet ray, AGL source, range cues, fusion."""

from __future__ import annotations

import math

from follow_me import projection, ranging
from follow_me.projection import CameraPose
from follow_me.ranging import RangeCue, Scan

_LEVEL = CameraPose(roll_rad=0.0, pitch_rad=0.0, yaw_rad=0.0)


def _cam(mount_pitch_deg: float = 0.0, yaw_rad: float = 0.0) -> CameraPose:
    return CameraPose(
        roll_rad=0.0, pitch_rad=0.0, yaw_rad=yaw_rad, mount_pitch_deg=mount_pitch_deg
    )


def _box_for_feet(px: float, py: float, h: float = 60.0) -> tuple[float, float, float, float]:
    """A box whose bottom-centre is at (px, py)."""
    w = 0.4 * h
    return (px - w / 2.0, py - h, w, h)


def test_intrinsics_from_fov_center_and_focal() -> None:
    intr = projection.CameraIntrinsics.from_fov(640, 480, 70.0)
    assert (intr.cx, intr.cy) == (320.0, 240.0)
    assert abs(intr.fx - 320.0 / math.tan(math.radians(35.0))) < 1e-9
    assert intr.fy == intr.fx


def test_feet_is_the_box_bottom_centre() -> None:
    assert projection.bbox_feet(100.0, 50.0, 20.0, 80.0) == (110.0, 130.0)


def test_ned_to_pixel_inverts_the_ray() -> None:
    intr = projection.CameraIntrinsics.from_fov(1280, 720, 87.0)
    cam = CameraPose(
        roll_rad=0.05, pitch_rad=-0.1, yaw_rad=1.0, mount_pitch_deg=15.0
    )
    ray = projection.camera_ray_to_ned(
        projection.pixel_to_camera_ray(900.0, 500.0, intr), cam
    )
    px = projection.ned_to_pixel(tuple(10.0 * c for c in ray), cam, intr)  # type: ignore[arg-type]
    assert px is not None
    assert abs(px[0] - 900.0) < 1e-6 and abs(px[1] - 500.0) < 1e-6


def test_flat_ground_feet_ray_gives_the_ground_range() -> None:
    # Camera pitched 45 deg down, 10 m up: feet at the frame centre are 10 m
    # ahead, due north.
    x, y, w, h = _box_for_feet(320.0, 240.0)
    fix = projection.measure_target(
        bbox_x=x, bbox_y=y, bbox_w=w, bbox_h=h,
        frame_width=640, frame_height=480, horizontal_fov_deg=70.0,
        cam=_cam(45.0), agl_m=10.0, agl_source=projection.AGL_RNG,
    )
    assert fix is not None
    ground = next(c for c in fix.cues if c.kind == projection.CUE_GROUND)
    assert abs(ground.range_m - 10.0) < 1e-6
    assert abs(fix.bearing_rad) < 1e-9
    assert abs(math.degrees(fix.depression_rad) - 45.0) < 1e-6
    assert fix.offset_north_m > 0.0 and abs(fix.offset_east_m) < 1e-6


def test_feet_not_centre_is_projected() -> None:
    # The same box: projecting its centre would land further away than its
    # feet. The ground cue must come from the feet.
    x, y, w, h = _box_for_feet(320.0, 240.0, h=100.0)
    fix = projection.measure_target(
        bbox_x=x, bbox_y=y, bbox_w=w, bbox_h=h,
        frame_width=640, frame_height=480, horizontal_fov_deg=70.0,
        cam=_cam(45.0), agl_m=10.0, agl_source=projection.AGL_RNG,
    )
    assert fix is not None
    ground = next(c for c in fix.cues if c.kind == projection.CUE_GROUND)
    centre_ray = projection.camera_ray_to_ned(
        projection.pixel_to_camera_ray(
            320.0, 190.0, projection.CameraIntrinsics.from_fov(640, 480, 70.0)
        ),
        _cam(45.0),
    )
    centre = projection.ground_cue(centre_ray, 10.0, projection.AGL_RNG)
    assert centre is not None and centre.range_m > ground.range_m


def test_sloped_ground_uses_the_rangefinder_height() -> None:
    # Over a slope the take-off height is wrong; the rangefinder height is the
    # one the ground cue must project through, and it is trusted more.
    agl, src = projection.select_agl(
        rangefinder_usable=True, rangefinder_m=6.0,
        roll_rad=0.0, pitch_rad=0.0, baro_agl_m=14.0,
    )
    assert (agl, src) == (6.0, projection.AGL_RNG)
    ray = projection.camera_ray_to_ned((0.0, 0.0, 1.0), _cam(30.0))
    rng = projection.ground_cue(ray, 6.0, projection.AGL_RNG)
    baro = projection.ground_cue(ray, 6.0, projection.AGL_BARO)
    assert rng is not None and baro is not None
    assert abs(rng.range_m - 6.0 / math.tan(math.radians(30.0))) < 1e-6
    assert baro.sigma_m > rng.sigma_m


def test_agl_falls_back_to_baro_when_tilted_or_unusable() -> None:
    assert projection.select_agl(
        rangefinder_usable=False, rangefinder_m=6.0,
        roll_rad=0.0, pitch_rad=0.0, baro_agl_m=9.0,
    ) == (9.0, projection.AGL_BARO)
    assert projection.select_agl(
        rangefinder_usable=True, rangefinder_m=6.0,
        roll_rad=math.radians(35.0), pitch_rad=0.0, baro_agl_m=9.0,
    ) == (9.0, projection.AGL_BARO)
    agl, src = projection.select_agl(
        rangefinder_usable=True, rangefinder_m=6.0,
        roll_rad=math.radians(20.0), pitch_rad=0.0, baro_agl_m=9.0,
    )
    assert src == projection.AGL_RNG and abs(agl - 6.0 * math.cos(math.radians(20.0))) < 1e-9


def test_box_height_cue_skips_a_clipped_box() -> None:
    intr = projection.CameraIntrinsics.from_fov(640, 480, 70.0)
    inside = projection.box_height_cue(
        bbox_x=300, bbox_y=100, bbox_w=40, bbox_h=100, frame_width=640,
        frame_height=480, intr=intr, depression_rad=0.0,
    )
    assert inside is not None
    assert abs(inside.range_m - intr.fy * 1.70 / 100.0) < 1e-9
    assert abs(inside.sigma_m - 0.15 * inside.range_m) < 1e-9
    clipped = projection.box_height_cue(
        bbox_x=300, bbox_y=380, bbox_w=40, bbox_h=100, frame_width=640,
        frame_height=480, intr=intr, depression_rad=0.0,
    )
    assert clipped is None


def test_fusion_is_inverse_variance() -> None:
    r, s = ranging.fuse(
        [RangeCue("ground", 10.0, 1.0), RangeCue("box_height", 12.0, 2.0)]
    )  # type: ignore[misc]
    assert abs(r - (10.0 / 1 + 12.0 / 4) / (1 + 0.25)) < 1e-9
    assert abs(s - math.sqrt(1.0 / 1.25)) < 1e-9
    assert ranging.fuse([]) is None


def _scan_with(bearing_deg: float, r: float) -> Scan:
    ranges = [0.0] * 360
    ranges[int(round(bearing_deg)) % 360] = r
    return Scan(0.0, math.radians(1.0), tuple(ranges))


def test_lidar_cue_is_used_only_when_it_agrees() -> None:
    x, y, w, h = _box_for_feet(320.0, 240.0)
    kwargs = dict(
        bbox_x=x, bbox_y=y, bbox_w=w, bbox_h=h,
        frame_width=640, frame_height=480, horizontal_fov_deg=70.0,
        cam=_cam(45.0), agl_m=10.0, agl_source=projection.AGL_RNG,
    )
    agree = projection.measure_target(**kwargs, scan=_scan_with(0.0, 10.3))
    assert agree is not None
    assert any(c.kind == projection.CUE_LIDAR for c in agree.cues)
    # The 5 cm lidar sigma dominates the fused range.
    assert abs(agree.range_m - 10.3) < 0.05
    behind = projection.measure_target(**kwargs, scan=_scan_with(0.0, 30.0))
    assert behind is not None
    assert not any(c.kind == projection.CUE_LIDAR for c in behind.cues)


def test_lidar_bin_is_body_relative() -> None:
    # Bins are body-frame bearings: a return at 0 deg is straight ahead.
    scan = _scan_with(0.0, 7.0)
    assert ranging.lidar_range_at(scan, 0.0) == 7.0
    assert ranging.lidar_range_at(scan, math.radians(10.0)) is None
    # A return within +/-2 degrees counts.
    assert ranging.lidar_range_at(_scan_with(2.0, 5.0), 0.0) == 5.0


def test_no_cue_returns_none() -> None:
    # Level camera, box at the top edge, no AGL: nothing to range from.
    fix = projection.measure_target(
        bbox_x=300, bbox_y=0, bbox_w=40, bbox_h=100,
        frame_width=640, frame_height=480, horizontal_fov_deg=70.0,
        cam=_LEVEL, agl_m=0.0, agl_source=projection.AGL_BARO,
    )
    assert fix is None


def test_predicted_box_round_trips_a_subject() -> None:
    cam = _cam(30.0)
    box = projection.predict_bbox(
        offset_north_m=8.0, offset_east_m=0.0, agl_m=4.0, cam=cam,
        frame_width=640, frame_height=480, horizontal_fov_deg=70.0,
    )
    assert box is not None
    x, y, w, h = box
    fix = projection.measure_target(
        bbox_x=x, bbox_y=y, bbox_w=w, bbox_h=h,
        frame_width=640, frame_height=480, horizontal_fov_deg=70.0,
        cam=cam, agl_m=4.0, agl_source=projection.AGL_RNG,
    )
    assert fix is not None
    ground = next(c for c in fix.cues if c.kind == projection.CUE_GROUND)
    assert abs(ground.range_m - 8.0) < 0.05
    # Off-frame (behind) predicts no box.
    assert projection.predict_bbox(
        offset_north_m=-8.0, offset_east_m=0.0, agl_m=4.0, cam=cam,
        frame_width=640, frame_height=480, horizontal_fov_deg=70.0,
    ) is None
