"""Projection geometry tests: pinhole ray, ground intersection, follow."""

from __future__ import annotations

import math

from follow_me import projection


def test_intrinsics_from_fov_center_and_focal() -> None:
    intr = projection.CameraIntrinsics.from_fov(640, 480, 70.0)
    assert intr.cx == 320.0
    assert intr.cy == 240.0
    # fx = (w/2) / tan(hfov/2); hfov=70deg.
    expected_fx = 320.0 / math.tan(math.radians(70.0) / 2.0)
    assert abs(intr.fx - expected_fx) < 1e-6
    assert intr.fy == intr.fx  # square pixels


def test_center_pixel_maps_to_boresight_ray() -> None:
    intr = projection.CameraIntrinsics.from_fov(640, 480, 70.0)
    ray = projection.pixel_to_camera_ray(320.0, 240.0, intr)
    # Center pixel points straight out the lens (+z), unit length.
    assert abs(ray[0]) < 1e-9
    assert abs(ray[1]) < 1e-9
    assert abs(ray[2] - 1.0) < 1e-9


def test_nadir_camera_center_hits_ground_below_vehicle() -> None:
    # Camera pointing straight down (mount pitch 90deg), level vehicle,
    # center pixel: the subject is directly under the vehicle.
    setpoint = projection.project_follow_setpoint(
        bbox_x=315.0,
        bbox_y=235.0,
        bbox_w=10.0,
        bbox_h=10.0,
        frame_width=640,
        frame_height=480,
        horizontal_fov_deg=70.0,
        roll_rad=0.0,
        pitch_rad=0.0,
        yaw_rad=0.0,
        vehicle_lat_deg=12.0,
        vehicle_lon_deg=77.0,
        agl_m=20.0,
        follow_distance_m=8.0,
        follow_height_m=4.0,
        mount_pitch_deg=90.0,
    )
    assert setpoint is not None
    tgt = setpoint.target
    # Subject is directly below: ground range ~ 0.
    assert tgt.ground_range_m < 0.5
    assert abs(tgt.lat_deg - 12.0) < 1e-4
    assert abs(tgt.lon_deg - 77.0) < 1e-4


def test_forward_pixel_offset_projects_ahead_of_vehicle() -> None:
    # Camera pitched 45deg down, level vehicle facing north (yaw 0). A
    # subject at the frame center sits ahead and below: positive north
    # offset, ground range close to the AGL height (45deg => range == agl).
    setpoint = projection.project_follow_setpoint(
        bbox_x=315.0,
        bbox_y=235.0,
        bbox_w=10.0,
        bbox_h=10.0,
        frame_width=640,
        frame_height=480,
        horizontal_fov_deg=70.0,
        roll_rad=0.0,
        pitch_rad=0.0,
        yaw_rad=0.0,
        vehicle_lat_deg=0.0,
        vehicle_lon_deg=0.0,
        agl_m=10.0,
        follow_distance_m=4.0,
        follow_height_m=5.0,
        mount_pitch_deg=45.0,
    )
    assert setpoint is not None
    tgt = setpoint.target
    assert tgt.offset_north_m > 5.0
    assert abs(tgt.offset_east_m) < 0.5
    # 45deg depression => ground range ~= AGL.
    assert abs(tgt.ground_range_m - 10.0) < 0.5
    # Bearing is due north.
    assert abs(tgt.bearing_rad) < 1e-3


def test_subject_above_horizon_has_no_ground_hit() -> None:
    # Camera pointing forward + level (no down tilt): the boresight ray is
    # horizontal, never reaching the ground plane.
    setpoint = projection.project_follow_setpoint(
        bbox_x=315.0,
        bbox_y=235.0,
        bbox_w=10.0,
        bbox_h=10.0,
        frame_width=640,
        frame_height=480,
        horizontal_fov_deg=70.0,
        roll_rad=0.0,
        pitch_rad=0.0,
        yaw_rad=0.0,
        vehicle_lat_deg=0.0,
        vehicle_lon_deg=0.0,
        agl_m=10.0,
        follow_distance_m=4.0,
        follow_height_m=5.0,
        mount_pitch_deg=0.0,
    )
    assert setpoint is None


def test_follow_setpoint_stands_off_behind_subject() -> None:
    # A subject 20 m north. With a 6 m follow distance the setpoint sits
    # 14 m north of the vehicle (6 m short of the subject), facing north.
    tgt = projection.GroundTarget(
        lat_deg=0.0001,
        lon_deg=0.0,
        offset_north_m=20.0,
        offset_east_m=0.0,
        slant_range_m=22.0,
        ground_range_m=20.0,
        bearing_rad=0.0,
    )
    sp = projection.follow_setpoint_from_target(
        tgt,
        vehicle_lat_deg=0.0,
        vehicle_lon_deg=0.0,
        follow_distance_m=6.0,
        follow_height_m=5.0,
    )
    # 14 m north => ~0.0001257 deg lat.
    assert sp.lat_deg > 0.0
    expected_north = 14.0 / 111_320.0
    assert abs(sp.lat_deg - expected_north) < 1e-6
    assert sp.alt_rel_m == 5.0
    assert abs(sp.yaw_rad) < 1e-6


def test_follow_setpoint_collapses_when_subject_within_standoff() -> None:
    # Subject only 3 m away but follow distance is 8 m: the setpoint
    # collapses onto the subject (no backing through it).
    tgt = projection.GroundTarget(
        lat_deg=0.0,
        lon_deg=0.0,
        offset_north_m=3.0,
        offset_east_m=0.0,
        slant_range_m=5.0,
        ground_range_m=3.0,
        bearing_rad=0.0,
    )
    sp = projection.follow_setpoint_from_target(
        tgt,
        vehicle_lat_deg=0.0,
        vehicle_lon_deg=0.0,
        follow_distance_m=8.0,
        follow_height_m=4.0,
    )
    # set_n = 3 - min(8,3)*cos(0) = 0 -> on the vehicle, i.e. no lat change.
    assert abs(sp.lat_deg) < 1e-9
