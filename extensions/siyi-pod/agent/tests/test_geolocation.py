"""Laser-range geolocation tests against hand-computed geometry."""

from __future__ import annotations

import math

from altnautica_siyi_pod.geolocation import geolocate

_M_PER_DEG_LAT = 111320.0


def test_straight_down_is_directly_below():
    t = geolocate(
        vehicle_lat_deg=12.9716,
        vehicle_lon_deg=77.5946,
        vehicle_rel_alt_m=50.0,
        vehicle_yaw_deg=90.0,
        gimbal_yaw_deg=0.0,
        gimbal_pitch_deg=-90.0,
        slant_range_m=50.0,
    )
    # Nadir: the subject is under the aircraft and the range equals the height.
    assert math.isclose(t.lat_deg, 12.9716, abs_tol=1e-6)
    assert math.isclose(t.lon_deg, 77.5946, abs_tol=1e-6)
    assert math.isclose(t.rel_alt_m, 0.0, abs_tol=1e-6)
    assert math.isclose(t.ground_range_m, 0.0, abs_tol=1e-6)


def test_level_north_is_north_of_the_aircraft():
    t = geolocate(
        vehicle_lat_deg=0.0,
        vehicle_lon_deg=0.0,
        vehicle_rel_alt_m=100.0,
        vehicle_yaw_deg=0.0,
        gimbal_yaw_deg=0.0,
        gimbal_pitch_deg=0.0,  # level with the horizon
        slant_range_m=100.0,
    )
    assert t.bearing_deg == 0.0
    assert math.isclose(t.lat_deg, 100.0 / _M_PER_DEG_LAT, rel_tol=1e-6)
    assert math.isclose(t.lon_deg, 0.0, abs_tol=1e-9)
    # Level shot: no altitude change.
    assert math.isclose(t.rel_alt_m, 100.0, abs_tol=1e-6)
    assert math.isclose(t.ground_range_m, 100.0, rel_tol=1e-6)


def test_bearing_adds_vehicle_heading_and_gimbal_yaw():
    t = geolocate(
        vehicle_lat_deg=0.0,
        vehicle_lon_deg=0.0,
        vehicle_rel_alt_m=100.0,
        vehicle_yaw_deg=60.0,
        gimbal_yaw_deg=30.0,
        gimbal_pitch_deg=0.0,
        slant_range_m=100.0,
    )
    assert t.bearing_deg == 90.0  # due east
    # East offset only: latitude unchanged, longitude increases.
    assert math.isclose(t.lat_deg, 0.0, abs_tol=1e-9)
    assert t.lon_deg > 0.0


def test_depressed_shot_lowers_the_target():
    t = geolocate(
        vehicle_lat_deg=0.0,
        vehicle_lon_deg=0.0,
        vehicle_rel_alt_m=100.0,
        vehicle_yaw_deg=0.0,
        gimbal_yaw_deg=0.0,
        gimbal_pitch_deg=-45.0,
        slant_range_m=100.0,
    )
    drop = 100.0 * math.sin(math.radians(45.0))
    assert math.isclose(t.rel_alt_m, 100.0 - drop, rel_tol=1e-6)
    assert math.isclose(t.ground_range_m, drop, rel_tol=1e-6)
    assert math.isclose(t.depression_deg, 45.0, abs_tol=1e-9)
