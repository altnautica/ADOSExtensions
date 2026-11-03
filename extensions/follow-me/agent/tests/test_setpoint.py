"""Standoff setpoint geometry tests."""

from __future__ import annotations

import math

from follow_me import setpoint


def _sp(**kw: float) -> setpoint.LocalSetpoint:
    args = dict(
        drone_north_m=0.0,
        drone_east_m=0.0,
        drone_down_m=-4.0,
        drone_yaw_rad=0.0,
        agl_m=4.0,
        target_north_m=20.0,
        target_east_m=0.0,
        target_vn_mps=0.0,
        target_ve_mps=0.0,
        follow_distance_m=8.0,
        follow_height_m=4.0,
        max_speed_mps=6.0,
    )
    args.update(kw)
    return setpoint.compute_setpoint(**args)  # type: ignore[arg-type]


def test_still_subject_is_held_along_the_line_of_sight() -> None:
    sp = _sp()
    assert abs(sp.north_m - 12.0) < 1e-9 and abs(sp.east_m) < 1e-9
    assert (sp.vn_mps, sp.ve_mps) == (0.0, 0.0)
    assert abs(sp.yaw_rad) < 1e-9


def test_moving_subject_is_trailed_on_its_path_with_feed_forward() -> None:
    # Subject 20 m north, walking east at 1.4 m/s: the vehicle stands off
    # behind it along its direction of travel (west of it).
    sp = _sp(target_ve_mps=1.4)
    assert abs(sp.north_m - 20.0) < 1e-9
    assert abs(sp.east_m - (-8.0)) < 1e-9
    assert abs(sp.ve_mps - 1.4) < 1e-9
    # Yaw still looks at the subject.
    assert abs(sp.yaw_rad - math.atan2(0.0, 20.0)) < 1e-9


def test_feed_forward_is_clamped_and_scaled() -> None:
    sp = _sp(target_vn_mps=9.0, max_speed_mps=6.0)
    assert abs(math.hypot(sp.vn_mps, sp.ve_mps) - 6.0) < 1e-9
    coast = _sp(target_vn_mps=2.0, ff_scale=0.5)
    assert abs(coast.vn_mps - 1.0) < 1e-9


def test_height_follows_the_terrain() -> None:
    # 6 m above the ground under the vehicle, want 4: descend 2 m (NED down +).
    sp = _sp(drone_down_m=-10.0, agl_m=6.0, follow_height_m=4.0)
    assert abs(sp.down_m - (-8.0)) < 1e-9
    assert sp.vd_mps == 0.0


def test_hold_is_zero_velocity_at_the_current_position() -> None:
    hold = setpoint.hold_setpoint(north_m=1.0, east_m=2.0, down_m=-3.0, yaw_rad=0.5)
    assert (hold.north_m, hold.east_m, hold.down_m, hold.yaw_rad) == (1.0, 2.0, -3.0, 0.5)
    assert (hold.vn_mps, hold.ve_mps, hold.vd_mps) == (0.0, 0.0, 0.0)
