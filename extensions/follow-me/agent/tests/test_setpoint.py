"""Follow and orbit goal geometry tests."""

from __future__ import annotations

import math

import pytest

from follow_me import setpoint


def _follow(**kw: float) -> setpoint.Goal:
    args: dict[str, float] = dict(
        target_n=10.0,
        target_e=0.0,
        target_vn=0.0,
        target_ve=0.0,
        ref_heading_rad=0.0,
        follow_distance_m=8.0,
        follow_angle_deg=180.0,
        max_speed_mps=6.0,
        ff_scale=1.0,
        drone_n=0.0,
        drone_e=0.0,
        drone_yaw_rad=0.0,
    )
    args.update(kw)
    return setpoint.follow_goal(**args)  # type: ignore[arg-type]


def test_behind_side_and_lead_place_the_goal_around_the_travel_heading() -> None:
    # Subject heading north (ref 0): 180 trails, 90 sits on its right (east),
    # 0 leads.
    behind = _follow(follow_angle_deg=180.0)
    assert (behind.north_m, behind.east_m) == pytest.approx((2.0, 0.0))
    side = _follow(follow_angle_deg=90.0)
    assert (side.north_m, side.east_m) == pytest.approx((10.0, 8.0))
    lead = _follow(follow_angle_deg=0.0)
    assert (lead.north_m, lead.east_m) == pytest.approx((18.0, 0.0))


def test_follow_yaw_points_at_the_subject() -> None:
    g = _follow(drone_n=10.0, drone_e=-5.0)
    assert g.yaw_rad == pytest.approx(math.pi / 2)


def test_feed_forward_is_clamped_and_scaled() -> None:
    g = _follow(target_vn=12.0, max_speed_mps=6.0)
    assert (g.vn_mps, g.ve_mps) == pytest.approx((6.0, 0.0))
    g = _follow(target_vn=2.0, ff_scale=0.5)
    assert g.vn_mps == pytest.approx(1.0)


def test_orbit_goal_sits_on_the_ring_and_moves_tangentially() -> None:
    g = setpoint.orbit_goal(
        center_n=0.0,
        center_e=0.0,
        center_vn=0.0,
        center_ve=0.0,
        phase_rad=0.0,
        radius_m=8.0,
        rate_dps=10.0,
        direction="cw",
        max_speed_mps=6.0,
        ff_scale=1.0,
    )
    assert (g.north_m, g.east_m) == pytest.approx((8.0, 0.0))
    # Clockwise from north is toward east; tangential speed R·ω.
    assert g.vn_mps == pytest.approx(0.0, abs=1e-9)
    assert g.ve_mps == pytest.approx(8.0 * math.radians(10.0))
    # Yaw toward the centre (south).
    assert abs(setpoint.wrap_pi(g.yaw_rad - math.pi)) < 1e-9
    ccw = setpoint.orbit_goal(
        center_n=0.0,
        center_e=0.0,
        center_vn=0.0,
        center_ve=0.0,
        phase_rad=0.0,
        radius_m=8.0,
        rate_dps=10.0,
        direction="ccw",
        max_speed_mps=6.0,
        ff_scale=1.0,
    )
    assert ccw.ve_mps == pytest.approx(-8.0 * math.radians(10.0))


def test_orbit_velocity_is_capped() -> None:
    g = setpoint.orbit_goal(
        center_n=0.0,
        center_e=0.0,
        center_vn=5.0,
        center_ve=0.0,
        phase_rad=0.0,
        radius_m=30.0,
        rate_dps=30.0,
        direction="cw",
        max_speed_mps=6.0,
        ff_scale=1.0,
    )
    assert math.hypot(g.vn_mps, g.ve_mps) == pytest.approx(6.0)


def test_orbit_phase_advances_only_near_the_ring_point() -> None:
    kw = dict(center_n=0.0, center_e=0.0, radius_m=8.0, rate_dps=10.0, dt_s=1.0)
    on = setpoint.advance_orbit_phase(0.0, direction="cw", drone_n=8.0, drone_e=0.5, **kw)
    assert on == pytest.approx(math.radians(10.0))
    off = setpoint.advance_orbit_phase(0.0, direction="cw", drone_n=0.0, drone_e=0.0, **kw)
    assert off == 0.0
    back = setpoint.advance_orbit_phase(0.0, direction="ccw", drone_n=8.0, drone_e=0.0, **kw)
    assert back == pytest.approx(-math.radians(10.0))


def test_reference_heading_slews_while_moving_and_holds_when_still() -> None:
    # Subject turns to east (90°); the reference moves ≤ 45°/s.
    r = setpoint.slew_ref_heading(0.0, 0.0, 2.0, 0.5)
    assert r == pytest.approx(math.radians(22.5))
    still = setpoint.slew_ref_heading(0.3, 0.0, 0.2, 1.0)
    assert still == 0.3


def test_height_follows_the_terrain() -> None:
    # 6 m above ground, want 4 m: 2 m lower (down grows).
    assert setpoint.height_down(-20.0, 6.0, 4.0) == pytest.approx(-18.0)
    assert setpoint.height_down(-20.0, 6.0, -3.0) == pytest.approx(-14.0)
