"""The follow height under a canopy the pilot reports ahead."""

from __future__ import annotations

from follow_me.canopy import HEIGHT_CANOPY, HEIGHT_CANOPY_FLOOR, HOLD_S, CanopyHeight
from follow_me.state import MIN_FOLLOW_HEIGHT_M, FollowConfig


def _cfg(height: float = 4.0) -> FollowConfig:
    return FollowConfig.resolve({"follow_height_m": height}, {})


def test_no_report_flies_the_set_height() -> None:
    c = CanopyHeight()
    cfg = c.apply_to(_cfg(), 10.0)
    assert cfg.follow_height_m == 4.0
    assert not cfg.follow_height_lowered
    assert cfg.follow_height_reason is None


def test_leaves_ahead_lower_the_follow_under_them() -> None:
    c = CanopyHeight()
    # Crown bases 4.2 m over the trail: the band clears them below 3.87 m.
    c.on_overhead({"fly_under_agl_m": 3.87, "ahead_m": 2.5}, 10.0)
    cfg = c.apply_to(_cfg(), 10.1)
    assert abs(cfg.follow_height_m - 3.87) < 1e-9
    assert cfg.follow_height_lowered and cfg.follow_height_reason == HEIGHT_CANOPY
    # Leaves clear of the set height change nothing.
    c.reset()
    c.on_overhead({"fly_under_agl_m": 4.5}, 11.0)
    assert c.apply_to(_cfg(), 11.0).follow_height_m == 4.0


def test_leaves_below_the_floor_hold_the_floor_and_say_so() -> None:
    c = CanopyHeight()
    c.on_overhead({"fly_under_agl_m": 2.9}, 10.0)
    cfg = c.apply_to(_cfg(), 10.0)
    assert cfg.follow_height_m == MIN_FOLLOW_HEIGHT_M
    assert cfg.follow_height_reason == HEIGHT_CANOPY_FLOOR
    assert cfg.follow_height_lowered
    # Set at the floor already: held there, not lowered, still said.
    cfg = c.apply_to(_cfg(MIN_FOLLOW_HEIGHT_M), 10.0)
    assert cfg.follow_height_m == MIN_FOLLOW_HEIGHT_M
    assert not cfg.follow_height_lowered
    assert cfg.follow_height_reason == HEIGHT_CANOPY_FLOOR


def test_a_report_holds_between_leans() -> None:
    c = CanopyHeight()
    c.on_overhead({"fly_under_agl_m": 3.8}, 10.0)
    c.on_overhead({"fly_under_agl_m": 3.6}, 11.0)
    c.on_overhead({"fly_under_agl_m": 3.9}, 12.0)
    # The lowest within the hold stands.
    assert abs(c.apply_to(_cfg(), 12.0).follow_height_m - 3.6) < 1e-9
    # No report for the hold: back to the set height.
    assert c.apply_to(_cfg(), 12.0 + HOLD_S + 0.1).follow_height_m == 4.0
    # A null with nothing reported before is clear air.
    c.on_overhead({"fly_under_agl_m": None}, 20.0)
    assert c.apply_to(_cfg(), 20.1).follow_height_m == 4.0


def test_a_null_after_leaves_holds_until_flown_past_them() -> None:
    c = CanopyHeight()
    # Leaves 4 m ahead along the way, reported at the origin.
    c.on_overhead({"fly_under_agl_m": 3.6, "ahead_m": 4.0}, 10.0, (0.0, 0.0))
    # The pilot forgets them in level flight (its returns lapse while it
    # still reports): null, still under the crowns.
    c.on_overhead({"fly_under_agl_m": None, "ahead_m": None}, 13.0, (5.0, 0.0))
    # Long after the time hold, short of 4 + max(2 x 4, 6) = 12 m flown.
    assert abs(c.apply_to(_cfg(), 40.0, (11.9, 0.0)).follow_height_m - 3.6) < 1e-9
    # With no position known it holds too.
    assert abs(c.apply_to(_cfg(), 40.0).follow_height_m - 3.6) < 1e-9
    # Past the last overhead point by twice its distance: back up.
    assert c.apply_to(_cfg(), 40.0, (12.0, 0.0)).follow_height_m == 4.0
    assert c.apply_to(_cfg(), 40.0, (0.0, 0.0)).follow_height_m == 4.0


def test_a_near_overhead_point_is_held_for_at_least_six_metres_past() -> None:
    c = CanopyHeight()
    c.on_overhead({"fly_under_agl_m": 3.7, "ahead_m": 0.5}, 10.0, (0.0, 0.0))
    c.on_overhead({"fly_under_agl_m": None}, 10.2, (0.2, 0.0))
    assert abs(c.apply_to(_cfg(), 30.0, (0.0, 6.4)).follow_height_m - 3.7) < 1e-9
    assert c.apply_to(_cfg(), 30.0, (0.0, 6.5)).follow_height_m == 4.0
    # New leaves end the post-null hold and start a fresh report.
    c.on_overhead({"fly_under_agl_m": 3.9, "ahead_m": 2.0}, 31.0, (0.0, 7.0))
    assert abs(c.apply_to(_cfg(), 31.1, (0.0, 7.0)).follow_height_m - 3.9) < 1e-9
    assert c.apply_to(_cfg(), 31.0 + HOLD_S + 0.1, (0.0, 7.0)).follow_height_m == 4.0


def test_a_malformed_report_changes_nothing() -> None:
    c = CanopyHeight()
    c.on_overhead({"fly_under_agl_m": 3.8}, 10.0)
    c.on_overhead({"fly_under_agl_m": "low"}, 10.5)
    c.on_overhead({"fly_under_agl_m": float("nan")}, 10.6)
    assert abs(c.apply_to(_cfg(), 10.7).follow_height_m - 3.8) < 1e-9
