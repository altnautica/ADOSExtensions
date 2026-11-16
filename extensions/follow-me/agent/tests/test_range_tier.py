"""The minimum standoff by range tier (see :mod:`follow_me.range_tier`): how
the tier is told from the range scatter, and how the minimum flown follows
it: never opened or closed before the tier is known, raised at once, and
lowered at INWARD_RATE_MPS only while the subject moves."""

from __future__ import annotations

import pytest

from follow_me.range_tier import (
    INWARD_RATE_MPS,
    MEASURED_ENTER_M,
    MEASURED_LEAVE_M,
    MEASURED_MIN_STANDOFF_M,
    MONOCULAR_MIN_STANDOFF_M,
    STANDOFF_CAMERA_RANGE,
    STANDOFF_CLOSING_IN,
    STANDOFF_RANGE_UNKNOWN,
    STANDOFF_SUBJECT_STANDING,
    RangeTier,
)
from follow_me.standoff import GOAL_DEADBAND_M
from follow_me.state import FollowConfig


def test_the_tier_is_told_from_the_scatter() -> None:
    t = RangeTier()
    assert t.measured is None
    # Before the scatter settles a measured-looking one says nothing yet...
    t.update(0.1, settled=False)
    assert t.measured is None
    # ...and a camera's says monocular at once.
    t.update(MEASURED_LEAVE_M + 0.1, settled=False)
    assert t.measured is False
    # Settled, the measured tier is entered at MEASURED_ENTER_M and left only
    # past MEASURED_LEAVE_M.
    t.update(MEASURED_ENTER_M + 0.05, settled=True)
    assert t.measured is False
    t.update(MEASURED_ENTER_M, settled=True)
    assert t.measured is True
    t.update(MEASURED_LEAVE_M, settled=True)
    assert t.measured is True
    t.update(MEASURED_LEAVE_M + 0.01, settled=True)
    assert t.measured is False


@pytest.mark.parametrize(
    ("range_m", "set_m", "least"),
    [
        # Engaged at the setting: nothing held.
        (3.2, 3.0, MEASURED_MIN_STANDOFF_M),
        (4.3, 4.0, MEASURED_MIN_STANDOFF_M),
        # Engaged farther out than a shorter setting: held there, less the
        # goal deadband, between the two tiers' minima.
        (4.3, 3.0, 4.3 - GOAL_DEADBAND_M),
        (8.0, 3.0, MONOCULAR_MIN_STANDOFF_M),
    ],
)
def test_the_minimum_starts_where_the_vehicle_engaged(range_m: float, set_m: float, least: float) -> None:
    t = RangeTier()
    assert not t.engaged
    assert t.min_standoff_m == MEASURED_MIN_STANDOFF_M
    t.engage(range_m, set_m)
    assert t.engaged
    assert t.min_standoff_m == pytest.approx(least)
    # Until the tier is known it does not move, still or not.
    for k in range(100):
        t.step(0.1 * k, still=k % 2 == 0)
    assert t.min_standoff_m == pytest.approx(least)


def test_a_camera_tier_raises_the_minimum_at_once() -> None:
    t = RangeTier()
    t.engage(3.0, 3.0)
    t.step(0.0, still=True)
    t.update(0.8, settled=False)
    t.step(0.05, still=True)
    assert t.min_standoff_m == MONOCULAR_MIN_STANDOFF_M


def test_a_measured_tier_lowers_the_minimum_only_while_the_subject_moves() -> None:
    t = RangeTier()
    t.engage(6.0, 3.0)
    t.update(0.1, settled=True)
    now = 0.0
    t.step(now, still=True)
    # Standing: never toward the subject.
    for _ in range(200):
        now += 0.05
        t.step(now, still=True)
    assert t.min_standoff_m == MONOCULAR_MIN_STANDOFF_M
    # Walking: at INWARD_RATE_MPS, down to the measured minimum and no lower.
    now += 0.05
    t.step(now, still=False)
    assert t.min_standoff_m == pytest.approx(MONOCULAR_MIN_STANDOFF_M - INWARD_RATE_MPS * 0.05)
    for _ in range(400):
        now += 0.05
        t.step(now, still=False)
    assert t.min_standoff_m == MEASURED_MIN_STANDOFF_M
    # Turned monocular: back out at once.
    t.update(0.6, settled=True)
    now += 0.05
    t.step(now, still=False)
    assert t.min_standoff_m == MONOCULAR_MIN_STANDOFF_M


def test_the_config_is_flown_at_the_minimum_and_says_so() -> None:
    cfg = FollowConfig.resolve({"follow_distance_m": 3.0, "orbit_radius_m": 8.0}, {})
    t = RangeTier()
    out = t.apply_to(cfg)
    assert (out.follow_distance_m, out.standoff_min_m, out.standoff_raised) == (3.0, MEASURED_MIN_STANDOFF_M, False)
    assert out.standoff_reason is None
    t.engage(3.0, 3.0)
    t.update(0.8, settled=True)
    t.step(0.0, still=False)
    out = t.apply_to(cfg)
    assert out.follow_distance_m == MONOCULAR_MIN_STANDOFF_M
    assert out.orbit_radius_m == 8.0
    assert out.standoff_min_m == MONOCULAR_MIN_STANDOFF_M
    assert out.standoff_raised is True
    assert out.standoff_reason == STANDOFF_CAMERA_RANGE
    t.reset()
    assert not t.engaged and t.measured is None


def test_the_reason_a_setting_is_held_farther_is_the_true_one() -> None:
    cfg = FollowConfig.resolve({"follow_distance_m": 3.0}, {})
    t = RangeTier()
    t.engage(6.0, 3.0)
    t.step(0.0, still=True)
    # Engaged 6 m out, the range not yet known.
    assert t.apply_to(cfg).standoff_reason == STANDOFF_RANGE_UNKNOWN
    # Measured, but the subject stands: held, not for a camera's range.
    t.update(0.1, settled=True)
    t.step(0.05, still=True)
    assert t.apply_to(cfg).standoff_reason == STANDOFF_SUBJECT_STANDING
    # Walking: closing in.
    t.step(0.1, still=False)
    out = t.apply_to(cfg)
    assert out.standoff_reason == STANDOFF_CLOSING_IN and out.standoff_min_m < MONOCULAR_MIN_STANDOFF_M
    # In at the measured minimum: nothing held, no reason.
    for k in range(400):
        t.step(0.15 + 0.05 * k, still=False)
    out = t.apply_to(cfg)
    assert (out.standoff_raised, out.standoff_reason) == (False, None)
    # A setting at or above the minimum is never held, whatever the tier.
    t.update(0.8, settled=True)
    t.step(30.0, still=False)
    assert t.apply_to(FollowConfig.resolve({"follow_distance_m": 6.0}, {})).standoff_reason is None
