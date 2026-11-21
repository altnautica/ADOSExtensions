"""Where the lost subject is predicted to be."""

from __future__ import annotations

import pytest
from follow_me.predict import predict_target, predicted_path, search_ellipse
from follow_me.target_filter import TargetFilter

_R = ((0.04, 0.0), (0.0, 0.04))


def _walking_north(speed: float = 1.4) -> TargetFilter:
    f = TargetFilter()
    for k in range(20):
        f.update(k * 0.1, speed * k * 0.1, 0.0, _R)
    return f


def test_prediction_carries_on_at_the_walking_velocity_then_lets_it_fade() -> None:
    f = _walking_north()
    # Inside the coast's grace the walk carries on as it was.
    early = predict_target(f, 1.9 + 0.5)
    assert early is not None
    assert early.mean_ne[0] == pytest.approx(1.4 * 2.4, abs=0.3)
    assert early.vel_ne[0] == pytest.approx(1.4, abs=0.15)
    # Past it the lost subject's predicted walk slows (the coast), short of
    # where the last velocity run on would put it.
    p = predict_target(f, 1.9 + 2.0)
    assert p is not None
    coast = f.coasted(3.9)
    assert coast is not None
    assert p.mean_ne[0] == pytest.approx(coast[0][0])
    assert 1.4 * 2.4 < p.mean_ne[0] < 1.4 * 3.9
    assert p.vel_ne[0] < 1.4


def test_path_is_half_second_spaced_and_at_most_ten_points() -> None:
    path = predicted_path(_walking_north(), 1.9)
    assert len(path) == 10
    assert path[1][0] - path[0][0] == pytest.approx(0.7, abs=0.1)


def test_ellipse_grows_with_time_and_points_along_travel() -> None:
    f = _walking_north()
    p1 = predict_target(f, 2.9)
    p2 = predict_target(f, 5.9)
    assert p1 is not None and p2 is not None
    e1 = search_ellipse(p1, k=2.0)
    e2 = search_ellipse(p2, k=2.0)
    assert e2.semi_major_m > e1.semi_major_m
    assert e1.semi_major_m >= e1.semi_minor_m
    assert (
        min(abs(e2.heading_deg), abs(e2.heading_deg - 180.0)) < 30.0
        or e2.semi_major_m - e2.semi_minor_m < 0.2
    )


def test_no_prediction_before_an_update() -> None:
    assert predict_target(TargetFilter(), 1.0) is None
