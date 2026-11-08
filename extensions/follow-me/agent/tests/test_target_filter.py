"""Constant-velocity Kalman filter tests."""

from __future__ import annotations

import math

from follow_me.target_filter import TargetFilter, position_sigma, subject_max_speed_mps

_R = ((0.09, 0.0), (0.0, 0.09))


def test_first_update_initialises_at_the_measurement() -> None:
    f = TargetFilter()
    assert not f.initialized
    assert f.predicted(0.0) is None
    f.update(1.0, 5.0, -2.0, _R)
    x, p = f.predicted(1.0)  # type: ignore[misc]
    assert x[:2] == [5.0, -2.0]
    assert x[2:] == [0.0, 0.0]
    assert abs(p[0][0] - 0.09) < 1e-12


def test_converges_to_a_walking_subject_velocity() -> None:
    f = TargetFilter()
    for i in range(60):  # 6 s at 10 Hz, 1 m/s north and 1 m/s east
        t = i * 0.1
        f.update(t, 1.0 * t, 1.0 * t, _R)
    x, _p = f.predicted(5.9)  # type: ignore[misc]
    assert abs(x[2] - 1.0) < 0.1 and abs(x[3] - 1.0) < 0.1


def test_prediction_moves_with_velocity_and_does_not_mutate() -> None:
    f = TargetFilter()
    for i in range(50):
        f.update(i * 0.1, 2.0 * i * 0.1, 0.0, _R)
    x1, p1 = f.predicted(6.0)  # type: ignore[misc]
    x2, _ = f.predicted(6.0)  # type: ignore[misc]
    assert x1 == x2
    assert abs(x1[0] - 12.0) < 0.5
    # Uncertainty grows with the coast.
    _x, p0 = f.predicted(4.9)  # type: ignore[misc]
    assert position_sigma(p1) > position_sigma(p0)


def test_mahalanobis_gates_by_the_prediction() -> None:
    f = TargetFilter()
    for i in range(30):
        f.update(i * 0.1, 0.0, 0.0, _R)
    near = f.mahalanobis(3.0, 0.2, 0.0, _R)
    far = f.mahalanobis(3.0, 6.0, 0.0, _R)
    assert near is not None and near < 3.0
    assert far is not None and far > 3.0


def test_reset_forgets_the_track() -> None:
    f = TargetFilter()
    f.update(0.0, 1.0, 1.0, _R)
    f.reset()
    assert not f.initialized
    assert f.mahalanobis(0.0, 0.0, 0.0, _R) is None


def test_position_sigma_is_the_major_axis() -> None:
    assert abs(position_sigma([[4.0, 0.0], [0.0, 1.0]]) - 2.0) < 1e-12
    assert math.isclose(position_sigma([[1.0, 0.0], [0.0, 1.0]]), 1.0)


def test_the_speed_cap_comes_from_the_class() -> None:
    assert subject_max_speed_mps("person") == 3.0
    assert subject_max_speed_mps(" Person ") == 3.0
    assert subject_max_speed_mps("car") is None
    assert subject_max_speed_mps("") is None
    assert subject_max_speed_mps(None) is None


# Range noise along the line of sight: a walker 8 m north whose range reads
# 4 m long, then 4 m short, at the detector's 3 Hz, each claimed to 1 m.
_RANGE_R = ((1.0, 0.0), (0.0, 0.25))


def _noisy_walker(f: TargetFilter) -> float:
    """Feed 1.2 m/s east with ±4 m north range noise; the largest speed."""
    top = 0.0
    for i in range(30):
        t = i / 3.0
        f.update(t, 8.0 + (4.0 if i % 2 else -4.0), 1.2 * t, _RANGE_R)
        x, _p = f.predicted(t)  # type: ignore[misc]
        top = max(top, math.hypot(x[2], x[3]))
    return top


def test_range_noise_cannot_make_a_person_faster_than_the_cap() -> None:
    free = TargetFilter()
    assert _noisy_walker(free) > 3.0
    capped = TargetFilter()
    capped.max_speed_mps = subject_max_speed_mps("person")
    assert _noisy_walker(capped) <= 3.0 + 1e-9


def test_a_capped_estimate_still_keeps_up_with_a_runner() -> None:
    # A 4 m/s runner (over the person cap), sighted at 3 Hz with ±0.5 m noise:
    # the estimate trails it by under a metre.
    f = TargetFilter()
    f.max_speed_mps = 3.0
    lag = 0.0
    for i in range(45):
        t = i / 3.0
        f.update(t, 4.0 * t + (0.5 if i % 2 else -0.5), 0.0, _RANGE_R)
        x, _p = f.predicted(t)  # type: ignore[misc]
        if t > 5.0:
            lag = max(lag, abs(x[0] - 4.0 * t))
    assert lag < 1.0, lag


def test_stop_leaves_the_estimate_where_it_was() -> None:
    f = TargetFilter()
    for i in range(30):
        f.update(i * 0.1, 2.0 * i * 0.1, 0.0, _R)
    last = f.last_update_s
    at_stop, _p = f.predicted(3.2)  # type: ignore[misc]
    f.stop(3.2)
    f.stop(3.7)
    x, _p = f.predicted(4.2)  # type: ignore[misc]
    assert x[2:] == [0.0, 0.0]
    assert abs(x[0] - at_stop[0]) < 1e-9 and abs(x[1] - at_stop[1]) < 1e-9
    # A stop is not a measurement.
    assert f.last_update_s == last
    # The next sightings estimate the motion again.
    for i in range(20):
        t = 4.2 + i * 0.1
        f.update(t, at_stop[0] + 2.0 * (t - 4.2), 0.0, _R)
    x, _p = f.predicted(6.1)  # type: ignore[misc]
    assert abs(x[2] - 2.0) < 0.3
