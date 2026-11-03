"""Constant-velocity Kalman filter tests."""

from __future__ import annotations

import math

from follow_me.target_filter import TargetFilter, position_sigma

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
