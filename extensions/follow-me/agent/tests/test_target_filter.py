"""Constant-velocity Kalman filter tests."""

from __future__ import annotations

import math
import random

from follow_me.target_filter import (
    SCATTER_MIN_FIXES,
    TargetFilter,
    position_sigma,
    subject_max_speed_mps,
)

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
    assert subject_max_speed_mps("person") == 5.0
    assert subject_max_speed_mps(" Person ") == 5.0
    assert subject_max_speed_mps("car") is None
    assert subject_max_speed_mps("") is None
    assert subject_max_speed_mps(None) is None


# Range noise along the line of sight: a walker 8 m north whose range reads
# 4 m long, then 4 m short, at the detector's 3 Hz, each claimed to 1 m.
_RANGE_R = ((1.0, 0.0), (0.0, 0.25))


def _noisy_walker(f: TargetFilter, *, gated: bool = True) -> float:
    """Feed 1.2 m/s east with ±4 m north range noise; the largest speed."""
    top = 0.0
    for i in range(30):
        t = i / 3.0
        f.update(t, 8.0 + (4.0 if i % 2 else -4.0), 1.2 * t, _RANGE_R, gated=gated)
        x, _p = f.predicted(t)  # type: ignore[misc]
        top = max(top, math.hypot(x[2], x[3]))
    return top


def test_range_noise_cannot_make_a_person_faster_than_the_cap() -> None:
    cap = subject_max_speed_mps("person")
    assert cap is not None
    free = TargetFilter()
    assert _noisy_walker(free, gated=False) > cap
    capped = TargetFilter()
    capped.max_speed_mps = cap
    assert _noisy_walker(capped, gated=False) <= cap + 1e-9


def test_the_innovation_gate_keeps_range_outliers_out_of_the_velocity() -> None:
    # The same ±4 m fixes claimed to 1 m are outliers: gated, the estimate
    # stays near the walker's 1.2 m/s with no speed cap at all.
    assert _noisy_walker(TargetFilter()) < 1.6


def _steady_walker(f: TargetFilter, n_fixes: int) -> float:
    """A walker at 1.4 m/s east sighted at 3 Hz with a 0.5 m sigma; the
    time of the last fix."""
    r = ((0.25, 0.0), (0.0, 0.25))
    t = 0.0
    for i in range(n_fixes):
        t = i / 3.0
        f.update(t, 0.0, 1.4 * t, r)
    return t


def test_a_single_range_outlier_moves_the_estimate_less_than_0_2_m() -> None:
    f = TargetFilter()
    t = _steady_walker(f, 15) + 1.0 / 3.0
    before, _p = f.predicted(t)  # type: ignore[misc]
    # One fix 3 m off along the line of sight (a lidar return off a trunk).
    assert f.update(t, 3.0, 1.4 * t, ((0.25, 0.0), (0.0, 0.25))) is False
    after, _p = f.predicted(t)  # type: ignore[misc]
    assert math.hypot(after[0] - before[0], after[1] - before[1]) < 0.2


def test_a_subject_that_really_moved_is_taken_back_within_three_fixes() -> None:
    # The subject is suddenly 4 m north of the track and stays there: the
    # gate gives way on the third fix in a row.
    f = TargetFilter()
    t0 = _steady_walker(f, 15)
    r = ((0.25, 0.0), (0.0, 0.25))
    fused = []
    for k in range(1, 4):
        t = t0 + k / 3.0
        fused.append(f.update(t, 4.0, 1.4 * t, r))
    assert fused == [False, False, True]
    x, _p = f.predicted(t0 + 1.0)  # type: ignore[misc]
    assert abs(x[0] - 4.0) < 1.0


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


def test_the_scatter_ratio_reads_how_far_the_fixes_really_scatter() -> None:
    # A walker 1.4 m/s along the line of sight (north), fixes fused with the
    # 0.75 m range sigma: when they really scatter by it the ratio reads
    # about 1, on clean ranges about 0, and a fresh track's first fixes
    # read nothing.
    r = ((0.75**2, 0.0), (0.0, 0.2**2))
    for noise, low, high in ((0.75, 0.7, 1.3), (0.0, 0.0, 0.1)):
        rnd = random.Random(3)
        f = TargetFilter()
        for i in range(200):
            t = i * 0.1
            f.update(t, 1.4 * t + rnd.gauss(0.0, noise), rnd.gauss(0.0, 0.05 * noise), r)
            if i < SCATTER_MIN_FIXES:
                assert f.scatter_ratio == 0.0
        assert low <= f.scatter_ratio <= high, (noise, f.scatter_ratio)
