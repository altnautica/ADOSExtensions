"""Unit tests for the telemetry freshness gate.

The gate is the difference between "we have seen this input" and "this
input is still true". The tests below pin the second reading, because the
first is what the follow loop used to do and it is what let a stalled
telemetry subscription steer the aircraft from frozen values.
"""

from __future__ import annotations

import pytest

from follow_me.freshness import FreshnessGate


def test_a_gate_with_no_arrival_is_never_fresh() -> None:
    # Nothing has arrived, so there is nothing to trust: a caller must hold
    # rather than command on whatever initial values it was constructed with.
    gate = FreshnessGate(0.5)
    assert gate.has_reported is False
    assert gate.is_fresh(0.0) is False
    assert gate.is_fresh(1_000.0) is False
    assert gate.age_s(1_000.0) is None


def test_a_recent_arrival_is_fresh() -> None:
    gate = FreshnessGate(0.5)
    gate.mark(100.0)
    assert gate.has_reported is True
    assert gate.is_fresh(100.0) is True
    assert gate.is_fresh(100.4) is True
    assert gate.age_s(100.4) == pytest.approx(0.4)


def test_freshness_expires_at_the_window_edge() -> None:
    gate = FreshnessGate(0.5)
    gate.mark(100.0)
    # Exactly at the window the arrival still counts; past it, it does not.
    assert gate.is_fresh(100.5) is True
    assert gate.is_fresh(100.51) is False


def test_a_stale_gate_still_reports_that_it_once_had_data() -> None:
    # "Stopped arriving" and "never arrived" are different faults and a
    # caller may want to say which; staleness must not erase the distinction.
    gate = FreshnessGate(0.5)
    gate.mark(100.0)
    assert gate.is_fresh(200.0) is False
    assert gate.has_reported is True
    assert gate.age_s(200.0) == 100.0


def test_a_later_arrival_refreshes_the_gate() -> None:
    gate = FreshnessGate(0.5)
    gate.mark(100.0)
    assert gate.is_fresh(101.0) is False
    gate.mark(101.0)
    assert gate.is_fresh(101.0) is True


def test_reset_returns_the_gate_to_never_reported() -> None:
    gate = FreshnessGate(0.5)
    gate.mark(100.0)
    gate.reset()
    assert gate.has_reported is False
    assert gate.is_fresh(100.0) is False


def test_a_negative_window_fails_safe_rather_than_fresh_forever() -> None:
    # A misconfigured negative window must not invert the gate into
    # "everything is fresh"; it clamps to the strictest possible window.
    gate = FreshnessGate(-5.0)
    assert gate.max_age_s == 0.0
    gate.mark(100.0)
    assert gate.is_fresh(100.0) is True
    assert gate.is_fresh(100.001) is False
