"""The widening search steps and the yaw sweep within one step."""

from __future__ import annotations

import math

import pytest
from follow_me.search import SearchBudget, search_steps, sweep_yaw


def test_steps_widen_and_fit_the_budget() -> None:
    steps = search_steps(SearchBudget(budget_s=25.0, radius_m=12.0, speed_mps=2.0))
    assert [s.sigma_k for s in steps][:3] == [1.0, 2.0, 3.0]
    assert sum(s.dwell_s for s in steps) <= 25.0
    assert all(s.yaw_offsets_deg[0] < 0 < s.yaw_offsets_deg[-1] for s in steps)


def test_a_short_budget_still_gets_one_step_when_it_fits() -> None:
    assert [s.index for s in search_steps(SearchBudget(5.0, 12.0, 2.0))] == [0]
    assert search_steps(SearchBudget(3.0, 12.0, 2.0)) == []


def test_sweep_starts_at_the_first_offset_and_ends_at_the_last() -> None:
    s = search_steps(SearchBudget(25.0, 12.0, 2.0))[0]
    assert sweep_yaw(s, 0.0, 0.0) == pytest.approx(math.radians(s.yaw_offsets_deg[0]))
    assert sweep_yaw(s, 0.0, s.dwell_s) == pytest.approx(math.radians(s.yaw_offsets_deg[-1]))
