"""The widening search at a vantage point: steps of growing ellipse scale,
each a yaw sweep across the predicted position and a pair of gimbal pitches,
and the yaw at a moment within a step."""

from __future__ import annotations

import math
from dataclasses import dataclass

# At most this many steps, however long the budget.
MAX_STEPS = 4
# The first step dwells this long; each later one this much longer.
_DWELL_BASE_S = 4.0
_DWELL_GROWTH_S = 2.0
# Half the yaw sweep of the first step, and how much each step widens it.
_SPAN_BASE_DEG = 45.0
_SPAN_GROWTH_DEG = 15.0
# Gimbal pitches (negative = down) looked through within each step: near
# ground first, then nearer the vehicle.
_GIMBAL_PITCHES_DEG = (-15.0, -30.0)


@dataclass(frozen=True)
class SearchBudget:
    budget_s: float
    radius_m: float
    speed_mps: float


@dataclass(frozen=True)
class SearchStep:
    index: int
    # Ellipse scale in sigmas the pilot's coverage test uses: 1, 2, 3 ...
    sigma_k: float
    yaw_offsets_deg: tuple[float, ...]
    gimbal_pitch_deg: tuple[float, ...]
    dwell_s: float


def search_steps(budget: SearchBudget) -> list[SearchStep]:
    """The steps that fit the budget, widening each time."""
    steps: list[SearchStep] = []
    used = 0.0
    for k in range(MAX_STEPS):
        dwell = _DWELL_BASE_S + _DWELL_GROWTH_S * k
        if used + dwell > budget.budget_s:
            break
        span = _SPAN_BASE_DEG + _SPAN_GROWTH_DEG * k
        steps.append(
            SearchStep(
                index=k,
                sigma_k=1.0 + k,
                yaw_offsets_deg=(-span, -span / 2.0, 0.0, span / 2.0, span),
                gimbal_pitch_deg=_GIMBAL_PITCHES_DEG,
                dwell_s=dwell,
            )
        )
        used += dwell
    return steps


def sweep_yaw(step: SearchStep, base_yaw_rad: float, t_in_step_s: float) -> float:
    """The yaw ``t_in_step_s`` into a step: linear through the offsets, from
    the first at the start to the last at the end of the dwell."""
    offs = step.yaw_offsets_deg
    f = min(1.0, max(0.0, t_in_step_s / step.dwell_s)) * (len(offs) - 1)
    i = min(int(f), len(offs) - 2)
    deg = offs[i] + (offs[i + 1] - offs[i]) * (f - i)
    return base_yaw_rad + math.radians(deg)


def sweep_pitch(step: SearchStep, t_in_step_s: float) -> float:
    """The gimbal pitch ``t_in_step_s`` into a step: the dwell split evenly
    across the step's pitches."""
    pitches = step.gimbal_pitch_deg
    f = min(0.999, max(0.0, t_in_step_s / step.dwell_s))
    return pitches[int(f * len(pitches))]


__all__ = ["MAX_STEPS", "SearchBudget", "SearchStep", "search_steps", "sweep_pitch", "sweep_yaw"]
