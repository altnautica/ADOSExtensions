"""Designation to first command: the follow's share of the lock latency.

The operator's tap designates the subject through the vision engine; its
first ``locked`` batch is the designation as the plugin sees it. The
target: tap to the first follow command within 0.2 s, and tap to the
vehicle moving faster than 0.3 m/s within 1.0 s. The plugin's share of the
first is one loop period at most (a batch waits for the next tick) plus its
own processing; the engine's share (the designate route and the crop pass
that locks) is measured on the bench.
"""

from __future__ import annotations

import inspect
import time
from typing import Any

import pytest
from follow_sim import fly

import follow_me
from follow_me.loop import _LOOP_PERIOD

_TAP_TO_COMMAND_S = 0.2


@pytest.mark.asyncio
async def test_a_designation_commands_before_the_next_batch(monkeypatch: Any) -> None:
    # The handling of every batch and tick, timed on the wall clock.
    spent: list[float] = []

    def timed(fn: Any) -> Any:
        if inspect.iscoroutinefunction(fn):

            async def wrapper(self: Any, *a: Any, **k: Any) -> Any:
                t0 = time.perf_counter()
                try:
                    return await fn(self, *a, **k)
                finally:
                    spent.append(time.perf_counter() - t0)

        else:

            def wrapper(self: Any, *a: Any, **k: Any) -> Any:
                t0 = time.perf_counter()
                try:
                    return fn(self, *a, **k)
                finally:
                    spent.append(time.perf_counter() - t0)

        return wrapper

    cls = follow_me.FollowMePlugin
    monkeypatch.setattr(cls, "_on_batch", timed(cls._on_batch))
    monkeypatch.setattr(cls, "_tick", timed(cls._tick))

    # A standing subject 14 m ahead, designated by the first batch at t = 0;
    # the agent's 10 Hz detector after it.
    f = await fly(
        monkeypatch,
        subject=lambda _t: (14.0, 0.0),
        start=(0.0, 0.0),
        duration_s=1.5,
        fix_hz=10.0,
        range_sigma_m=0.0,
        outlier_share=0.0,
    )
    # The first locked batch is fused and commanded on the tick it arrives
    # before: in the running loop that tick is at most one period later,
    # before the next batch.
    first = f.goals[0]
    assert first.get("kind") == "follow"
    assert first.get("vel_mode") == "command"
    assert _LOOP_PERIOD < 0.1
    # Handling a batch and a tick costs milliseconds, so the plugin's share
    # of tap to command is the loop period.
    worst = max(spent)
    print(f"plugin share: {_LOOP_PERIOD * 1000:.0f} ms wait + {worst * 1000:.2f} ms worst handling")
    assert _LOOP_PERIOD + worst <= _TAP_TO_COMMAND_S / 2
    # The command speeds up from the first tick, and the vehicle (the
    # pilot's Normal profile) is past 0.3 m/s within a second of the batch.
    speeds = [((g["vn"] ** 2 + g["ve"] ** 2) ** 0.5) for g in f.goals[:3]]
    assert speeds[0] < speeds[1] < speeds[2]
    moving = next(f.t[i] for i in range(len(f.t)) if f.speed(i) > 0.3)
    print(f"vehicle past 0.3 m/s {moving:.2f} s after the designation")
    assert moving + _LOOP_PERIOD <= 1.0
