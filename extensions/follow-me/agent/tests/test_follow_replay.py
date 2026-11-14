"""Replay of a recorded simulated follow flight through the plugin.

The fixture (``fixtures/follow_replay_mine_drift.json``) holds a follow in a
5 m mine drift: the flight controller's ATTITUDE, LOCAL_POSITION_NED and
downward rangefinder, every detector batch with its arrival and its frame's
exposure on the same clock, and the simulator's true bearing and range from
the vehicle to the subject. The plugin runs on the bench's configuration (a
fixed camera 87° wide, 15° below the nose, gimbal pointing left on), fed in
time order, ticking at its own 20 Hz.

The vehicle replays its recorded path, so the goals are not flown; what is
checked is what the plugin asks for while the subject walks across the frame
and the lock turns uncertain (frames 1063 to 1131):

* every goal's yaw keeps the subject inside the camera's half field of view;
* the subject's estimated speed stays at or under the person cap.
"""

from __future__ import annotations

import bisect
import json
import math
from pathlib import Path
from typing import Any

import pytest
from ados.sdk.vision import BoundingBox, Detection, DetectionBatch
from follow_fakes import (
    _arm_guided,
    _attitude,
    _Ctx,
    _distance_sensor,
    _global_position,
    _install_clock,
    _local_position,
    _make_plugin,
)
from follow_me.state import FOLLOW_STATE_TOPIC

_FIXTURE = Path(__file__).parent / "fixtures" / "follow_replay_mine_drift.json"
_FIRST_FRAME, _LAST_FRAME = 1063, 1131
_TICK_S = 0.05
# A person's speed cap (follow_me.target_filter): a run, not a sprint.
_PERSON_CAP_MPS = 3.0


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class _Replay:
    """What the plugin asked for during the replay, keyed by the newest batch
    it had received."""

    def __init__(self, data: dict[str, Any]) -> None:
        self.data = data
        self.goals: list[tuple[float, int, dict[str, Any]]] = []
        self.states: list[tuple[int, dict[str, Any]]] = []
        self._truth_t = [row[0] for row in data["truth"]]

    def truth_bearing(self, t: float) -> float:
        i = min(len(self._truth_t) - 1, bisect.bisect_left(self._truth_t, t))
        return float(self.data["truth"][i][1])

    def window_goals(self) -> list[tuple[float, int, dict[str, Any]]]:
        return [g for g in self.goals if _FIRST_FRAME <= g[1] <= _LAST_FRAME]


async def _replay(monkeypatch: Any) -> _Replay:
    data = json.loads(_FIXTURE.read_text())
    cam = data["camera"]
    ctx = _Ctx(
        config={
            "active": True,
            "designate_camera": "main",
            **data["config"],
        }
    )
    plugin = _make_plugin(ctx)
    clock = _install_clock(monkeypatch)
    fw, fh = cam["frame_wh"]

    # (time, order at equal times, telemetry callable | batch | None for a tick)
    events: list[tuple[float, int, Any]] = []
    for t, r, p, y in data["attitude"]:
        events.append((t, 0, lambda r=r, p=p, y=y: plugin._on_attitude(_attitude(r, p, y))))
    for t, n, e, d, vn, ve in data["local_position"]:
        events.append(
            (t, 0, lambda n=n, e=e, d=d, vn=vn, ve=ve: plugin._on_local_position(_local_position(n, e, d, vn, ve)))
        )
    for t, m in data["rangefinder"]:
        events.append((t, 0, lambda m=m: plugin._on_distance_sensor(_distance_sensor(m))))
    for t, alt in data["global_position"]:
        events.append((t, 0, lambda alt=alt: plugin._on_global_position(_global_position(12.0, 77.0, alt))))
    for rx, exposure_ms, frame_id, dets in data["batches"]:
        batch = DetectionBatch(
            model_id="person",
            camera_id="main",
            frame_id=frame_id,
            ts_ms=exposure_ms,
            frame_width=fw,
            frame_height=fh,
            detections=[
                Detection(
                    bbox=BoundingBox(x=x, y=y, width=w, height=h),
                    class_label=cls,
                    confidence=conf,
                    track_id=tid,
                    lock_state=lock,
                )
                for x, y, w, h, conf, cls, tid, lock in dets
            ],
        )
        events.append((rx, 1, batch))
    t_end = data["batches"][-1][0] + 0.5
    t = data["batches"][0][0]
    while t < t_end:
        events.append((t, 2, None))
        t += _TICK_S
    events.sort(key=lambda ev: (ev[0], ev[1]))

    replay = _Replay(data)
    frame = 0
    for t, kind, item in events:
        clock.now = t
        if kind == 0:
            item()
            continue
        if kind == 1:
            frame = item.frame_id
            plugin._on_batch(item)
            continue
        _arm_guided(plugin)
        sent = len(ctx.flight.setpoints)
        published = len(ctx.events.published)
        await plugin._tick()
        for goal in ctx.flight.setpoints[sent:]:
            replay.goals.append((t, frame, goal))
        for topic, payload in ctx.events.published[published:]:
            if topic == FOLLOW_STATE_TOPIC:
                replay.states.append((frame, payload))
    return replay


@pytest.mark.asyncio
async def test_the_yaw_goal_keeps_the_subject_in_the_frame(monkeypatch: Any) -> None:
    replay = await _replay(monkeypatch)
    half_fov = math.radians(replay.data["camera"]["hfov_deg"]) / 2.0
    goals = [g for g in replay.window_goals() if g[2].get("yaw") is not None]
    # The subject is followed on a live lock for most of the window.
    assert sum(1 for g in goals if g[2]["kind"] == "follow") >= 20, len(goals)
    off = [
        (frame, round(t, 2), g["kind"], round(math.degrees(_wrap(replay.truth_bearing(t) - g["yaw"])), 1))
        for t, frame, g in goals
    ]
    outside = [o for o in off if abs(o[3]) > math.degrees(half_fov)]
    assert not outside, outside


@pytest.mark.asyncio
async def test_the_subject_speed_stays_under_the_person_cap(monkeypatch: Any) -> None:
    replay = await _replay(monkeypatch)
    cap = _PERSON_CAP_MPS
    speeds = [
        (frame, s["target_speed_mps"])
        for frame, s in replay.states
        if _FIRST_FRAME <= frame <= _LAST_FRAME and s.get("target_speed_mps") is not None
    ]
    assert speeds
    assert all(v <= cap + 1e-6 for _f, v in speeds), speeds
    for _t, _frame, g in replay.window_goals():
        assert math.hypot(g["vn"], g["ve"]) <= cap + 1e-6, g
