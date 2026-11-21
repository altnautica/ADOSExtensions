"""The engine's subject state (``vision.subject.v1``): flown on while fresh and
on the locked track, the own estimate otherwise, a release taken as a loss,
and the phone beacon alone keeping the follow on its feet within the
separation guard."""

from __future__ import annotations

import math
from types import SimpleNamespace
from typing import Any

import ados.sdk.vision as sdk_vision
import pytest
from follow_fakes import (
    _Clock,
    _Ctx,
    _events,
    _install_clock,
    _make_plugin,
    _state_events,
    _telemetry_config,
)
from follow_sim import fly
from test_follow_v03 import _feed, _follow_one, _fresh, _person
from test_separation import _RANGEFINDER_RESOLUTION_M, _clearance_3d

import follow_me
from follow_me.state import (
    FOLLOW_LOST_TOPIC,
    HOLD_NO_RANGE,
    LOCK_LOST,
    OVERHEAD_CLEARANCE_M,
    PHASE_AWAIT_OPERATOR,
    PHASE_FOLLOWING,
    PHASE_PREDICTING,
    PHASE_REPOSITIONING,
)
from follow_me.subject import SUBJECT_FRESH_S, SubjectFeed

_HORIZONS_S = (0.5, 1.0, 1.5, 2.0, 3.0)


def _cov(var: float) -> tuple[float, ...]:
    return (var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var)


def _state(
    now: float,
    *,
    pos: tuple[float, float] = (12.0, -5.0),
    vel: tuple[float, float] = (0.0, 0.0),
    sigma: float = 0.5,
    sources: tuple[str, ...] = ("box", "ground"),
    age_s: float = 0.0,
    fix_age_ms: int = 0,
    track_id: int = 7,
    camera_id: str = "uvc-0",
    bend: tuple[float, float] = (0.0, 0.0),
) -> SimpleNamespace:
    """A state record shaped as ``ados.sdk.vision.SubjectState``: the
    predictions run on at ``vel`` plus ``bend`` per second (a map-aware
    prediction that turns off the straight line)."""

    def at(dt: float) -> tuple[float, float, float]:
        return (
            pos[0] + (vel[0] + bend[0]) * dt,
            pos[1] + (vel[1] + bend[1]) * dt,
            0.0,
        )

    return SimpleNamespace(
        camera_id=camera_id,
        id=track_id,
        stamp_ns=round((now - age_s) * 1e9),
        fix_age_ms=fix_age_ms,
        pos_ned=(pos[0], pos[1], 0.0),
        cov=_cov(sigma * sigma),
        vel_ned=(vel[0], vel[1], 0.0),
        mode_probs=(1.0, 0.0, 0.0),
        height_m=1.7,
        predictions=tuple(
            SimpleNamespace(
                dt_s=dt, pos_ned=at(dt), cov=_cov(sigma * sigma + 0.25 * dt * dt)
            )
            for dt in _HORIZONS_S
        ),
        sources=frozenset(sources),
    )


def _released(track_id: int = 7, camera_id: str = "uvc-0") -> SimpleNamespace:
    return SimpleNamespace(camera_id=camera_id, id=track_id, stamp_ns=0)


def _plugin(
    monkeypatch: Any, **over: Any
) -> tuple[_Ctx, follow_me.FollowMePlugin, _Clock]:
    cfg = _telemetry_config()
    cfg.update(over)
    ctx = _Ctx(config=cfg)
    plugin = _make_plugin(ctx)
    plugin._active_camera = "uvc-0"
    return ctx, plugin, _install_clock(monkeypatch)


def test_a_state_is_read_between_its_predictions_with_their_spread() -> None:
    feed = SubjectFeed()
    feed.on_record(
        _state(10.0, pos=(4.0, 1.0), vel=(2.0, 0.0), bend=(0.0, 1.0), age_s=0.25)
    )
    view = feed.view(10.0, 7, "uvc-0")
    assert view is not None and view.source == "vision" and not view.beacon_only
    x, p = view.at(10.0)
    # A quarter second on, half way to the 0.5 s prediction.
    assert x[:2] == pytest.approx([4.5, 1.25])
    assert x[2:] == pytest.approx([2.0, 0.0])
    assert p[0][0] == pytest.approx(0.25 + 0.5 * 0.0625)
    # The velocity spread is how fast the predictions' spread grows: 0.5 m/s.
    assert p[2][2] == pytest.approx(0.25) and p[3][3] == pytest.approx(0.25)
    assert view.path_ne(10.0)[0] == [4.5, 1.25]
    # Measured just now: no coast spread; 1.5 s without a fix: 1 s of it.
    assert view.coast_spread_m(10.0) == 0.0
    feed.on_record(_state(10.0, fix_age_ms=1500, sources=()))
    view = feed.view(10.0, 7, "uvc-0")
    assert view is not None and view.source == "predicted"
    assert view.coast_spread_m(10.0) == pytest.approx(0.5)


@pytest.mark.parametrize(
    "why, kw, track, camera",
    [
        ("stale", {"age_s": SUBJECT_FRESH_S + 0.01}, 7, "uvc-0"),
        ("ahead of the clock", {"age_s": -0.5}, 7, "uvc-0"),
        ("another track", {}, 8, "uvc-0"),
        ("another camera", {"camera_id": "uvc-1"}, 7, "uvc-0"),
        ("no lock", {}, None, "uvc-0"),
    ],
)
def test_a_state_is_flown_only_fresh_and_on_the_locked_track(
    why: str, kw: dict[str, Any], track: int | None, camera: str
) -> None:
    feed = SubjectFeed()
    feed.on_record(_state(10.0, **kw))
    assert feed.view(10.0, track, camera) is None, why


def test_a_release_drops_the_state_and_is_taken_once() -> None:
    feed = SubjectFeed()
    feed.on_record(_state(10.0))
    feed.on_record(_released(8))
    assert feed.view(10.0, 7, "uvc-0") is not None
    assert not feed.take_release(7, "uvc-0")
    feed.on_record(_released(7))
    assert feed.view(10.0, 7, "uvc-0") is None
    assert feed.take_release(7, "uvc-0")
    assert not feed.take_release(7, "uvc-0")


def test_the_sdk_record_decodes_into_what_the_feed_reads() -> None:
    decode = getattr(sdk_vision, "decode_subject_record", None)
    if decode is None:
        pytest.skip("host SDK without vision.subject.v1")
    rec = _state(10.0, sources=("beacon",))
    payload = {
        "v": 1,
        "camera_id": rec.camera_id,
        "id": rec.id,
        "stamp_ns": rec.stamp_ns,
        "fix_age_ms": 200,
        "pos_ned": list(rec.pos_ned),
        "cov": list(rec.cov),
        "vel_ned": list(rec.vel_ned),
        "mode_probs": list(rec.mode_probs),
        "height_m": 1.7,
        "predictions": [
            {"dt_s": p.dt_s, "pos_ned": list(p.pos_ned), "cov": list(p.cov)}
            for p in rec.predictions
        ],
        "sources": ["beacon"],
    }
    feed = SubjectFeed()
    feed.on_record(decode(payload))
    view = feed.view(10.0, 7, "uvc-0")
    assert view is not None and view.beacon_only and view.source == "beacon"
    assert view.at(10.0)[0][:2] == pytest.approx([12.0, -5.0])
    feed.on_record(
        decode({"v": 1, "camera_id": "uvc-0", "id": 7, "stamp_ns": 1, "released": True})
    )
    assert feed.take_release(7, "uvc-0")


@pytest.mark.asyncio
async def test_start_subscribes_to_the_subject_when_the_host_has_it(
    monkeypatch: Any,
) -> None:
    calls: list[str | None] = []

    async def subscribe_subject(cb: Any, *, camera_id: str | None = None) -> None:
        calls.append(camera_id)

    ctx = _Ctx(config={"active": False})
    ctx.vision.subscribe_subject = subscribe_subject  # type: ignore[attr-defined]
    plugin = follow_me.FollowMePlugin()
    await plugin.on_start(ctx)
    await plugin.on_stop(ctx)
    assert calls == [None]
    # A host without the topic leaves the follow on its own estimate.
    bare = _Ctx(config={"active": False})
    plugin = follow_me.FollowMePlugin()
    await plugin.on_start(bare)
    await plugin.on_stop(bare)


def _ring_center(goal: dict[str, Any]) -> list[float]:
    return list(goal["candidates"]["ring_center_ne"])


@pytest.mark.asyncio
async def test_a_fresh_subject_state_drives_the_goal(monkeypatch: Any) -> None:
    ctx, plugin, clock = _plugin(monkeypatch)
    plugin._on_subject(_state(clock.now, pos=(12.0, -5.0), vel=(1.0, 0.5)))
    await _follow_one(plugin, clock)
    goal = ctx.flight.setpoints[-1]
    assert goal["kind"] == "follow"
    assert _ring_center(goal) == pytest.approx([12.0, -5.0])
    assert goal["subject_ne"] == pytest.approx([12.0, -5.0])
    state = _state_events(ctx)[-1]
    assert state["commanding"] is True and state["subject_source"] == "vision"
    assert state["target_ned"] == [12.0, -5.0] and state["target_vel_ned"] == [1.0, 0.5]
    assert state["predicted_ned"][0] == [12.0, -5.0]


@pytest.mark.parametrize(
    "kw",
    [{"age_s": SUBJECT_FRESH_S + 0.05}, {"track_id": 8}, {"camera_id": "uvc-1"}],
    ids=["stale", "other-track", "other-camera"],
)
@pytest.mark.asyncio
async def test_a_stale_or_foreign_state_falls_back_to_the_own_estimate(
    monkeypatch: Any, kw: dict[str, Any]
) -> None:
    ctx, plugin, clock = _plugin(monkeypatch)
    plugin._on_subject(_state(clock.now, pos=(12.0, -5.0), **kw))
    await _follow_one(plugin, clock)
    goal = ctx.flight.setpoints[-1]
    own = plugin._filter.coasted(clock.now)
    assert own is not None
    assert _ring_center(goal) == pytest.approx(own[0][:2])
    assert math.dist(_ring_center(goal), [12.0, -5.0]) > 1.0
    assert _state_events(ctx)[-1]["subject_source"] is None


@pytest.mark.asyncio
async def test_a_fresh_subject_state_commands_without_an_own_range_fix(
    monkeypatch: Any,
) -> None:
    ctx, plugin, clock = _plugin(monkeypatch)
    # No box ever ranges: the own filter never starts.
    monkeypatch.setattr(plugin, "_measure_box", lambda *_a, **_k: None)
    await _follow_one(plugin, clock)
    assert _state_events(ctx)[-1]["hold_reason"] == HOLD_NO_RANGE
    clock.advance(0.05)
    plugin._on_subject(_state(clock.now))
    await _follow_one(plugin, clock)
    assert ctx.flight.setpoints[-1]["kind"] == "follow"
    assert _state_events(ctx)[-1]["commanding"] is True


@pytest.mark.parametrize("mode", ["smart", "strict"])
@pytest.mark.asyncio
async def test_a_release_is_a_lost_subject(monkeypatch: Any, mode: str) -> None:
    ctx, plugin, clock = _plugin(monkeypatch, reacquire_mode=mode)
    plugin._on_subject(_state(clock.now))
    await _follow_one(plugin, clock)
    clock.advance(0.05)
    plugin._on_subject(_released(7))
    # The engine still reports the box locked: the release decides.
    await _follow_one(plugin, clock)
    assert plugin._recovery is not None
    state = _state_events(ctx)[-1]
    assert state["subject_source"] is None
    if mode == "smart":
        assert state["phase"] == PHASE_PREDICTING
    else:
        assert state["phase"] == PHASE_AWAIT_OPERATOR
        assert _events(ctx, FOLLOW_LOST_TOPIC)[-1]["reason"] == "lock_lost"
    # The released track never re-latches.
    _feed(plugin, _person(track_id=7, lock_state="locked"))
    assert plugin._tracker.has_lock is False


async def _lose(plugin: follow_me.FollowMePlugin, clock: _Clock, record: Any) -> None:
    clock.advance(0.05)
    _fresh(plugin)
    _feed(plugin, _person(track_id=7, lock_state=LOCK_LOST))
    plugin._on_subject(record)
    await plugin._tick()


@pytest.mark.asyncio
async def test_the_beacon_alone_keeps_following_a_lost_lock_and_says_so(
    monkeypatch: Any,
) -> None:
    ctx, plugin, clock = _plugin(monkeypatch)
    await _follow_one(plugin, clock)
    beacon = _state(
        clock.now + 0.05,
        pos=(20.0, 3.0),
        vel=(1.4, 0.0),
        sigma=2.0,
        sources=("beacon",),
    )
    await _lose(plugin, clock, beacon)
    assert plugin._recovery is None and plugin._tracker.has_lock
    goal = ctx.flight.setpoints[-1]
    assert goal["kind"] == "follow" and _ring_center(goal) == pytest.approx([20.0, 3.0])
    # The beacon's spread is the error the pilot keeps the subject clear by.
    assert goal["subject_sigma_m"] > 0.0
    state = _state_events(ctx)[-1]
    assert state["phase"] == PHASE_FOLLOWING and state["lock_state"] == LOCK_LOST
    assert state["commanding"] is True and state["subject_source"] == "beacon"
    assert state["range_m"] == pytest.approx(math.hypot(20.0, 3.0), abs=0.01)
    # The beacon goes quiet: the loss path starts.
    clock.advance(SUBJECT_FRESH_S + 0.05)
    _fresh(plugin)
    _feed(plugin, _person(track_id=7, lock_state=LOCK_LOST))
    await plugin._tick()
    assert plugin._recovery is not None
    assert _state_events(ctx)[-1]["phase"] == PHASE_PREDICTING


@pytest.mark.parametrize(
    "over",
    [{"reacquire_mode": "strict"}, {"uncertain_mode": "hold"}],
    ids=["strict", "hold"],
)
@pytest.mark.asyncio
async def test_the_beacon_alone_never_flies_a_strict_or_holding_follow(
    monkeypatch: Any, over: dict[str, Any]
) -> None:
    ctx, plugin, clock = _plugin(monkeypatch, **over)
    await _follow_one(plugin, clock)
    await _lose(plugin, clock, _state(clock.now + 0.05, sources=("beacon",)))
    # Strict waits for the operator; hold starts the recovery: neither keeps
    # following on the phone.
    assert plugin._recovery is not None
    assert _state_events(ctx)[-1]["phase"] != PHASE_FOLLOWING


@pytest.mark.asyncio
async def test_a_recovery_predicts_with_the_subject_state(monkeypatch: Any) -> None:
    ctx, plugin, clock = _plugin(monkeypatch)
    await _follow_one(plugin, clock)
    # Vision lost, no beacon: the engine coasts on its own prediction.
    coast = {"pos": (30.0, 10.0), "vel": (1.0, 0.0), "bend": (0.0, 2.0), "sources": ()}
    await _lose(plugin, clock, _state(clock.now + 0.05, **coast))
    assert plugin._recovery is not None
    assert _state_events(ctx)[-1]["phase"] == PHASE_PREDICTING
    # Past the prediction leg the vantage request names where the engine
    # predicts the subject now, off the straight line.
    clock.advance(2.1)
    _fresh(plugin)
    _feed(plugin)
    plugin._on_subject(_state(clock.now, age_s=0.3, **coast))
    await plugin._tick()
    goal = ctx.flight.setpoints[-1]
    assert goal["kind"] == "vantage"
    assert _state_events(ctx)[-1]["phase"] == PHASE_REPOSITIONING
    assert goal["vantage"]["predicted_ne"] == pytest.approx([30.3, 10.6])
    assert goal["subject_ne"] == pytest.approx([30.3, 10.6])


def _walker(t: float) -> tuple[float, float]:
    return (30.0 + 1.4 * t, 0.0)


def _turns_back(t: float) -> tuple[float, float]:
    """The walker, who at 14 s turns straight back at the vehicle."""
    return _walker(t) if t <= 14.0 else (30.0 + 1.4 * (28.0 - t), 0.0)


async def _occluded_flight(
    monkeypatch: Any, *, beacon: bool, subject: Any = _walker, **kw: Any
) -> Any:
    """A walker followed from 6 m behind, vision lost from 10 s to 20 s;
    with ``beacon`` the phone's fix (2 m spread, a slowly wandering error of
    up to 1.1 m) feeds the engine's state alone through the gap."""
    tick = follow_me.FollowMePlugin._tick

    async def with_beacon(self: follow_me.FollowMePlugin) -> None:
        now = follow_me.fc_state.monotonic()
        t = now - 1000.0
        if beacon and 10.0 <= t < 20.0:
            n, e = subject(t)
            ahead = subject(t + 0.1)
            err = (0.8 * math.sin(0.4 * t), 0.8 * math.cos(0.3 * t))
            self._on_subject(
                _state(
                    now,
                    pos=(n + err[0], e + err[1]),
                    vel=((ahead[0] - n) / 0.1, (ahead[1] - e) / 0.1),
                    sigma=2.0,
                    sources=("beacon",),
                    fix_age_ms=100,
                )
            )
        await tick(self)

    monkeypatch.setattr(follow_me.FollowMePlugin, "_tick", with_beacon)
    return await fly(
        monkeypatch,
        subject=subject,
        start=(24.0, 0.0),
        duration_s=26.0,
        fix_hz=10.0,
        # Frames 101-200: t 10.0-19.9 s.
        miss_frames=tuple(range(101, 201)),
        **kw,
    )


@pytest.mark.asyncio
async def test_the_beacon_alone_holds_the_follow_through_a_ten_second_occlusion(
    monkeypatch: Any,
) -> None:
    f = await _occluded_flight(monkeypatch, beacon=True)
    gap = [i for i in f.after(10.5) if f.t[i] < 20.0]
    assert all(f.phases[i] == PHASE_FOLLOWING for i in gap)
    assert all(f.state[i]["subject_source"] == "beacon" for i in gap)
    # The separation guard holds on the beacon's estimate, and the 6 m band
    # holds through the gap and after it.
    assert min(f.range(i) for i in range(len(f.t))) >= 4.5
    assert all(4.5 <= f.range(i) <= 8.0 for i in f.after(12.0))
    assert f.state[-1]["subject_source"] is None


@pytest.mark.asyncio
async def test_the_beacon_alone_keeps_a_subject_turning_back_clear_of_the_rotors(
    monkeypatch: Any,
) -> None:
    # Straight back at the vehicle 4 s into the gap, at the lowest follow
    # height: the same bar as a turn-back seen by the camera, the body under
    # the rotors with a metre to spare, and here not under them at all.
    f = await _occluded_flight(
        monkeypatch, beacon=True, subject=_turns_back, config={"follow_height_m": 1.5}
    )
    idx = f.after(14.0)
    assert (
        min(_clearance_3d(f, i) for i in idx)
        >= OVERHEAD_CLEARANCE_M - _RANGEFINDER_RESOLUTION_M
    )
    assert min(f.range(i) for i in idx) >= 1.0


@pytest.mark.asyncio
async def test_without_the_beacon_the_same_occlusion_is_a_loss(
    monkeypatch: Any,
) -> None:
    f = await _occluded_flight(monkeypatch, beacon=False)
    gap = [i for i in f.after(14.0) if f.t[i] < 20.0]
    assert all(f.phases[i] != PHASE_FOLLOWING for i in gap)
