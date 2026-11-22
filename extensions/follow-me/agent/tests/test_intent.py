"""Follow intents: the goal sent to a pilot that plans the view itself
(``pilot.v1`` version 6), and the hard guard kept as a check on it."""

from __future__ import annotations

from dataclasses import replace
from typing import Any

import pytest
from ados.sdk.vision import Detection, DetectionBatch
from follow_fakes import (
    _arm_guided,
    _bbox_center_frame,
    _Ctx,
    _install_clock,
    _level_pose,
    _make_plugin,
    _state_events,
    _telemetry_config,
)

from follow_me import intent
from follow_me.standoff import SAFE_DISTANCE_M, Command, clear_distance
from follow_me.state import LOCK_LOCKED, MIN_FOLLOW_HEIGHT_M, FollowConfig


@pytest.fixture
def intents(monkeypatch: pytest.MonkeyPatch) -> None:
    """The pilot takes follow intents."""
    monkeypatch.setattr(intent, "pilot_takes_intents", lambda: True)


def _ring(kind: str = "follow") -> dict[str, Any]:
    return intent.ring_goal(
        kind,
        (1.0, 2.0, -4.0),
        Command(1.0, 2.0, 0.8, 0.1, 0.3),
        FollowConfig(),
        ring={
            "ring_center_ne": [7.0, 2.0],
            "radius_m": 6.0,
            "angle_deg": 180.0,
            "angle_span_deg": 90.0,
        },
        subject_ne=(7.0, 2.0),
        subject_sigma_m=0.4,
    )


def _send(
    goal: dict[str, Any], cfg: FollowConfig, drone: tuple[float, ...]
) -> dict[str, Any]:
    return intent.follow_goal(
        goal,
        cfg=cfg,
        pace_ne=(1.4, 0.0),
        standoff_m=6.0,
        subject_ref=12,
        drone=drone,  # type: ignore[arg-type]
        guard=((7.0, 2.0), (1.4, 0.0), 0.0),
    )


def test_an_older_pilot_gets_the_ring_goal() -> None:
    # The autouse fixture: no intents.
    g = _send(_ring(), FollowConfig(), (1.0, 2.0, 0.0, 0.0))
    assert g["kind"] == "follow"
    assert g["vel_mode"] == intent.VEL_MODE_COMMAND
    assert g["candidates"]["radius_m"] == 6.0


def test_a_pilot_at_version_6_gets_the_intent(intents: None) -> None:
    cfg = FollowConfig(follow_angle_deg=90.0, follow_height_m=5.0)
    g = _send(_ring(), cfg, (1.0, 2.0, 0.0, 0.0))
    assert g["kind"] == intent.KIND_FOLLOW_INTENT
    # Its own point, height, heading, subject and margin as they were.
    assert (g["n"], g["e"], g["d"]) == (1.0, 2.0, -4.0)
    assert g["yaw"] == 0.3 and g["subject_ne"] == [7.0, 2.0]
    assert g["subject_sigma_m"] == 0.4 and g["margin_m"] == 1.5
    # The subject's velocity fed forward, no ring.
    assert (g["vn"], g["ve"]) == (1.4, 0.0)
    assert g["vel_mode"] == "feed_forward" and g["candidates"] is None
    assert g["standoff_band_m"] == [5.0, 7.0]
    assert g["bearing_pref_deg"] == 90.0
    assert g["height_band_m"] == [5.0, 6.0]
    assert g["subject_ref"] == 12 and g["require_los"] is True
    assert g["bearing_frame"] == intent.BEARING_FRAME_SUBJECT


def test_bearings_bands_and_heights(intents: None) -> None:
    assert intent.bearing_pref_deg(FollowConfig(follow_angle_deg=270.0)) == -90.0
    assert intent.bearing_pref_deg(FollowConfig(follow_angle_deg=180.0)) == -180.0
    assert intent.bearing_pref_deg(FollowConfig(follow_angle_auto=True)) is None
    # The band's near edge stays a metre past the guard's safe distance.
    assert intent.standoff_band_m(3.0) == [SAFE_DISTANCE_M + 1.0, 4.0]
    # The height band never starts under the lowest follow height, and a
    # height lowered under leaves is held.
    assert (
        intent.height_band_m(FollowConfig(follow_height_m=2.0))[0]
        == MIN_FOLLOW_HEIGHT_M
    )
    lowered = FollowConfig(follow_height_m=3.6, follow_height_lowered=True)
    assert intent.height_band_m(lowered) == [3.6, 3.6]


def test_an_orbit_is_an_intent_at_its_phase_over_the_ground(intents: None) -> None:
    goal = _ring("orbit")
    goal["candidates"]["angle_deg"] = 300.0
    g = _send(goal, FollowConfig(), (1.0, 2.0, 0.0, 0.0))
    assert g["kind"] == intent.KIND_FOLLOW_INTENT
    assert g["bearing_frame"] == intent.BEARING_FRAME_WORLD
    assert g["bearing_pref_deg"] == -60.0
    assert g["candidates"] is None


def test_the_hard_guard_takes_back_the_goal(intents: None) -> None:
    clear = clear_distance(6.0, 0.0, 0.0)
    # Inside the clear distance: the follow's own guarded command.
    g = _send(_ring(), FollowConfig(), (7.0 - clear + 0.1, 2.0, 0.0, 0.0))
    assert g["kind"] == "follow"
    # Outside it but closing fast enough to be there within the guard's lag.
    g = _send(_ring(), FollowConfig(), (7.0 - clear - 1.0, 2.0, 3.0, 0.0))
    assert g["kind"] == "follow"
    # Outside it and keeping pace: the intent.
    g = _send(_ring(), FollowConfig(), (7.0 - clear - 1.0, 2.0, 1.4, 0.0))
    assert g["kind"] == intent.KIND_FOLLOW_INTENT


async def _follow_one_tick(
    monkeypatch: Any, goal_reply: dict[str, Any] | None = None
) -> _Ctx:
    """One tick of a locked follow; the context it sent to. ``goal_reply`` is
    what the pilot answers every goal with (accepted when None)."""
    ctx = _Ctx(config=_telemetry_config())
    if goal_reply is not None:
        ctx.flight.goal_reply = goal_reply
    plugin = _make_plugin(ctx)
    plugin._active_camera = "uvc-0"
    clock = _install_clock(monkeypatch)
    _level_pose(plugin)
    _arm_guided(plugin)
    plugin._on_batch(
        DetectionBatch(
            model_id="coco-person",
            camera_id="uvc-0",
            frame_id=1,
            ts_ms=int(clock.now * 1000) - 80,
            frame_width=640,
            frame_height=480,
            detections=[
                Detection(
                    bbox=_bbox_center_frame(),
                    class_label="person",
                    confidence=0.9,
                    track_id=7,
                    lock_state=LOCK_LOCKED,
                )
            ],
        )
    )
    await plugin._tick()
    return ctx


@pytest.mark.asyncio
async def test_the_follow_sends_intents_to_a_pilot_that_takes_them(
    monkeypatch: Any, intents: None
) -> None:
    ctx = await _follow_one_tick(monkeypatch)
    goal = ctx.flight.setpoints[-1]
    assert goal["kind"] == intent.KIND_FOLLOW_INTENT
    assert goal["bearing_pref_deg"] == -180.0
    assert goal["standoff_band_m"] == [5.0, 7.0]
    assert goal["height_band_m"][0] >= MIN_FOLLOW_HEIGHT_M
    # No engine subject state: the pilot takes whichever it reads.
    assert goal["subject_ref"] is None
    # The auto angle leaves the bearing to the pilot.
    cfg = replace(FollowConfig(), follow_angle_auto=True)
    auto = intent.intent_goal(
        goal, cfg=cfg, pace_ne=(0.0, 0.0), standoff_m=6.0, subject_ref=None
    )
    assert auto["bearing_pref_deg"] is None


@pytest.mark.asyncio
@pytest.mark.parametrize("takes", [True, False], ids=["intent", "ring"])
async def test_the_read_back_names_the_goal_kind_sent(
    monkeypatch: Any, takes: bool
) -> None:
    # Which goal the pilot was sent is read back, not inferred from the
    # pilot's flight.
    monkeypatch.setattr(intent, "pilot_takes_intents", lambda: takes)
    ctx = await _follow_one_tick(monkeypatch)
    state = _state_events(ctx)[-1]
    assert state["commanding"] is True
    assert state["goal_kind"] == ctx.flight.setpoints[-1]["kind"]
    assert state["goal_kind"] == (intent.KIND_FOLLOW_INTENT if takes else "follow")


@pytest.mark.asyncio
async def test_a_refused_goal_is_logged_and_names_no_goal_kind(
    monkeypatch: Any, intents: None
) -> None:
    # A refused goal is not flown: the read-back must not claim it is.
    ctx = await _follow_one_tick(
        monkeypatch, goal_reply={"ok": False, "err": "not_holder"}
    )
    state = _state_events(ctx)[-1]
    assert state["commanding"] is True
    assert state["goal_kind"] is None
    refused = [r for r in ctx.log.records if r[1] == "follow_me_goal_refused"]
    assert refused == [
        ("warning", "follow_me_goal_refused",
         {"kind": intent.KIND_FOLLOW_INTENT, "err": "not_holder"})
    ]


@pytest.mark.asyncio
async def test_a_repeated_refusal_is_logged_once_and_its_end_once() -> None:
    ctx = _Ctx(config=_telemetry_config())
    plugin = _make_plugin(ctx)
    plugin._lease_id = 7
    goal = {"kind": "follow", "n": 0.0, "e": 0.0, "d": -4.0}
    ctx.flight.goal_reply = {"ok": False}
    for t in (1.0, 1.05, 1.1):
        await plugin._send_goal(goal, t)
    assert plugin._goal_kind is None
    ctx.flight.goal_reply = {"ok": True}
    await plugin._send_goal(goal, 1.15)
    assert plugin._goal_kind == "follow"
    names = ("follow_me_goal_refused", "follow_me_goal_accepted")
    events = [r for r in ctx.log.records if r[1] in names]
    assert events == [
        ("warning", "follow_me_goal_refused", {"kind": "follow", "err": "unavailable"}),
        ("info", "follow_me_goal_accepted", {"kind": "follow"}),
    ]
    # Every send counts toward the published rate, refused or not.
    assert len(ctx.flight.setpoints) == 4


@pytest.mark.asyncio
async def test_a_goal_the_host_rejects_is_logged_once_and_names_no_kind() -> None:
    ctx = _Ctx(config=_telemetry_config())
    plugin = _make_plugin(ctx)
    plugin._lease_id = 7
    plugin._goal_kind = "follow"

    async def reject(_lease_id: int, _goal: dict[str, Any]) -> dict:
        raise RuntimeError("bad_args: unknown goal field")

    ctx.flight.goal_send = reject  # type: ignore[method-assign]
    goal = {"kind": "follow_intent", "n": 0.0, "e": 0.0, "d": -4.0}
    for t in (1.0, 1.05):
        await plugin._send_goal(goal, t)
    assert plugin._goal_kind is None
    refused = [r for r in ctx.log.records if r[1] == "follow_me_goal_refused"]
    assert refused == [
        ("warning", "follow_me_goal_refused",
         {"kind": "follow_intent", "err": "bad_args: unknown goal field"})
    ]


def test_the_auto_angle_resolves_from_config() -> None:
    assert (
        FollowConfig.resolve({"follow_angle_auto": True}, {}).follow_angle_auto is True
    )
    assert FollowConfig.resolve({}, {}).follow_angle_auto is False


def test_the_registry_decides_intents(monkeypatch: pytest.MonkeyPatch) -> None:
    # The real check, not the autouse fixture's.
    monkeypatch.undo()
    intent.pilot_takes_intents.cache_clear()
    try:
        from ados.core.contracts import contract_version
    except ImportError:
        assert intent.pilot_takes_intents() is False
        return
    want = (contract_version("pilot.v1") or 0) >= intent.PILOT_INTENT_VERSION
    assert intent.pilot_takes_intents() is want
