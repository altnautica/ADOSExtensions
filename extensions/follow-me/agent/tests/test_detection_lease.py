"""The detection lease on the follow camera.

The vision engine runs detection on a camera only while a consumer holds a
lease. While active, the plugin renews one on its follow camera every 2 s with
a 6 s hold, moves it when the follow camera changes, and releases it (hold 0)
when the behaviour goes inactive or the plugin stops. An SDK without
``request_detection`` is tolerated.
"""

from __future__ import annotations

from typing import Any

import pytest
from ados.sdk.vision import BoundingBox, Detection, DetectionBatch
from follow_fakes import _Ctx, _install_clock, _make_plugin, _Vision
from follow_me.demand import HOLD_S, RENEW_S, DetectionLease
from follow_me.state import LOCK_LOCKED


class _LeaseVision(_Vision):
    def __init__(self, fail: bool = False) -> None:
        super().__init__()
        self.requests: list[tuple[str, float]] = []
        self.fail = fail

    async def request_detection(self, camera_id: str, hold_s: float) -> dict:
        self.requests.append((camera_id, hold_s))
        if self.fail:
            raise RuntimeError("camera not captured")
        return {"camera_id": camera_id, "detecting": hold_s > 0, "until_ms": None}


class _RecordingLog:
    def __init__(self) -> None:
        self.records: list[tuple[str, str]] = []

    def info(self, event: str, *_a: Any, **_k: Any) -> None:
        self.records.append(("info", event))

    def warning(self, event: str, *_a: Any, **_k: Any) -> None:
        self.records.append(("warning", event))

    def debug(self, event: str, *_a: Any, **_k: Any) -> None:
        self.records.append(("debug", event))


def _ctx(config: dict[str, Any] | None = None, *, fail: bool = False) -> _Ctx:
    ctx = _Ctx(config=config)
    ctx.vision = _LeaseVision(fail=fail)
    ctx.log = _RecordingLog()
    return ctx


# ---------------------------------------------------------------------------
# DetectionLease
# ---------------------------------------------------------------------------


def test_contract_cadence() -> None:
    assert RENEW_S == 2.0
    assert HOLD_S == 6.0


@pytest.mark.asyncio
async def test_lease_renews_on_the_period() -> None:
    ctx = _ctx()
    lease = DetectionLease()
    await lease.update(ctx, "main", 100.0)
    await lease.update(ctx, "main", 100.05)
    await lease.update(ctx, "main", 101.9)
    assert ctx.vision.requests == [("main", 6.0)]
    await lease.update(ctx, "main", 102.0)
    assert ctx.vision.requests == [("main", 6.0), ("main", 6.0)]
    assert lease.camera == "main"


@pytest.mark.asyncio
async def test_camera_change_releases_the_old_camera_first() -> None:
    ctx = _ctx()
    lease = DetectionLease()
    await lease.update(ctx, "main", 100.0)
    await lease.update(ctx, "belly", 100.5)
    assert ctx.vision.requests == [("main", 6.0), ("main", 0.0), ("belly", 6.0)]
    assert lease.camera == "belly"


@pytest.mark.asyncio
async def test_no_camera_releases_once() -> None:
    ctx = _ctx()
    lease = DetectionLease()
    await lease.update(ctx, "main", 100.0)
    await lease.update(ctx, None, 100.5)
    await lease.update(ctx, None, 103.0)
    assert ctx.vision.requests == [("main", 6.0), ("main", 0.0)]
    assert lease.camera is None


@pytest.mark.asyncio
async def test_release_with_nothing_held_sends_nothing() -> None:
    ctx = _ctx()
    await DetectionLease().release(ctx)
    assert ctx.vision.requests == []


@pytest.mark.asyncio
async def test_reacquiring_after_release_requests_immediately() -> None:
    ctx = _ctx()
    lease = DetectionLease()
    await lease.update(ctx, "main", 100.0)
    await lease.update(ctx, None, 100.5)
    await lease.update(ctx, "main", 100.6)
    assert ctx.vision.requests[-1] == ("main", 6.0)
    assert len(ctx.vision.requests) == 3


@pytest.mark.asyncio
async def test_refused_request_retries_on_the_period_not_every_tick() -> None:
    ctx = _ctx(fail=True)
    lease = DetectionLease()
    await lease.update(ctx, "ghost", 100.0)
    await lease.update(ctx, "ghost", 100.05)
    assert ctx.vision.requests == [("ghost", 6.0)]
    assert lease.camera is None
    assert ("warning", "follow_me_detection_lease_failed") in ctx.log.records
    await lease.update(ctx, "ghost", 102.0)
    assert len(ctx.vision.requests) == 2
    # Nothing was granted, so going inactive has nothing to release.
    await lease.update(ctx, None, 102.1)
    assert len(ctx.vision.requests) == 2


@pytest.mark.asyncio
async def test_sdk_without_request_detection_logs_once_and_carries_on() -> None:
    ctx = _Ctx()
    ctx.log = _RecordingLog()
    assert not hasattr(ctx.vision, "request_detection")
    lease = DetectionLease()
    for t in (100.0, 102.0, 104.0):
        await lease.update(ctx, "main", t)
    await lease.update(ctx, None, 105.0)
    await lease.release(ctx)
    assert ctx.log.records.count(("info", "follow_me_detection_lease_unsupported")) == 1
    assert lease.camera is None


# ---------------------------------------------------------------------------
# Plugin wiring
# ---------------------------------------------------------------------------


def _batch(camera_id: str) -> DetectionBatch:
    return DetectionBatch(
        model_id="coco-person",
        camera_id=camera_id,
        frame_id=1,
        ts_ms=0,
        detections=[
            Detection(
                bbox=BoundingBox(x=300.0, y=200.0, width=40.0, height=120.0),
                class_label="person",
                confidence=0.9,
                track_id=7,
                lock_state=LOCK_LOCKED,
            )
        ],
    )


async def _step(plugin: Any, now: float) -> None:
    """One loop iteration's config read and housekeeping."""
    await plugin._config_for_tick()
    await plugin._housekeeping(now)


@pytest.mark.asyncio
async def test_active_holds_the_pinned_camera_and_inactive_releases(
    monkeypatch: Any,
) -> None:
    clock = _install_clock(monkeypatch)
    ctx = _ctx({"active": True, "designate_camera": "belly"})
    plugin = _make_plugin(ctx)

    await _step(plugin, clock.now)
    assert ctx.vision.requests == [("belly", 6.0)]

    clock.advance(RENEW_S)
    await _step(plugin, clock.now)
    assert ctx.vision.requests == [("belly", 6.0), ("belly", 6.0)]

    await ctx.config_kv.set("active", False)
    clock.advance(0.05)
    await _step(plugin, clock.now)
    assert ctx.vision.requests[-1] == ("belly", 0.0)

    clock.advance(RENEW_S)
    await _step(plugin, clock.now)
    assert len(ctx.vision.requests) == 3


@pytest.mark.asyncio
async def test_inactive_never_requests(monkeypatch: Any) -> None:
    clock = _install_clock(monkeypatch)
    ctx = _ctx({"active": False, "designate_camera": "belly"})
    plugin = _make_plugin(ctx)
    await _step(plugin, clock.now)
    assert ctx.vision.requests == []


@pytest.mark.asyncio
async def test_auto_follows_the_designated_subjects_camera(monkeypatch: Any) -> None:
    clock = _install_clock(monkeypatch)
    ctx = _ctx({"active": True, "designate_camera": "auto"})
    plugin = _make_plugin(ctx)

    # No subject yet: nothing to hold on auto.
    await _step(plugin, clock.now)
    assert ctx.vision.requests == []

    await _step(plugin, clock.now)
    plugin._on_batch(_batch("main"))
    clock.advance(0.05)
    await _step(plugin, clock.now)
    assert ctx.vision.requests == [("main", 6.0)]


@pytest.mark.asyncio
async def test_teardown_releases_the_lease(monkeypatch: Any) -> None:
    clock = _install_clock(monkeypatch)
    ctx = _ctx({"active": True, "designate_camera": "belly"})
    plugin = _make_plugin(ctx)
    await _step(plugin, clock.now)
    await plugin.on_stop(ctx)
    assert ctx.vision.requests == [("belly", 6.0), ("belly", 0.0)]
