"""Camera geometry from the detection batches, attitude on the flight
controller's clock, frames at their exposure, and the camera-lost hold.

The projection needs the follow camera's field of view and mount tilt: from
the camera roster on the vision engine's batches, else the operator's
settings, else the reference camera. A box is
projected through the attitude at its frame's exposure, and the attitude
history is kept on the flight controller's clock so a MAVLink delivery stall
does not stamp a whole burst of samples at one arrival instant.
"""

from __future__ import annotations

import asyncio
from types import SimpleNamespace
from typing import Any

import pytest
from ados.sdk.vision import BoundingBox, Detection
from follow_fakes import (
    _attitude,
    _Ctx,
    _global_position,
    _install_clock,
    _local_position,
    _make_plugin,
    _spy_projection,
    _state_events,
)

from follow_me.camera import DEFAULT_GEOMETRY, configured_geometry, geometry_of
from follow_me.state import LOCK_LOCKED, FollowConfig

_W, _H = 1280, 720
_WALKER = BoundingBox(x=615.0, y=250.0, width=55.0, height=138.0)


def _batch(ts_ms: int, *, detections: list[Any] | None = None, **extra: Any) -> Any:
    """A follow-camera detection batch; ``extra`` carries the optional
    camera fields an older SDK class does not declare."""
    return SimpleNamespace(
        model_id="coco-person",
        camera_id="uvc-0",
        frame_id=1,
        ts_ms=ts_ms,
        detections=detections if detections is not None else [],
        frame_width=_W,
        frame_height=_H,
        **extra,
    )


def _locked(bbox: BoundingBox = _WALKER) -> Detection:
    return Detection(
        bbox=bbox,
        class_label="person",
        confidence=0.9,
        track_id=3,
        lock_state=LOCK_LOCKED,
    )


def _pose(plugin: Any) -> None:
    plugin._on_local_position(_local_position(0.0, 0.0, -2.5))
    plugin._on_global_position(_global_position(12.0, 77.0, 2.5))


def _measure(plugin: Any, now: float) -> None:
    s = plugin._sighting
    assert s is not None
    plugin._measure_sighting(FollowConfig(), now, s.bbox)


# -- (a) geometry from the batch --------------------------------------------


def _state(plugin: Any, cfg: FollowConfig, now: float) -> dict[str, Any]:
    return plugin._build_state(
        cfg,
        now,
        active=True,
        phase="holding",
        lock_state=None,
        commanding=False,
        hold_reason="no-range",
        coast_remaining_s=0.0,
        predicted_bbox=None,
    ).to_dict()


# The operator's settings: a belly camera, wide and straight down.
_SETTINGS = FollowConfig(camera_hfov_deg=120.0, mount_pitch_deg=90.0)


def test_the_projection_uses_the_batch_camera_geometry(monkeypatch: Any) -> None:
    clock = _install_clock(monkeypatch)
    captured = _spy_projection(monkeypatch)
    plugin = _make_plugin(_Ctx())
    # The roster on the batch wins over the settings.
    plugin._cfg = _SETTINGS
    plugin._on_attitude(_attitude(0.0, 0.0, 0.0))
    _pose(plugin)
    # Roster sign: negative is below the nose. The projection's tilt is
    # positive down.
    plugin._on_batch(
        _batch(
            int(clock.now * 1000.0),
            detections=[_locked()],
            hfov_deg=60.0,
            mount_pitch_deg=-20.0,
        )
    )
    _measure(plugin, clock.now)
    assert captured["horizontal_fov_deg"] == pytest.approx(60.0)
    assert captured["mount_pitch_deg"] == pytest.approx(20.0)
    state = _state(plugin, _SETTINGS, clock.now)
    assert state["camera_hfov_deg"] == pytest.approx(60.0)
    assert state["camera_mount_pitch_deg"] == pytest.approx(-20.0)
    assert state["camera_geometry_source"] == "roster"


def test_a_batch_without_geometry_falls_back_to_the_reference_front_camera(
    monkeypatch: Any,
) -> None:
    clock = _install_clock(monkeypatch)
    captured = _spy_projection(monkeypatch)
    plugin = _make_plugin(_Ctx())
    plugin._on_attitude(_attitude(0.0, 0.0, 0.0))
    _pose(plugin)
    plugin._on_batch(_batch(int(clock.now * 1000.0), detections=[_locked()]))
    _measure(plugin, clock.now)
    assert captured["horizontal_fov_deg"] == pytest.approx(87.0)
    assert captured["mount_pitch_deg"] == pytest.approx(15.0)
    state = _state(plugin, FollowConfig(), clock.now)
    assert state["camera_hfov_deg"] == pytest.approx(87.0)
    assert state["camera_mount_pitch_deg"] == pytest.approx(-15.0)
    assert state["camera_geometry_source"] == "default"


def test_a_batch_without_geometry_projects_with_the_settings(monkeypatch: Any) -> None:
    clock = _install_clock(monkeypatch)
    captured = _spy_projection(monkeypatch)
    plugin = _make_plugin(_Ctx())
    plugin._cfg = _SETTINGS
    plugin._on_attitude(_attitude(0.0, 0.0, 0.0))
    _pose(plugin)
    plugin._on_batch(_batch(int(clock.now * 1000.0), detections=[_locked()]))
    _measure(plugin, clock.now)
    assert captured["horizontal_fov_deg"] == pytest.approx(120.0)
    assert captured["mount_pitch_deg"] == pytest.approx(90.0)
    state = _state(plugin, _SETTINGS, clock.now)
    assert state["camera_hfov_deg"] == pytest.approx(120.0)
    assert state["camera_mount_pitch_deg"] == pytest.approx(-90.0)
    assert state["camera_geometry_source"] == "settings"


def test_the_settings_override_the_default_and_unset_keeps_it() -> None:
    cfg = FollowConfig.resolve({"camera_hfov_deg": 50.0}, {"mount_pitch_deg": 40.0})
    assert cfg.camera_hfov_deg == 50.0
    assert cfg.mount_pitch_deg == 40.0
    geometry = configured_geometry(cfg)
    assert (geometry.hfov_deg, geometry.mount_pitch_deg) == (50.0, -40.0)
    assert geometry.source == "settings"
    unset = FollowConfig.resolve({}, {})
    assert unset.camera_hfov_deg is None and unset.mount_pitch_deg is None
    assert configured_geometry(unset) == DEFAULT_GEOMETRY
    # Before any batch the read-back already reports the settings.
    plugin = _make_plugin(_Ctx())
    state = _state(plugin, cfg, 0.0)
    assert state["camera_hfov_deg"] == pytest.approx(50.0)
    assert state["camera_geometry_source"] == "settings"


def test_a_partial_roster_fills_from_the_settings_and_reports_them() -> None:
    geometry = geometry_of(SimpleNamespace(hfov_deg=70.0), configured_geometry(_SETTINGS))
    assert (geometry.hfov_deg, geometry.mount_pitch_deg) == (70.0, -90.0)
    assert geometry.source == "settings"


# -- (b) attitude on the FC clock through a delivery stall -------------------


def test_a_delivery_stall_keeps_each_attitude_at_its_fc_instant(
    monkeypatch: Any,
) -> None:
    clock = _install_clock(monkeypatch)
    captured = _spy_projection(monkeypatch)
    plugin = _make_plugin(_Ctx())
    _pose(plugin)
    # The FC clock runs 100 s behind the host; deliveries take 20 ms.
    fc_ms = 50_000
    clock.now = fc_ms / 1000.0 + 100.02
    for _ in range(20):
        clock.advance(0.05)
        fc_ms += 50
        plugin._on_attitude(_attitude(0.0, 0.0, 0.0, fc_ms))
    stall_start_ms = fc_ms
    # 0.8 s of samples at 20 Hz, pitching the nose down 0.01 rad per sample,
    # held up and delivered together within 1 ms.
    burst = []
    for k in range(1, 17):
        burst.append((stall_start_ms + 50 * k, -0.01 * k))
    clock.advance(0.8)
    for t_ms, pitch in burst:
        clock.advance(0.00005)
        plugin._on_attitude(_attitude(0.0, pitch, 0.0, t_ms))
    # A frame exposed mid-stall, stamped on the host clock.
    exposure_fc_ms = stall_start_ms + 400
    ts_ms = int(round(exposure_fc_ms + 100_020))
    plugin._on_batch(_batch(ts_ms, detections=[_locked()]))
    _measure(plugin, clock.now)
    assert captured["cam"].pitch_rad == pytest.approx(-0.08, abs=0.002)


def test_the_fc_clock_offset_falls_at_once_and_rises_slowly() -> None:
    from follow_me.fc_state import FcClock

    c = FcClock()
    assert c.offset_s is None
    c.note(10.0, 110.3)
    assert c.offset_s == pytest.approx(100.3)
    # A faster delivery: taken at once.
    c.note(10.1, 110.15)
    assert c.offset_s == pytest.approx(100.05)
    # Every later delivery slower (the window still holds the fast one).
    c.note(10.2, 110.45)
    assert c.offset_s == pytest.approx(100.05)
    # Once the fast one leaves the window the offset rises at 0.05 of the
    # host time that passed.
    for k in range(64):
        c.note(11.0 + k * 0.1, 111.5 + k * 0.1)
    assert 100.05 < c.offset_s < 100.5
    # The FC restarted (its time stepped back by more than 1 s): start over.
    c.note(1.0, 200.0)
    assert c.offset_s == pytest.approx(199.0)
    assert c.fc_s(200.5) == pytest.approx(1.5)


# -- (c) the sensor stamp ----------------------------------------------------


def _ramp(plugin: Any, clock: Any) -> int:
    """ATTITUDE at 20 Hz for 2 s with the nose pitching down 0.1 rad/s; the
    FC clock 100 s behind the host, delivered at once. The FC ms of the
    first sample."""
    fc0 = 20_000
    for k in range(41):
        clock.now = (fc0 + 50 * k) / 1000.0 + 100.0
        plugin._on_attitude(_attitude(0.0, -0.1 * k * 0.05, 0.0, fc0 + 50 * k))
    return fc0


def test_a_sim_frame_is_projected_at_its_sensor_stamp(monkeypatch: Any) -> None:
    clock = _install_clock(monkeypatch)
    captured = _spy_projection(monkeypatch)
    plugin = _make_plugin(_Ctx())
    _pose(plugin)
    fc0 = _ramp(plugin, clock)
    exposure_ms = fc0 + 1000
    # The host stamp is the frame's arrival, 0.25 s after its exposure.
    host_ms = exposure_ms + 100_000 + 250
    plugin._on_batch(
        _batch(host_ms, detections=[_locked()], ts_sensor_ms=exposure_ms)
    )
    assert plugin._sighting.ts_sensor_ms == exposure_ms
    _measure(plugin, clock.now)
    assert captured["cam"].pitch_rad == pytest.approx(-0.1, abs=1e-6)
    # Latency accounting stays on the host stamp.
    assert plugin._pending_latency_ts_ms == host_ms


def test_a_sensor_stamp_far_from_the_host_stamp_is_ignored(monkeypatch: Any) -> None:
    clock = _install_clock(monkeypatch)
    captured = _spy_projection(monkeypatch)
    plugin = _make_plugin(_Ctx())
    _pose(plugin)
    fc0 = _ramp(plugin, clock)
    exposure_ms = fc0 + 1000
    host_ms = exposure_ms + 100_000 + 250
    plugin._on_batch(
        _batch(host_ms, detections=[_locked()], ts_sensor_ms=exposure_ms + 5000)
    )
    _measure(plugin, clock.now)
    # The host stamp mapped onto the FC clock: 0.25 s later on the ramp.
    assert captured["cam"].pitch_rad == pytest.approx(-0.125, abs=1e-6)


# -- (d) camera lost ----------------------------------------------------------


class _RecordingLog:
    def __init__(self) -> None:
        self.warnings: list[str] = []

    def info(self, *_a: Any, **_k: Any) -> None:
        pass

    def debug(self, *_a: Any, **_k: Any) -> None:
        pass

    def warning(self, event: str, *_a: Any, **_k: Any) -> None:
        self.warnings.append(event)


def test_a_silent_follow_camera_holds_camera_lost(monkeypatch: Any) -> None:
    clock = _install_clock(monkeypatch)
    ctx = _Ctx({"active": True, "designate_camera": "uvc-0", "gimbal_point": False})
    ctx.log = _RecordingLog()
    plugin = _make_plugin(ctx)

    def reason() -> Any:
        return _state_events(ctx)[-1]["hold_reason"]

    async def run() -> None:
        await plugin._tick()
        # Inside the start grace: no camera-lost even with no batch yet.
        clock.advance(4.0)
        await plugin._tick()
        assert reason() != "camera-lost"
        clock.advance(1.5)
        await plugin._tick()
        assert reason() == "camera-lost"
        assert _state_events(ctx)[-1]["phase"] == "acquiring"
        # A batch (even an empty one) clears it.
        plugin._on_batch(_batch(int(clock.now * 1000.0)))
        clock.advance(0.05)
        await plugin._tick()
        assert reason() != "camera-lost"
        # 1.9 s of silence is not lost yet; past 2 s it is.
        clock.advance(1.85)
        await plugin._tick()
        assert reason() != "camera-lost"
        clock.advance(0.2)
        await plugin._tick()
        assert reason() == "camera-lost"
        clock.advance(0.5)
        await plugin._tick()
        assert reason() == "camera-lost"

    asyncio.run(run())
    # Logged once per episode.
    assert ctx.log.warnings.count("follow_me_camera_lost") == 2
