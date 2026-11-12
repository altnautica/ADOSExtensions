"""Plugin control-loop tests against a fake host context.

Two contracts these cover:

* The overlay's ``camera.thermal.frame`` topic must have a live producer (a
  spot temperature the GCS renders over the video leg) whenever a device is
  open, so the thermal spot-meter is never a producer-less dead surface.

* Nothing may reach that topic when no device is open. With no injected UVC
  backend the plugin has no sensor at all, and it must say so rather than
  publish a fabricated reading.

The plugin runs on asyncio; these drive the deterministic ``apply_config_once``
entry point through ``asyncio.run`` so the suite stays sync-only (no extra
test dependency).
"""

from __future__ import annotations

import asyncio
from types import SimpleNamespace

import altnautica_thermal_camera.plugin as plugin_mod

from altnautica_thermal_camera.driver import LeptonUvcDriver
from altnautica_thermal_camera.plugin import (
    REASON_NO_BACKEND,
    REASON_NO_DEVICE,
    REASON_STREAM_FAILED,
    ThermalUsbPlugin,
)
from mock_backend import MockUvcBackend


class _ConfigKV:
    def __init__(self) -> None:
        self._d: dict = {}

    def static(self, key, default):
        return default

    async def get(self, key, default=None):
        return self._d.get(key, default)

    async def set(self, key, value) -> None:
        self._d[key] = value


class _Telemetry:
    def __init__(self) -> None:
        self.extended: list[tuple[str, dict]] = []

    async def extend(self, channel, payload) -> None:
        self.extended.append((channel, payload))


class _Events:
    def __init__(self) -> None:
        self.published: list[tuple[str, dict]] = []

    async def publish(self, topic, payload) -> None:
        self.published.append((topic, payload))


class _Ctx:
    def __init__(self) -> None:
        self.config_kv = _ConfigKV()
        self.telemetry = _Telemetry()
        self.events = _Events()


def _channel(ctx: _Ctx, name: str) -> list[dict]:
    return [payload for channel, payload in ctx.telemetry.extended if channel == name]


def _factory(**kwargs):
    return lambda: MockUvcBackend(**kwargs)


def test_publishes_thermal_frame_readout_for_the_overlay():
    # The GCS overlay subscribes to camera.thermal.frame for the live spot
    # temperature. The agent must produce it: the centre-reticle temperature
    # tracks the mock's hot disc (60 C) over a ~22 C background.
    ctx = _Ctx()
    plugin = ThermalUsbPlugin(backend_factory=_factory(peak_celsius=60.0))

    async def _run() -> None:
        await plugin.on_start(ctx)
        await plugin.apply_config_once()
        await plugin.on_stop(ctx)

    asyncio.run(_run())

    channels = [ch for ch, _ in ctx.telemetry.extended]
    assert "camera.thermal.frame" in channels, channels

    payload = next(
        p for ch, p in ctx.telemetry.extended if ch == "camera.thermal.frame"
    )
    assert payload["width"] == 160
    assert payload["height"] == 120
    assert payload["spot"]["x"] == 80
    assert payload["spot"]["y"] == 60
    # The centre spot reads the hot disc; the extrema bracket the scene.
    assert abs(payload["spot"]["temperatureC"] - 60.0) < 1.0
    assert abs(payload["maxC"] - 60.0) < 1.0
    assert abs(payload["minC"] - 22.0) < 1.0


def test_no_device_publishes_no_frame_readout():
    # With no device on the bus, no frame stream opens and the overlay channel
    # stays unproduced; the state channel says why.
    ctx = _Ctx()
    plugin = ThermalUsbPlugin(backend_factory=_factory(device_count=0))

    async def _run() -> None:
        await plugin.on_start(ctx)
        await plugin.apply_config_once()
        await plugin.on_stop(ctx)

    asyncio.run(_run())

    assert _channel(ctx, "camera.thermal.frame") == []
    states = _channel(ctx, "thermal")
    assert states, ctx.telemetry.extended
    assert states[0]["connected"] is False
    assert states[0]["reason"] == REASON_NO_DEVICE


def test_no_backend_fails_closed_and_publishes_no_reading():
    # The package ships no UVC backend. With none injected the plugin has no
    # sensor at all: it must report an unavailable camera, run no control loop,
    # and put nothing on the frame channel -- never a synthesised temperature.
    ctx = _Ctx()
    plugin = ThermalUsbPlugin()
    control_tasks: list[int] = []

    async def _run() -> None:
        await plugin.on_start(ctx)
        # Only this coroutine's own task may be alive: on_start must not have
        # spawned the control loop.
        control_tasks.append(len(asyncio.all_tasks()) - 1)
        await plugin.apply_config_once()
        await plugin.on_stop(ctx)

    asyncio.run(_run())

    assert control_tasks == [0]
    assert plugin.session is None
    assert plugin.driver is None
    assert _channel(ctx, "camera.thermal.frame") == []

    states = _channel(ctx, "thermal")
    assert states, ctx.telemetry.extended
    assert states[0]["connected"] is False
    assert states[0]["reason"] == REASON_NO_BACKEND


def test_skills_report_disabled_without_a_sensor():
    # Both cockpit Skills need an open session to do anything. With no sensor
    # the Skill Bar must be told they are disabled, not left unpopulated.
    ctx = _Ctx()
    plugin = ThermalUsbPlugin()

    asyncio.run(plugin.on_start(ctx))

    published = dict(ctx.events.published)
    assert set(published) == {"camera.thermal.palette", "camera.thermal.ffc"}
    for payload in published.values():
        assert payload["state"] == "disabled"
        assert payload["reason"] == REASON_NO_BACKEND


def test_enumeration_failure_fails_closed():
    # A backend that cannot enumerate the bus has established no device. The
    # plugin must report that rather than let the exception escape on_start,
    # which would leave the GCS with no state at all.
    class _BrokenBackend(MockUvcBackend):
        def enumerate(self):
            raise OSError("libuvc: could not access USB device list")

    ctx = _Ctx()
    plugin = ThermalUsbPlugin(backend_factory=_BrokenBackend)

    asyncio.run(plugin.on_start(ctx))

    assert plugin.session is None
    assert _channel(ctx, "camera.thermal.frame") == []
    states = _channel(ctx, "thermal")
    assert states, ctx.telemetry.extended
    assert states[0]["connected"] is False
    assert states[0]["reason"] == REASON_NO_DEVICE


def test_stream_failure_releases_the_session_and_fails_closed(monkeypatch):
    # A session that cannot deliver frames produces no temperature at all.
    # Reporting it as a connected camera would leave the overlay waiting for
    # frames that are never coming, so the session is released instead.
    async def _boom(self, session):
        raise OSError("libuvc: could not start the isochronous stream")

    monkeypatch.setattr(LeptonUvcDriver, "frame_iterator", _boom)

    ctx = _Ctx()
    backend = MockUvcBackend()
    plugin = ThermalUsbPlugin(backend_factory=lambda: backend)

    asyncio.run(plugin.on_start(ctx))

    assert plugin.session is None
    assert backend.opened_devices == set()
    assert _channel(ctx, "camera.thermal.frame") == []
    states = _channel(ctx, "thermal")
    assert states, ctx.telemetry.extended
    assert states[-1]["connected"] is False
    assert states[-1]["reason"] == REASON_STREAM_FAILED


def test_frame_readout_carries_a_timestamp():
    # The overlay ages the spot reading; a read-back without a timestamp
    # cannot be told apart from a repeat of an old one.
    ctx = _Ctx()
    plugin = ThermalUsbPlugin(backend_factory=_factory())

    async def _run() -> None:
        await plugin.on_start(ctx)
        await plugin.apply_config_once()
        await plugin.on_stop(ctx)

    asyncio.run(_run())

    frames = _channel(ctx, "camera.thermal.frame")
    assert frames, ctx.telemetry.extended
    assert isinstance(frames[0]["timestampMs"], int)
    assert frames[0]["timestampMs"] > 0


def test_repeated_frame_read_failures_fail_closed_as_stream_failed():
    # A stream that opened and then stops delivering must not leave the
    # camera reported as connected with the last reading on screen.
    ctx = _Ctx()
    backend = MockUvcBackend()
    plugin = ThermalUsbPlugin(backend_factory=lambda: backend)

    class _DeadFrames:
        async def __anext__(self):
            raise OSError("libuvc: frame timeout")

        async def aclose(self) -> None:
            return None

    async def _run() -> None:
        await plugin.on_start(ctx)
        plugin._frames = _DeadFrames()
        for _ in range(5):
            await plugin.apply_config_once()
        await plugin.on_stop(ctx)

    asyncio.run(_run())

    states = _channel(ctx, "thermal")
    assert any(
        s["connected"] is False and s.get("reason") == REASON_STREAM_FAILED
        for s in states
    ), states
    assert backend.opened_devices == set()


class _FlakyBackend(MockUvcBackend):
    """A backend whose stream can stall and whose device can drop off the
    bus, then come back, the way a USB stall plays out on the drone."""

    def __init__(self) -> None:
        super().__init__(width=16, height=12)
        self.stalled = False
        self.unplugged = False

    def enumerate(self):
        return [] if self.unplugged else super().enumerate()

    def frames(self, device):
        for frame in super().frames(device):
            if self.stalled:
                raise OSError("libuvc: frame timeout")
            yield frame


async def _until(cond, what: str, timeout: float = 5.0) -> None:
    loop = asyncio.get_running_loop()
    end = loop.time() + timeout
    while not cond():
        if loop.time() > end:
            raise AssertionError(f"timed out waiting for {what}")
        await asyncio.sleep(0.001)


def test_stream_failed_reconnects_when_the_device_returns(monkeypatch):
    # A short USB stall must not lose the camera for the rest of the flight:
    # after the stream is declared failed the control loop keeps trying to
    # reopen it (2 s, then every 5 s), and once the device answers again the
    # camera reads connected, fresh read-backs flow and settings apply again.
    clock = [1_000.0]
    monkeypatch.setattr(plugin_mod, "time", SimpleNamespace(time=lambda: clock[0]))

    ctx = _Ctx()
    backend = _FlakyBackend()
    plugin = ThermalUsbPlugin(backend_factory=lambda: backend)
    delays: list[float] = []
    backoff_started_ms: list[int] = []

    async def fake_sleep(seconds: float) -> None:
        if seconds >= 1.0 and not backoff_started_ms:
            backoff_started_ms.append(int(clock[0] * 1000))
        delays.append(seconds)
        clock[0] += seconds
        # The device comes back while the third reopen attempt is waiting.
        if sum(1 for d in delays if d >= 1.0) == 3:
            backend.unplugged = False
            backend.stalled = False
        await asyncio.sleep(0)

    plugin._sleep = fake_sleep

    def failed() -> bool:
        return any(
            s["connected"] is False and s.get("reason") == REASON_STREAM_FAILED
            for s in _channel(ctx, "thermal")
        )

    async def _run() -> dict:
        await plugin.on_start(ctx)
        await _until(lambda: _channel(ctx, "camera.thermal.frame"), "a first read-back")
        backend.stalled = True
        backend.unplugged = True
        await _until(failed, "stream-failed")
        n_frames = len(_channel(ctx, "camera.thermal.frame"))
        await _until(
            lambda: plugin.session is not None
            and len(_channel(ctx, "camera.thermal.frame")) > n_frames,
            "the stream to reopen and deliver",
        )
        await ctx.config_kv.set("palette", "rainbow")
        await _until(
            lambda: _channel(ctx, "thermal")[-1]["palette"] == "rainbow",
            "the palette to apply after the reopen",
        )
        await plugin.on_stop(ctx)
        return {"n_frames": n_frames}

    seen = asyncio.run(_run())

    assert [d for d in delays if d >= 1.0] == [2.0, 5.0, 5.0]
    states = _channel(ctx, "thermal")
    i_failed = next(
        i for i, s in enumerate(states) if s.get("reason") == REASON_STREAM_FAILED
    )
    assert any(
        s["connected"] is True and "reason" not in s for s in states[i_failed:]
    ), states
    resumed = _channel(ctx, "camera.thermal.frame")[seen["n_frames"]:]
    assert resumed[0]["timestampMs"] >= backoff_started_ms[0] + 12_000
    assert plugin.session is None  # released by on_stop
    assert backend.opened_devices == set()


def test_reconnect_retries_without_a_cap():
    # The device may stay away for a long time; the loop keeps trying every
    # 5 s rather than giving up after a few attempts.
    ctx = _Ctx()
    backend = _FlakyBackend()
    plugin = ThermalUsbPlugin(backend_factory=lambda: backend)
    delays: list[float] = []

    async def fake_sleep(seconds: float) -> None:
        delays.append(seconds)
        await asyncio.sleep(0)

    plugin._sleep = fake_sleep

    async def _run() -> None:
        await plugin.on_start(ctx)
        backend.stalled = True
        backend.unplugged = True
        await _until(
            lambda: sum(1 for d in delays if d >= 1.0) >= 40, "forty reopen attempts"
        )
        assert plugin.session is None
        assert plugin.unavailable_reason == REASON_STREAM_FAILED
        await plugin.on_stop(ctx)

    asyncio.run(_run())
    backoff = [d for d in delays if d >= 1.0]
    assert backoff[0] == 2.0
    assert set(backoff[1:]) == {5.0}


def test_palette_is_reported_only_after_the_camera_accepts_it(monkeypatch):
    # A palette the camera rejected must not be shown as the applied palette.
    real_set_param = LeptonUvcDriver.set_param

    async def _reject_palette(self, session, param, value):
        if param == "palette":
            raise OSError("libuvc: control transfer failed")
        await real_set_param(self, session, param, value)

    monkeypatch.setattr(LeptonUvcDriver, "set_param", _reject_palette)

    ctx = _Ctx()
    plugin = ThermalUsbPlugin(backend_factory=_factory(width=16, height=12))

    async def _run() -> None:
        await plugin.on_start(ctx)
        await ctx.config_kv.set("palette", "grayscale")
        await plugin.apply_config_once()
        await ctx.config_kv.set("palette_cycle", True)
        await plugin.apply_config_once()
        await plugin.on_stop(ctx)

    asyncio.run(_run())

    palettes = [s["palette"] for s in _channel(ctx, "thermal")]
    assert palettes and set(palettes) == {"ironbow"}, palettes
    # The cycle did not write a palette the camera refused back to settings.
    assert ctx.config_kv._d.get("palette") == "grayscale"
