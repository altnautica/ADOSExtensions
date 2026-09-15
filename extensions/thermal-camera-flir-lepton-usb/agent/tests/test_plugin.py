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
