"""Plugin control-loop tests against a fake host context.

The key regression these cover: the overlay's ``camera.thermal.frame`` topic
must have a live producer (a spot temperature the GCS renders over the video
leg), so the thermal spot-meter is never a producer-less dead surface.

The plugin runs on asyncio; these drive the deterministic ``apply_config_once``
entry point through ``asyncio.run`` so the suite stays sync-only (no extra
test dependency).
"""

from __future__ import annotations

import asyncio

from altnautica_thermal_camera.plugin import ThermalUsbPlugin
from altnautica_thermal_camera.uvc_backend import MockUvcBackend


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


class _Ctx:
    def __init__(self) -> None:
        self.config_kv = _ConfigKV()
        self.telemetry = _Telemetry()


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
    # With no device, no frame stream opens and the overlay channel stays
    # unproduced (no phantom readout).
    ctx = _Ctx()
    plugin = ThermalUsbPlugin(backend_factory=_factory(device_count=0))

    async def _run() -> None:
        await plugin.on_start(ctx)
        await plugin.apply_config_once()
        await plugin.on_stop(ctx)

    asyncio.run(_run())

    channels = [ch for ch, _ in ctx.telemetry.extended]
    assert "camera.thermal.frame" not in channels
