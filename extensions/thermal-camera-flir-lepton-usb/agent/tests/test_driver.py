"""Unit tests for the LeptonUvcDriver."""

from __future__ import annotations

import asyncio
from typing import Any

import pytest

from altnautica_thermal_camera.driver import (
    LeptonUvcDriver,
    LeptonUvcSession,
    _firmware_meets,
)
from altnautica_thermal_camera.plugin import ThermalUsbPlugin
from altnautica_thermal_camera.uvc_backend import (
    DEFAULT_HEIGHT,
    DEFAULT_WIDTH,
    MockUvcBackend,
)


def _run(coro: Any) -> Any:
    return asyncio.run(coro)


@pytest.fixture
def backend() -> MockUvcBackend:
    return MockUvcBackend()


@pytest.fixture
def driver(backend: MockUvcBackend) -> LeptonUvcDriver:
    return LeptonUvcDriver(backend)


def test_discover_returns_one_candidate_per_mock_device(driver: LeptonUvcDriver) -> None:
    candidates = asyncio.run(driver.discover())
    assert len(candidates) == 1
    cand = candidates[0]
    assert cand.driver_id == "altnautica.thermal-flir-lepton-usb"
    assert cand.bus == "usb"
    assert cand.vid_pid == (0x1E4E, 0x0100)
    assert cand.metadata is not None
    assert cand.metadata["radiometric"] is True


def test_open_marks_device_open_and_returns_session(
    driver: LeptonUvcDriver, backend: MockUvcBackend
) -> None:
    candidates = asyncio.run(driver.discover())
    session = asyncio.run(driver.open(candidates[0], {"palette": "rainbow"}))
    assert isinstance(session, LeptonUvcSession)
    assert session.palette == "rainbow"
    assert session.device.serial in backend.opened_devices


def test_close_releases_the_device(
    driver: LeptonUvcDriver, backend: MockUvcBackend
) -> None:
    candidates = asyncio.run(driver.discover())
    session = asyncio.run(driver.open(candidates[0], {}))
    asyncio.run(driver.close(session))
    assert session.device.serial not in backend.opened_devices


def test_capabilities_report_y16_radiometric(
    driver: LeptonUvcDriver,
) -> None:
    candidates = asyncio.run(driver.discover())
    session = asyncio.run(driver.open(candidates[0], {}))
    caps = driver.capabilities(session)
    assert caps.radiometric is True
    assert caps.bit_depth == 14
    assert caps.width == DEFAULT_WIDTH
    assert caps.height == DEFAULT_HEIGHT
    assert caps.pixel_format == "Y16"
    assert caps.streaming_protocol == "uvc"
    assert "Y16" in caps.color_spaces


def test_set_param_palette_validates_the_value(
    driver: LeptonUvcDriver,
) -> None:
    candidates = asyncio.run(driver.discover())
    session = asyncio.run(driver.open(candidates[0], {}))
    asyncio.run(driver.set_param(session, "palette", "grayscale"))
    assert session.palette == "grayscale"
    with pytest.raises(ValueError):
        asyncio.run(driver.set_param(session, "palette", "not-a-palette"))


def test_set_param_ffc_calls_backend(
    driver: LeptonUvcDriver, backend: MockUvcBackend
) -> None:
    candidates = asyncio.run(driver.discover())
    session = asyncio.run(driver.open(candidates[0], {}))
    asyncio.run(driver.set_param(session, "ffc", None))
    assert backend.ffc_calls.get(session.device.serial) == 1


def test_set_param_unknown_parameter_raises(
    driver: LeptonUvcDriver,
) -> None:
    candidates = asyncio.run(driver.discover())
    session = asyncio.run(driver.open(candidates[0], {}))
    with pytest.raises(ValueError):
        asyncio.run(driver.set_param(session, "shutter", 0.5))


def test_open_rejects_outdated_firmware() -> None:
    backend = MockUvcBackend(firmware_version="1.1.0")
    driver = LeptonUvcDriver(backend)
    candidates = asyncio.run(driver.discover())
    with pytest.raises(Exception) as excinfo:
        asyncio.run(driver.open(candidates[0], {}))
    msg = str(excinfo.value)
    assert "firmware" in msg.lower()
    assert "1.1.0" in msg


def test_firmware_meets_handles_short_versions() -> None:
    assert _firmware_meets("1.2.2", (1, 2, 2)) is True
    assert _firmware_meets("1.2.3", (1, 2, 2)) is True
    assert _firmware_meets("1.2.1", (1, 2, 2)) is False
    assert _firmware_meets("1.2", (1, 2, 0)) is True
    assert _firmware_meets("garbage", (1, 2, 2)) is False


def test_open_locks_default_tlinear_resolution(
    driver: LeptonUvcDriver, backend: MockUvcBackend
) -> None:
    candidates = asyncio.run(driver.discover())
    session = asyncio.run(driver.open(candidates[0], {}))
    assert backend.tlinear_resolutions[session.device.serial] == 0.01
    assert session.tlinear_resolution_k_per_count == 0.01


def test_open_honours_configured_tlinear_resolution(
    driver: LeptonUvcDriver, backend: MockUvcBackend
) -> None:
    candidates = asyncio.run(driver.discover())
    session = asyncio.run(
        driver.open(candidates[0], {"tlinear_resolution_k_per_count": 0.1})
    )
    assert backend.tlinear_resolutions[session.device.serial] == 0.1
    assert session.tlinear_resolution_k_per_count == 0.1


def test_open_rejects_unsupported_tlinear_resolution(
    driver: LeptonUvcDriver,
) -> None:
    candidates = asyncio.run(driver.discover())
    with pytest.raises(Exception) as excinfo:
        asyncio.run(
            driver.open(candidates[0], {"tlinear_resolution_k_per_count": 0.5})
        )
    assert "tlinear" in str(excinfo.value).lower()


def test_set_param_tlinear_resolution_runtime_change(
    driver: LeptonUvcDriver, backend: MockUvcBackend
) -> None:
    candidates = asyncio.run(driver.discover())
    session = asyncio.run(driver.open(candidates[0], {}))
    asyncio.run(driver.set_param(session, "tlinear_resolution", 0.1))
    assert session.tlinear_resolution_k_per_count == 0.1
    assert backend.tlinear_resolutions[session.device.serial] == 0.1


def test_frame_iterator_stamps_resolution_into_metadata(
    driver: LeptonUvcDriver,
) -> None:
    async def first_frame() -> Any:
        candidates = await driver.discover()
        session = await driver.open(candidates[0], {})
        try:
            agen = await driver.frame_iterator(session)
            async for frame in agen:
                return frame
        finally:
            await driver.close(session)
        raise AssertionError("frame_iterator yielded nothing")

    frame = asyncio.run(first_frame())
    assert frame.metadata is not None
    assert (
        frame.metadata["tlinear_resolution_k_per_count"] == 0.01
    ), "default tlinear resolution should propagate"
    assert frame.pixel_format == "Y16"
    assert frame.width > 0 and frame.height > 0
    assert len(bytes(frame.data)) == frame.width * frame.height * 2


class _FakeConfigKv:
    def __init__(self, values: dict[str, Any] | None = None) -> None:
        self._values = dict(values or {})

    def static(self, key: str, default: Any = None) -> Any:
        return default

    async def get(self, key: str, default: Any = None) -> Any:
        return self._values.get(key, default)

    async def set(self, key: str, value: Any, scope: str = "drone") -> dict:
        self._values[key] = value
        return {"ok": True}


class _FakeVideo:
    def __init__(self) -> None:
        self.sources: list[list[dict]] = []

    async def set_source(self, cameras: Any) -> dict:
        legs = list(cameras)
        self.sources.append(legs)
        return {"ok": True, "count": len(legs)}


class _FakeTelemetry:
    def __init__(self) -> None:
        self.extended: list[tuple[str, dict]] = []

    async def extend(self, channel: str, payload: dict) -> None:
        self.extended.append((channel, payload))


class _FakeContext:
    def __init__(
        self,
        config: dict[str, Any] | None = None,
        with_video: bool = False,
        with_telemetry: bool = False,
    ) -> None:
        self.config_kv = _FakeConfigKv(config)
        if with_video:
            self.video = _FakeVideo()
        if with_telemetry:
            self.telemetry = _FakeTelemetry()

        class _Log:
            def info(self, *args: Any, **kwargs: Any) -> None:
                pass

        self.log = _Log()


def test_plugin_opens_and_closes_the_device() -> None:
    async def scenario() -> None:
        plugin = ThermalUsbPlugin()
        ctx = _FakeContext()
        await plugin.on_start(ctx)
        assert plugin.driver is not None
        assert plugin.session is not None
        await plugin.on_stop(ctx)
        assert plugin.session is None
        assert plugin.driver is None

    asyncio.run(scenario())


def test_plugin_uses_injected_backend_factory() -> None:
    async def scenario() -> None:
        constructed: list[MockUvcBackend] = []

        def factory() -> MockUvcBackend:
            backend = MockUvcBackend(device_count=2)
            constructed.append(backend)
            return backend

        plugin = ThermalUsbPlugin(backend_factory=factory)
        ctx = _FakeContext()
        await plugin.on_start(ctx)
        try:
            assert len(constructed) == 1
            assert plugin.driver is not None
            candidates = await plugin.driver.discover()
            assert len(candidates) == 2
        finally:
            await plugin.on_stop(ctx)

    asyncio.run(scenario())


def test_palette_config_applies_to_the_driver() -> None:
    async def scenario() -> None:
        plugin = ThermalUsbPlugin()
        ctx = _FakeContext(config={"palette": "ironbow"})
        await plugin.on_start(ctx)
        try:
            ctx.config_kv._values["palette"] = "rainbow"
            await plugin.apply_config_once()
            assert plugin.session.palette == "rainbow"
        finally:
            await plugin.on_stop(ctx)

    asyncio.run(scenario())


def test_gain_config_sets_the_radiometric_resolution() -> None:
    async def scenario() -> None:
        plugin = ThermalUsbPlugin()
        ctx = _FakeContext()
        await plugin.on_start(ctx)
        try:
            ctx.config_kv._values["gain"] = False  # low gain -> wider range
            await plugin.apply_config_once()
            assert plugin.session.tlinear_resolution_k_per_count == 0.1
            ctx.config_kv._values["gain"] = True  # high gain -> sensitivity
            await plugin.apply_config_once()
            assert plugin.session.tlinear_resolution_k_per_count == 0.01
        finally:
            await plugin.on_stop(ctx)

    asyncio.run(scenario())


def test_cycle_palette_advances_and_writes_back() -> None:
    from altnautica_thermal_camera.palettes import list_palettes

    async def scenario() -> None:
        plugin = ThermalUsbPlugin()
        ctx = _FakeContext(config={"palette": "ironbow"})
        await plugin.on_start(ctx)
        try:
            palettes = list_palettes()
            expected = palettes[(palettes.index("ironbow") + 1) % len(palettes)]
            ctx.config_kv._values["palette_cycle"] = True
            await plugin.apply_config_once()
            assert plugin.session.palette == expected
            # The cycled palette is written back so the picker reflects it.
            assert await ctx.config_kv.get("palette") == expected
            # The one-shot key is cleared so a re-press cycles again.
            assert await ctx.config_kv.get("palette_cycle") is False
        finally:
            await plugin.on_stop(ctx)

    asyncio.run(scenario())


def test_ffc_action_triggers_and_resets() -> None:
    async def scenario() -> None:
        backend = MockUvcBackend()
        plugin = ThermalUsbPlugin(backend_factory=lambda: backend)
        ctx = _FakeContext()
        await plugin.on_start(ctx)
        try:
            serial = plugin.session.device.serial
            ctx.config_kv._values["ffc"] = True
            await plugin.apply_config_once()
            assert backend.ffc_calls.get(serial) == 1
            assert await ctx.config_kv.get("ffc") is False
        finally:
            await plugin.on_stop(ctx)

    asyncio.run(scenario())


def test_thermal_state_is_published_on_telemetry() -> None:
    async def scenario() -> None:
        plugin = ThermalUsbPlugin()
        ctx = _FakeContext(config={"palette": "ironbow"}, with_telemetry=True)
        await plugin.on_start(ctx)
        try:
            channels = [c for c, _ in ctx.telemetry.extended]
            assert "thermal" in channels
            payload = dict(ctx.telemetry.extended[-1][1])
            assert payload["connected"] is True
            assert payload["palette"] == "ironbow"
        finally:
            await plugin.on_stop(ctx)

    asyncio.run(scenario())


def test_video_source_declared_only_when_an_endpoint_is_configured() -> None:
    async def scenario() -> None:
        # No stream endpoint -> no leg advertised (Rule 44: no phantom stream).
        plugin = ThermalUsbPlugin()
        ctx = _FakeContext(with_video=True)
        await plugin.on_start(ctx)
        assert ctx.video.sources == []
        await plugin.on_stop(ctx)

        # With an endpoint -> the thermal leg is advertised to the pipeline.
        plugin2 = ThermalUsbPlugin()
        ctx2 = _FakeContext(
            config={"stream_source": "rtsp://127.0.0.1:8554/thermal"},
            with_video=True,
        )
        await plugin2.on_start(ctx2)
        try:
            assert len(ctx2.video.sources) == 1
            leg = ctx2.video.sources[0][0]
            assert leg["id"] == "thermal"
            assert leg["role"] == "ir"
            assert leg["source"] == "rtsp://127.0.0.1:8554/thermal"
        finally:
            await plugin2.on_stop(ctx2)

    asyncio.run(scenario())
