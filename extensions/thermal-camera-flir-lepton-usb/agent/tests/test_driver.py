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
    return asyncio.get_event_loop().run_until_complete(coro)


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


class _FakePeripheralManager:
    def __init__(self) -> None:
        self.registered: list[Any] = []

    def register_camera_driver(self, driver: Any) -> None:
        self.registered.append(driver)

    def unregister_camera_driver(self, driver: Any) -> None:
        self.registered.remove(driver)


class _FakeContext:
    def __init__(self) -> None:
        self.peripheral_manager = _FakePeripheralManager()

        class _Log:
            def info(self, *args: Any, **kwargs: Any) -> None:
                pass

        self.log = _Log()


def test_plugin_registers_and_unregisters_driver() -> None:
    plugin = ThermalUsbPlugin()
    ctx = _FakeContext()
    asyncio.run(plugin.on_start(ctx))
    assert plugin.driver is not None
    assert ctx.peripheral_manager.registered == [plugin.driver]
    asyncio.run(plugin.on_stop(ctx))
    assert ctx.peripheral_manager.registered == []
    assert plugin.driver is None


def test_plugin_uses_injected_backend_factory() -> None:
    constructed: list[MockUvcBackend] = []

    def factory() -> MockUvcBackend:
        backend = MockUvcBackend(device_count=2)
        constructed.append(backend)
        return backend

    plugin = ThermalUsbPlugin(backend_factory=factory)
    ctx = _FakeContext()
    asyncio.run(plugin.on_start(ctx))
    assert len(constructed) == 1
    assert plugin.driver is not None
    candidates = asyncio.run(plugin.driver.discover())
    assert len(candidates) == 2
