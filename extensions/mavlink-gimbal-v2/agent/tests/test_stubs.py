"""Stub driver tests.

The serial-protocol drivers are stubs at v1.0. We assert that they
are concrete subclasses of ``GimbalDriver`` (so the supervisor can
register them) and that ``open()`` raises ``NotImplementedError`` so
the supervisor surfaces a configuration error rather than stalling
silently.
"""

from __future__ import annotations

import inspect

import pytest

from ados.sdk.drivers.gimbal import GimbalCandidate, GimbalDriver
from altnautica_gimbal_v2 import (
    GremsyGimbalDriver,
    SimpleBgcGimbalDriver,
    Storm32NtGimbalDriver,
)


@pytest.mark.parametrize(
    "cls",
    [SimpleBgcGimbalDriver, Storm32NtGimbalDriver, GremsyGimbalDriver],
)
def test_stub_driver_is_concrete_subclass_of_gimbal_driver(cls: type) -> None:
    assert issubclass(cls, GimbalDriver)
    assert not inspect.isabstract(cls)


@pytest.mark.parametrize(
    "cls",
    [SimpleBgcGimbalDriver, Storm32NtGimbalDriver, GremsyGimbalDriver],
)
@pytest.mark.asyncio
async def test_stub_driver_discover_returns_empty(cls: type) -> None:
    driver = cls()
    candidates = await driver.discover()
    assert candidates == []


@pytest.mark.parametrize(
    "cls",
    [SimpleBgcGimbalDriver, Storm32NtGimbalDriver, GremsyGimbalDriver],
)
@pytest.mark.asyncio
async def test_stub_driver_open_raises_not_implemented(cls: type) -> None:
    driver = cls()
    candidate = GimbalCandidate(
        driver_id=cls.driver_id,
        device_id="placeholder",
        label="test",
        bus="serial",
    )
    with pytest.raises(NotImplementedError):
        await driver.open(candidate, config={})


def test_each_stub_has_unique_driver_id() -> None:
    ids = {
        SimpleBgcGimbalDriver.driver_id,
        Storm32NtGimbalDriver.driver_id,
        GremsyGimbalDriver.driver_id,
    }
    assert len(ids) == 3
