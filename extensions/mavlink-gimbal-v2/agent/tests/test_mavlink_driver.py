"""Driver tests.

The driver emits four kinds of commands. The mock router records each
``send_command`` call so the tests assert command id and parameter
values directly. The round-trip test injects a synthesized
``GIMBAL_DEVICE_ATTITUDE_STATUS`` and verifies the session state
updates accordingly.
"""

from __future__ import annotations

import asyncio
from typing import Any

import pytest

from altnautica_gimbal_v2.mavlink_driver import MavlinkGimbalDriver
from altnautica_gimbal_v2.mavlink_messages import (
    MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
    MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
    MAV_CMD_DO_SET_ROI_LOCATION,
    MAV_CMD_DO_SET_ROI_NONE,
)


class MockRouter:
    def __init__(self) -> None:
        self.sent: list[Any] = []

    def send_command(self, cmd: Any) -> bool:
        self.sent.append(cmd)
        return True


@pytest.fixture
def router() -> MockRouter:
    return MockRouter()


@pytest.fixture
def driver(router: MockRouter) -> MavlinkGimbalDriver:
    return MavlinkGimbalDriver(router=router)


@pytest.mark.asyncio
async def test_discover_returns_one_logical_candidate(
    driver: MavlinkGimbalDriver,
) -> None:
    candidates = await driver.discover()
    assert len(candidates) == 1
    assert candidates[0].driver_id == "mavlink-gimbal-v2"
    assert candidates[0].bus == "mavlink"


@pytest.mark.asyncio
async def test_open_emits_configure_and_returns_session(
    driver: MavlinkGimbalDriver, router: MockRouter
) -> None:
    candidates = await driver.discover()
    session = await driver.open(candidates[0], config={})
    assert session is not None
    assert any(
        getattr(c, "command", None) == MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
        for c in router.sent
    )


@pytest.mark.asyncio
async def test_command_attitude_emits_pitchyaw_with_clamped_values(
    driver: MavlinkGimbalDriver, router: MockRouter
) -> None:
    candidates = await driver.discover()
    session = await driver.open(
        candidates[0],
        config={
            "limits": {
                "pitch_min_deg": -90.0,
                "pitch_max_deg": 30.0,
                "yaw_min_deg": -180.0,
                "yaw_max_deg": 180.0,
                "roll_min_deg": -45.0,
                "roll_max_deg": 45.0,
            }
        },
    )
    await driver.command_attitude(session, pitch_deg=-30.0, yaw_deg=45.0)
    pitchyaw = [
        c for c in router.sent
        if getattr(c, "command", None) == MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
    ]
    assert len(pitchyaw) == 1
    assert pitchyaw[0].param1 == -30.0
    assert pitchyaw[0].param2 == 45.0


@pytest.mark.asyncio
async def test_command_attitude_clamps_out_of_range_setpoints(
    driver: MavlinkGimbalDriver, router: MockRouter
) -> None:
    candidates = await driver.discover()
    session = await driver.open(
        candidates[0],
        config={"limits": {"pitch_min_deg": -45.0, "pitch_max_deg": 30.0}},
    )
    await driver.command_attitude(session, pitch_deg=-90.0, yaw_deg=0.0)
    pitchyaw = [
        c for c in router.sent
        if getattr(c, "command", None) == MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
    ]
    # Pitch should be clamped to -45 (the configured minimum).
    assert pitchyaw[-1].param1 == -45.0


@pytest.mark.asyncio
async def test_set_roi_location_emits_command_int_with_scaled_lat_lon(
    driver: MavlinkGimbalDriver, router: MockRouter
) -> None:
    candidates = await driver.discover()
    session = await driver.open(candidates[0], config={})
    await driver.set_roi_location(
        session, lat_deg=12.971, lon_deg=77.594, alt_m=50.0
    )
    roi = [
        c for c in router.sent
        if getattr(c, "command", None) == MAV_CMD_DO_SET_ROI_LOCATION
    ]
    assert len(roi) == 1
    assert roi[0].x == 129710000
    assert roi[0].y == 775940000
    assert roi[0].z == 50.0


@pytest.mark.asyncio
async def test_clear_roi_emits_set_roi_none(
    driver: MavlinkGimbalDriver, router: MockRouter
) -> None:
    candidates = await driver.discover()
    session = await driver.open(candidates[0], config={})
    await driver.clear_roi(session)
    none_cmds = [
        c for c in router.sent
        if getattr(c, "command", None) == MAV_CMD_DO_SET_ROI_NONE
    ]
    assert len(none_cmds) == 1


@pytest.mark.asyncio
async def test_round_trip_command_then_status_updates_state(
    driver: MavlinkGimbalDriver, router: MockRouter
) -> None:
    candidates = await driver.discover()
    session = await driver.open(candidates[0], config={})

    # Step 1: GCS asks the gimbal to point pitch=-30, yaw=45.
    await driver.command_attitude(session, pitch_deg=-30.0, yaw_deg=45.0)

    # Step 2: a router-level subscription mirrors a synthesized
    # GIMBAL_DEVICE_ATTITUDE_STATUS reply back into the driver. The
    # mock simulates SITL acknowledging the new attitude.
    driver.on_attitude_status(
        session, pitch_deg=-29.5, yaw_deg=44.7, roll_deg=0.1
    )
    state = driver.get_state(session)
    assert state.pitch_deg == pytest.approx(-29.5)
    assert state.yaw_deg == pytest.approx(44.7)
    assert state.roll_deg == pytest.approx(0.1)


@pytest.mark.asyncio
async def test_close_releases_roi(
    driver: MavlinkGimbalDriver, router: MockRouter
) -> None:
    candidates = await driver.discover()
    session = await driver.open(candidates[0], config={})
    router.sent.clear()
    await driver.close(session)
    none_cmds = [
        c for c in router.sent
        if getattr(c, "command", None) == MAV_CMD_DO_SET_ROI_NONE
    ]
    assert len(none_cmds) == 1


@pytest.mark.asyncio
async def test_capabilities_reflect_configured_axis_limits(
    driver: MavlinkGimbalDriver, router: MockRouter
) -> None:
    candidates = await driver.discover()
    session = await driver.open(
        candidates[0],
        config={
            "limits": {
                "pitch_min_deg": -60.0,
                "pitch_max_deg": 20.0,
                "yaw_min_deg": -170.0,
                "yaw_max_deg": 170.0,
                "roll_min_deg": -30.0,
                "roll_max_deg": 30.0,
            }
        },
    )
    caps = driver.capabilities(session)
    assert caps.pitch_min_deg == -60.0
    assert caps.pitch_max_deg == 20.0
    assert caps.yaw_min_deg == -170.0
    assert caps.yaw_max_deg == 170.0
    assert caps.has_pitch is True
    assert caps.has_yaw is True
    assert caps.has_roll is True


@pytest.mark.asyncio
async def test_get_state_returns_neutral_at_open(
    driver: MavlinkGimbalDriver, router: MockRouter
) -> None:
    candidates = await driver.discover()
    session = await driver.open(candidates[0], config={})
    state = driver.get_state(session)
    assert state.pitch_deg == 0.0
    assert state.yaw_deg == 0.0
    assert state.roll_deg == 0.0


@pytest.mark.asyncio
async def test_command_rate_emits_pitchyaw_with_rate_fields(
    driver: MavlinkGimbalDriver, router: MockRouter
) -> None:
    candidates = await driver.discover()
    session = await driver.open(candidates[0], config={})
    router.sent.clear()
    await driver.command_rate(
        session, pitch_rate_dps=10.0, yaw_rate_dps=-5.0
    )
    pitchyaw = [
        c for c in router.sent
        if getattr(c, "command", None) == MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
    ]
    assert len(pitchyaw) == 1
    assert pitchyaw[0].param3 == 10.0
    assert pitchyaw[0].param4 == -5.0


@pytest.mark.asyncio
async def test_state_after_command_attitude_records_mode_manual(
    driver: MavlinkGimbalDriver, router: MockRouter
) -> None:
    candidates = await driver.discover()
    session = await driver.open(candidates[0], config={})
    await driver.command_attitude(session, pitch_deg=-10.0, yaw_deg=20.0)
    state = driver.get_state(session)
    assert state.mode == "manual"


@pytest.mark.asyncio
async def test_state_after_set_roi_records_mode_roi_lock(
    driver: MavlinkGimbalDriver, router: MockRouter
) -> None:
    candidates = await driver.discover()
    session = await driver.open(candidates[0], config={})
    await driver.set_roi_location(
        session, lat_deg=1.0, lon_deg=2.0, alt_m=3.0
    )
    state = driver.get_state(session)
    assert state.mode == "roi-lock"


@pytest.mark.asyncio
async def test_typed_session_guard_rejects_foreign_session(
    driver: MavlinkGimbalDriver,
) -> None:
    class Foreign:
        pass

    with pytest.raises(TypeError):
        driver.capabilities(Foreign())  # type: ignore[arg-type]


@pytest.mark.asyncio
async def test_state_iterator_produces_pushed_samples(
    driver: MavlinkGimbalDriver, router: MockRouter
) -> None:
    candidates = await driver.discover()
    session = await driver.open(candidates[0], config={})
    iterator = await driver.state_iterator(session)

    driver.on_attitude_status(session, pitch_deg=-5.0, yaw_deg=10.0, roll_deg=0.0)
    sample = await asyncio.wait_for(iterator.__anext__(), timeout=0.5)
    assert sample.pitch_deg == -5.0
    assert sample.yaw_deg == 10.0
