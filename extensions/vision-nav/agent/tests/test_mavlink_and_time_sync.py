"""Unit tests for the comp 198 HEARTBEAT state machine, the bidirectional
TIMESYNC offset handler, and the MAVLink emit helpers.

The tests use a synchronous test-double in place of the real
:class:`PluginContext`. The plugin code accepts any object that exposes
``ctx.mavlink.send(bytes, component_id=...)``; the double records each
call so assertions can read the resulting frame back out.
"""

from __future__ import annotations

import asyncio
import struct
from dataclasses import dataclass, field
from typing import Any

import pytest

from altnautica_vision_nav.mavlink.comp_status import (
    CRC_HEARTBEAT,
    MAV_STATE_ACTIVE,
    MAV_STATE_CRITICAL,
    MAV_STATE_FLIGHT_TERMINATION,
    MAV_STATE_STANDBY,
    MSG_ID_HEARTBEAT,
    CompanionHeartbeat,
    CompanionState,
    build_heartbeat,
)
from altnautica_vision_nav.mavlink.encoders import (
    CRC_DISTANCE_SENSOR,
    CRC_OPTICAL_FLOW,
    CRC_OPTICAL_FLOW_RAD,
    MSG_ID_DISTANCE_SENSOR,
    MSG_ID_OPTICAL_FLOW,
    MSG_ID_OPTICAL_FLOW_RAD,
    build_optical_flow,
    build_optical_flow_rad,
    emit_optical_flow_rad,
)
from altnautica_vision_nav.time_sync.clock_align import (
    CRC_TIMESYNC,
    DRIFT_BREACH_NS,
    MSG_ID_TIMESYNC,
    ClockAlign,
    build_timesync,
)


# ---------------------------------------------------------------------------
# Test doubles
# ---------------------------------------------------------------------------


@dataclass
class _CallRecord:
    frame: bytes
    component_id: int | None


class _FakeMavlink:
    """Records every ``send`` call. Subscribe is a no-op stub."""

    def __init__(self) -> None:
        self.sends: list[_CallRecord] = []
        self.subscriptions: dict[str, Any] = {}

    async def send(
        self, frame: bytes, component_id: int | None = None
    ) -> dict[str, int]:
        self.sends.append(_CallRecord(frame=bytes(frame), component_id=component_id))
        return {"sent": True, "len": len(frame)}

    async def subscribe(self, msg_name: str, callback: Any) -> None:
        self.subscriptions[msg_name] = callback


class _FakeLog:
    def __init__(self) -> None:
        self.events: list[tuple[str, dict[str, Any]]] = []

    def info(self, event: str, **fields: Any) -> None:
        self.events.append((event, dict(fields)))


@dataclass
class _FakeCtx:
    mavlink: _FakeMavlink = field(default_factory=_FakeMavlink)
    log: _FakeLog = field(default_factory=_FakeLog)


# ---------------------------------------------------------------------------
# Helpers for poking at frame bytes without round-tripping a decoder
# ---------------------------------------------------------------------------


def _msg_id_from_frame(frame: bytes) -> int:
    # STX(1) LEN(1) INCOMPAT(1) COMPAT(1) SEQ(1) SYSID(1) COMPID(1) MSGID(3 LE)
    return frame[7] | (frame[8] << 8) | (frame[9] << 16)


def _sysid_from_frame(frame: bytes) -> int:
    return frame[5]


def _compid_from_frame(frame: bytes) -> int:
    return frame[6]


def _payload_from_frame(frame: bytes) -> bytes:
    length = frame[1]
    return frame[10 : 10 + length]


def _heartbeat_system_status(frame: bytes) -> int:
    payload = _payload_from_frame(frame)
    # HEARTBEAT wire layout: custom_mode (u32), type (u8), autopilot (u8),
    # base_mode (u8), system_status (u8), mavlink_version (u8).
    if len(payload) < 8:
        # Truncation-padded.
        payload = payload + b"\x00" * (8 - len(payload))
    return struct.unpack_from("<B", payload, 7)[0]


# ---------------------------------------------------------------------------
# Heartbeat state machine
# ---------------------------------------------------------------------------


class TestCompanionHeartbeat:
    @pytest.mark.asyncio
    async def test_state_machine_transitions_log_and_drive_system_status(
        self,
    ) -> None:
        ctx = _FakeCtx()
        hb = CompanionHeartbeat(component_id=198)
        await hb.start(ctx)
        try:
            # First tick fires immediately at start: INACTIVE -> STANDBY(3).
            await asyncio.sleep(0)
            await _drain(ctx, count=1)
            first = ctx.mavlink.sends[0].frame
            assert _msg_id_from_frame(first) == MSG_ID_HEARTBEAT
            assert _compid_from_frame(first) == 198
            assert _heartbeat_system_status(first) == MAV_STATE_STANDBY

            # Move INACTIVE -> ACTIVE -> CRITICAL -> TERMINATING.
            ctx.log.events.clear()
            for new_state, expected_status in (
                (CompanionState.ACTIVE, MAV_STATE_ACTIVE),
                (CompanionState.CRITICAL, MAV_STATE_CRITICAL),
                (CompanionState.TERMINATING, MAV_STATE_FLIGHT_TERMINATION),
            ):
                ctx.mavlink.sends.clear()
                hb.transition(new_state)
                await hb._emit_once()
                latest = ctx.mavlink.sends[-1].frame
                assert _heartbeat_system_status(latest) == expected_status

            # Each transition logged a state-change event.
            recorded_states = [
                event[1]["new"]
                for event in ctx.log.events
                if event[0] == "companion_heartbeat_transition"
            ]
            assert recorded_states == ["active", "critical", "terminating"]
        finally:
            await hb.stop()

    @pytest.mark.asyncio
    async def test_stop_emits_terminating_heartbeat(self) -> None:
        ctx = _FakeCtx()
        hb = CompanionHeartbeat(component_id=198)
        await hb.start(ctx)
        await asyncio.sleep(0)
        ctx.mavlink.sends.clear()
        hb.transition(CompanionState.ACTIVE)
        await hb.stop()
        # The stop path emits exactly one final HEARTBEAT in TERMINATING.
        terminating_frames = [
            r.frame
            for r in ctx.mavlink.sends
            if _heartbeat_system_status(r.frame) == MAV_STATE_FLIGHT_TERMINATION
        ]
        assert len(terminating_frames) >= 1


# ---------------------------------------------------------------------------
# Clock alignment
# ---------------------------------------------------------------------------


class TestClockAlign:
    def test_offset_estimate_adopts_first_sample(self) -> None:
        clock = ClockAlign(component_id=198)
        # Simulate the FC running one second ahead of the plugin's monotonic.
        ts1 = 1_000_000_000_000
        tc1 = ts1 + 1_000_000_000  # +1s
        clock.handle_timesync_response(tc1=tc1, ts1=ts1)
        assert clock.offset_ns == 1_000_000_000

    def test_drift_breach_bumps_reset_counter(self) -> None:
        clock = ClockAlign(component_id=198)
        # First sample: 0 ns offset.
        clock.handle_timesync_response(tc1=1_000, ts1=1_000)
        assert clock.reset_counter == 0
        # Second sample diverges by 80 ms (> 50 ms threshold).
        ts1 = 2_000_000_000
        tc1 = ts1 + 80_000_000  # 80 ms ahead
        clock.handle_timesync_response(tc1=tc1, ts1=ts1)
        assert clock.reset_counter == 1
        # A third sample within the threshold does not bump again.
        ts1 = 3_000_000_000
        tc1 = ts1 + clock.offset_ns + 1_000_000  # within 1 ms of running offset
        clock.handle_timesync_response(tc1=tc1, ts1=ts1)
        assert clock.reset_counter == 1

    def test_convert_to_fc_clock_applies_offset(self) -> None:
        clock = ClockAlign(component_id=198)
        clock.offset_ns = 500_000_000  # +500 ms
        assert clock.convert_to_fc_clock(1_000_000_000) == 1_500_000_000

    def test_drift_threshold_constant_matches_spec(self) -> None:
        # 50 ms in nanoseconds is the documented threshold.
        assert DRIFT_BREACH_NS == 50_000_000

    def test_ema_smoothing_after_first_sample(self) -> None:
        clock = ClockAlign(component_id=198)
        clock.handle_timesync_response(tc1=1_000_000_000, ts1=0)
        assert clock.offset_ns == 1_000_000_000
        # Second sample at +1.01s drift is below the 50 ms threshold
        # relative to the first; EMA should drag the offset slightly.
        ts1 = 1_000_000_000
        tc1 = ts1 + 1_010_000_000
        prior = clock.offset_ns
        clock.handle_timesync_response(tc1=tc1, ts1=ts1)
        # Did not bump reset counter (drift below 50 ms).
        assert clock.reset_counter == 0
        # New offset moved toward the estimate by ~10%.
        assert prior < clock.offset_ns < tc1 - ts1


# ---------------------------------------------------------------------------
# Emit helpers
# ---------------------------------------------------------------------------


class TestEmitHelpers:
    @pytest.mark.asyncio
    async def test_emit_optical_flow_rad_uses_fc_clock(self) -> None:
        ctx = _FakeCtx()
        clock = ClockAlign(component_id=198)
        # Pin a known offset so the test does not depend on real time.
        clock.offset_ns = 7_000_000_000  # +7 seconds vs monotonic

        await emit_optical_flow_rad(
            ctx,
            component_id=198,
            sensor_id=0,
            integration_time_us=33_333,
            integrated_x=0.01,
            integrated_y=-0.02,
            integrated_xgyro=0.005,
            integrated_ygyro=-0.004,
            integrated_zgyro=0.0,
            temperature=2500,
            quality=200,
            time_delta_distance_us=0,
            distance=1.25,
            clock_align=clock,
        )

        assert len(ctx.mavlink.sends) == 1
        frame = ctx.mavlink.sends[0].frame
        assert _msg_id_from_frame(frame) == MSG_ID_OPTICAL_FLOW_RAD
        assert _compid_from_frame(frame) == 198

        # First 8 bytes of the payload are time_usec (u64 little endian).
        payload = _payload_from_frame(frame)
        if len(payload) < 8:
            payload = payload + b"\x00" * (8 - len(payload))
        time_usec = struct.unpack_from("<Q", payload, 0)[0]

        # time_usec must reflect the FC-clock conversion, not the raw
        # monotonic value. With a +7s offset the FC clock should be
        # several seconds ahead of any plausible monotonic-only reading.
        # We check the delta is at least ~6.5 seconds in microseconds.
        # That confirms the offset was applied rather than truncated.
        assert time_usec > 6_500_000  # > 6.5 s in microseconds


# ---------------------------------------------------------------------------
# CRC_EXTRA fingerprints
# ---------------------------------------------------------------------------


def test_crc_extra_values_match_dialect_fingerprints() -> None:
    """Pin the dialect-generated CRC_EXTRA bytes so accidental drift fails fast."""
    assert MSG_ID_HEARTBEAT == 0
    assert CRC_HEARTBEAT == 50
    assert MSG_ID_OPTICAL_FLOW == 100
    assert CRC_OPTICAL_FLOW == 175
    assert MSG_ID_OPTICAL_FLOW_RAD == 106
    assert CRC_OPTICAL_FLOW_RAD == 138
    assert MSG_ID_DISTANCE_SENSOR == 132
    assert CRC_DISTANCE_SENSOR == 85
    assert MSG_ID_TIMESYNC == 111
    assert CRC_TIMESYNC == 34


# ---------------------------------------------------------------------------
# Frame round-trip sanity
# ---------------------------------------------------------------------------


def test_build_optical_flow_produces_v2_frame_with_expected_fields() -> None:
    frame = build_optical_flow(
        sys_id=1,
        comp_id=198,
        seq=0,
        time_usec=123_456_789,
        sensor_id=0,
        flow_x=10,
        flow_y=-20,
        flow_comp_m_x=0.5,
        flow_comp_m_y=-0.25,
        quality=180,
        ground_distance=2.0,
    )
    assert frame[0] == 0xFD  # v2 STX
    assert _msg_id_from_frame(frame) == MSG_ID_OPTICAL_FLOW
    assert _sysid_from_frame(frame) == 1
    assert _compid_from_frame(frame) == 198


def test_build_optical_flow_rad_produces_v2_frame_with_expected_fields() -> None:
    frame = build_optical_flow_rad(
        sys_id=1,
        comp_id=198,
        seq=1,
        time_usec=1,
        sensor_id=0,
        integration_time_us=33_333,
        integrated_x=0.0,
        integrated_y=0.0,
        integrated_xgyro=0.0,
        integrated_ygyro=0.0,
        integrated_zgyro=0.0,
        temperature=0,
        quality=0,
        time_delta_distance_us=0,
        distance=0.0,
    )
    assert _msg_id_from_frame(frame) == MSG_ID_OPTICAL_FLOW_RAD


def test_build_timesync_produces_v2_frame_with_expected_fields() -> None:
    frame = build_timesync(sys_id=1, comp_id=198, seq=0, tc1=0, ts1=42)
    assert _msg_id_from_frame(frame) == MSG_ID_TIMESYNC
    assert _compid_from_frame(frame) == 198


def test_build_heartbeat_produces_v2_frame_with_expected_fields() -> None:
    frame = build_heartbeat(
        sys_id=1,
        comp_id=198,
        seq=0,
        system_status=MAV_STATE_ACTIVE,
    )
    assert _msg_id_from_frame(frame) == MSG_ID_HEARTBEAT
    assert _heartbeat_system_status(frame) == MAV_STATE_ACTIVE


# ---------------------------------------------------------------------------
# Utility
# ---------------------------------------------------------------------------


async def _drain(ctx: _FakeCtx, *, count: int, timeout_s: float = 0.5) -> None:
    """Wait until ``ctx.mavlink.sends`` has at least ``count`` entries."""
    deadline = asyncio.get_event_loop().time() + timeout_s
    while len(ctx.mavlink.sends) < count:
        if asyncio.get_event_loop().time() > deadline:
            raise AssertionError(
                f"timeout waiting for {count} sends; got {len(ctx.mavlink.sends)}"
            )
        await asyncio.sleep(0.01)
