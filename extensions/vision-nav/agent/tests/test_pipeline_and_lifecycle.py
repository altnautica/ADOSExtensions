"""Unit tests for the OF pipeline, pre-arm helper, health publisher,
and config validation.

The tests use fakes for everything that would otherwise need real
hardware:

* a fake capture source that yields a fixed list of frames;
* a fake optical-flow processor that returns a canned result;
* a fake rangefinder that returns a canned reading;
* a fake MAVLink facade that records every ``send`` call.

Each fake exposes only what the production code paths reach into; the
goal is to test the orchestration logic, not the OpenCV math (which is
covered by the capture/processor test file).
"""

from __future__ import annotations

import asyncio
import struct
from dataclasses import dataclass, field
from typing import Any, AsyncIterator, Optional, Tuple

import pytest
from pydantic import ValidationError

from altnautica_vision_nav.config import (
    PreArmConfig,
    RangefinderConfig,
    VisionNavConfig,
    load_config,
)
from altnautica_vision_nav.health import HealthPublisher
from altnautica_vision_nav.mavlink.comp_status import CompanionState
from altnautica_vision_nav.pipelines.of_pipeline import OFPipeline
from altnautica_vision_nav.pre_arm import (
    CRC_SET_GPS_GLOBAL_ORIGIN,
    CRC_SET_HOME_POSITION,
    MSG_ID_SET_GPS_GLOBAL_ORIGIN,
    MSG_ID_SET_HOME_POSITION,
    PreArmHelper,
)
from altnautica_vision_nav.processors.optical_flow_lk import OpticalFlowResult
from altnautica_vision_nav.rangefinder.base import RangeReading


# ---------------------------------------------------------------------------
# Frame-decoding helpers (mirror the ones in test_mavlink_and_time_sync.py
# but live here so this file is self-contained).
# ---------------------------------------------------------------------------


def _msg_id_from_frame(frame: bytes) -> int:
    return frame[7] | (frame[8] << 8) | (frame[9] << 16)


def _compid_from_frame(frame: bytes) -> int:
    return frame[6]


# ---------------------------------------------------------------------------
# Fakes
# ---------------------------------------------------------------------------


@dataclass
class _SendCall:
    frame: bytes
    component_id: Optional[int]


class _FakeMavlink:
    """Records every ``send`` call. Subscribe and unsubscribe are stubs."""

    def __init__(self) -> None:
        self.sends: list[_SendCall] = []
        self.subscriptions: dict[str, Any] = {}

    async def send(
        self, frame: bytes, component_id: Optional[int] = None
    ) -> dict[str, int]:
        self.sends.append(_SendCall(frame=bytes(frame), component_id=component_id))
        return {"sent": True, "len": len(frame)}

    def subscribe(self, msg_name: str, callback: Any) -> Any:
        self.subscriptions[msg_name] = callback

        def _unsubscribe() -> None:
            self.subscriptions.pop(msg_name, None)

        return _unsubscribe

    def unsubscribe(self, msg_name: str, callback: Any) -> None:
        self.subscriptions.pop(msg_name, None)


class _FakeTelemetry:
    """Records every ``extend`` call as ``(topic, payload)`` tuples."""

    def __init__(self) -> None:
        self.extends: list[tuple[str, dict]] = []

    async def extend(self, topic: str, payload: dict) -> None:
        self.extends.append((topic, dict(payload)))


class _FakeLog:
    def __init__(self) -> None:
        self.events: list[tuple[str, str, dict]] = []

    def info(self, event: str, **fields: Any) -> None:
        self.events.append(("info", event, dict(fields)))

    def warning(self, event: str, **fields: Any) -> None:
        self.events.append(("warning", event, dict(fields)))


@dataclass
class _FakeCtx:
    mavlink: _FakeMavlink = field(default_factory=_FakeMavlink)
    telemetry: _FakeTelemetry = field(default_factory=_FakeTelemetry)
    log: _FakeLog = field(default_factory=_FakeLog)


class _FakeCapture:
    """Capture source that yields a fixed list of frames once."""

    def __init__(
        self, frames: list[Tuple[int, Any]], *, hold_after: bool = True
    ) -> None:
        self._frames = list(frames)
        self._hold_after = hold_after

    async def frames(self) -> AsyncIterator[Tuple[int, Any]]:
        for ts, frame in self._frames:
            yield ts, frame
        if self._hold_after:
            # Keep the iterator alive so the pipeline reaches its
            # ``stop()`` cleanly without raising StopAsyncIteration
            # from the loop body.
            while True:
                await asyncio.sleep(0.01)
                # Never yield more; the test will stop the pipeline.


class _FakeProcessor:
    """Returns a canned OpticalFlowResult on every ``process`` call."""

    def __init__(self, result: OpticalFlowResult) -> None:
        self._result = result
        self.calls = 0

    def process(
        self,
        prev_gray: Any,
        curr_gray: Any,
        dt_seconds: float,
        gyro: Optional[Any] = None,
        distance_m: Optional[float] = None,
    ) -> OpticalFlowResult:
        self.calls += 1
        return self._result


class _FakeRangefinder:
    """Driver-shaped fake that returns a canned reading."""

    def __init__(
        self,
        reading: Optional[RangeReading],
        *,
        min_m: float = 0.1,
        max_m: float = 12.0,
    ) -> None:
        self._reading = reading
        self._min_m = min_m
        self._max_m = max_m
        self.opened = False
        self.closed = False

    @property
    def name(self) -> str:
        return "fake"

    @property
    def min_range_m(self) -> float:
        return self._min_m

    @property
    def max_range_m(self) -> float:
        return self._max_m

    async def open(self) -> None:
        self.opened = True

    async def close(self) -> None:
        self.closed = True

    async def read(self) -> Optional[RangeReading]:
        return self._reading


class _FakeHeartbeat:
    """Heartbeat shape used by the pipeline; tracks state transitions."""

    def __init__(self) -> None:
        self.state: CompanionState = CompanionState.INACTIVE
        self.transitions: list[CompanionState] = []

    def transition(self, new_state: CompanionState) -> None:
        if new_state is self.state:
            return
        self.state = new_state
        self.transitions.append(new_state)


# ---------------------------------------------------------------------------
# Config validation
# ---------------------------------------------------------------------------


class TestConfig:
    def test_config_load_defaults(self) -> None:
        cfg = load_config({})
        assert cfg.mode == "optical_flow"
        assert cfg.camera.device_path == "/dev/video0"
        assert cfg.camera.bus_type == "uvc"
        assert cfg.camera.width == 640
        assert cfg.camera.height == 480
        assert cfg.camera.fps == 30
        assert cfg.rangefinder.topology == "fc"
        assert cfg.rangefinder.driver == "fc_relay"
        assert cfg.firmware.type == "ardupilot"
        assert cfg.pre_arm.auto_set_origin is False
        assert cfg.flow_quality_min == 50

    def test_config_validation_rejects_invalid_driver(self) -> None:
        with pytest.raises(ValidationError):
            load_config(
                {
                    "rangefinder": {
                        "topology": "companion",
                        "driver": "fake",
                    }
                }
            )

    def test_config_validation_rejects_out_of_range_lat(self) -> None:
        with pytest.raises(ValidationError):
            load_config({"pre_arm": {"origin_lat": 91.0}})

    def test_config_accepts_partial_overrides(self) -> None:
        cfg = load_config({"camera": {"bus_type": "csi", "width": 1280}})
        assert cfg.camera.bus_type == "csi"
        assert cfg.camera.width == 1280
        # Other fields still default.
        assert cfg.camera.height == 480
        assert cfg.camera.fps == 30

    def test_config_rejects_invalid_camera_device_path(self) -> None:
        """An attacker cannot redirect the capture path at an arbitrary
        device by smuggling a non-video node through the per-drone
        config."""

        with pytest.raises(ValidationError):
            load_config({"camera": {"device_path": "/dev/sda"}})
        with pytest.raises(ValidationError):
            load_config({"camera": {"device_path": "/etc/passwd"}})
        with pytest.raises(ValidationError):
            load_config({"camera": {"device_path": "/dev/video0; rm -rf /"}})

    def test_config_accepts_valid_camera_device_path(self) -> None:
        cfg = load_config({"camera": {"device_path": "/dev/video2"}})
        assert cfg.camera.device_path == "/dev/video2"

    def test_config_rejects_invalid_rangefinder_device(self) -> None:
        """The UART rangefinder driver opens any path as a tty; the
        schema must keep operators from pointing it at a sensitive
        file or block device."""

        with pytest.raises(ValidationError):
            load_config(
                {
                    "rangefinder": {
                        "topology": "companion",
                        "driver": "tfluna_uart",
                        "device": "/etc/shadow",
                    }
                }
            )
        with pytest.raises(ValidationError):
            load_config(
                {
                    "rangefinder": {
                        "topology": "companion",
                        "driver": "tfluna_uart",
                        "device": "/dev/sda",
                    }
                }
            )
        with pytest.raises(ValidationError):
            load_config(
                {
                    "rangefinder": {
                        "topology": "companion",
                        "driver": "tfluna_uart",
                        "device": "../../etc/shadow",
                    }
                }
            )

    def test_config_accepts_valid_rangefinder_uart_device(self) -> None:
        cfg = load_config(
            {
                "rangefinder": {
                    "topology": "companion",
                    "driver": "tfluna_uart",
                    "device": "/dev/ttyUSB0",
                }
            }
        )
        assert cfg.rangefinder.device == "/dev/ttyUSB0"

    def test_config_accepts_valid_rangefinder_i2c_bus_identifier(self) -> None:
        # I2C drivers consume the same ``device`` field as either a bare
        # bus number or the conventional path form.
        cfg_digit = load_config(
            {
                "rangefinder": {
                    "topology": "companion",
                    "driver": "vl53l1x_i2c",
                    "device": "1",
                }
            }
        )
        assert cfg_digit.rangefinder.device == "1"
        cfg_path = load_config(
            {
                "rangefinder": {
                    "topology": "companion",
                    "driver": "garmin_lidarlite_i2c",
                    "device": "/dev/i2c-3",
                }
            }
        )
        assert cfg_path.rangefinder.device == "/dev/i2c-3"


# ---------------------------------------------------------------------------
# OF pipeline
# ---------------------------------------------------------------------------


def _high_quality_result(quality: int = 200) -> OpticalFlowResult:
    return OpticalFlowResult(
        flow_x_dpi=4.0,
        flow_y_dpi=-3.0,
        flow_comp_m_x=0.1,
        flow_comp_m_y=-0.05,
        flow_rate_x=0.01,
        flow_rate_y=-0.02,
        flow_rate_z=0.0,
        quality=quality,
        integration_time_us=33_000,
    )


def _low_quality_result() -> OpticalFlowResult:
    return OpticalFlowResult(
        flow_x_dpi=0.0,
        flow_y_dpi=0.0,
        flow_comp_m_x=0.0,
        flow_comp_m_y=0.0,
        flow_rate_x=0.0,
        flow_rate_y=0.0,
        flow_rate_z=0.0,
        quality=10,
        integration_time_us=33_000,
    )


def _canned_reading(distance_m: float = 1.5) -> RangeReading:
    return RangeReading(
        distance_m=distance_m,
        quality=90,
        timestamp_monotonic_ns=0,
    )


class TestOFPipeline:
    @pytest.mark.asyncio
    async def test_of_pipeline_emit_on_high_quality(self) -> None:
        ctx = _FakeCtx()
        # Two frames are enough: first becomes ``prev``, second triggers
        # one process() / one emit pair.
        capture = _FakeCapture(
            [(1_000_000_000, "f0"), (1_033_000_000, "f1")],
            hold_after=True,
        )
        processor = _FakeProcessor(_high_quality_result(quality=200))
        rangefinder = _FakeRangefinder(_canned_reading(1.5))
        heartbeat = _FakeHeartbeat()

        pipeline = OFPipeline(
            ctx=ctx,
            capture_source=capture,
            of_processor=processor,
            gyro_tap=None,
            rangefinder=rangefinder,
            clock_align=None,
            companion_heartbeat=heartbeat,
            health_publisher=None,
            flow_quality_min=50,
            component_id=198,
            sensor_id=0,
        )

        await pipeline.start()
        # Give the loop a chance to consume both frames and emit.
        for _ in range(20):
            if processor.calls >= 1 and len(ctx.mavlink.sends) >= 2:
                break
            await asyncio.sleep(0.01)
        await pipeline.stop()

        assert processor.calls >= 1
        # Two emits per healthy frame: OPTICAL_FLOW_RAD + DISTANCE_SENSOR.
        msg_ids = {_msg_id_from_frame(s.frame) for s in ctx.mavlink.sends}
        assert 106 in msg_ids  # OPTICAL_FLOW_RAD
        assert 132 in msg_ids  # DISTANCE_SENSOR
        # All emits ride the comp 198 peripheral.
        for sent in ctx.mavlink.sends:
            assert _compid_from_frame(sent.frame) == 198
            assert sent.component_id == 198
        # Heartbeat transitioned to ACTIVE.
        assert CompanionState.ACTIVE in heartbeat.transitions

    @pytest.mark.asyncio
    async def test_of_pipeline_companion_state_transitions(self) -> None:
        """Two seconds of low-quality frames must push companion to CRITICAL."""

        ctx = _FakeCtx()
        # Six frames spaced 0.5 s apart in *frame-stamp* nanoseconds.
        # The pipeline measures the streak against the frame timestamps,
        # not against wall-clock, so the loop runs in tight async time
        # while the in-pipeline streak counter sees 2.5 s of frames.
        frames = [(int(0.5e9) * i, f"f{i}") for i in range(6)]
        capture = _FakeCapture(frames, hold_after=True)
        processor = _FakeProcessor(_low_quality_result())
        heartbeat = _FakeHeartbeat()

        pipeline = OFPipeline(
            ctx=ctx,
            capture_source=capture,
            of_processor=processor,
            gyro_tap=None,
            rangefinder=None,
            clock_align=None,
            companion_heartbeat=heartbeat,
            health_publisher=None,
            flow_quality_min=50,
        )

        await pipeline.start()
        # Wait for all 6 frames to drain.
        for _ in range(50):
            if processor.calls >= 5:
                break
            await asyncio.sleep(0.01)
        await pipeline.stop()

        # No emits because every frame was below the threshold.
        emit_ids = {_msg_id_from_frame(s.frame) for s in ctx.mavlink.sends}
        assert 106 not in emit_ids
        # CRITICAL must have fired.
        assert CompanionState.CRITICAL in heartbeat.transitions
        # ACTIVE must NOT have fired (we never saw a good frame).
        assert CompanionState.ACTIVE not in heartbeat.transitions


# ---------------------------------------------------------------------------
# Pre-arm helper
# ---------------------------------------------------------------------------


class TestPreArmHelper:
    @pytest.mark.asyncio
    async def test_pre_arm_auto_set_origin_sends_both_messages(self) -> None:
        ctx = _FakeCtx()
        helper = PreArmHelper(component_id=198)

        await helper.auto_set_origin(ctx, 12.97, 77.59, 920.0)

        assert len(ctx.mavlink.sends) == 2
        origin_frame = ctx.mavlink.sends[0].frame
        home_frame = ctx.mavlink.sends[1].frame
        assert _msg_id_from_frame(origin_frame) == MSG_ID_SET_GPS_GLOBAL_ORIGIN
        assert _msg_id_from_frame(home_frame) == MSG_ID_SET_HOME_POSITION
        # Both carry the plugin's component id.
        assert _compid_from_frame(origin_frame) == 198
        assert _compid_from_frame(home_frame) == 198
        # The send call also tagged the routed component id.
        assert ctx.mavlink.sends[0].component_id == 198
        assert ctx.mavlink.sends[1].component_id == 198

    @pytest.mark.asyncio
    async def test_pre_arm_payload_encodes_lat_lon_alt(self) -> None:
        ctx = _FakeCtx()
        helper = PreArmHelper(component_id=198)
        await helper.auto_set_origin(ctx, 12.97, 77.59, 920.0)

        origin_frame = ctx.mavlink.sends[0].frame
        length = origin_frame[1]
        payload = origin_frame[10 : 10 + length]
        # Truncation may strip trailing zero bytes from extension fields.
        if len(payload) < 13:
            payload = payload + b"\x00" * (13 - len(payload))
        lat_e7, lon_e7, alt_mm, target_system = struct.unpack_from(
            "<iiiB", payload, 0
        )
        assert lat_e7 == int(round(12.97 * 1e7))
        assert lon_e7 == int(round(77.59 * 1e7))
        assert alt_mm == int(round(920.0 * 1000.0))
        assert target_system == 1

    def test_pre_arm_crc_extra_values_match_dialect_fingerprints(self) -> None:
        # Pin the documented values so accidental edits fail the suite.
        assert MSG_ID_SET_GPS_GLOBAL_ORIGIN == 48
        assert CRC_SET_GPS_GLOBAL_ORIGIN == 41
        assert MSG_ID_SET_HOME_POSITION == 243
        assert CRC_SET_HOME_POSITION == 85


# ---------------------------------------------------------------------------
# Health publisher payload shape
# ---------------------------------------------------------------------------


class TestHealthPublisher:
    @pytest.mark.asyncio
    async def test_health_publisher_payload_shape(self) -> None:
        ctx = _FakeCtx()
        health = HealthPublisher(
            rangefinder_topology="companion",
            recommended_camera_id="/dev/video0",
        )
        health.update_from_pipeline(
            result=_high_quality_result(quality=180),
            flow_rate_hz=29.5,
            distance_m=1.25,
        )
        health.update_companion_state(CompanionState.ACTIVE)

        snapshot = health.snapshot()
        # Field names are camelCase end-to-end.
        expected_keys = {
            "opticalFlowSupported",
            "vioSupported",
            "rangefinderTopology",
            "recommendedCameraId",
            "flowQuality",
            "flowRateHz",
            "flowDistanceM",
            "vioState",
            "vioResetCounter",
            "vioQuality",
            "companionState",
        }
        assert set(snapshot.keys()) == expected_keys
        # Snake case must not leak into the payload.
        for key in snapshot:
            assert "_" not in key, f"non-camelCase key: {key!r}"
        # Types match the cloud relay validator.
        assert snapshot["opticalFlowSupported"] is True
        assert snapshot["vioSupported"] is False
        assert snapshot["rangefinderTopology"] == "companion"
        assert snapshot["recommendedCameraId"] == "/dev/video0"
        assert snapshot["flowQuality"] == 180
        assert snapshot["flowRateHz"] == pytest.approx(29.5)
        assert snapshot["flowDistanceM"] == pytest.approx(1.25)
        assert snapshot["vioState"] == "absent"
        assert snapshot["vioResetCounter"] == 0
        assert snapshot["vioQuality"] is None
        assert snapshot["companionState"] == "active"

    @pytest.mark.asyncio
    async def test_health_publisher_emits_via_telemetry_extend(self) -> None:
        ctx = _FakeCtx()
        health = HealthPublisher(rangefinder_topology="fc")
        health.start(ctx)
        try:
            # First tick fires immediately; give the loop one yield.
            for _ in range(20):
                if ctx.telemetry.extends:
                    break
                await asyncio.sleep(0.01)
        finally:
            await health.stop()

        assert len(ctx.telemetry.extends) >= 1
        topic, payload = ctx.telemetry.extends[0]
        assert topic == "navigation"
        assert payload["opticalFlowSupported"] is True
        assert payload["rangefinderTopology"] == "fc"

    def test_health_publisher_distance_m_can_be_none(self) -> None:
        health = HealthPublisher()
        # Default snapshot before any pipeline update: distance is None.
        snapshot = health.snapshot()
        assert snapshot["flowDistanceM"] is None
        assert snapshot["companionState"] == "inactive"
