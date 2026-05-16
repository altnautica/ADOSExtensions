"""Tests for the new estimator-agnostic pipeline + plugin live-wire.

Pins the runtime contract: the pipeline reads frames + IMU samples,
calls the active estimator, routes the output through the component
router, drives the companion-198 heartbeat off the estimator state,
and publishes a pre-arm report on every tick.

In-flight mode swap (the operator's mode-card click flow) is also
covered: swap_estimator changes the active estimator without
restarting the pipeline loop.
"""

from __future__ import annotations

import asyncio
import time
from typing import Optional

import pytest

from altnautica_vision_nav.estimators.base import (
    BaseEstimator,
    EstimatorOutput,
    EstimatorState,
)
from altnautica_vision_nav.estimators.null_estimator import NullEstimator
from altnautica_vision_nav.imu.base import BaseImuSource
from altnautica_vision_nav.imu.time_aligner import TimeAligner
from altnautica_vision_nav.mavlink.comp_status import (
    CompanionHeartbeat,
    CompanionState,
)
from altnautica_vision_nav.mavlink.component_router import (
    MavlinkComponentRouter,
)
from altnautica_vision_nav.pipelines.estimator_pipeline import (
    EstimatorPipeline,
)
from altnautica_vision_nav.pre_arm_gate import PreArmGate


# ----------------------------------------------------------------------
# Test doubles
# ----------------------------------------------------------------------


class _StubEstimator(BaseEstimator):
    """Programmable estimator returning a queued output."""

    estimator_id = "stub"
    output_mode = "optical_flow"

    def __init__(self, *, output: Optional[EstimatorOutput] = None) -> None:
        self._output = output
        self.shutdown_called = False
        self.step_count = 0

    def step(self, **_kwargs):
        self.step_count += 1
        return self._output

    def shutdown(self) -> None:
        self.shutdown_called = True


class _StubImu(BaseImuSource):
    source_id = "stub-imu"

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


class _StubCtx:
    """Minimal ctx satisfying the bits the pipeline reads."""

    class _Mavlink:
        async def send(self, *_args, **_kwargs):
            return None

    def __init__(self) -> None:
        self.mavlink = self._Mavlink()
        self.event = None
        self.log = None


class _StubHealth:
    """Records the calls the pipeline makes onto the health publisher."""

    def __init__(self) -> None:
        self.pre_arm_reports: list = []
        self.companion_states: list = []
        self.flow_updates: list = []
        # Mimic the real HealthPublisher's intrinsics flag so the
        # pipeline's pre-arm input builder reads it correctly.
        self._intrinsics_loaded = False

    def update_companion_state(self, state) -> None:
        self.companion_states.append(state)

    def update_from_pipeline(self, **kwargs) -> None:
        self.flow_updates.append(kwargs)

    def set_pre_arm_report(self, report) -> None:
        self.pre_arm_reports.append(report)


class _StubConfig:
    """Minimal config the pre-arm input builder needs."""

    class _RF:
        topology = "companion"

    def __init__(self, *, mode: str = "optical_flow") -> None:
        self.mode = mode
        self.rangefinder = self._RF()


def _of_output(state: EstimatorState = "converged", quality: int = 180):
    return EstimatorOutput(
        timestamp_us=1_000_000,
        output_mode="optical_flow",
        state=state,
        flow_rate_x=0.1,
        flow_rate_y=-0.05,
        flow_rate_z=0.0,
        flow_quality=quality,
        flow_distance_m=1.5,
        flow_scale_source="rangefinder",
        integration_time_us=33_333,
    )


# ----------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------


@pytest.mark.asyncio
async def test_pipeline_construction_smoke() -> None:
    """The pipeline constructs cleanly with all required parts."""

    ctx = _StubCtx()
    imu = _StubImu()
    aligner = TimeAligner(imu)
    estimator = NullEstimator()
    router = MavlinkComponentRouter(ctx=ctx)
    gate = PreArmGate()
    health = _StubHealth()
    heartbeat = CompanionHeartbeat(component_id=198)
    config = _StubConfig(mode="off")

    pipeline = EstimatorPipeline(
        ctx=ctx,
        capture_source=_StubCapture(),
        imu_source=imu,
        time_aligner=aligner,
        rangefinder=None,
        scale_source=None,
        estimator=estimator,
        component_router=router,
        pre_arm_gate=gate,
        health_publisher=health,
        companion_heartbeat=heartbeat,
        clock_align=None,
        config=config,
    )
    assert pipeline.estimator is estimator


@pytest.mark.asyncio
async def test_pipeline_swap_estimator() -> None:
    """``swap_estimator`` shuts down the old estimator + installs the new."""

    ctx = _StubCtx()
    imu = _StubImu()
    aligner = TimeAligner(imu)
    old = _StubEstimator()
    new = _StubEstimator()
    pipeline = EstimatorPipeline(
        ctx=ctx,
        capture_source=_StubCapture(),
        imu_source=imu,
        time_aligner=aligner,
        rangefinder=None,
        scale_source=None,
        estimator=old,
        component_router=MavlinkComponentRouter(ctx=ctx),
        pre_arm_gate=PreArmGate(),
        health_publisher=_StubHealth(),
        companion_heartbeat=CompanionHeartbeat(component_id=198),
        clock_align=None,
        config=_StubConfig(mode="optical_flow"),
    )
    await pipeline.swap_estimator(new)
    assert pipeline.estimator is new
    assert old.shutdown_called is True


@pytest.mark.asyncio
async def test_pipeline_emits_pre_arm_on_every_tick() -> None:
    """Each frame pair invokes the gate and pushes a report onto health."""

    ctx = _StubCtx()
    imu = _StubImu()
    aligner = TimeAligner(imu)
    health = _StubHealth()
    config = _StubConfig(mode="optical_flow")
    estimator = _StubEstimator(output=_of_output())
    pipeline = EstimatorPipeline(
        ctx=ctx,
        capture_source=_TwoFrameCapture(),
        imu_source=imu,
        time_aligner=aligner,
        rangefinder=None,
        scale_source=None,
        estimator=estimator,
        component_router=MavlinkComponentRouter(ctx=ctx),
        pre_arm_gate=PreArmGate(),
        health_publisher=health,
        companion_heartbeat=CompanionHeartbeat(component_id=198),
        clock_align=None,
        config=config,
    )
    await pipeline.start()
    # Wait briefly for the loop to consume the two-frame capture and
    # produce one step output (the first frame is the prev seed, the
    # second triggers a step call).
    await asyncio.sleep(0.05)
    await pipeline.stop()
    # At least one pre-arm report should have landed.
    assert len(health.pre_arm_reports) >= 1
    # The report carries the right mode.
    assert health.pre_arm_reports[0].mode == "optical_flow"


@pytest.mark.asyncio
async def test_pipeline_drives_companion_state_from_estimator() -> None:
    """A converged estimator flips the companion-198 heartbeat to ACTIVE."""

    ctx = _StubCtx()
    imu = _StubImu()
    aligner = TimeAligner(imu)
    health = _StubHealth()
    estimator = _StubEstimator(output=_of_output(state="converged"))
    heartbeat = CompanionHeartbeat(component_id=198)
    pipeline = EstimatorPipeline(
        ctx=ctx,
        capture_source=_TwoFrameCapture(),
        imu_source=imu,
        time_aligner=aligner,
        rangefinder=None,
        scale_source=None,
        estimator=estimator,
        component_router=MavlinkComponentRouter(ctx=ctx),
        pre_arm_gate=PreArmGate(),
        health_publisher=health,
        companion_heartbeat=heartbeat,
        clock_align=None,
        config=_StubConfig(mode="optical_flow"),
    )
    await pipeline.start()
    await asyncio.sleep(0.05)
    await pipeline.stop()
    # Heartbeat should have moved to ACTIVE because the estimator
    # converged on the very first emit.
    assert heartbeat.state is CompanionState.ACTIVE


# ----------------------------------------------------------------------
# Capture stubs
# ----------------------------------------------------------------------


class _StubCapture:
    """Capture that yields nothing; for construction smoke tests."""

    async def __aiter__(self):
        if False:
            yield None  # pragma: no cover

    def frames(self):
        return self.__aiter__()


class _TwoFrameCapture:
    """Yields two frames in quick succession then stops."""

    async def _gen(self):
        t0 = time.monotonic_ns()
        yield (t0, b"frame0")
        yield (t0 + 33_000_000, b"frame1")

    def frames(self):
        return self._gen()
