"""Optical-flow pipeline orchestrator.

Stitches the four pieces of the vision-nav plugin together:

* a capture source (V4L2 or libcamera) yielding ``(ts_ns, frame)``;
* the Lucas-Kanade processor that turns a frame pair into a flow sample;
* the rangefinder driver that supplies height-above-ground;
* the MAVLink emit helpers that send ``OPTICAL_FLOW_RAD`` and
  ``DISTANCE_SENSOR`` to the flight controller.

The pipeline also drives the companion-198 ``CompanionState`` machine
and feeds the heartbeat extras publisher so the GCS card shows live
flow quality and rate.
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Optional, Protocol

from altnautica_vision_nav.capture.gyro_tap import GyroTap
from altnautica_vision_nav.mavlink import comp_status as _comp_status
from altnautica_vision_nav.mavlink import encoders as _encoders
from altnautica_vision_nav.processors.optical_flow_lk import (
    OpticalFlowLk,
    OpticalFlowResult,
)
from altnautica_vision_nav.rangefinder.base import (
    RangefinderDriver,
    RangeReading,
)
from altnautica_vision_nav.time_sync.clock_align import ClockAlign

log = logging.getLogger(__name__)


# Tunables. Two seconds of consecutive low-quality frames flips the
# companion into CRITICAL; one good frame flips it back to ACTIVE.
LOW_QUALITY_GRACE_NS = 2_000_000_000

# Default DISTANCE_SENSOR enum values used when the rangefinder driver
# does not advertise its own sensor metadata.
DEFAULT_SENSOR_TYPE_LASER = 0  # MAV_DISTANCE_SENSOR_LASER
DEFAULT_ORIENTATION_DOWN = 25  # MAV_SENSOR_ROTATION_PITCH_270 (downward)


class _Capture(Protocol):
    """Minimal capture-source protocol.

    Yields ``(monotonic_ns, frame)`` tuples. ``frame`` is either a BGR
    or single-channel ``ndarray``; the processor handles both shapes.
    """

    def frames(self) -> "object": ...


class OFPipeline:
    """Run the OF loop until told to stop.

    Parameters
    ----------
    ctx:
        Plugin context. The mavlink emit helpers and the heartbeat
        publisher both call methods on it.
    capture_source:
        Object exposing an async iterator ``frames()`` that yields
        ``(monotonic_ns, frame)`` tuples.
    of_processor:
        The Lucas-Kanade estimator instance.
    gyro_tap:
        Latest-reading gyro tap, or ``None`` to disable derotation.
    rangefinder:
        Distance source. ``None`` when ``rangefinder.topology = none``.
    clock_align:
        FC-clock alignment estimator. Optional; falls back to raw
        monotonic timestamps when absent.
    companion_heartbeat:
        Drives the comp 198 ``system_status`` field. The pipeline
        moves it through INACTIVE / ACTIVE / CRITICAL.
    health_publisher:
        Receives per-emit flow stats so the GCS card reads live data.
        Optional.
    flow_quality_min:
        Quality threshold below which a frame is treated as low-quality
        for the purposes of the CRITICAL transition.
    """

    def __init__(
        self,
        *,
        ctx: object,
        capture_source: _Capture,
        of_processor: OpticalFlowLk,
        gyro_tap: Optional[GyroTap],
        rangefinder: Optional[RangefinderDriver],
        clock_align: Optional[ClockAlign],
        companion_heartbeat: _comp_status.CompanionHeartbeat,
        health_publisher: Optional[object] = None,
        flow_quality_min: int = 50,
        component_id: int = 198,
        sensor_id: int = 0,
    ) -> None:
        self._ctx = ctx
        self._capture = capture_source
        self._of = of_processor
        self._gyro_tap = gyro_tap
        self._rangefinder = rangefinder
        self._clock_align = clock_align
        self._heartbeat = companion_heartbeat
        self._health = health_publisher
        self._quality_min = int(flow_quality_min)
        self._component_id = int(component_id)
        self._sensor_id = int(sensor_id)

        self._task: Optional[asyncio.Task[None]] = None
        self._stop_event = asyncio.Event()

        # Streak window: timestamp of the first low-quality frame in the
        # current run, or ``None`` while frames are good. The CRITICAL
        # transition fires when the streak exceeds the grace window.
        self._low_quality_streak_start_ns: Optional[int] = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    async def start(self) -> None:
        """Launch the loop as a background task. Idempotent."""

        if self._task is not None and not self._task.done():
            return
        self._stop_event.clear()
        self._task = asyncio.create_task(self.run())

    async def stop(self) -> None:
        """Signal the loop to exit and wait for it (1 s grace)."""

        self._stop_event.set()
        task = self._task
        self._task = None
        if task is None:
            return
        try:
            await asyncio.wait_for(task, timeout=1.0)
        except (asyncio.TimeoutError, asyncio.CancelledError):
            task.cancel()
            try:
                await task
            except (asyncio.CancelledError, Exception):  # noqa: BLE001
                pass
        except Exception:  # noqa: BLE001
            pass

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    async def run(self) -> None:
        """Consume frames and emit MAVLink samples until stopped."""

        prev_frame: object | None = None
        prev_ts_ns: Optional[int] = None
        try:
            frame_iter = self._capture.frames()
            async for ts_ns, frame in frame_iter:
                if self._stop_event.is_set():
                    break
                if prev_frame is None or prev_ts_ns is None:
                    prev_frame = frame
                    prev_ts_ns = ts_ns
                    continue
                dt = max((ts_ns - prev_ts_ns) / 1e9, 1e-6)

                gyro = self._gyro_tap.latest() if self._gyro_tap else None
                reading = await self._maybe_read_rangefinder()
                distance_m = reading.distance_m if reading else None

                result = self._of.process(
                    prev_frame,
                    frame,
                    dt,
                    gyro=gyro,
                    distance_m=distance_m,
                )

                await self._handle_result(
                    result=result,
                    dt=dt,
                    reading=reading,
                    now_ns=ts_ns,
                )

                prev_frame = frame
                prev_ts_ns = ts_ns
        except asyncio.CancelledError:
            return
        except Exception as exc:  # noqa: BLE001
            log.warning("OF pipeline loop terminated: %s", exc)

    # ------------------------------------------------------------------
    # Per-frame handling
    # ------------------------------------------------------------------

    async def _handle_result(
        self,
        *,
        result: OpticalFlowResult,
        dt: float,
        reading: Optional[RangeReading],
        now_ns: int,
    ) -> None:
        """React to one OF sample: emit, update state, refresh health."""

        flow_rate_hz = 1.0 / dt if dt > 0 else 0.0
        distance_m = reading.distance_m if reading else None

        if int(result.quality) >= self._quality_min:
            # Healthy sample: emit and clear any pending CRITICAL streak.
            self._low_quality_streak_start_ns = None
            if self._heartbeat.state is not _comp_status.CompanionState.ACTIVE:
                self._heartbeat.transition(_comp_status.CompanionState.ACTIVE)
                self._notify_companion_state(_comp_status.CompanionState.ACTIVE)

            await self._emit_optical_flow_rad(result=result, reading=reading)

            if reading is not None:
                await self._emit_distance_sensor(reading=reading)

            self._update_health(result=result, flow_rate_hz=flow_rate_hz, distance_m=distance_m)
            return

        # Low-quality sample: do not emit MAVLink; watch the streak.
        if self._low_quality_streak_start_ns is None:
            self._low_quality_streak_start_ns = now_ns
        elif (
            now_ns - self._low_quality_streak_start_ns >= LOW_QUALITY_GRACE_NS
            and self._heartbeat.state is not _comp_status.CompanionState.CRITICAL
        ):
            self._heartbeat.transition(_comp_status.CompanionState.CRITICAL)
            self._notify_companion_state(_comp_status.CompanionState.CRITICAL)

        # Even on a poor frame, keep the GCS card current with the
        # quality value the operator can see deteriorating.
        self._update_health(result=result, flow_rate_hz=flow_rate_hz, distance_m=distance_m)

    async def _emit_optical_flow_rad(
        self,
        *,
        result: OpticalFlowResult,
        reading: Optional[RangeReading],
    ) -> None:
        distance = float(reading.distance_m) if reading else 0.0
        gyro = self._gyro_tap.latest() if self._gyro_tap else None
        x_gyro = gyro.xgyro if gyro else 0.0
        y_gyro = gyro.ygyro if gyro else 0.0
        z_gyro = gyro.zgyro if gyro else 0.0
        await _encoders.emit_optical_flow_rad(
            self._ctx,
            component_id=self._component_id,
            sensor_id=self._sensor_id,
            integration_time_us=int(result.integration_time_us),
            integrated_x=float(result.flow_rate_x),
            integrated_y=float(result.flow_rate_y),
            integrated_xgyro=float(x_gyro),
            integrated_ygyro=float(y_gyro),
            integrated_zgyro=float(z_gyro),
            temperature=0,
            quality=int(result.quality),
            time_delta_distance_us=0,
            distance=distance,
            clock_align=self._clock_align,
        )

    async def _emit_distance_sensor(self, *, reading: RangeReading) -> None:
        min_m = (
            self._rangefinder.min_range_m if self._rangefinder is not None else 0.0
        )
        max_m = (
            self._rangefinder.max_range_m if self._rangefinder is not None else 40.0
        )
        current_cm = max(0, min(0xFFFF, int(round(reading.distance_m * 100))))
        min_cm = max(0, min(0xFFFF, int(round(min_m * 100))))
        max_cm = max(0, min(0xFFFF, int(round(max_m * 100))))
        covariance = max(0, min(255, 100 - int(reading.quality)))
        await _encoders.emit_distance_sensor(
            self._ctx,
            component_id=self._component_id,
            min_distance_cm=min_cm,
            max_distance_cm=max_cm,
            current_distance_cm=current_cm,
            sensor_type=DEFAULT_SENSOR_TYPE_LASER,
            sensor_id=self._sensor_id,
            orientation=DEFAULT_ORIENTATION_DOWN,
            covariance=covariance,
            clock_align=self._clock_align,
        )

    async def _maybe_read_rangefinder(self) -> Optional[RangeReading]:
        if self._rangefinder is None:
            return None
        try:
            return await self._rangefinder.read()
        except Exception as exc:  # noqa: BLE001
            log.warning("rangefinder read failed: %s", exc)
            return None

    def _notify_companion_state(self, state: _comp_status.CompanionState) -> None:
        if self._health is None:
            return
        update = getattr(self._health, "update_companion_state", None)
        if callable(update):
            try:
                update(state)
            except Exception as exc:  # noqa: BLE001
                log.warning("health update_companion_state raised: %s", exc)

    def _update_health(
        self,
        *,
        result: OpticalFlowResult,
        flow_rate_hz: float,
        distance_m: Optional[float],
    ) -> None:
        if self._health is None:
            return
        update = getattr(self._health, "update_from_pipeline", None)
        if callable(update):
            try:
                update(
                    result=result,
                    flow_rate_hz=flow_rate_hz,
                    distance_m=distance_m,
                )
            except Exception as exc:  # noqa: BLE001
                log.warning("health update_from_pipeline raised: %s", exc)


# Re-exported for callers that monkeypatch the clock in tests.
def _now_ns() -> int:
    return time.monotonic_ns()
