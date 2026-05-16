"""Estimator-agnostic pipeline orchestrator.

This pipeline replaces the legacy :class:`OFPipeline` on the live
runtime path. Where OFPipeline hard-wired the Lucas-Kanade tracker
plus the rangefinder + gyro tap, the new pipeline holds a generic
:class:`BaseEstimator` plus an :class:`BaseImuSource` plus a
:class:`TimeAligner` plus an optional :class:`BaseScaleSource` plus
a :class:`MavlinkComponentRouter`. Any of the six estimator modes
flows through the same loop.

The pipeline owns:

* the frame-pair loop (prev, curr, dt) driven by the capture source
* the time-aligned IMU lookup per frame
* the rangefinder read per frame when a rangefinder is wired
* the call into ``estimator.step()`` and the route of the output
  through ``MavlinkComponentRouter.emit()``
* the companion-198 heartbeat state machine driven by the estimator's
  reported state
* the per-tick :class:`PreArmGate` evaluation + publication onto the
  agent heartbeat
* the optional ``DISTANCE_SENSOR`` co-emission when the rangefinder
  topology is ``companion``

It does not own:

* the capture device (the plugin opens + closes that)
* the IMU subscription (the plugin starts + stops the source)
* the rangefinder driver lifecycle
* the MAVLink component registration (the plugin handles 198 + 197)

An in-flight :meth:`swap_estimator` lets the plugin change modes
without restarting the loop. The new estimator is constructed
externally and handed in; this class only swaps the reference and
shuts down the previous one.
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Optional, Protocol

from altnautica_vision_nav.estimators.base import BaseEstimator, EstimatorOutput
from altnautica_vision_nav.imu.base import BaseImuSource
from altnautica_vision_nav.imu.time_aligner import TimeAligner
from altnautica_vision_nav.mavlink import comp_status as _comp_status
from altnautica_vision_nav.mavlink import encoders as _encoders
from altnautica_vision_nav.mavlink.component_router import MavlinkComponentRouter
from altnautica_vision_nav.pre_arm_gate import (
    PreArmGate,
    PreArmInputs,
    PreArmReport,
)
from altnautica_vision_nav.rangefinder.base import (
    RangefinderDriver,
    RangeReading,
)
from altnautica_vision_nav.scale.base import BaseScaleSource
from altnautica_vision_nav.time_sync.clock_align import ClockAlign

log = logging.getLogger(__name__)


# Two seconds of consecutive degraded / failed states flip the
# companion-198 heartbeat into CRITICAL. One healthy sample flips it
# back to ACTIVE. Matches the OFPipeline policy.
DEGRADED_GRACE_NS = 2_000_000_000

# Default DISTANCE_SENSOR enum values used when the rangefinder
# driver does not advertise its own sensor metadata.
DEFAULT_SENSOR_TYPE_LASER = 0  # MAV_DISTANCE_SENSOR_LASER
DEFAULT_ORIENTATION_DOWN = 25  # MAV_SENSOR_ROTATION_PITCH_270


class _Capture(Protocol):
    """Minimal capture-source protocol matching OFPipeline's contract."""

    def frames(self) -> "object": ...


class EstimatorPipeline:
    """The new estimator-agnostic runtime pipeline.

    Parameters
    ----------
    ctx:
        Plugin context. Forwarded to the component router + the
        rangefinder + the heartbeat for MAVLink emissions.
    capture_source:
        Camera frame producer (V4L2 or libcamera).
    imu_source:
        Live IMU subscriber. Mutated by `start()`; the pipeline does
        not own its lifecycle but reads `recent()` per frame.
    time_aligner:
        Frame-IMU pairing + drift watcher. The pipeline calls
        :meth:`TimeAligner.lookup` per frame.
    rangefinder:
        Distance source. ``None`` when the mode does not need one.
    scale_source:
        Altitude ladder used by the rangefinder-free OF mode. The
        pipeline reads `pick()` per frame when the active estimator
        consumes one. ``None`` when not applicable.
    estimator:
        The active estimator (instance of :class:`BaseEstimator`).
        Owned by the pipeline; on :meth:`stop` the pipeline calls
        :meth:`BaseEstimator.shutdown` so subprocess-backed engines
        teardown cleanly.
    component_router:
        Routes each :class:`EstimatorOutput` to the right MAVLink
        component (198 for OF, 197 for VIO).
    pre_arm_gate:
        Evaluated on every health tick; result lands on the heartbeat.
    health_publisher:
        Receives flow stats + estimator state + pre-arm report.
    companion_heartbeat:
        The pipeline transitions this through INACTIVE / ACTIVE /
        CRITICAL per the estimator state.
    clock_align:
        FC-clock alignment used by the component router and the
        DISTANCE_SENSOR co-emission.
    config:
        Active per-drone config. Used by the pre-arm gate to know
        which mode is active and what topology the rangefinder has.
    component_id:
        OF MAVLink component id (default 198, peripheral).
    sensor_id:
        Sensor id reported on emitted messages.
    """

    def __init__(
        self,
        *,
        ctx: object,
        capture_source: _Capture,
        imu_source: BaseImuSource,
        time_aligner: TimeAligner,
        rangefinder: Optional[RangefinderDriver],
        scale_source: Optional[BaseScaleSource],
        estimator: BaseEstimator,
        component_router: MavlinkComponentRouter,
        pre_arm_gate: PreArmGate,
        health_publisher: object,
        companion_heartbeat: _comp_status.CompanionHeartbeat,
        clock_align: Optional[ClockAlign],
        config: object,
        component_id: int = 198,
        sensor_id: int = 0,
    ) -> None:
        self._ctx = ctx
        self._capture = capture_source
        self._imu = imu_source
        self._aligner = time_aligner
        self._rangefinder = rangefinder
        self._scale = scale_source
        self._estimator = estimator
        self._router = component_router
        self._gate = pre_arm_gate
        self._health = health_publisher
        self._heartbeat = companion_heartbeat
        self._clock_align = clock_align
        self._config = config
        self._component_id = int(component_id)
        self._sensor_id = int(sensor_id)

        self._task: Optional[asyncio.Task[None]] = None
        self._stop_event = asyncio.Event()
        # Degraded-state streak window for the CRITICAL transition.
        self._degraded_streak_start_ns: Optional[int] = None

    @property
    def estimator(self) -> BaseEstimator:
        return self._estimator

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    async def start(self) -> None:
        if self._task is not None and not self._task.done():
            return
        self._stop_event.clear()
        self._task = asyncio.create_task(self.run())

    async def stop(self) -> None:
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
        # Shut down the estimator after the loop has drained so an
        # in-flight subprocess gets a clean shutdown signal.
        try:
            self._estimator.shutdown()
        except Exception:  # noqa: BLE001
            pass

    async def swap_estimator(
        self,
        new_estimator: BaseEstimator,
        new_scale_source: Optional[BaseScaleSource] = None,
    ) -> None:
        """In-flight mode change. Shut down the old estimator + scale; install the new.

        The pipeline loop reads ``self._estimator`` each frame so the
        swap takes effect on the next tick. The previous estimator's
        ``shutdown()`` runs synchronously; subprocess-backed engines
        terminate their vendor binary here.
        """

        old_estimator = self._estimator
        old_scale = self._scale
        self._estimator = new_estimator
        self._scale = new_scale_source
        self._degraded_streak_start_ns = None
        try:
            old_estimator.shutdown()
        except Exception:  # noqa: BLE001
            pass
        if old_scale is not None and old_scale is not new_scale_source:
            try:
                old_scale.stop()
            except Exception:  # noqa: BLE001
                pass
        if new_scale_source is not None and new_scale_source is not old_scale:
            try:
                new_scale_source.start()
            except Exception:  # noqa: BLE001
                pass

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    async def run(self) -> None:
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

                aligned = self._aligner.lookup(ts_ns)
                reading = await self._maybe_read_rangefinder()
                distance_m = reading.distance_m if reading else None

                output = self._estimator.step(
                    prev_frame=prev_frame,
                    curr_frame=frame,
                    dt_seconds=dt,
                    imu_batch=aligned,
                    range_reading=reading,
                )

                if output is not None:
                    await self._handle_output(
                        output=output,
                        reading=reading,
                        ts_ns=ts_ns,
                        dt=dt,
                    )
                else:
                    # Estimator produced no sample this tick (warm-up,
                    # off mode, or insufficient features). Still publish
                    # the pre-arm report so the GCS sees an honest
                    # state.
                    self._publish_pre_arm(distance_m=distance_m)

                prev_frame = frame
                prev_ts_ns = ts_ns
        except asyncio.CancelledError:
            return
        except Exception as exc:  # noqa: BLE001
            log.warning("Estimator pipeline loop terminated: %s", exc)

    # ------------------------------------------------------------------
    # Per-frame handling
    # ------------------------------------------------------------------

    async def _handle_output(
        self,
        *,
        output: EstimatorOutput,
        reading: Optional[RangeReading],
        ts_ns: int,
        dt: float,
    ) -> None:
        # Route the MAVLink emission first so the FC sees the sample
        # with minimum delay; the health + pre-arm updates run after.
        try:
            await self._router.emit(output)
        except Exception as exc:  # noqa: BLE001
            log.warning("Component router emit failed: %s", exc)

        # Co-emit DISTANCE_SENSOR on comp 198 when the rangefinder is
        # companion-owned. PX4 needs it as a separate input; ArduPilot
        # also benefits from the explicit terrain altitude.
        if reading is not None and self._rangefinder is not None:
            try:
                await self._emit_distance_sensor(reading=reading)
            except Exception as exc:  # noqa: BLE001
                log.warning("DISTANCE_SENSOR emit failed: %s", exc)

        # Drive the companion-198 heartbeat from the estimator state.
        self._update_companion_state(state=output.state, ts_ns=ts_ns)

        # Refresh OF flow stats on the health publisher (no-op for
        # VIO outputs since those fields are None).
        self._update_flow_health(output=output, dt=dt)

        # Pre-arm gate runs on every emission so the GCS card sees
        # the latest assessment.
        self._publish_pre_arm(
            distance_m=reading.distance_m if reading else None,
            output=output,
        )

    async def _maybe_read_rangefinder(self) -> Optional[RangeReading]:
        if self._rangefinder is None:
            return None
        try:
            return await self._rangefinder.read()
        except Exception as exc:  # noqa: BLE001
            log.warning("Rangefinder read failed: %s", exc)
            return None

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

    # ------------------------------------------------------------------
    # Companion state machine
    # ------------------------------------------------------------------

    def _update_companion_state(self, *, state: str, ts_ns: int) -> None:
        """Map estimator state to the comp-198 heartbeat value.

        ``converged`` flips the companion to ACTIVE immediately.
        ``degraded`` / ``failed`` start a 2 s streak window; if it
        persists the companion moves to CRITICAL. ``off`` /
        ``init`` / ``converging`` keep the heartbeat where it was
        (no transition; the GCS shows the estimator state separately).
        """

        if state == "converged":
            self._degraded_streak_start_ns = None
            if self._heartbeat.state is not _comp_status.CompanionState.ACTIVE:
                self._heartbeat.transition(_comp_status.CompanionState.ACTIVE)
                self._notify_companion_state(_comp_status.CompanionState.ACTIVE)
            return
        if state in ("degraded", "failed"):
            if self._degraded_streak_start_ns is None:
                self._degraded_streak_start_ns = ts_ns
            elif (
                ts_ns - self._degraded_streak_start_ns >= DEGRADED_GRACE_NS
                and self._heartbeat.state
                is not _comp_status.CompanionState.CRITICAL
            ):
                self._heartbeat.transition(
                    _comp_status.CompanionState.CRITICAL
                )
                self._notify_companion_state(
                    _comp_status.CompanionState.CRITICAL
                )
            return
        # init / converging / off: leave the heartbeat where it was.
        self._degraded_streak_start_ns = None

    def _notify_companion_state(
        self, state: _comp_status.CompanionState
    ) -> None:
        update = getattr(self._health, "update_companion_state", None)
        if callable(update):
            try:
                update(state)
            except Exception as exc:  # noqa: BLE001
                log.warning(
                    "health update_companion_state raised: %s", exc
                )

    # ------------------------------------------------------------------
    # Flow stats + pre-arm
    # ------------------------------------------------------------------

    def _update_flow_health(
        self,
        *,
        output: EstimatorOutput,
        dt: float,
    ) -> None:
        # update_from_pipeline expects a flow-shaped OpticalFlowResult.
        # For VIO outputs we synthesise the right surface so the GCS
        # flow-quality field shows the VIO quality (already wired in
        # the heartbeat from earlier work).
        if output.flow_quality is None:
            return
        update = getattr(self._health, "update_from_pipeline", None)
        if not callable(update):
            return

        # Build a thin object matching the OpticalFlowResult shape the
        # health publisher reads. We don't import OpticalFlowResult to
        # keep the pipeline decoupled from the legacy module.
        class _Result:
            quality = int(output.flow_quality or 0)
            integration_time_us = int(output.integration_time_us or 0)
            flow_rate_x = float(output.flow_rate_x or 0.0)
            flow_rate_y = float(output.flow_rate_y or 0.0)
            flow_rate_z = float(output.flow_rate_z or 0.0)
            flow_x_dpi = 0.0
            flow_y_dpi = 0.0
            flow_comp_m_x = 0.0
            flow_comp_m_y = 0.0

        flow_rate_hz = 1.0 / dt if dt > 0 else 0.0
        try:
            update(
                result=_Result(),
                flow_rate_hz=flow_rate_hz,
                distance_m=output.flow_distance_m,
            )
        except Exception as exc:  # noqa: BLE001
            log.warning("health update_from_pipeline raised: %s", exc)

    def _publish_pre_arm(
        self,
        *,
        distance_m: Optional[float] = None,
        output: Optional[EstimatorOutput] = None,
    ) -> None:
        inputs = self._build_pre_arm_inputs(
            distance_m=distance_m, output=output
        )
        report: PreArmReport = self._gate.evaluate(inputs)
        set_report = getattr(self._health, "set_pre_arm_report", None)
        if callable(set_report):
            try:
                set_report(report)
            except Exception as exc:  # noqa: BLE001
                log.warning("health set_pre_arm_report raised: %s", exc)

    def _build_pre_arm_inputs(
        self,
        *,
        distance_m: Optional[float],
        output: Optional[EstimatorOutput],
    ) -> PreArmInputs:
        mode = getattr(self._config, "mode", "off")
        companion_state = self._heartbeat.state.value
        estimator_state = output.state if output is not None else "off"
        flow_quality = output.flow_quality if output is not None else None
        flow_scale_source = (
            output.flow_scale_source if output is not None else None
        )
        rangefinder_topology = self._config.rangefinder.topology if hasattr(
            self._config, "rangefinder"
        ) else None
        if rangefinder_topology == "none":
            rangefinder_topology = None
        sync_offset_ms = self._aligner.mean_residual_ms()
        feature_count = output.feature_count if output is not None else None
        intrinsics_loaded = self._is_intrinsics_loaded()
        extrinsics_loaded = self._is_extrinsics_loaded()
        return PreArmInputs(
            mode=mode,
            companion_state=companion_state,
            estimator_state=estimator_state,
            flow_quality=flow_quality,
            flow_scale_source=flow_scale_source,
            rangefinder_topology=rangefinder_topology,
            intrinsics_loaded=intrinsics_loaded,
            extrinsics_loaded=extrinsics_loaded,
            sync_offset_ms=sync_offset_ms,
            feature_count=feature_count,
        )

    def _is_intrinsics_loaded(self) -> bool:
        # Read through the health publisher's internal flag the
        # calibration upload hook flips. Fall back to False so the
        # VIO pre-arm gate refuses until the upload happens.
        loaded = getattr(self._health, "_intrinsics_loaded", False)
        return bool(loaded)

    def _is_extrinsics_loaded(self) -> bool:
        # The current health publisher only carries a single
        # ``intrinsics_loaded`` flag because the calibration is uploaded
        # as one file. Treat the two as locked together: when the file
        # is uploaded, both intrinsics and extrinsics are present.
        return self._is_intrinsics_loaded()


def _now_ns() -> int:
    return time.monotonic_ns()
