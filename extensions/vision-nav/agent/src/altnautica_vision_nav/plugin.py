"""Vision Navigation plugin entry point.

The agent supervisor imports :class:`VisionNavPlugin` via the
``ados.plugins`` entry-point declared in ``pyproject.toml`` and runs
its lifecycle hooks.

This module is the wiring layer. The interesting work happens in:

* :mod:`altnautica_vision_nav.pipelines.estimator_pipeline` — the
  runtime loop that turns frames + IMU into MAVLink emissions
* :mod:`altnautica_vision_nav.estimators` — the swappable estimator
  classes
* :mod:`altnautica_vision_nav.imu` — IMU sources + time alignment
* :mod:`altnautica_vision_nav.scale` — rangefinder-free altitude ladder
* :mod:`altnautica_vision_nav.shim` — VIO vendor binary IPC layer
* :mod:`altnautica_vision_nav.mavlink.component_router` — emission
  routing across components 198 (OF) and 197 (VIO)
* :mod:`altnautica_vision_nav.pre_arm_gate` — mode-aware arm-readiness

The plugin's job here is to construct all of the above in the right
order on ``on_start``, swap estimators on ``on_configure`` when the
mode changes, and route mode-change + calibration-upload events from
the GCS over the plugin event bus.

Two operator-facing event topics are owned by this plugin:

* ``com.altnautica.vision-nav.set_mode`` — payload ``{"mode": "<one
  of six>"}``. The GCS publishes when the operator picks a mode in
  the ModeCard.
* ``com.altnautica.vision-nav.upload_calibration`` — payload
  ``{"camchain_yaml": "<contents>"}``. The GCS publishes when the
  operator uploads a Kalibr-style camera-IMU calibration.

Both are subscribed in ``on_start`` and unsubscribed on ``on_stop``.
"""

from __future__ import annotations

import asyncio
import logging
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Optional

from altnautica_vision_nav import __version__ as PLUGIN_VERSION
from altnautica_vision_nav.calibration import (
    CameraImuExtrinsicsError,
    CameraIntrinsicsError,
    load_extrinsics,
    load_intrinsics,
)
from altnautica_vision_nav.config import VisionNavConfig, load_config
from altnautica_vision_nav.estimators import (
    ESTIMATOR_REGISTRY,
    BaseEstimator,
    HybridEstimator,
    NullEstimator,
    OpticalFlowEstimator,
    OpticalFlowNoRangeEstimator,
    VioOpenvinsEstimator,
    VioVinsFusionEstimator,
    available_estimators,
)
from altnautica_vision_nav.health import HealthPublisher
from altnautica_vision_nav.imu import MavlinkRawImu, TimeAligner
from altnautica_vision_nav.imu.base import BaseImuSource
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
from altnautica_vision_nav.pre_arm import PreArmHelper
from altnautica_vision_nav.pre_arm_gate import PreArmGate
from altnautica_vision_nav.processors.optical_flow_lk import OpticalFlowLk
from altnautica_vision_nav.rangefinder import (
    GarminLidarLiteI2c,
    RangefinderDriver,
    RelayDistanceSensor,
    TfLunaUart,
    Vl53l1xI2c,
)
from altnautica_vision_nav.scale import MavlinkScaleLadder
from altnautica_vision_nav.scale.base import BaseScaleSource
from altnautica_vision_nav.shim import EngineConfig
from altnautica_vision_nav.shim.engine import (
    OpenVinsEngine,
    VinsFusionEngine,
)
from altnautica_vision_nav.time_sync.clock_align import ClockAlign

log = logging.getLogger(__name__)


COMPONENT_OF_PERIPHERAL = 198
COMPONENT_VIO = 197
SENSOR_ID = 0

EVENT_SET_MODE = "com.altnautica.vision-nav.set_mode"
EVENT_UPLOAD_CALIBRATION = "com.altnautica.vision-nav.upload_calibration"
EVENT_START_CALIBRATION = "com.altnautica.vision-nav.start_calibration"
EVENT_CALIBRATION_PROGRESS = "com.altnautica.vision-nav.calibration_progress"
EVENT_CALIBRATION_COMPLETE = "com.altnautica.vision-nav.calibration_complete"
EVENT_RECALIBRATE_IMU_BIASES = "com.altnautica.vision-nav.recalibrate_imu_biases"


def _i2c_bus_number(device: str, default: int) -> int:
    if not device:
        return default
    if device.isdigit():
        return int(device)
    if "i2c-" in device:
        tail = device.rsplit("i2c-", 1)[-1]
        try:
            return int(tail)
        except ValueError:
            return default
    return default


@dataclass(frozen=True)
class _AutoDetectSummary:
    """Compact result of the start-up auto-detect pass.

    Wired into the health publisher so the heartbeat surfaces every
    field directly. The plugin keeps this internal-only; the GCS sees
    the data via the heartbeat shape rather than this dataclass.
    """

    camera_count: int
    picked_camera_path: Optional[str]
    rangefinder_driver: Optional[str]
    suggested_mode: str
    suggested_reason: str


class VisionNavPlugin:
    """Plugin entry point loaded by the agent supervisor."""

    plugin_id = "com.altnautica.vision-nav"
    version = PLUGIN_VERSION

    def __init__(self) -> None:
        self._config: Optional[VisionNavConfig] = None
        self._capture: object | None = None
        self._imu_source: Optional[BaseImuSource] = None
        self._time_aligner: Optional[TimeAligner] = None
        self._scale_source: Optional[BaseScaleSource] = None
        self._rangefinder: Optional[RangefinderDriver] = None
        self._heartbeat: Optional[CompanionHeartbeat] = None
        self._clock_align: Optional[ClockAlign] = None
        self._pipeline: Optional[EstimatorPipeline] = None
        self._health: Optional[HealthPublisher] = None
        self._pre_arm: Optional[PreArmHelper] = None
        self._gate: Optional[PreArmGate] = None
        self._router: Optional[MavlinkComponentRouter] = None
        self._component_of_registered = False
        self._component_vio_registered = False
        self._event_unsubscribes: list[Callable[[], None]] = []
        self._ctx: object | None = None

    # ------------------------------------------------------------------
    # Lifecycle hooks
    # ------------------------------------------------------------------

    async def on_install(self, ctx: object) -> None:
        self._log_info(ctx, "vision_nav_installed", version=PLUGIN_VERSION)

    async def on_enable(self, ctx: object) -> None:
        # Refuse to enable on ground-station-profile hosts. The plugin
        # owns a video capture device + an FC MAVLink channel; both
        # are exclusive resources a ground station does not have.
        # Raising here causes the plugin host runner to log a
        # plugin_runtime_error and exit before on_start runs, which is
        # the behaviour we want: a misplaced install refuses to take
        # over the host's hardware.
        from altnautica_vision_nav.autodetect import (
            detect_host_profile,
            is_drone_profile,
        )

        profile = detect_host_profile()
        if not is_drone_profile(profile):
            self._log_warning(
                ctx,
                "vision_nav_refused_on_ground_station",
                profile=profile.profile,
                source=profile.source,
            )
            raise RuntimeError(
                "vision-nav is a drone-side plugin and cannot enable on a "
                f"{profile.profile} host"
            )
        self._log_info(ctx, "vision_nav_enabled", profile=profile.profile)

    async def on_configure(self, ctx: object, config: object) -> None:
        raw = config if isinstance(config, dict) else {}
        previous_mode = self._config.mode if self._config else None
        try:
            self._config = load_config(raw)
        except Exception as exc:  # noqa: BLE001
            self._log_warning(
                ctx, "vision_nav_config_invalid", error=str(exc)
            )
            if self._config is None:
                self._config = load_config({})
            return

        # If the pipeline is running and the mode changed, perform an
        # in-flight swap. The pipeline's swap_estimator handles the
        # shutdown of the old estimator and the start of any new
        # scale source.
        if (
            self._pipeline is not None
            and previous_mode is not None
            and previous_mode != self._config.mode
        ):
            await self._swap_to_mode(ctx, self._config.mode)

        if self._health is not None:
            self._health.set_mode(self._config.mode)

    async def on_start(self, ctx: object) -> None:
        if self._config is None:
            self._config = load_config({})
        self._ctx = ctx

        # Auto-detect hardware + merge into config. Runs before any
        # capture or peripheral opens so the picked camera + auto-flip
        # mode are the values the rest of the start-up sees.
        autodetect_summary = self._auto_detect_hardware(ctx)
        self._auto_flip_mode_for_hardware(ctx, autodetect_summary)

        # MAVLink components register first so subscribers see the
        # peripherals on the bus before the first emit.
        self._register_components(ctx)

        self._clock_align = ClockAlign(component_id=COMPONENT_OF_PERIPHERAL)
        await self._safe_clock_start(ctx)

        self._heartbeat = CompanionHeartbeat(
            component_id=COMPONENT_OF_PERIPHERAL
        )
        await self._heartbeat.start(ctx)

        # IMU source. The new full-IMU subscriber replaces the gyro-only
        # tap. The time aligner pairs each camera frame with the
        # closest IMU sample by hardware timestamp.
        self._imu_source = self._build_imu_source(ctx)
        self._time_aligner = TimeAligner(
            self._imu_source or _NoOpImuSource(),
            timeshift_cam_imu_s=0.0,
        )

        # Capture source unchanged.
        capture = self._open_capture(ctx, self._config)
        self._capture = capture

        # Rangefinder unchanged.
        rangefinder = await self._open_rangefinder(ctx, self._config)
        self._rangefinder = rangefinder

        # Scale source only when the mode needs it.
        self._scale_source = self._build_scale_source(ctx, self._config.mode)

        topology = self._topology_label(self._config)
        recommended_camera = self._config.camera.device_path or None

        # Health publisher with all the new attach hooks wired.
        self._health = HealthPublisher(
            rangefinder_topology=topology,
            recommended_camera_id=recommended_camera,
            mode=self._config.mode,
            available_estimators=available_estimators(),
            suggested_mode=autodetect_summary.suggested_mode,
            suggested_mode_reason=autodetect_summary.suggested_reason,
            detected_camera_count=autodetect_summary.camera_count,
            detected_rangefinder_driver=autodetect_summary.rangefinder_driver,
        )
        self._health.attach_imu_source(self._imu_source)
        self._health.attach_time_aligner(self._time_aligner)
        # Attempt to load a previously-saved calibration so VIO modes
        # can pre-arm without an upload on every boot.
        self._try_load_persisted_calibration(ctx)
        self._health.start(ctx)
        self._health.update_companion_state(CompanionState.INACTIVE)

        # Component router for routing OF emissions to comp 198 and
        # VIO emissions to comp 197.
        self._router = MavlinkComponentRouter(
            ctx=ctx,
            clock_align=self._clock_align,
            of_component_id=COMPONENT_OF_PERIPHERAL,
            vio_component_id=COMPONENT_VIO,
            sensor_id=SENSOR_ID,
        )

        # Pre-arm gate; thresholds default to the values pinned by
        # the prior session's research.
        self._gate = PreArmGate(
            flow_quality_gate=self._config.flow_quality_min,
        )

        # Active estimator from the registry.
        estimator = self._build_estimator(
            ctx,
            self._config.mode,
            scale_source=self._scale_source,
        )

        # The new estimator-agnostic pipeline.
        self._pipeline = EstimatorPipeline(
            ctx=ctx,
            capture_source=capture if capture is not None else _NoOpCapture(),
            imu_source=self._imu_source or _NoOpImuSource(),
            time_aligner=self._time_aligner,
            rangefinder=rangefinder,
            scale_source=self._scale_source,
            estimator=estimator,
            component_router=self._router,
            pre_arm_gate=self._gate,
            health_publisher=self._health,
            companion_heartbeat=self._heartbeat,
            clock_align=self._clock_align,
            config=self._config,
            component_id=COMPONENT_OF_PERIPHERAL,
            sensor_id=SENSOR_ID,
        )
        if capture is not None:
            await self._pipeline.start()
        else:
            self._log_warning(ctx, "vision_nav_capture_unavailable")

        # Subscribe to operator events from the GCS.
        await self._subscribe_to_events(ctx)

        # Pre-arm helper for the synthetic-origin push. ArduPilot and
        # PX4 expect SET_GPS_GLOBAL_ORIGIN + SET_HOME_POSITION when
        # flying vision-only. iNav drives optical-flow position-hold
        # without a global origin (position hold is purely flow-driven
        # in nav-poshold), so the synthetic origin push is a no-op and
        # we skip it to avoid noisy MAVLink chatter.
        self._pre_arm = PreArmHelper(component_id=COMPONENT_OF_PERIPHERAL)
        if (
            self._config.pre_arm.auto_set_origin
            and self._config.firmware.type != "inav"
        ):
            await self._pre_arm.auto_set_origin(
                ctx,
                self._config.pre_arm.origin_lat,
                self._config.pre_arm.origin_lon,
                self._config.pre_arm.origin_alt_m,
            )
        elif (
            self._config.pre_arm.auto_set_origin
            and self._config.firmware.type == "inav"
        ):
            self._log_info(
                ctx,
                "vision_nav_inav_origin_push_skipped",
                reason=(
                    "iNav position-hold does not require a global EKF "
                    "origin; flow drives nav-poshold directly."
                ),
            )

        self._log_info(ctx, "vision_nav_started", version=PLUGIN_VERSION)

    async def on_pause(self, ctx: object) -> None:
        if self._pipeline is not None:
            await self._pipeline.stop()
        if self._heartbeat is not None:
            self._heartbeat.transition(CompanionState.INACTIVE)
        if self._health is not None:
            self._health.update_companion_state(CompanionState.INACTIVE)
        self._log_info(ctx, "vision_nav_paused")

    async def on_resume(self, ctx: object) -> None:
        if self._pipeline is not None:
            await self._pipeline.start()
        self._log_info(ctx, "vision_nav_resumed")

    async def on_stop(self, ctx: object) -> None:
        for unsub in self._event_unsubscribes:
            try:
                unsub()
            except Exception:  # noqa: BLE001
                pass
        self._event_unsubscribes.clear()

        if self._pipeline is not None:
            await self._pipeline.stop()
            self._pipeline = None

        if self._scale_source is not None:
            try:
                self._scale_source.stop()
            except Exception:  # noqa: BLE001
                pass
            self._scale_source = None

        if self._capture is not None:
            stop = getattr(self._capture, "stop", None)
            if callable(stop):
                try:
                    stop()
                except Exception as exc:  # noqa: BLE001
                    self._log_warning(
                        ctx, "vision_nav_capture_stop_failed", error=str(exc)
                    )
            self._capture = None

        if self._rangefinder is not None:
            try:
                await self._rangefinder.close()
            except Exception as exc:  # noqa: BLE001
                self._log_warning(
                    ctx, "vision_nav_rangefinder_close_failed", error=str(exc)
                )
            self._rangefinder = None

        if self._imu_source is not None:
            try:
                self._imu_source.stop()
            except Exception as exc:  # noqa: BLE001
                self._log_warning(
                    ctx, "vision_nav_imu_stop_failed", error=str(exc)
                )
            self._imu_source = None

        if self._heartbeat is not None:
            self._heartbeat.transition(CompanionState.TERMINATING)
            await self._heartbeat.stop()
            self._heartbeat = None

        if self._clock_align is not None:
            try:
                await self._clock_align.stop()
            except Exception as exc:  # noqa: BLE001
                self._log_warning(
                    ctx, "vision_nav_clock_stop_failed", error=str(exc)
                )
            self._clock_align = None

        if self._health is not None:
            await self._health.stop()
            self._health = None

        self._unregister_components(ctx)
        self._ctx = None
        self._log_info(ctx, "vision_nav_stopped")

    async def on_disable(self, ctx: object) -> None:
        await self.on_stop(ctx)

    async def on_uninstall(self, ctx: object) -> None:
        self._log_info(ctx, "vision_nav_uninstalled")

    # ------------------------------------------------------------------
    # Estimator factory + mode swap
    # ------------------------------------------------------------------

    def _build_estimator(
        self,
        ctx: object,
        mode: str,
        *,
        scale_source: Optional[BaseScaleSource],
    ) -> BaseEstimator:
        """Construct the estimator class for ``mode``.

        The registry maps mode → class. The class constructors differ
        so this factory translates the mode + available inputs into
        the right call. VIO modes that fail to construct fall back to
        :class:`NullEstimator` with a logged warning.
        """

        # Belt-and-suspenders for the config-level iNav+VIO rejection.
        # A hand-edited config or a programmatic set-mode that bypasses
        # the validator would otherwise instantiate a VIO estimator the
        # firmware cannot consume.
        if self._config is not None and self._config.firmware.type == "inav":
            if mode in {"vio_openvins", "vio_vins_fusion", "hybrid_of_plus_vio"}:
                self._log_warning(
                    ctx,
                    "vision_nav_vio_not_supported_on_inav",
                    mode=mode,
                )
                return NullEstimator()

        cls = ESTIMATOR_REGISTRY.get(mode)
        if cls is None or cls is NullEstimator:
            return NullEstimator()
        if cls is OpticalFlowEstimator:
            return OpticalFlowEstimator(
                processor=OpticalFlowLk(),
                quality_gate=self._config.flow_quality_min
                if self._config
                else 50,
            )
        if cls is OpticalFlowNoRangeEstimator:
            return OpticalFlowNoRangeEstimator(
                processor=OpticalFlowLk(),
                scale_source=scale_source,
                quality_gate=self._config.flow_quality_min
                if self._config
                else 50,
            )
        if cls in (VioOpenvinsEstimator, VioVinsFusionEstimator):
            return self._build_vio_estimator(ctx, cls)
        if cls is HybridEstimator:
            return self._build_hybrid_estimator(ctx, scale_source)
        # Unknown registry entry: fail safe.
        self._log_warning(ctx, "vision_nav_unknown_mode", mode=mode)
        return NullEstimator()

    def _build_vio_estimator(
        self, ctx: object, cls: type
    ) -> BaseEstimator:
        """Best-effort VIO construction.

        Falls back to :class:`NullEstimator` (with a logged warning)
        when the vendor binary is missing, the calibration is not
        loaded, or any other prerequisite fails. The pre-arm gate
        independently refuses to arm a VIO mode without calibration,
        so this fallback is a defensive double-check, not the
        operator's safety net.
        """

        engine_cls = (
            OpenVinsEngine if cls is VioOpenvinsEstimator else VinsFusionEngine
        )
        if not self._vendor_binary_present(engine_cls.basename):
            self._log_warning(
                ctx,
                "vision_nav_vio_binary_missing",
                basename=engine_cls.basename,
            )
            return NullEstimator()
        try:
            engine = engine_cls(
                spawn_callable=self._make_spawn_callable(ctx),
                transport_factory=None,
            )
            engine_config = self._build_engine_config()
            return cls(engine=engine, engine_config=engine_config)
        except Exception as exc:  # noqa: BLE001
            self._log_warning(
                ctx, "vision_nav_vio_build_failed", error=str(exc)
            )
            return NullEstimator()

    def _build_hybrid_estimator(
        self,
        ctx: object,
        scale_source: Optional[BaseScaleSource],
    ) -> BaseEstimator:
        """Hybrid runs an OF child plus a VIO child.

        When the VIO half fails to construct (vendor binary missing /
        no calibration), the hybrid degrades to a wrapped OF
        estimator with the VIO half stubbed to :class:`NullEstimator`.
        """

        of_child = OpticalFlowEstimator(processor=OpticalFlowLk())
        vio_child = self._build_vio_estimator(ctx, VioOpenvinsEstimator)
        return HybridEstimator(of_estimator=of_child, vio_estimator=vio_child)

    def _build_engine_config(self) -> EngineConfig:
        """Build the EngineConfig the VIO shim passes to the vendor binary.

        Reads the persisted calibration when present. When no
        calibration is loaded the config carries defaults; the
        vendor binary will refuse to handshake without intrinsics,
        which surfaces as a binary-startup failure to the caller.
        """

        # Persisted calibration is loaded into the health publisher
        # earlier; we don't currently keep the raw values on the
        # plugin instance. For the engine config we re-read the YAML
        # file if it exists.
        cam_data = self._read_persisted_camchain()
        if cam_data is None:
            # Default placeholder; the vendor binary will reject it.
            return EngineConfig(
                camera_model="pinhole",
                fx=500.0,
                fy=500.0,
                cx=320.0,
                cy=240.0,
                width=640,
                height=480,
                distortion_model="none",
                distortion_coeffs=(),
                t_cam_imu=(
                    1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0,
                ),
                timeshift_cam_imu_s=0.0,
                imu_rate_hz=100.0,
                camera_rate_hz=30.0,
            )
        intrinsics, extrinsics = cam_data
        t_rows = extrinsics.T_cam_imu
        flat = tuple(float(v) for row in t_rows for v in row)
        return EngineConfig(
            camera_model=intrinsics.camera_model,
            fx=intrinsics.fx,
            fy=intrinsics.fy,
            cx=intrinsics.cx,
            cy=intrinsics.cy,
            width=intrinsics.width,
            height=intrinsics.height,
            distortion_model=intrinsics.distortion_model,
            distortion_coeffs=tuple(intrinsics.distortion_coeffs),
            t_cam_imu=flat,
            timeshift_cam_imu_s=extrinsics.timeshift_cam_imu,
            imu_rate_hz=100.0,
            camera_rate_hz=30.0,
        )

    async def _swap_to_mode(self, ctx: object, new_mode: str) -> None:
        """In-flight mode change. Build the new estimator + scale; swap."""

        if self._pipeline is None:
            return
        new_scale = self._build_scale_source(ctx, new_mode)
        new_estimator = self._build_estimator(
            ctx, new_mode, scale_source=new_scale
        )
        await self._pipeline.swap_estimator(
            new_estimator=new_estimator,
            new_scale_source=new_scale,
        )
        self._scale_source = new_scale
        if self._health is not None:
            self._health.set_mode(new_mode)
        self._log_info(ctx, "vision_nav_mode_swapped", mode=new_mode)

    def _build_scale_source(
        self, ctx: object, mode: str
    ) -> Optional[BaseScaleSource]:
        """The rangefinder-free OF mode is the only mode that needs one today."""

        if mode != "optical_flow_degraded":
            return None
        try:
            ladder = MavlinkScaleLadder(ctx, outdoor_flag=False)
            ladder.start()
            return ladder
        except Exception as exc:  # noqa: BLE001
            self._log_warning(
                ctx, "vision_nav_scale_ladder_failed", error=str(exc)
            )
            return None

    def _vendor_binary_present(self, basename: str) -> bool:
        """Check whether the named vendor binary is on disk.

        Looks under ``<plugin_install_dir>/vendor/<basename>``. The
        install dir is discovered via the env var the plugin host
        sets (``ADOS_PLUGIN_INSTALL_DIR``) or via the plugin context.
        """

        install_dir = os.environ.get("ADOS_PLUGIN_INSTALL_DIR")
        if install_dir is None:
            install_dir = self._install_dir_from_ctx()
        if install_dir is None:
            return False
        candidate = Path(install_dir) / "vendor" / basename
        return candidate.is_file() and os.access(candidate, os.X_OK)

    def _install_dir_from_ctx(self) -> Optional[str]:
        ctx = self._ctx
        if ctx is None:
            return None
        return getattr(ctx, "install_dir", None) or getattr(
            ctx, "plugin_install_dir", None
        )

    def _make_spawn_callable(self, ctx: object) -> Optional[Callable]:
        """Wrap ``ctx.process.spawn`` so the VIO engine can spawn its binary."""

        process = getattr(ctx, "process", None)
        if process is None:
            return None
        spawn = getattr(process, "spawn", None)
        if not callable(spawn):
            return None
        return spawn

    # ------------------------------------------------------------------
    # Hardware auto-detect
    # ------------------------------------------------------------------

    def _auto_detect_hardware(self, ctx: object) -> "_AutoDetectSummary":
        """Probe cameras, rangefinder, IMU; merge into the live config.

        Returns a small typed summary the heartbeat surfaces verbatim.
        Side effects: when the operator's config left
        ``camera.device_path`` at the default ``/dev/video0`` and a
        camera was actually found at a different node, the detected
        path wins. The rangefinder topology + driver get the same
        treatment when the operator's config is at the defaults.
        """

        from altnautica_vision_nav.autodetect import (
            derive_suggested_mode,
            detect_cameras,
            detect_rangefinder,
        )
        from altnautica_vision_nav.autodetect.camera import pick_camera_for_mode

        config = self._config
        assert config is not None  # callers set this in on_start

        cameras = detect_cameras()
        picked = pick_camera_for_mode(cameras, config.mode)
        rangefinder = detect_rangefinder()

        # Merge picked camera path into config when the operator left
        # it at the default. Operator-set paths always win.
        if picked is not None:
            cfg_default = config.camera.device_path in ("", "/dev/video0")
            if cfg_default and picked.device_path != config.camera.device_path:
                # Pydantic v2: rebuild the camera sub-model with the
                # picked device path.
                config.camera = config.camera.model_copy(
                    update={"device_path": picked.device_path}
                )
                self._log_info(
                    ctx,
                    "vision_nav_auto_picked_camera",
                    device=picked.device_path,
                    kind=picked.kind,
                )

        # Merge detected rangefinder when the operator left topology
        # at the default ("none").
        if rangefinder is not None and config.rangefinder.topology == "none":
            config.rangefinder = config.rangefinder.model_copy(
                update={
                    "topology": rangefinder.topology,
                    "driver": rangefinder.driver,
                }
            )
            self._log_info(
                ctx,
                "vision_nav_auto_picked_rangefinder",
                driver=rangefinder.driver,
                bus=rangefinder.bus,
            )

        suggestion = derive_suggested_mode(
            has_camera=picked is not None,
            has_rangefinder=rangefinder is not None,
            # Future hooks: forward-camera and NPU-board detection
            # roll in once the HAL board fingerprint becomes
            # available to the plugin via ctx.hal. For v1 the
            # suggestion only differentiates OF / OF-degraded / off.
            has_forward_camera=False,
            has_npu_board=False,
        )

        return _AutoDetectSummary(
            camera_count=len(cameras),
            picked_camera_path=picked.device_path if picked else None,
            rangefinder_driver=(
                rangefinder.driver if rangefinder is not None else None
            ),
            suggested_mode=suggestion.mode,
            suggested_reason=suggestion.reason,
        )

    def _auto_flip_mode_for_hardware(
        self, ctx: object, summary: "_AutoDetectSummary"
    ) -> None:
        """Flip the active mode when the hardware does not support it.

        The single auto-flip today: ``optical_flow`` -> ``optical_flow_degraded``
        when no rangefinder was detected. The plugin logs the flip so
        the operator sees it in the agent journal; the GCS surfaces it
        via the suggestedMode mismatch with the active mode.
        """

        config = self._config
        if config is None:
            return
        if (
            config.mode == "optical_flow"
            and summary.rangefinder_driver is None
        ):
            self._log_info(
                ctx,
                "vision_nav_auto_flip_to_degraded",
                reason="no rangefinder detected; falling back to OF degraded",
            )
            config.mode = "optical_flow_degraded"

    # ------------------------------------------------------------------
    # IMU
    # ------------------------------------------------------------------

    def _build_imu_source(self, ctx: object) -> Optional[BaseImuSource]:
        # Pick the highest-rate IMU source the host exposes. The
        # selector probes I2C buses for a BMI088 chip-id and falls
        # back to MAVLink RAW_IMU when no direct bus answers.
        from altnautica_vision_nav.autodetect.imu import (
            detect_preferred_imu_source,
        )
        from altnautica_vision_nav.imu.direct_i2c import DirectI2cImu

        preferred = detect_preferred_imu_source()

        if preferred.source_id == "direct-i2c-bmi088" and preferred.bus_num is not None:
            try:
                source = DirectI2cImu(preferred.bus_num)
                source.start()
                self._log_info(
                    ctx,
                    "vision_nav_imu_direct_i2c_started",
                    bus=preferred.bus_num,
                )
                return source
            except Exception as exc:  # noqa: BLE001
                # Direct-I2C failed (chip-id mismatch, permission, etc).
                # Fall through to the MAVLink path so the plugin keeps
                # working even when the bench IMU is misbehaving.
                self._log_warning(
                    ctx,
                    "vision_nav_imu_direct_i2c_failed",
                    error=str(exc),
                )

        mavlink = getattr(ctx, "mavlink", None)
        if mavlink is None or not callable(
            getattr(mavlink, "subscribe", None)
        ):
            return None
        source = MavlinkRawImu(ctx)
        try:
            source.start()
            return source
        except Exception as exc:  # noqa: BLE001
            self._log_warning(
                ctx, "vision_nav_imu_subscribe_failed", error=str(exc)
            )
            return None

    # ------------------------------------------------------------------
    # Events: set_mode + upload_calibration
    # ------------------------------------------------------------------

    async def _subscribe_to_events(self, ctx: object) -> None:
        event_bus = getattr(ctx, "event", None) or getattr(ctx, "events", None)
        if event_bus is None:
            return
        subscribe = getattr(event_bus, "subscribe", None)
        if not callable(subscribe):
            return

        async def _on_set_mode(payload: Any) -> None:
            await self._handle_set_mode_event(ctx, payload)

        async def _on_upload(payload: Any) -> None:
            await self._handle_upload_calibration_event(ctx, payload)

        async def _on_start_cal(payload: Any) -> None:
            await self._handle_start_calibration_event(ctx, payload)

        async def _on_recal_imu(_payload: Any) -> None:
            self._trigger_imu_bias_recalibration(ctx)

        try:
            unsub_a = subscribe(EVENT_SET_MODE, _on_set_mode)
            unsub_b = subscribe(EVENT_UPLOAD_CALIBRATION, _on_upload)
            unsub_c = subscribe(EVENT_START_CALIBRATION, _on_start_cal)
            unsub_d = subscribe(EVENT_RECALIBRATE_IMU_BIASES, _on_recal_imu)
            for unsub in (unsub_a, unsub_b, unsub_c, unsub_d):
                if asyncio.iscoroutine(unsub):
                    unsub = await unsub
                if callable(unsub):
                    self._event_unsubscribes.append(unsub)
        except Exception as exc:  # noqa: BLE001
            self._log_warning(
                ctx, "vision_nav_event_subscribe_failed", error=str(exc)
            )

    async def _handle_set_mode_event(
        self, ctx: object, payload: Any
    ) -> None:
        mode = self._extract_str(payload, "mode")
        if mode is None or mode not in ESTIMATOR_REGISTRY:
            self._log_warning(
                ctx, "vision_nav_set_mode_invalid", mode=str(mode)
            )
            return
        if self._config is None:
            self._config = load_config({})
        merged = self._config.model_dump()
        merged["mode"] = mode
        try:
            self._config = load_config(merged)
        except Exception as exc:  # noqa: BLE001
            self._log_warning(
                ctx, "vision_nav_set_mode_apply_failed", error=str(exc)
            )
            return
        await self._swap_to_mode(ctx, mode)

    async def _handle_upload_calibration_event(
        self, ctx: object, payload: Any
    ) -> None:
        text = self._extract_str(payload, "camchain_yaml")
        if text is None:
            self._log_warning(ctx, "vision_nav_calibration_missing_yaml")
            return
        cam_dir = self._calibration_dir(ctx)
        try:
            cam_dir.mkdir(parents=True, exist_ok=True)
            path = cam_dir / "camchain.yaml"
            path.write_text(text, encoding="utf-8")
        except OSError as exc:
            self._log_warning(
                ctx, "vision_nav_calibration_persist_failed", error=str(exc)
            )
            return
        # Validate and apply.
        try:
            intrinsics = load_intrinsics(path)
            extrinsics = load_extrinsics(path)
        except (CameraIntrinsicsError, CameraImuExtrinsicsError) as exc:
            self._log_warning(
                ctx, "vision_nav_calibration_invalid", error=str(exc)
            )
            return
        if self._health is not None:
            self._health.set_intrinsics_loaded(True)
        if self._time_aligner is not None:
            self._time_aligner.set_timeshift(extrinsics.timeshift_cam_imu)
        self._log_info(
            ctx,
            "vision_nav_calibration_loaded",
            fx=intrinsics.fx,
            fy=intrinsics.fy,
            timeshift=extrinsics.timeshift_cam_imu,
        )

    async def _handle_start_calibration_event(
        self, ctx: object, payload: Any
    ) -> None:
        """Bridge the GCS ``start_calibration`` event into the runner.

        Constructs a :class:`RunnerHooks` adapter that publishes
        progress + completion events back to the same plugin event
        bus, fetches the IMU samples from the live :class:`MavlinkRawImu`
        buffer for the captured window, and persists the resulting
        camchain.yaml. The runner then walks substeps (tag detection,
        intrinsics solve, timeshift fit) at its own pace; substep
        updates surface on the wizard's progress bar live.
        """

        from altnautica_vision_nav.calibration.runner import (
            CalibrationProgress,
            CalibrationResult,
            RunnerHooks,
            run_calibration,
        )
        from altnautica_vision_nav.calibration.runner import (
            _ImuSample as _RunnerImuSample,
        )

        if not isinstance(payload, dict):
            self._log_warning(ctx, "vision_nav_start_cal_invalid_payload")
            return

        event_bus = getattr(ctx, "event", None) or getattr(ctx, "events", None)
        if event_bus is None or not callable(
            getattr(event_bus, "publish", None)
        ):
            self._log_warning(ctx, "vision_nav_start_cal_no_publish")
            return

        async def _publish_progress(p: CalibrationProgress) -> None:
            try:
                result = event_bus.publish(
                    EVENT_CALIBRATION_PROGRESS,
                    {
                        "type": "calibration_progress",
                        "stage": p.stage,
                        "percent": p.percent,
                        "detail": p.detail,
                    },
                )
                if asyncio.iscoroutine(result):
                    await result
            except Exception as exc:  # noqa: BLE001
                self._log_warning(
                    ctx, "vision_nav_cal_publish_failed", error=str(exc)
                )

        async def _publish_complete(
            result: CalibrationResult | None,
            error: str | None,
        ) -> None:
            payload_out: dict[str, Any] = {
                "type": "calibration_complete",
                "result": result.to_dict() if result is not None else None,
            }
            if error is not None:
                payload_out["error"] = error
            try:
                outcome = event_bus.publish(
                    EVENT_CALIBRATION_COMPLETE, payload_out
                )
                if asyncio.iscoroutine(outcome):
                    await outcome
            except Exception as exc:  # noqa: BLE001
                self._log_warning(
                    ctx, "vision_nav_cal_complete_publish_failed",
                    error=str(exc),
                )
            # Apply intrinsics + timeshift on success so the live
            # estimator picks them up on the next tick.
            if result is not None:
                if self._health is not None:
                    self._health.set_intrinsics_loaded(True)
                if self._time_aligner is not None:
                    self._time_aligner.set_timeshift(
                        result.timeshift_cam_imu_s
                    )

        def _fetch_imu_window(
            start_ns: int, end_ns: int
        ) -> list[_RunnerImuSample]:
            source = self._imu_source
            if source is None:
                return []
            samples = []
            for s in source.recent():
                if start_ns <= s.ts_ns <= end_ns:
                    samples.append(
                        _RunnerImuSample(
                            timestamp_ns=s.ts_ns,
                            gyro=(s.xgyro, s.ygyro, s.zgyro),
                            accel=(s.xacc, s.yacc, s.zacc),
                        )
                    )
            return samples

        def _persist(yaml_text: str) -> Path:
            cam_dir = self._calibration_dir(ctx)
            cam_dir.mkdir(parents=True, exist_ok=True)
            path = cam_dir / "camchain.yaml"
            path.write_text(yaml_text, encoding="utf-8")
            return path

        hooks = RunnerHooks(
            publish_progress=_publish_progress,
            publish_complete=_publish_complete,
            fetch_imu_window=_fetch_imu_window,
            persist_camchain_yaml=_persist,
        )
        try:
            await run_calibration(payload, hooks)
        except Exception as exc:  # noqa: BLE001
            self._log_warning(
                ctx, "vision_nav_cal_runner_crashed", error=str(exc)
            )
            await _publish_complete(None, f"runner crashed: {exc}")

    def _trigger_imu_bias_recalibration(self, ctx: object) -> None:
        """Forward the GCS-side recalibrate event to the IMU source.

        Only the direct-I2C source supports bias calibration; the
        MAVLink path uses the FC's own calibration tooling. The
        method is a no-op when the active source is anything other
        than :class:`DirectI2cImu`.
        """

        from altnautica_vision_nav.imu.direct_i2c import DirectI2cImu

        source = self._imu_source
        if not isinstance(source, DirectI2cImu):
            self._log_warning(
                ctx,
                "vision_nav_recalibrate_ignored_non_direct_imu",
                source=type(source).__name__,
            )
            return
        try:
            source.start_bias_calibration()
            self._log_info(ctx, "vision_nav_recalibrate_imu_biases_started")
        except Exception as exc:  # noqa: BLE001
            self._log_warning(
                ctx,
                "vision_nav_recalibrate_imu_biases_failed",
                error=str(exc),
            )

    def _try_load_persisted_calibration(self, ctx: object) -> None:
        """Load a previously-uploaded calibration on plugin start."""

        path = self._calibration_dir(ctx) / "camchain.yaml"
        if not path.is_file():
            return
        try:
            extrinsics = load_extrinsics(path)
            intrinsics = load_intrinsics(path)
        except Exception as exc:  # noqa: BLE001
            self._log_warning(
                ctx,
                "vision_nav_persisted_calibration_invalid",
                error=str(exc),
            )
            return
        if self._health is not None:
            self._health.set_intrinsics_loaded(True)
        if self._time_aligner is not None:
            self._time_aligner.set_timeshift(extrinsics.timeshift_cam_imu)
        self._log_info(
            ctx,
            "vision_nav_persisted_calibration_loaded",
            fx=intrinsics.fx,
            timeshift=extrinsics.timeshift_cam_imu,
        )

    def _read_persisted_camchain(self):
        """Re-read the persisted YAML for VIO engine config construction."""

        ctx = self._ctx
        if ctx is None:
            return None
        path = self._calibration_dir(ctx) / "camchain.yaml"
        if not path.is_file():
            return None
        try:
            intrinsics = load_intrinsics(path)
            extrinsics = load_extrinsics(path)
        except Exception:  # noqa: BLE001
            return None
        return intrinsics, extrinsics

    def _calibration_dir(self, ctx: object) -> Path:
        data_dir = getattr(ctx, "data_dir", None) or os.environ.get(
            "ADOS_PLUGIN_DATA_DIR"
        )
        if data_dir is None:
            data_dir = "/var/ados/plugins/com.altnautica.vision-nav/data"
        return Path(data_dir)

    @staticmethod
    def _extract_str(payload: Any, key: str) -> Optional[str]:
        if isinstance(payload, dict):
            value = payload.get(key)
        else:
            value = getattr(payload, key, None)
        if isinstance(value, str):
            return value
        return None

    # ------------------------------------------------------------------
    # MAVLink component registration
    # ------------------------------------------------------------------

    def _register_components(self, ctx: object) -> None:
        mavlink = getattr(ctx, "mavlink", None)
        if mavlink is None:
            return
        register = getattr(mavlink, "register_component", None)
        if not callable(register):
            return
        try:
            register(COMPONENT_OF_PERIPHERAL, kind="peripheral")
            self._component_of_registered = True
        except Exception as exc:  # noqa: BLE001
            self._log_warning(
                ctx,
                "vision_nav_component_register_failed",
                component=COMPONENT_OF_PERIPHERAL,
                error=str(exc),
            )
        try:
            register(COMPONENT_VIO, kind="vio")
            self._component_vio_registered = True
        except Exception as exc:  # noqa: BLE001
            self._log_warning(
                ctx,
                "vision_nav_component_register_failed",
                component=COMPONENT_VIO,
                error=str(exc),
            )

    def _unregister_components(self, ctx: object) -> None:
        mavlink = getattr(ctx, "mavlink", None)
        if mavlink is None:
            self._component_of_registered = False
            self._component_vio_registered = False
            return
        unregister = getattr(mavlink, "unregister_component", None)
        if not callable(unregister):
            self._component_of_registered = False
            self._component_vio_registered = False
            return
        for comp_id, flag in (
            (COMPONENT_OF_PERIPHERAL, "_component_of_registered"),
            (COMPONENT_VIO, "_component_vio_registered"),
        ):
            if not getattr(self, flag):
                continue
            try:
                unregister(comp_id)
            except Exception as exc:  # noqa: BLE001
                self._log_warning(
                    ctx,
                    "vision_nav_component_unregister_failed",
                    component=comp_id,
                    error=str(exc),
                )
            finally:
                setattr(self, flag, False)

    async def _safe_clock_start(self, ctx: object) -> None:
        if self._clock_align is None:
            return
        try:
            await self._clock_align.start(ctx)
        except Exception as exc:  # noqa: BLE001
            self._log_warning(
                ctx, "vision_nav_clock_start_failed", error=str(exc)
            )

    # ------------------------------------------------------------------
    # Capture + rangefinder (unchanged from prior plugin.py)
    # ------------------------------------------------------------------

    def _open_capture(
        self, ctx: object, config: VisionNavConfig
    ) -> object | None:
        if config.camera.bus_type == "csi":
            try:
                from altnautica_vision_nav.capture.libcamera_source import (
                    LibcameraSource,
                )

                cam = LibcameraSource(
                    width=config.camera.width,
                    height=config.camera.height,
                    fps=config.camera.fps,
                )
                cam.start()
                return cam
            except Exception as exc:  # noqa: BLE001
                self._log_warning(
                    ctx, "vision_nav_libcamera_failed", error=str(exc)
                )
        try:
            from altnautica_vision_nav.capture.v4l2_source import V4l2Source

            cam = V4l2Source(
                device_path=config.camera.device_path,
                width=config.camera.width,
                height=config.camera.height,
                fps=config.camera.fps,
            )
            cam.start()
            return cam
        except Exception as exc:  # noqa: BLE001
            self._log_warning(
                ctx, "vision_nav_v4l2_failed", error=str(exc)
            )
            return None

    async def _open_rangefinder(
        self, ctx: object, config: VisionNavConfig
    ) -> Optional[RangefinderDriver]:
        rf = config.rangefinder
        if rf.topology == "none":
            return None
        if rf.topology == "fc" or rf.driver == "fc_relay":
            driver: RangefinderDriver = RelayDistanceSensor(ctx)
        else:
            try:
                driver = self._build_companion_driver(rf)
            except Exception as exc:  # noqa: BLE001
                self._log_warning(
                    ctx, "vision_nav_rangefinder_build_failed", error=str(exc)
                )
                return None
        try:
            await driver.open()
        except Exception as exc:  # noqa: BLE001
            self._log_warning(
                ctx, "vision_nav_rangefinder_open_failed", error=str(exc)
            )
            return None
        return driver

    def _build_companion_driver(self, rf: object) -> RangefinderDriver:
        driver_name = getattr(rf, "driver", "")
        device = getattr(rf, "device", None) or ""
        baud = getattr(rf, "baud", None) or 115200
        if driver_name == "tfluna_uart":
            return TfLunaUart(device=device or "/dev/ttyUSB0", baud=int(baud))
        if driver_name == "garmin_lidarlite_i2c":
            return GarminLidarLiteI2c(bus_number=_i2c_bus_number(device, 1))
        if driver_name == "vl53l1x_i2c":
            return Vl53l1xI2c(bus_number=_i2c_bus_number(device, 1))
        raise ValueError(f"unknown rangefinder driver: {driver_name!r}")

    def _topology_label(self, config: VisionNavConfig) -> Optional[str]:
        topology = config.rangefinder.topology
        if topology in ("companion", "fc"):
            return topology
        return None

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    def _log_info(self, ctx: object, event: str, **fields: object) -> None:
        self._log(ctx, "info", event, fields)

    def _log_warning(self, ctx: object, event: str, **fields: object) -> None:
        self._log(ctx, "warning", event, fields)

    def _log(
        self,
        ctx: object,
        level: str,
        event: str,
        fields: dict,
    ) -> None:
        plugin_log = getattr(ctx, "log", None) if ctx is not None else None
        if plugin_log is not None:
            method = getattr(plugin_log, level, None)
            if callable(method):
                try:
                    method(event, **fields)
                    return
                except TypeError:
                    try:
                        method("%s %s", event, fields)
                        return
                    except Exception:  # noqa: BLE001
                        pass
                except Exception:  # noqa: BLE001
                    pass
        fallback = getattr(log, level, log.info)
        fallback("%s %s", event, fields)


class _NoOpCapture:
    """Stand-in when capture cannot be opened. The pipeline loop never iterates."""

    async def __aiter__(self):
        if False:
            yield None  # pragma: no cover - unreachable

    def frames(self):
        return self.__aiter__()

    def stop(self) -> None:
        return None


class _NoOpImuSource(BaseImuSource):
    """Empty IMU source used when no MAVLink subscription is available."""

    source_id = "noop"

    def start(self) -> None:
        return None

    def stop(self) -> None:
        return None
