"""Vision Navigation plugin entry point.

The agent supervisor imports :class:`VisionNavPlugin` via the
``ados.plugins`` entry-point declared in ``pyproject.toml`` and runs
its lifecycle hooks. The runner's exact hook surface evolves with the
agent; this module implements the seven hooks the runner currently
calls (``on_install``, ``on_enable``, ``on_configure``, ``on_start``,
``on_stop``, ``on_disable``, ``on_uninstall``) plus the per-drone
pause/resume pair the host fires when an operator toggles the
extension off and on without removing it.

The SDK at ``ados.sdk`` does not yet expose an ``ADOSPlugin`` base
class. The plugin runner treats any object with the matching async
methods as a plugin (duck-typed). This module follows that contract
and stays a plain class so it remains importable in isolation for
unit tests that do not have the agent on ``sys.path``.

Two paths through ``on_start`` are supported:

1. Production: the host provides a context with ``mavlink``, ``log``,
   ``telemetry``, etc. The plugin opens the configured camera and
   rangefinder, starts the OF pipeline, and the comp-198 heartbeat is
   visible on the bus within one tick.
2. Bench / fake context: a test or a host that does not yet expose
   all facets passes a stub. Anything that fails to construct (camera
   open, rangefinder open, MAVLink subscribe) is logged and skipped;
   the lifecycle still reaches ``on_start`` end so ``on_stop`` is
   reachable.
"""

from __future__ import annotations

import logging
from typing import Optional

from altnautica_vision_nav import __version__ as PLUGIN_VERSION
from altnautica_vision_nav.capture.gyro_tap import GyroTap
from altnautica_vision_nav.config import VisionNavConfig, load_config
from altnautica_vision_nav.health import HealthPublisher
from altnautica_vision_nav.mavlink.comp_status import (
    CompanionHeartbeat,
    CompanionState,
)
from altnautica_vision_nav.pipelines.of_pipeline import OFPipeline
from altnautica_vision_nav.pre_arm import PreArmHelper
from altnautica_vision_nav.processors.optical_flow_lk import OpticalFlowLk
from altnautica_vision_nav.rangefinder import (
    GarminLidarLiteI2c,
    RangefinderDriver,
    RelayDistanceSensor,
    TfLunaUart,
    Vl53l1xI2c,
)
from altnautica_vision_nav.time_sync.clock_align import ClockAlign

log = logging.getLogger(__name__)


COMPONENT_ID = 198
SENSOR_ID = 0


def _i2c_bus_number(device: str, default: int) -> int:
    """Parse ``/dev/i2c-N`` into ``N``; return ``default`` on any drift."""

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


class VisionNavPlugin:
    """Plugin entry point loaded by the agent supervisor.

    The class is intentionally small. Each lifecycle hook is the
    minimum amount of glue needed to drive the modules in this
    package; the heavy lifting lives in the OFPipeline, the
    rangefinder drivers, and the MAVLink helpers.
    """

    plugin_id = "com.altnautica.vision-nav"
    version = PLUGIN_VERSION

    def __init__(self) -> None:
        self._config: Optional[VisionNavConfig] = None
        self._capture: object | None = None
        self._gyro_tap: Optional[GyroTap] = None
        self._rangefinder: Optional[RangefinderDriver] = None
        self._heartbeat: Optional[CompanionHeartbeat] = None
        self._clock_align: Optional[ClockAlign] = None
        self._pipeline: Optional[OFPipeline] = None
        self._health: Optional[HealthPublisher] = None
        self._pre_arm: Optional[PreArmHelper] = None
        self._component_registered = False

    # ------------------------------------------------------------------
    # Lifecycle hooks
    # ------------------------------------------------------------------

    async def on_install(self, ctx: object) -> None:
        """Called once when the plugin is first installed.

        Logs the install and confirms the runtime is compatible. The
        manifest already constrains supported boards, so the agent
        host rejects an incompatible installation before this hook
        runs.
        """

        self._log_info(ctx, "vision_nav_installed", version=PLUGIN_VERSION)

    async def on_enable(self, ctx: object) -> None:
        """Called when the plugin is enabled. No-op until ``on_start``."""

        self._log_info(ctx, "vision_nav_enabled")

    async def on_configure(self, ctx: object, config: object) -> None:
        """Receive per-drone config from the host.

        ``config`` is a plain dict shipped by the supervisor. The
        Pydantic model defends against drift; on validation failure
        the plugin keeps the previous config (if any) and logs the
        error so the operator sees it in the GCS log stream.
        """

        raw = config if isinstance(config, dict) else {}
        try:
            self._config = load_config(raw)
        except Exception as exc:  # noqa: BLE001
            self._log_warning(
                ctx,
                "vision_nav_config_invalid",
                error=str(exc),
            )
            if self._config is None:
                # First-time configure: fall back to defaults so the
                # lifecycle can keep moving and a corrected config
                # later via on_configure will pick up cleanly.
                self._config = load_config({})

    async def on_start(self, ctx: object) -> None:
        """Open all resources and start the OF pipeline."""

        if self._config is None:
            self._config = load_config({})

        # Register MAVLink component first so subscribers see the
        # peripheral on the bus before the first emit.
        self._register_component(ctx)

        self._clock_align = ClockAlign(component_id=COMPONENT_ID)
        await self._safe_clock_start(ctx)

        self._heartbeat = CompanionHeartbeat(component_id=COMPONENT_ID)
        await self._heartbeat.start(ctx)

        self._gyro_tap = self._build_gyro_tap(ctx)

        capture = self._open_capture(ctx, self._config)
        self._capture = capture

        rangefinder = await self._open_rangefinder(ctx, self._config)
        self._rangefinder = rangefinder

        of_processor = OpticalFlowLk()

        topology = self._topology_label(self._config)
        recommended_camera = self._config.camera.device_path or None
        self._health = HealthPublisher(
            rangefinder_topology=topology,
            recommended_camera_id=recommended_camera,
        )
        self._health.start(ctx)
        self._health.update_companion_state(CompanionState.INACTIVE)

        self._pipeline = OFPipeline(
            ctx=ctx,
            capture_source=capture,
            of_processor=of_processor,
            gyro_tap=self._gyro_tap,
            rangefinder=rangefinder,
            clock_align=self._clock_align,
            companion_heartbeat=self._heartbeat,
            health_publisher=self._health,
            flow_quality_min=self._config.flow_quality_min,
            component_id=COMPONENT_ID,
            sensor_id=SENSOR_ID,
        )
        if capture is not None:
            await self._pipeline.start()
        else:
            self._log_warning(ctx, "vision_nav_capture_unavailable")

        self._pre_arm = PreArmHelper(component_id=COMPONENT_ID)
        if self._config.pre_arm.auto_set_origin:
            await self._pre_arm.auto_set_origin(
                ctx,
                self._config.pre_arm.origin_lat,
                self._config.pre_arm.origin_lon,
                self._config.pre_arm.origin_alt_m,
            )

        self._log_info(ctx, "vision_nav_started", version=PLUGIN_VERSION)

    async def on_pause(self, ctx: object) -> None:
        """Stop the OF pipeline but keep MAVLink subscriptions live.

        Used when an operator toggles the extension off without
        removing it. The companion drops to INACTIVE so the GCS
        renders a paused state immediately.
        """

        if self._pipeline is not None:
            await self._pipeline.stop()
        if self._heartbeat is not None:
            self._heartbeat.transition(CompanionState.INACTIVE)
        if self._health is not None:
            self._health.update_companion_state(CompanionState.INACTIVE)
        self._log_info(ctx, "vision_nav_paused")

    async def on_resume(self, ctx: object) -> None:
        """Restart the OF pipeline after a previous ``on_pause``."""

        if self._pipeline is not None:
            await self._pipeline.start()
        self._log_info(ctx, "vision_nav_resumed")

    async def on_stop(self, ctx: object) -> None:
        """Full shutdown: pipeline, capture, rangefinder, heartbeat."""

        if self._pipeline is not None:
            await self._pipeline.stop()
            self._pipeline = None

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

        if self._gyro_tap is not None:
            try:
                self._gyro_tap.stop()
            except Exception as exc:  # noqa: BLE001
                self._log_warning(
                    ctx, "vision_nav_gyro_stop_failed", error=str(exc)
                )
            self._gyro_tap = None

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

        self._unregister_component(ctx)
        self._log_info(ctx, "vision_nav_stopped")

    async def on_disable(self, ctx: object) -> None:
        """Alias for ``on_stop`` in v1.

        A future revision may keep state around so a re-enable does
        not have to re-open hardware; for now disable means fully
        teardown.
        """

        await self.on_stop(ctx)

    async def on_uninstall(self, ctx: object) -> None:
        """Final cleanup hook. No persistent state in v1."""

        self._log_info(ctx, "vision_nav_uninstalled")

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _register_component(self, ctx: object) -> None:
        mavlink = getattr(ctx, "mavlink", None)
        if mavlink is None:
            return
        register = getattr(mavlink, "register_component", None)
        if not callable(register):
            return
        try:
            register(COMPONENT_ID, kind="peripheral")
            self._component_registered = True
        except Exception as exc:  # noqa: BLE001
            self._log_warning(
                ctx, "vision_nav_component_register_failed", error=str(exc)
            )

    def _unregister_component(self, ctx: object) -> None:
        if not self._component_registered:
            return
        mavlink = getattr(ctx, "mavlink", None)
        if mavlink is None:
            return
        unregister = getattr(mavlink, "unregister_component", None)
        if not callable(unregister):
            self._component_registered = False
            return
        try:
            unregister(COMPONENT_ID)
        except Exception as exc:  # noqa: BLE001
            self._log_warning(
                ctx, "vision_nav_component_unregister_failed", error=str(exc)
            )
        finally:
            self._component_registered = False

    async def _safe_clock_start(self, ctx: object) -> None:
        if self._clock_align is None:
            return
        try:
            await self._clock_align.start(ctx)
        except Exception as exc:  # noqa: BLE001
            self._log_warning(
                ctx, "vision_nav_clock_start_failed", error=str(exc)
            )

    def _build_gyro_tap(self, ctx: object) -> Optional[GyroTap]:
        mavlink = getattr(ctx, "mavlink", None)
        if mavlink is None or not callable(getattr(mavlink, "subscribe", None)):
            return None
        tap = GyroTap(ctx)
        try:
            tap.start()
            return tap
        except Exception as exc:  # noqa: BLE001
            self._log_warning(
                ctx, "vision_nav_gyro_subscribe_failed", error=str(exc)
            )
            return None

    def _open_capture(
        self, ctx: object, config: VisionNavConfig
    ) -> object | None:
        """Open the configured capture source.

        Returns ``None`` if neither libcamera nor V4L2 can open. The
        plugin lifecycle still reaches ``on_start`` end so ``on_stop``
        is reachable; the host shows the failure on the GCS card.
        """

        # CSI path first when explicitly requested.
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
                    ctx,
                    "vision_nav_libcamera_failed",
                    error=str(exc),
                )

        # UVC / V4L2 fallback.
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
                ctx,
                "vision_nav_v4l2_failed",
                error=str(exc),
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
                    ctx,
                    "vision_nav_rangefinder_build_failed",
                    error=str(exc),
                )
                return None

        try:
            await driver.open()
        except Exception as exc:  # noqa: BLE001
            self._log_warning(
                ctx,
                "vision_nav_rangefinder_open_failed",
                error=str(exc),
            )
            return None
        return driver

    def _build_companion_driver(self, rf: object) -> RangefinderDriver:
        """Construct a companion-side rangefinder per config.

        Driver constructors raise ``ImportError`` when pyserial /
        smbus2 are not installed, or ``OSError`` when the device is
        absent. Both bubble up to the caller's broad except so the
        lifecycle keeps moving.
        """

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
    # Logging helpers
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
