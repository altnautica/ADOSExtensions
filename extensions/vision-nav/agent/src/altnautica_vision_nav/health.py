"""Heartbeat extras publisher.

The plugin owns a ``navigation`` block on the agent's heartbeat. The
publisher keeps a 1 Hz tick that pushes the latest pipeline snapshot
through ``ctx.telemetry.extend("navigation", payload)``. The payload
shape mirrors the GCS-side ``VisionNavTelemetry`` type and the
``cmd_droneStatus.navigation`` validator on the cloud relay.

Field names are camelCase end-to-end. A previous lite-agent gate-9
finding showed that snake_case drift on the wire silently drops the
entire enriched block at the cloud relay, so this module is the only
authority on the spelling.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Optional

from altnautica_vision_nav.mavlink.comp_status import CompanionState
from altnautica_vision_nav.processors.optical_flow_lk import OpticalFlowResult

log = logging.getLogger(__name__)

TICK_INTERVAL_S = 1.0

# Map the comp-198 companion state to the GCS-facing estimator state.
# The mapping mirrors how the operator reads the card today: when the
# companion is active the OF estimator is steady-state, when it is
# critical the OF samples are being dropped (so the estimator is
# degraded), and when the companion is offline there is no estimator.
_COMPANION_TO_ESTIMATOR_STATE = {
    CompanionState.INACTIVE: "off",
    CompanionState.ACTIVE: "converged",
    CompanionState.CRITICAL: "degraded",
    CompanionState.TERMINATING: "failed",
}


def _scale_source_for_topology(topology: Optional[str]) -> Optional[str]:
    """Derive the flow-scale-source label from the rangefinder topology.

    Today this only emits ``"rangefinder"`` (when a rangefinder of
    any topology is present) or ``None``. Once the rangefinder-free
    estimator lands and the active scale source is known per-sample,
    ``"baro"`` / ``"gps"`` / ``"vision"`` join the value set.
    """

    if topology in ("companion", "fc", "both"):
        return "rangefinder"
    return None


class HealthPublisher:
    """Periodically publish a ``navigation`` block on the agent telemetry bus.

    The OFPipeline calls :meth:`update_from_pipeline` after every emit
    so the snapshot stays fresh. The tick runs at 1 Hz and is
    idempotent: calling :meth:`start` twice is a no-op, and
    :meth:`stop` cancels the task cleanly.
    """

    def __init__(
        self,
        *,
        rangefinder_topology: Optional[str] = None,
        recommended_camera_id: Optional[str] = None,
        mode: Optional[str] = None,
        available_estimators: Optional[list[str]] = None,
    ) -> None:
        self._rangefinder_topology = rangefinder_topology
        self._recommended_camera_id = recommended_camera_id
        self._mode = mode
        self._available_estimators = (
            list(available_estimators) if available_estimators else []
        )
        self._latest_flow_quality: Optional[int] = None
        self._latest_flow_rate_hz: Optional[float] = None
        self._latest_distance_m: Optional[float] = None
        self._companion_state: CompanionState = CompanionState.INACTIVE
        # The IMU source + aligner are attached when the estimator
        # framework wires the full IMU pipeline. Each is None until
        # then; the snapshot reads ``None`` through to the wire, and
        # the GCS knows to render the sensors card as "not yet wired"
        # rather than as an empty / zeroed state.
        self._imu_source_id: Optional[str] = None
        self._imu_rate_hz_provider: Optional[object] = None
        self._sync_offset_provider: Optional[object] = None
        self._intrinsics_loaded: bool = False
        self._pre_arm_report: Optional[object] = None
        self._task: Optional[asyncio.Task[None]] = None
        self._ctx: object | None = None

    # ------------------------------------------------------------------
    # Public mutation API
    # ------------------------------------------------------------------

    def set_topology(self, topology: Optional[str]) -> None:
        """Set the rangefinder topology label.

        ``topology`` is one of ``"companion"``, ``"fc"``, ``"both"`` or
        ``None``. The cloud relay schema accepts exactly these values.
        """

        self._rangefinder_topology = topology

    def set_recommended_camera_id(self, camera_id: Optional[str]) -> None:
        """Set the camera id surfaced as the GCS recommended device."""

        self._recommended_camera_id = camera_id

    def set_mode(self, mode: Optional[str]) -> None:
        """Set the active estimator-mode label surfaced on the heartbeat.

        Values mirror the config ``mode`` literal (``"off"``,
        ``"optical_flow"`` today; ``"optical_flow_degraded"`` and the
        VIO modes are added in later phases). The GCS mode picker
        renders this as the currently-selected option.
        """

        self._mode = mode

    def set_available_estimators(self, estimators: list[str]) -> None:
        """Set the list of estimator keys this plugin instance can run.

        Mirrors the registry at start-up. The GCS uses this to gate the
        mode-picker options so the operator never sees an estimator the
        agent can't instantiate.
        """

        self._available_estimators = list(estimators)

    def attach_imu_source(self, imu_source: Optional[object]) -> None:
        """Attach an :class:`BaseImuSource` so its id + rate ride the heartbeat.

        ``None`` detaches and the snapshot reports ``imuSource = None``
        and ``imuRateHz = None`` so the GCS renders the sensors card
        as awaiting wiring rather than as a zeroed-out state.
        """

        if imu_source is None:
            self._imu_source_id = None
            self._imu_rate_hz_provider = None
            return
        source_id = getattr(imu_source, "source_id", None)
        self._imu_source_id = source_id if isinstance(source_id, str) else None
        self._imu_rate_hz_provider = imu_source

    def attach_time_aligner(self, aligner: Optional[object]) -> None:
        """Attach the :class:`TimeAligner` so its rolling residual rides the heartbeat.

        The GCS surfaces ``cameraImuSyncOffsetMs`` in the sensors card
        and gates the VIO pre-arm check on the drift band.
        """

        self._sync_offset_provider = aligner

    def set_intrinsics_loaded(self, loaded: bool) -> None:
        """Set whether camera intrinsics have been loaded from calibration.

        The VIO pre-arm gate refuses to arm when intrinsics are not
        loaded; the GCS sensors card shows a Calibrate CTA when this
        is false.
        """

        self._intrinsics_loaded = bool(loaded)

    def set_pre_arm_report(self, report: Optional[object]) -> None:
        """Attach the latest :class:`PreArmReport` from the gate.

        The heartbeat publishes ``preArmReport`` as a structured dict
        the GCS pre-arm card reads. ``None`` clears the field so an
        outdated report does not linger across mode changes.
        """

        self._pre_arm_report = report

    def update_companion_state(self, state: CompanionState) -> None:
        """Mirror the comp 198 companion state into the heartbeat block."""

        self._companion_state = state

    def update_from_pipeline(
        self,
        *,
        result: OpticalFlowResult,
        flow_rate_hz: float,
        distance_m: Optional[float],
    ) -> None:
        """Refresh the OF-derived snapshot from one pipeline emit."""

        self._latest_flow_quality = int(result.quality)
        self._latest_flow_rate_hz = float(flow_rate_hz)
        self._latest_distance_m = (
            float(distance_m) if distance_m is not None else None
        )

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self, ctx: object) -> None:
        """Begin the 1 Hz publish tick. Idempotent."""

        if self._task is not None and not self._task.done():
            return
        self._ctx = ctx
        self._task = asyncio.create_task(self._tick_loop())

    async def stop(self) -> None:
        """Cancel the publish tick."""

        task = self._task
        self._task = None
        if task is not None:
            task.cancel()
            try:
                await task
            except (asyncio.CancelledError, Exception):  # noqa: BLE001
                pass
        self._ctx = None

    # ------------------------------------------------------------------
    # Snapshot
    # ------------------------------------------------------------------

    def snapshot(self) -> dict:
        """Return the current ``navigation`` payload as a plain dict.

        All keys are camelCase. ``flowDistanceM`` is the only field that
        can be ``None``; the rest carry concrete defaults so the GCS
        renders a stable card even when no frames have flown yet.
        """

        estimator_state = _COMPANION_TO_ESTIMATOR_STATE.get(
            self._companion_state, "off"
        )
        scale_source = _scale_source_for_topology(self._rangefinder_topology)
        return {
            "opticalFlowSupported": True,
            "vioSupported": False,
            "rangefinderTopology": self._rangefinder_topology,
            "recommendedCameraId": self._recommended_camera_id,
            "flowQuality": self._latest_flow_quality,
            "flowRateHz": self._latest_flow_rate_hz,
            "flowDistanceM": self._latest_distance_m,
            "vioState": "absent",
            "vioResetCounter": 0,
            "vioQuality": None,
            "companionState": self._companion_state.name.lower(),
            # Additive fields from the estimator-framework. All
            # optional on the GCS side so an older GCS render path
            # stays correct, and so this snapshot stays a strict
            # superset of the previous shape.
            "mode": self._mode,
            "availableEstimators": list(self._available_estimators),
            "estimatorState": estimator_state,
            "flowScaleSource": scale_source,
            "imuSource": self._imu_source_id,
            "imuRateHz": self._read_imu_rate_hz(),
            "cameraImuSyncOffsetMs": self._read_sync_offset_ms(),
            "cameraIntrinsicsLoaded": self._intrinsics_loaded,
            "preArmReport": self._read_pre_arm_report(),
        }

    def _read_pre_arm_report(self) -> Optional[dict]:
        """Coerce the attached report into a plain dict for the wire.

        Accepts a :class:`PreArmReport` (which has an :meth:`as_dict`
        helper) or a raw dict for tests. Returns ``None`` when no
        report has been set so the GCS treats the row as "unknown"
        rather than rendering an empty card.
        """

        report = self._pre_arm_report
        if report is None:
            return None
        as_dict = getattr(report, "as_dict", None)
        if callable(as_dict):
            try:
                return as_dict()
            except Exception:  # noqa: BLE001 - never let a probe break the tick
                return None
        if isinstance(report, dict):
            return report
        return None

    def _read_imu_rate_hz(self) -> Optional[float]:
        """Read the IMU rate via the source's ``rate_hz()`` if attached."""

        provider = self._imu_rate_hz_provider
        if provider is None:
            return None
        rate = getattr(provider, "rate_hz", None)
        if not callable(rate):
            return None
        try:
            value = rate()
        except Exception:  # noqa: BLE001 - never let a probe break the tick
            return None
        if isinstance(value, (int, float)):
            return float(value)
        return None

    def _read_sync_offset_ms(self) -> Optional[float]:
        """Read the rolling sync residual via the aligner if attached."""

        provider = self._sync_offset_provider
        if provider is None:
            return None
        residual = getattr(provider, "mean_residual_ms", None)
        if not callable(residual):
            return None
        try:
            value = residual()
        except Exception:  # noqa: BLE001 - never let a probe break the tick
            return None
        if isinstance(value, (int, float)):
            return float(value)
        return None

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    async def _tick_loop(self) -> None:
        try:
            while True:
                await self._emit_once()
                await asyncio.sleep(TICK_INTERVAL_S)
        except asyncio.CancelledError:
            return
        except Exception as exc:  # noqa: BLE001
            log.warning("health publisher tick loop failed: %s", exc)

    async def _emit_once(self) -> None:
        ctx = self._ctx
        if ctx is None:
            return
        telemetry = getattr(ctx, "telemetry", None)
        if telemetry is None:
            return
        extend = getattr(telemetry, "extend", None)
        if extend is None:
            return
        try:
            result = extend("navigation", self.snapshot())
            if asyncio.iscoroutine(result):
                await result
        except Exception as exc:  # noqa: BLE001
            log.warning("health publisher emit failed: %s", exc)
