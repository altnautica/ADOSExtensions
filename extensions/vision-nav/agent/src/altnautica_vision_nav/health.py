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
        }

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
