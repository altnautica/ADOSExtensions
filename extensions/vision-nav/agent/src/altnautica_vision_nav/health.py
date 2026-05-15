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
    ) -> None:
        self._rangefinder_topology = rangefinder_topology
        self._recommended_camera_id = recommended_camera_id
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
