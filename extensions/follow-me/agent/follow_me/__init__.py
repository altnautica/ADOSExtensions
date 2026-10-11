"""Follow-Me agent plugin.

The companion-side half of the Follow-Me behavior. It subscribes to the
vision engine's detection stream and the flight controller's pose, locks
onto an operator-designated subject, and flies a fixed-distance standoff
follow by projecting the subject's image position onto the ground and
emitting guided position setpoints to the autopilot at a steady rate.

Designation is operator-driven and owned by the vision engine: the operator
designates a subject through the engine (the GCS overlay calls the engine's
designate route), the engine's single-object tracker locks that subject and
stamps it with a track id + lock state on the detection stream, and this
plugin follows that one tracked subject. A lock-state safety gate stops
commanding the instant the tracker reports the subject uncertain or lost,
and the engine never silently re-locks onto a different subject — the
operator must designate again.

The behavior is gated by a per-drone ``active`` config flag the GCS skill
toggles; the loop reads it live each cycle so arming/disarming the skill
takes effect without a restart. The plugin publishes a ``follow.state``
read-back the GCS skill bar and config tab render.

The detector is a generic person/COCO model registered elsewhere in the
vision pipeline; this plugin consumes whatever detection stream the
configured camera produces and follows the operator's chosen track.
"""

from __future__ import annotations

import asyncio
from dataclasses import replace
from typing import Any

from ados.sdk.cameras import CAMERA_SELECTOR_AUTO
from ados.sdk.tracking import LockedTargetTracker
from ados.sdk.vision import DetectionBatch

from follow_me import fc_state
from follow_me.fc_state import FcState, FcStateMixin, GimbalState, Pose
from follow_me.loop import FollowLoopMixin, _LastCommand
from follow_me.state import HOLD_INACTIVE, FollowConfig, FollowState

PLUGIN_ID = "com.altnautica.follow-me"

# How long the loop coasts on the last good lock before declaring the
# subject lost when its detection stops arriving.
_COAST_WINDOW_S = 1.5

# How often the full per-drone config key set is re-read. The follow loop
# runs faster than this; between refreshes only the live ``active`` toggle
# is re-read each tick so arm/disarm stays responsive, while the rarely-
# changed geometry keys are served from cache instead of an IPC round-trip
# per key on every loop.
_CONFIG_REFRESH_S = 1.0


class FollowMePlugin(FcStateMixin, FollowLoopMixin):
    """Lifecycle-hook plugin class the runner instantiates with no args."""

    def __init__(self) -> None:
        self._ctx: Any = None
        self._pose = Pose()
        self._fc = FcState()
        self._gimbal = GimbalState()
        # The last gimbal angle the plugin itself commanded (projection
        # convention), used as the boresight estimate until the FC reports a
        # real gimbal attitude.
        self._commanded_gimbal: tuple[float, float] | None = None
        # The real source-frame size from the most recent detection batch that
        # carried one; None until the first sized batch arrives.
        self._last_frame_wh: tuple[int, int] | None = None
        # The shared locked-target safety gate: adopt only the engine's
        # designated target, coast briefly, stop on uncertain/lost, never
        # silently re-lock. One audited implementation for every behaviour.
        self._tracker = LockedTargetTracker(coast_window_s=_COAST_WINDOW_S)
        self._loop_task: asyncio.Task | None = None
        self._published = FollowState()
        self._last_heartbeat: float = 0.0
        # Cached per-drone config, refreshed on a slow interval (see
        # _config_for_tick) instead of re-fetched every loop.
        self._cfg: FollowConfig | None = None
        self._cfg_read_at: float = 0.0
        # The camera whose detections the follow loop consumes. ``None`` (no
        # config read yet) accepts any camera; once set, _on_batch filters to
        # it, so a runtime designate_camera change takes effect immediately.
        self._active_camera: str | None = None
        # The last commanded follow setpoint, held (re-sent) while coasting on
        # a frozen bbox so the setpoint does not drift on stale image data.
        self._last_command: _LastCommand | None = None

    # -- lifecycle ----------------------------------------------------

    async def on_start(self, ctx: Any) -> None:
        self._ctx = ctx
        cfg = await self._read_config()
        self._cfg = cfg
        self._cfg_read_at = fc_state.monotonic()
        self._active_camera = self._resolve_designate_camera(cfg.designate_camera)

        # FC state. ATTITUDE gives roll/pitch/yaw; GLOBAL_POSITION_INT gives
        # lat/lon and relative altitude (height AGL above home); HEARTBEAT
        # gives the armed + flight-mode state the command gate needs.
        await ctx.mavlink.subscribe("ATTITUDE", self._on_attitude)
        await ctx.mavlink.subscribe(
            "GLOBAL_POSITION_INT", self._on_global_position
        )
        await ctx.mavlink.subscribe("HEARTBEAT", self._on_heartbeat)
        # The gimbal's reported boresight attitude, so the image-to-ground
        # projection uses where the gimbal actually points, not a fixed guess.
        await ctx.mavlink.subscribe(
            "MOUNT_ORIENTATION", self._on_mount_orientation
        )

        # Detection stream. The operator designates through the vision engine;
        # we follow whatever track it locks (see _on_batch), so there is no
        # plugin-side designate path. The SDK has no unsubscribe, so we
        # subscribe to every camera once and filter to the configured camera
        # locally in _on_batch; changing designate_camera at runtime then takes
        # effect immediately without re-subscribing.
        await ctx.vision.subscribe_detections(self._on_batch, camera_id=None)

        # MCP tools (guarded: the host injects ctx.tools only when the
        # mcp.expose capability is granted; tests pass a ctx without it). The
        # tools are read-only + stop; there is no AI-initiated follow-start.
        self._register_tools(ctx)

        self._loop_task = asyncio.create_task(self._follow_loop())
        ctx.log.info("follow_me_started", camera=cfg.designate_camera)
        await self._publish_state(force=True, hold_reason=HOLD_INACTIVE)

    # -- camera binding + MCP tools -----------------------------------

    def _resolve_designate_camera(self, selection: Any) -> str | None:
        """Resolve the camera-selector value to the detection-subscription
        filter. By-requirement (auto / empty) accepts detections from any
        camera (``None``) and follows the operator's designated track wherever
        the engine feeds it; a pinned id filters to that camera. The full
        roster-based resolution runs host-side; the follow loop only needs the
        pinned-vs-any distinction."""
        if selection is None:
            return None
        s = str(selection).strip()
        if s == "" or s == CAMERA_SELECTOR_AUTO:
            return None
        return s

    def _register_tools(self, ctx: Any) -> None:
        tools = getattr(ctx, "tools", None)
        if tools is None or not hasattr(tools, "register"):
            return
        tools.register("follow_status", self._tool_follow_status)
        tools.register("stop_follow", self._tool_stop_follow)

    async def _tool_follow_status(self, _args: dict) -> dict:
        """Report the current follow read-back (read-only)."""
        return self._published.to_dict()

    async def _tool_stop_follow(self, _args: dict) -> dict:
        """Disarm the follow behaviour. Only ever stops — an assistant cannot
        start a follow (that requires an operator to designate a subject)."""
        ctx = self._ctx
        if ctx is None:
            return {"ok": False, "reason": "not running"}
        setter = getattr(ctx.config_kv, "set", None)
        if setter is None:
            return {"ok": False, "reason": "no config seam"}
        await setter("active", False)
        return {"ok": True, "active": False}

    async def on_stop(self, ctx: Any) -> None:
        await self._teardown()
        ctx.log.info("follow_me_stopped")

    async def on_disable(self, ctx: Any) -> None:
        await self._teardown()

    async def _teardown(self) -> None:
        if self._loop_task is not None:
            self._loop_task.cancel()
            try:
                await self._loop_task
            except asyncio.CancelledError:
                pass
            self._loop_task = None
        self._tracker.drop()
        self._last_command = None
        if self._ctx is not None:
            await self._publish_state(
                force=True, active=False, hold_reason=HOLD_INACTIVE
            )

    # -- config -------------------------------------------------------

    async def _read_config(self) -> FollowConfig:
        live: dict[str, object] = {}
        for key in (
            "active",
            "follow_distance_m",
            "follow_height_m",
            "gimbal_point",
            "designate_camera",
            "camera_hfov_deg",
            "mount_pitch_deg",
        ):
            value = await self._ctx.config_kv.get(key, None)
            if value is not None:
                live[key] = value
        static = dict(getattr(self._ctx, "config", {}) or {})
        return FollowConfig.resolve(live, static)

    async def _config_for_tick(self) -> FollowConfig:
        """The per-drone config for one loop iteration.

        The full key set is re-read only every ``_CONFIG_REFRESH_S``; between
        refreshes the live ``active`` toggle is re-read each tick so arm/disarm
        stays responsive, while the rarely-changed geometry keys are served
        from cache. This cuts the per-loop IPC from one ``get`` per key to one
        ``get`` for ``active`` on most ticks.
        """
        now = fc_state.monotonic()
        if self._cfg is None or (now - self._cfg_read_at) >= _CONFIG_REFRESH_S:
            self._cfg = await self._read_config()
            self._cfg_read_at = now
        else:
            active = await self._ctx.config_kv.get("active", None)
            if active is not None:
                self._cfg = replace(self._cfg, active=bool(active))
        self._active_camera = self._resolve_designate_camera(
            self._cfg.designate_camera
        )
        return self._cfg

    # -- detections --------------------------------------------------

    def _on_batch(self, batch: DetectionBatch) -> None:
        # Subscribed to every camera, so filter to the configured designate
        # camera here (see on_start); a None active camera (before the first
        # config read) accepts any camera.
        if (
            self._active_camera is not None
            and batch.camera_id != self._active_camera
        ):
            return
        # Cache the real source-frame size the bbox pixels live in, so the
        # projection uses the true resolution instead of a guess.
        fw = getattr(batch, "frame_width", None)
        fh = getattr(batch, "frame_height", None)
        if fw and fh:
            self._last_frame_wh = (int(fw), int(fh))
        # Hand the batch to the shared gate. It adopts the engine's designated
        # target (the one detection stamped with a track id + lock state) and
        # never chooses a target itself; the follow loop reads the effective
        # lock (with the coast window) each tick.
        self._tracker.record(batch, fc_state.monotonic())


__all__ = ["FollowMePlugin", "PLUGIN_ID"]
