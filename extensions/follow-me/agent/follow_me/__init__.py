"""Follow-Me agent plugin.

The companion-side half of the Follow-Me behavior. It subscribes to the
vision engine's detection stream and the flight controller's local pose,
height-above-ground rangefinder and arm/mode state, locks onto an
operator-designated subject, estimates where the subject stands and how it
moves, and flies a follow (a distance and an angle from the subject's travel
heading) or an orbit by sending goals to the agent's pilot at 20 Hz under a
motion lease. The pilot owns obstacle avoidance and the speed limits; the
operator's sticks nudge the follow parameters while the lease is held.

Designation is operator-driven and owned by the vision engine: the operator
designates a subject through the engine (the GCS overlay calls the engine's
designate route), the engine's single-object tracker locks that subject and
stamps it with a track id + lock state on the detection stream, and this
plugin follows that one tracked subject. A lock-state safety gate stops
commanding the instant the tracker reports the subject uncertain or lost.

On a loss the operator-chosen ``reacquire_mode`` applies. ``strict`` gives
the subject up and waits for a new designate. ``smart`` coasts on the
prediction for a short window and, if a person matching the subject's
appearance appears where the subject should be, designates it through the
engine and announces it on ``follow.reacquired``; the engine itself never
re-locks silently.

The vision engine runs detection on a camera only while a consumer holds a
lease on it; while the behaviour is active this plugin holds one on its follow
camera (see ``follow_me.demand``).

The behavior is gated by a per-drone ``active`` config flag the GCS skill
toggles; the loop reads it live each cycle so arming/disarming the skill
takes effect without a restart. The plugin publishes a ``follow.state``
read-back the GCS skill bar and config tab render.
"""

from __future__ import annotations

import asyncio
import math
from collections import deque
from dataclasses import replace
from typing import Any

from ados.sdk.cameras import CAMERA_SELECTOR_AUTO
from ados.sdk.tracking import LockedTargetTracker
from ados.sdk.vision import DetectionBatch

from follow_me import fc_state
from follow_me.coast import CoastMixin
from follow_me.demand import DetectionLease
from follow_me.fc_state import FcState, FcStateMixin, GimbalState, Pose, Rangefinder
from follow_me.loop import FollowLoopMixin, Sighting
from follow_me.motion import LEASE_REVOKED_TOPIC, STICKS_TOPIC, Nudges
from follow_me.ranging import Scan
from follow_me.readback import ReadbackMixin
from follow_me.reacquire import Coast, Embedding, embedding_of
from follow_me.state import (
    HOLD_INACTIVE,
    PHASE_IDLE,
    FollowConfig,
    FollowState,
)
from follow_me.target_filter import TargetFilter

PLUGIN_ID = "com.altnautica.follow-me"

# How long the shared lock gate coasts on the last good sighting before
# declaring the subject lost when its detection stops arriving.
_TRACK_GAP_S = 1.5

# How often the full per-drone config key set is re-read. The follow loop
# runs faster than this; between refreshes only the live ``active`` toggle
# is re-read each tick so arm/disarm stays responsive.
_CONFIG_REFRESH_S = 1.0

# The planar scan the world-engine extension shares. Optional: when that
# extension is absent the subscription is refused and retried on this period,
# and the lidar range cue is simply unavailable.
SCAN_TOPIC = "plugin.world-engine.scan"
_SCAN_SUBSCRIBE_RETRY_S = 10.0

_CONFIG_KEYS = (
    "active",
    "follow_distance_m",
    "follow_height_m",
    "gimbal_point",
    "designate_camera",
    "camera_hfov_deg",
    "mount_pitch_deg",
    "reacquire_mode",
    "reacquire_window_s",
    "reacquire_min_similarity",
    "max_speed_mps",
    "follow_mode",
    "follow_angle_deg",
    "orbit_radius_m",
    "orbit_rate_dps",
    "orbit_direction",
    "obstacle_margin_m",
)


def parse_scan(payload: dict[str, Any]) -> Scan | None:
    """A body-frame planar scan from a ``plugin.world-engine.scan`` payload, or
    ``None`` when the payload is not one."""
    if payload.get("frame") != "body_frd":
        return None
    ranges = payload.get("ranges_m")
    try:
        start = float(payload["angle_start_rad"])
        inc = float(payload["angle_increment_rad"])
        values = tuple(float(r) for r in ranges)  # type: ignore[union-attr]
    except (KeyError, TypeError, ValueError):
        return None
    if not values or not (inc > 0.0) or not math.isfinite(start):
        return None
    return Scan(angle_start_rad=start, angle_increment_rad=inc, ranges_m=values)


class FollowMePlugin(FcStateMixin, ReadbackMixin, CoastMixin, FollowLoopMixin):
    """Lifecycle-hook plugin class the runner instantiates with no args."""

    def __init__(self) -> None:
        self._ctx: Any = None
        self._pose = Pose()
        self._fc = FcState()
        self._gimbal = GimbalState()
        self._rangefinder = Rangefinder()
        # The last gimbal angle the plugin itself commanded (projection
        # convention), used as the boresight estimate until the FC reports a
        # real gimbal attitude.
        self._commanded_gimbal: tuple[float, float] | None = None
        self._last_gimbal_s: float = -math.inf
        # Send times of the pilot goals, for the published send rate
        # (bounded: 20 Hz over a 2 s window is 40 entries).
        self._setpoint_times: deque[float] = deque(maxlen=256)
        # The real source-frame size from the most recent detection batch that
        # carried one; None until the first sized batch arrives.
        self._last_frame_wh: tuple[int, int] | None = None
        # The shared locked-target safety gate: adopt only the engine's
        # designated target, stop on uncertain/lost, never re-lock on its own.
        self._tracker = LockedTargetTracker(coast_window_s=_TRACK_GAP_S)
        # The subject track in the local NED frame, and which engine track it
        # belongs to.
        self._filter = TargetFilter()
        self._filter_track_id: int | None = None
        self._template: Embedding | None = None
        self._sighting: Sighting | None = None
        self._measured_bbox: Any = None
        self._last_fix: Any = None
        self._coast: Coast | None = None
        self._phase = PHASE_IDLE
        # The pilot's motion lease, the last refusal and when it was asked.
        self._lease_id: str | None = None
        self._lease_tried_s: float = -math.inf
        self._lease_refusal: str | None = None
        self._revoked_reason: str | None = None
        self._nudges = Nudges()
        # Follow reference heading and orbit phase, reset on a new track or a
        # mode change.
        self._ref_heading: float | None = None
        self._orbit_phase: float | None = None
        self._geom_mode: str | None = None
        self._geom_tick_s: float | None = None
        self._pending_latency_ts_ms: int | None = None
        self._latency_ms: int | None = None
        self._last_lost_track_id: int | None = None
        # The newest camera-filtered batch, examined for re-acquire candidates.
        self._latest_batch: DetectionBatch | None = None
        self._latest_batch_s: float = -math.inf
        self._scan: Scan | None = None
        self._scan_at: float = -math.inf
        self._scan_subscribed = False
        self._scan_subscribe_tried_s: float = -math.inf
        self._loop_task: asyncio.Task | None = None
        self._published = FollowState()
        self._last_heartbeat: float = -math.inf
        self._last_publish_s: float = -math.inf
        # Cached per-drone config, refreshed on a slow interval (see
        # _config_for_tick) instead of re-fetched every loop.
        self._cfg: FollowConfig | None = None
        self._cfg_read_at: float = 0.0
        # The camera whose detections the follow loop consumes. ``None`` (no
        # config read yet) accepts any camera; once set, _on_batch filters to
        # it, so a runtime designate_camera change takes effect immediately.
        self._active_camera: str | None = None
        # The detection lease on the follow camera, held while active.
        self._lease = DetectionLease()

    # -- lifecycle ----------------------------------------------------

    async def on_start(self, ctx: Any) -> None:
        self._ctx = ctx
        cfg = await self._read_config()
        self._cfg = cfg
        self._cfg_read_at = fc_state.monotonic()
        self._active_camera = self._resolve_designate_camera(cfg.designate_camera)

        # Flight-controller state: attitude, the local NED position and
        # velocity control runs in, the downward rangefinder (height above
        # ground), the barometric altitude fallback, and the arm + mode state
        # the command gate needs.
        await ctx.mavlink.subscribe("ATTITUDE", self._on_attitude)
        await ctx.mavlink.subscribe("LOCAL_POSITION_NED", self._on_local_position)
        await ctx.mavlink.subscribe("DISTANCE_SENSOR", self._on_distance_sensor)
        await ctx.mavlink.subscribe("GLOBAL_POSITION_INT", self._on_global_position)
        await ctx.mavlink.subscribe("HEARTBEAT", self._on_heartbeat)
        # The gimbal's reported boresight attitude, so the image-to-ground
        # projection uses where the gimbal actually points.
        await ctx.mavlink.subscribe("MOUNT_ORIENTATION", self._on_mount_orientation)

        # Detection stream, with re-identification embeddings for the smart
        # re-acquire appearance match. The SDK has no unsubscribe, so we
        # subscribe to every camera once and filter to the configured camera
        # locally in _on_batch.
        await ctx.vision.subscribe_detections(
            self._on_batch, camera_id=None, embeddings=True
        )
        await self._subscribe_scan(fc_state.monotonic())

        # The pilot revokes the motion lease (pause, stick takeover, land,
        # RTH, FC mode change) and forwards the operator's sticks while it is
        # held; both arrive only to the holder.
        await ctx.events.subscribe(LEASE_REVOKED_TOPIC, self._on_lease_revoked)
        await ctx.events.subscribe(STICKS_TOPIC, self._on_sticks)
        await self._subscribe_sticks(ctx)

        # MCP tools (guarded: the host injects ctx.tools only when the
        # mcp.expose capability is granted). Read-only + stop; there is no
        # AI-initiated follow-start.
        self._register_tools(ctx)

        self._loop_task = asyncio.create_task(self._follow_loop())
        ctx.log.info("follow_me_started", camera=cfg.designate_camera)
        await self._publish_state(
            cfg, phase=PHASE_IDLE, hold_reason=HOLD_INACTIVE, force=True
        )

    async def _housekeeping(self, now: float) -> None:
        await self._lease.update(self._ctx, self._lease_camera(), now)
        if (
            not self._scan_subscribed
            and (now - self._scan_subscribe_tried_s) >= _SCAN_SUBSCRIBE_RETRY_S
        ):
            await self._subscribe_scan(now)

    def _lease_camera(self) -> str | None:
        """The camera to hold a detection lease on: none while inactive,
        else the follow camera. On auto with no subject the engine's own
        designate lease covers the camera the operator designates on."""
        if self._cfg is None or not self._cfg.active:
            return None
        return self._follow_camera()

    def _follow_camera(self) -> str | None:
        """The one camera follow-me reads: the pinned designate camera, or on
        auto the camera of the designated subject while it is locked or being
        re-acquired. None (any camera) only on auto with no subject, so the
        operator can designate on any camera; once a subject exists another
        camera's batches (a belly camera's whole-frame box) never feed the
        lock gate or a re-acquire."""
        if self._active_camera is not None:
            return self._active_camera
        if self._sighting is not None and (
            self._tracker.has_lock or self._coast is not None
        ):
            return self._sighting.camera_id
        return None

    async def _subscribe_sticks(self, ctx: Any) -> None:
        """Ask the pilot for the operator's sticks while the lease is held.
        An agent without the method (or one that refuses it) leaves follow
        flying without nudges rather than failing the plugin start."""
        subscribe = getattr(ctx.flight, "sticks_subscribe", None)
        if subscribe is None:
            ctx.log.warning("follow_me_sticks_unavailable", reason="host has no sticks_subscribe")
            return
        try:
            await subscribe()
        except Exception as exc:  # noqa: BLE001
            ctx.log.warning("follow_me_sticks_unavailable", reason=str(exc))

    async def _subscribe_scan(self, now: float) -> None:
        self._scan_subscribe_tried_s = now
        try:
            await self._ctx.events.subscribe(SCAN_TOPIC, self._on_scan)
        except Exception as exc:  # noqa: BLE001
            self._ctx.log.debug("follow_me_scan_unavailable", error=str(exc))
            return
        self._scan_subscribed = True

    def _on_scan(self, payload: dict[str, Any]) -> None:
        scan = parse_scan(payload)
        if scan is not None:
            self._scan = scan
            self._scan_at = fc_state.monotonic()

    # -- camera binding + MCP tools -----------------------------------

    def _resolve_designate_camera(self, selection: Any) -> str | None:
        """Resolve the camera-selector value to the detection-subscription
        filter. By-requirement (auto / empty) accepts detections from any
        camera (``None``); a pinned id filters to that camera."""
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
        self._coast = None
        self._filter.reset()
        self._filter_track_id = None
        if self._ctx is not None:
            await self._release_lease()
            await self._lease.release(self._ctx)
            await self._publish_state(
                self._cfg, phase=PHASE_IDLE, hold_reason=HOLD_INACTIVE, force=True
            )

    # -- config -------------------------------------------------------

    async def _read_config(self) -> FollowConfig:
        live: dict[str, object] = {}
        for key in _CONFIG_KEYS:
            value = await self._ctx.config_kv.get(key, None)
            if value is not None:
                live[key] = value
        static = dict(getattr(self._ctx, "config", {}) or {})
        cfg = FollowConfig.resolve(live, static)
        # Stick nudges fly at once; the config catches up on the write-back.
        self._nudges.settle(cfg)
        return self._nudges.apply_to(cfg)

    async def _config_for_tick(self) -> FollowConfig:
        """The per-drone config for one loop iteration: the full key set every
        ``_CONFIG_REFRESH_S``, the live ``active`` toggle every tick."""
        now = fc_state.monotonic()
        if self._cfg is None or (now - self._cfg_read_at) >= _CONFIG_REFRESH_S:
            self._cfg = await self._read_config()
            self._cfg_read_at = now
        else:
            active = await self._ctx.config_kv.get("active", None)
            if active is not None:
                self._cfg = replace(self._cfg, active=bool(active))
            # A stick nudge reaches the next goal, not the next config read.
            self._cfg = self._nudges.apply_to(self._cfg)
        self._active_camera = self._resolve_designate_camera(
            self._cfg.designate_camera
        )
        return self._cfg

    # -- detections --------------------------------------------------

    def _on_batch(self, batch: DetectionBatch) -> None:
        # Subscribed to every camera, so filter to the follow camera here:
        # the pinned designate camera, or on auto the designated subject's
        # camera once there is one (any camera before).
        camera = self._follow_camera()
        if camera is not None and str(batch.camera_id) != camera:
            return
        now = fc_state.monotonic()
        fw = getattr(batch, "frame_width", None)
        fh = getattr(batch, "frame_height", None)
        frame_wh = (int(fw), int(fh)) if fw and fh else None
        if frame_wh is not None:
            self._last_frame_wh = frame_wh
        self._latest_batch = batch
        self._latest_batch_s = now
        for det in batch.detections or ():
            if det.track_id is not None and det.lock_state is not None:
                if det.bbox is not None:
                    self._sighting = Sighting(
                        track_id=int(det.track_id),
                        bbox=det.bbox,
                        camera_id=str(batch.camera_id),
                        frame_wh=frame_wh,
                        ts_ms=int(getattr(batch, "ts_ms", 0) or 0),
                        received_s=now,
                        confidence=float(det.confidence),
                        embedding=embedding_of(det),
                    )
                break
        # During a smart coast the lost track must not re-latch the gate
        # (the engine keeps reporting it as lost); only the batches after our
        # own re-acquire designate are fed.
        if self._coast is not None and not self._coast.designated:
            return
        # The shared gate adopts the engine's designated target and never
        # chooses one itself.
        self._tracker.record(batch, now)


__all__ = ["FollowMePlugin", "PLUGIN_ID", "SCAN_TOPIC", "parse_scan"]
