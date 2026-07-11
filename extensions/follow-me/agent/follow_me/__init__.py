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
import time
from typing import Any

from ados.plugins.manifest import (
    AgentBlock,
    Compatibility,
    PluginManifest,
)
from ados.sdk.tracking import EffectiveLock, LockedTargetTracker
from ados.sdk.vision import BoundingBox, DetectionBatch

from follow_me import mavlink_frames, projection
from follow_me.state import (
    FOLLOW_STATE_TOPIC,
    LOCK_LOCKED,
    LOCK_LOST,
    FollowConfig,
    FollowState,
)

PLUGIN_ID = "com.altnautica.follow-me"

# The fixed cadence the follow loop commands at, decoupled from the
# inference rate so the setpoint stream is smooth regardless of detector
# jitter.
_LOOP_HZ = 6.0
_LOOP_PERIOD = 1.0 / _LOOP_HZ

# How long the loop coasts on the last good lock before declaring the
# subject lost when its detection stops arriving.
_COAST_WINDOW_S = 1.5

# A slow heartbeat for the follow.state read-back even when nothing
# changed, so the GCS knows the agent is alive.
_STATE_HEARTBEAT_S = 1.0


def get_manifest() -> PluginManifest:
    """In-code manifest mirror (the packed archive ships manifest.yaml)."""
    return PluginManifest(
        schema_version=2,
        id=PLUGIN_ID,
        version="0.2.1",
        name="ADOS Follow-Me",
        description=(
            "Locks onto an operator-designated subject and flies a "
            "fixed-distance standoff follow from the companion."
        ),
        author="Altnautica",
        license="GPL-3.0-or-later",
        risk="high",
        compatibility=Compatibility(ados_version=">=0.99.125"),
        agent=AgentBlock(
            entrypoint="follow_me:FollowMePlugin",
            isolation="subprocess",
            permissions=[
                "vision.detection.subscribe",
                "mavlink.read",
                "mavlink.write",
                "event.publish",
                "event.subscribe",
            ],
        ),
    )


manifest = get_manifest()


class _Pose:
    """Latest FC pose cached from the MAVLink telemetry stream."""

    __slots__ = (
        "roll",
        "pitch",
        "yaw",
        "lat_deg",
        "lon_deg",
        "rel_alt_m",
        "have_attitude",
        "have_position",
    )

    def __init__(self) -> None:
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.lat_deg = 0.0
        self.lon_deg = 0.0
        self.rel_alt_m = 0.0
        self.have_attitude = False
        self.have_position = False

    @property
    def ready(self) -> bool:
        return self.have_attitude and self.have_position


class FollowMePlugin:
    """Lifecycle-hook plugin class the runner instantiates with no args."""

    def __init__(self) -> None:
        self._ctx: Any = None
        self._pose = _Pose()
        # The shared locked-target safety gate: adopt only the engine's
        # designated target, coast briefly, stop on uncertain/lost, never
        # silently re-lock. One audited implementation for every behaviour.
        self._tracker = LockedTargetTracker(coast_window_s=_COAST_WINDOW_S)
        self._loop_task: asyncio.Task | None = None
        self._published = FollowState()
        self._last_heartbeat: float = 0.0

    # -- lifecycle ----------------------------------------------------

    async def on_start(self, ctx: Any) -> None:
        self._ctx = ctx
        cfg = await self._read_config()

        # FC pose. ATTITUDE gives roll/pitch/yaw; GLOBAL_POSITION_INT gives
        # lat/lon and relative altitude (height AGL above home).
        await ctx.mavlink.subscribe("ATTITUDE", self._on_attitude)
        await ctx.mavlink.subscribe(
            "GLOBAL_POSITION_INT", self._on_global_position
        )

        # Detection stream from the configured camera. The operator designates
        # through the vision engine; we follow whatever track it locks (see
        # _on_batch), so there is no plugin-side designate path.
        await ctx.vision.subscribe_detections(
            self._on_batch, camera_id=cfg.designate_camera
        )

        self._loop_task = asyncio.create_task(self._follow_loop())
        ctx.log.info("follow_me_started", camera=cfg.designate_camera)
        await self._publish_state(force=True)

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
        if self._ctx is not None:
            await self._publish_state(force=True, active=False)

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

    # -- MAVLink pose handlers ---------------------------------------

    def _on_attitude(self, msg: dict[str, Any]) -> None:
        self._pose.roll = float(msg.get("roll", 0.0))
        self._pose.pitch = float(msg.get("pitch", 0.0))
        self._pose.yaw = float(msg.get("yaw", 0.0))
        self._pose.have_attitude = True

    def _on_global_position(self, msg: dict[str, Any]) -> None:
        # lat/lon are 1e7 integer degrees; relative_alt is millimetres.
        lat = msg.get("lat")
        lon = msg.get("lon")
        rel = msg.get("relative_alt")
        if lat is not None:
            self._pose.lat_deg = float(lat) / 1e7
        if lon is not None:
            self._pose.lon_deg = float(lon) / 1e7
        if rel is not None:
            self._pose.rel_alt_m = float(rel) / 1000.0
        self._pose.have_position = True

    # -- detections --------------------------------------------------

    def _on_batch(self, batch: DetectionBatch) -> None:
        # Hand the batch to the shared gate. It adopts the engine's designated
        # target (the one detection stamped with a track id + lock state) and
        # never chooses a target itself; the follow loop reads the effective
        # lock (with the coast window) each tick.
        self._tracker.record(batch, time.monotonic())

    # -- the follow loop ---------------------------------------------

    async def _follow_loop(self) -> None:
        while True:
            await asyncio.sleep(_LOOP_PERIOD)
            try:
                await self._tick()
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # noqa: BLE001
                self._ctx.log.warning("follow_me_tick_error", error=str(exc))

    async def _tick(self) -> None:
        cfg = await self._read_config()

        if not cfg.active or not self._tracker.has_lock:
            await self._publish_state(active=cfg.active, commanding=False)
            return

        now = time.monotonic()
        lock = self._tracker.effective_lock(now)

        if lock is EffectiveLock.LOST:
            # Drop the lock entirely; require a fresh operator designate.
            self._tracker.drop()
            await self._publish_state(
                active=True, lock_state=LOCK_LOST, commanding=False, force=True
            )
            return

        target = self._tracker.locked_target(now)
        if target is None:
            # uncertain (or coasting): hold, stop commanding.
            await self._publish_state(
                active=True, lock_state=lock.value, commanding=False
            )
            return

        if not self._pose.ready:
            await self._publish_state(
                active=True, lock_state=LOCK_LOCKED, commanding=False
            )
            return

        await self._command_follow(cfg, target.bbox)

    async def _command_follow(self, cfg: FollowConfig, bbox: BoundingBox) -> None:
        frame_w, frame_h = self._frame_size_for(bbox)

        gimbal_pitch = 0.0
        gimbal_yaw = 0.0
        setpoint = projection.project_follow_setpoint(
            bbox_x=bbox.x,
            bbox_y=bbox.y,
            bbox_w=bbox.width,
            bbox_h=bbox.height,
            frame_width=frame_w,
            frame_height=frame_h,
            horizontal_fov_deg=cfg.camera_hfov_deg,
            roll_rad=self._pose.roll,
            pitch_rad=self._pose.pitch,
            yaw_rad=self._pose.yaw,
            vehicle_lat_deg=self._pose.lat_deg,
            vehicle_lon_deg=self._pose.lon_deg,
            agl_m=self._pose.rel_alt_m,
            follow_distance_m=cfg.follow_distance_m,
            follow_height_m=cfg.follow_height_m,
            mount_pitch_deg=cfg.mount_pitch_deg,
            gimbal_pitch_deg=gimbal_pitch,
            gimbal_yaw_deg=gimbal_yaw,
        )
        if setpoint is None:
            # No ground intersection (subject above the horizon / no AGL).
            await self._publish_state(
                active=True, lock_state=LOCK_LOCKED, commanding=False
            )
            return

        frame = mavlink_frames.build_position_target(
            lat_deg=setpoint.lat_deg,
            lon_deg=setpoint.lon_deg,
            alt_rel_m=setpoint.alt_rel_m,
            yaw_rad=setpoint.yaw_rad,
        )
        await self._ctx.mavlink.send(frame)

        if cfg.gimbal_point:
            target = setpoint.target
            pitch_deg, yaw_deg = mavlink_frames.gimbal_angles_for_target(
                slant_range_m=target.slant_range_m,
                agl_m=self._pose.rel_alt_m,
                bearing_rad=target.bearing_rad,
                vehicle_yaw_rad=self._pose.yaw,
            )
            gimbal_frame = mavlink_frames.build_gimbal_pitchyaw(
                pitch_deg=pitch_deg, yaw_deg=yaw_deg
            )
            # A missing gimbal component is non-fatal: the FC ignores the
            # command and the body-only follow continues.
            try:
                await self._ctx.mavlink.send(gimbal_frame)
            except Exception as exc:  # noqa: BLE001
                self._ctx.log.info(
                    "follow_me_gimbal_skipped", error=str(exc)
                )

        await self._publish_state(
            active=True,
            lock_state=LOCK_LOCKED,
            commanding=True,
            range_m=setpoint.target.ground_range_m,
            distance_setpoint_m=cfg.follow_distance_m,
            height_setpoint_m=cfg.follow_height_m,
        )

    def _frame_size_for(self, bbox: BoundingBox) -> tuple[int, int]:
        """Frame dimensions the bbox lives in.

        The detection batch does not carry the frame size through the
        SDK callback, so the projection uses the configured capture
        resolution. The bbox is in source-frame pixels, so we infer a
        frame at least as large as the box and default to the common UVC
        640x480 frame otherwise.
        """
        w = max(640, int(bbox.x + bbox.width))
        h = max(480, int(bbox.y + bbox.height))
        return (w, h)

    # -- state read-back ---------------------------------------------

    async def _publish_state(
        self,
        *,
        force: bool = False,
        active: bool | None = None,
        lock_state: str | None = None,
        commanding: bool = False,
        range_m: float | None = None,
        distance_setpoint_m: float | None = None,
        height_setpoint_m: float | None = None,
    ) -> None:
        new = FollowState(
            active=bool(active) if active is not None else False,
            lock_state=lock_state,
            target_id=self._tracker.track_id,
            range_m=range_m,
            distance_setpoint_m=distance_setpoint_m,
            height_setpoint_m=height_setpoint_m,
            commanding=commanding,
        )
        now = time.monotonic()
        heartbeat_due = (now - self._last_heartbeat) >= _STATE_HEARTBEAT_S
        if not force and not new.changed_from(self._published) and not heartbeat_due:
            return
        self._published = new
        self._last_heartbeat = now
        if self._ctx is None:
            return
        await self._ctx.events.publish(FOLLOW_STATE_TOPIC, new.to_dict())


__all__ = ["FollowMePlugin", "get_manifest", "manifest", "PLUGIN_ID"]
