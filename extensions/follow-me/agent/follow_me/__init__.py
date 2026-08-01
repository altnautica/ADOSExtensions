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
from dataclasses import dataclass, replace
from typing import Any

from ados.plugins.manifest import (
    AgentBlock,
    Compatibility,
    PluginManifest,
)
from ados.sdk.cameras import CAMERA_SELECTOR_AUTO
from ados.sdk.tracking import EffectiveLock, LockedTargetTracker
from ados.sdk.vision import BoundingBox, DetectionBatch

from follow_me import mavlink_frames, projection
from follow_me.freshness import FreshnessGate
from follow_me.state import (
    FOLLOW_STATE_TOPIC,
    HOLD_FC_DISARMED,
    HOLD_FC_NOT_GUIDED,
    HOLD_FC_STALE,
    HOLD_INACTIVE,
    HOLD_LOCK_LOST,
    HOLD_LOCK_UNCERTAIN,
    HOLD_NO_GROUND_FIX,
    HOLD_NO_LOCK,
    HOLD_POSE_STALE,
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

# Freshness windows for the flight-controller telemetry the loop projects
# through. Every one of these inputs can stop arriving while the write path
# still works — a stream-rate change, a lagged broadcast receiver, a router
# reconnect, an FC that simply stops emitting one message — so each is aged
# rather than latched, and each gets a window sized from the cadence that
# message is actually expected at. One shared window would be wrong for all
# three: they are requested at 10 Hz, 5 Hz and 1 Hz respectively.
#
# Pose. The agent's MAVLink router requests ATTITUDE at 10 Hz and
# GLOBAL_POSITION_INT at 5 Hz, so the slower of the two nominally arrives
# every 0.2 s. 0.5 s is three follow-loop periods (the loop runs at 6 Hz)
# and two and a half position periods: one dropped message plus link jitter
# still reads current, while a real stall is caught within about three
# ticks. Deliberately much tighter than the 1.5 s detection coast window,
# because a frozen pose corrupts the ORIGIN and heading the projection
# measures from, which grows the setpoint error without bound as the
# aircraft flies on. A frozen bounding box only misplaces the subject
# within the frame, and the loop already holds its last setpoint for that.
_POSE_MAX_AGE_S = 0.5

# Flight-controller arm + mode. HEARTBEAT is requested at 1 Hz, so a window
# sized like the pose one would report stale between every message. Three
# seconds is three missed heartbeats — the same "the link is gone" rule of
# thumb ground stations apply to a MAVLink system.
_FC_MAX_AGE_S = 3.0

# Gimbal boresight. MOUNT_ORIENTATION is NOT one of the streams the router
# requests, so its cadence is whatever the firmware happens to emit; this
# window is correspondingly generous. Expiring here is benign rather than a
# stop: the projection falls back to the angle the plugin last commanded the
# gimbal to, which is a better estimate than an attitude report old enough
# to predate the current aim.
_GIMBAL_MAX_AGE_S = 2.0

# A slow heartbeat for the follow.state read-back even when nothing
# changed, so the GCS knows the agent is alive.
_STATE_HEARTBEAT_S = 1.0

# How often the full per-drone config key set is re-read. The follow loop
# runs faster than this; between refreshes only the live ``active`` toggle
# is re-read each tick so arm/disarm stays responsive, while the rarely-
# changed geometry keys are served from cache instead of an IPC round-trip
# per key on every loop.
_CONFIG_REFRESH_S = 1.0

# HEARTBEAT decode. MAV_MODE_FLAG_SAFETY_ARMED is bit 7 of base_mode.
_BASE_MODE_ARMED = 0x80
# MAV_AUTOPILOT ids we can decode a guided/offboard mode for.
_AP_ARDUPILOTMEGA = 3
_AP_PX4 = 12
# ArduPilot Copter custom_mode values that accept companion position
# setpoints: GUIDED and GUIDED_NOGPS.
_ARDUCOPTER_GUIDED = 4
_ARDUCOPTER_GUIDED_NOGPS = 20
# PX4 packs the main mode into bits 16-23 of custom_mode; OFFBOARD is 6.
_PX4_MAIN_MODE_OFFBOARD = 6

# Fallback frame size when a detection batch carries no source-frame
# dimensions (a pre-frame-size agent). The common UVC capture size.
_DEFAULT_FRAME_W = 640
_DEFAULT_FRAME_H = 480


@dataclass
class _LastCommand:
    """The last follow command emitted, retained so the loop can HOLD it while
    coasting on a frozen bounding box instead of re-projecting stale image data
    through fresh vehicle attitude."""

    bbox: Any
    position_setpoint: dict[str, Any]
    gimbal_frame: bytes | None
    range_m: float
    distance_setpoint_m: float
    height_setpoint_m: float


def _monotonic() -> float:
    """The one clock every freshness mark and window check in this plugin
    reads. Routed through a single function so the whole staleness surface
    can be driven from a test without sleeping through real windows."""
    return time.monotonic()


def _is_guided_mode(autopilot: int, custom_mode: int) -> bool:
    """Whether ``custom_mode`` is a guided/offboard mode for ``autopilot``.

    Firmware-aware: ArduPilot Copter GUIDED / GUIDED_NOGPS, PX4 OFFBOARD. An
    autopilot we do not decode returns False so the follow loop reports
    ``fc_guided`` honestly rather than claiming to command an FC that would
    ignore the setpoints.
    """
    if autopilot == _AP_PX4:
        return ((custom_mode >> 16) & 0xFF) == _PX4_MAIN_MODE_OFFBOARD
    if autopilot == _AP_ARDUPILOTMEGA:
        return custom_mode in (_ARDUCOPTER_GUIDED, _ARDUCOPTER_GUIDED_NOGPS)
    return False


def get_manifest() -> PluginManifest:
    """In-code manifest mirror (the packed archive ships manifest.yaml)."""
    return PluginManifest(
        schema_version=3,
        id=PLUGIN_ID,
        version="0.2.5",
        name="ADOS Follow-Me",
        description=(
            "Locks onto an operator-designated subject and flies a "
            "fixed-distance standoff follow from the companion."
        ),
        author="Altnautica",
        license="GPL-3.0-or-later",
        risk="high",
        compatibility=Compatibility(ados_version=">=0.99.180"),
        agent=AgentBlock(
            entrypoint="follow_me:FollowMePlugin",
            isolation="subprocess",
            permissions=[
                "vision.detection.subscribe",
                "mavlink.read",
                "mavlink.write",
                "flight.guided_setpoint",
                "event.publish",
                "event.subscribe",
                "mcp.expose",
            ],
        ),
    )


manifest = get_manifest()


class _Pose:
    """Latest FC pose cached from the MAVLink telemetry stream.

    Attitude and position arrive on two different messages, at two different
    rates, and can stop independently — an FC can keep emitting ATTITUDE long
    after GLOBAL_POSITION_INT dries up. They therefore carry two separate
    freshness gates rather than one shared mark, which whichever message was
    still flowing would keep refreshing, hiding the partial stall entirely.
    """

    __slots__ = (
        "roll",
        "pitch",
        "yaw",
        "lat_deg",
        "lon_deg",
        "rel_alt_m",
        "attitude",
        "position",
    )

    def __init__(self, max_age_s: float = _POSE_MAX_AGE_S) -> None:
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.lat_deg = 0.0
        self.lon_deg = 0.0
        self.rel_alt_m = 0.0
        self.attitude = FreshnessGate(max_age_s)
        self.position = FreshnessGate(max_age_s)

    def ready(self, now_monotonic_s: float) -> bool:
        """Whether BOTH halves of the pose are current enough to project
        through. Not "have we ever had a pose" — a pose the aircraft has
        since flown away from is worse than no pose, because it looks
        usable."""
        return self.attitude.is_fresh(now_monotonic_s) and self.position.is_fresh(
            now_monotonic_s
        )


class _FcState:
    """Latest flight-controller arm + mode cached from HEARTBEAT.

    ``guided`` is True only when the autopilot is positively identified
    (ArduPilot or PX4) AND reports a mode that accepts companion position
    setpoints. An unknown autopilot or a non-guided mode leaves it False so
    the follow loop never claims to command an FC that would ignore it.

    Both bits are readings, not facts: they describe the FC as of the last
    HEARTBEAT. Once that heartbeat ages out they are no longer evidence of
    anything, so the accessors below report False rather than continuing to
    assert an arm state nothing has confirmed for seconds.
    """

    __slots__ = ("armed", "guided", "heartbeat")

    def __init__(self, max_age_s: float = _FC_MAX_AGE_S) -> None:
        self.armed = False
        self.guided = False
        self.heartbeat = FreshnessGate(max_age_s)

    def is_fresh(self, now_monotonic_s: float) -> bool:
        return self.heartbeat.is_fresh(now_monotonic_s)

    def armed_at(self, now_monotonic_s: float) -> bool:
        return self.armed and self.heartbeat.is_fresh(now_monotonic_s)

    def guided_at(self, now_monotonic_s: float) -> bool:
        return self.guided and self.heartbeat.is_fresh(now_monotonic_s)


class _GimbalState:
    """Gimbal boresight attitude reported by the FC, in the PROJECTION's
    convention (pitch degrees positive = down, yaw degrees positive = right of
    the nose). MOUNT_ORIENTATION reports pitch negative = down, so the handler
    negates it on the way in.

    The report is only the boresight while it is current; a report old enough
    to predate the gimbal's current aim is a worse estimate than the angle the
    plugin itself last commanded. The gate therefore decays back to that
    fallback (and then to the fixed mount) instead of latching."""

    __slots__ = ("pitch_deg", "yaw_deg", "report")

    def __init__(self, max_age_s: float = _GIMBAL_MAX_AGE_S) -> None:
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.report = FreshnessGate(max_age_s)


class FollowMePlugin:
    """Lifecycle-hook plugin class the runner instantiates with no args."""

    def __init__(self) -> None:
        self._ctx: Any = None
        self._pose = _Pose()
        self._fc = _FcState()
        self._gimbal = _GimbalState()
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
        self._cfg_read_at = _monotonic()
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
        now = _monotonic()
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

    # -- MAVLink pose handlers ---------------------------------------

    def _on_heartbeat(self, msg: dict[str, Any]) -> None:
        base_mode = int(msg.get("base_mode", 0) or 0)
        custom_mode = int(msg.get("custom_mode", 0) or 0)
        autopilot = int(msg.get("autopilot", 0) or 0)
        self._fc.armed = bool(base_mode & _BASE_MODE_ARMED)
        self._fc.guided = _is_guided_mode(autopilot, custom_mode)
        self._fc.heartbeat.mark(_monotonic())

    def _on_mount_orientation(self, msg: dict[str, Any]) -> None:
        # MOUNT_ORIENTATION carries degrees: pitch negative = down, yaw
        # relative to the vehicle heading (positive to the right). The
        # projection wants pitch positive = down, so negate it; the yaw
        # convention already matches.
        pitch = msg.get("pitch")
        yaw = msg.get("yaw")
        if pitch is None or yaw is None:
            return
        self._gimbal.pitch_deg = -float(pitch)
        self._gimbal.yaw_deg = float(yaw)
        self._gimbal.report.mark(_monotonic())

    def _on_attitude(self, msg: dict[str, Any]) -> None:
        self._pose.roll = float(msg.get("roll", 0.0))
        self._pose.pitch = float(msg.get("pitch", 0.0))
        self._pose.yaw = float(msg.get("yaw", 0.0))
        # Marked separately from position: ATTITUDE and GLOBAL_POSITION_INT
        # are different messages at different rates and either can stall on
        # its own.
        self._pose.attitude.mark(_monotonic())

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
        self._pose.position.mark(_monotonic())

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
        self._tracker.record(batch, _monotonic())

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
        # One clock for the whole tick, so every freshness question is asked
        # about the same instant and the read-back cannot disagree with the
        # gate that produced it.
        now = _monotonic()
        cfg = await self._config_for_tick()

        if not cfg.active or not self._tracker.has_lock:
            await self._publish_state(
                now=now,
                active=cfg.active,
                commanding=False,
                hold_reason=HOLD_INACTIVE if not cfg.active else HOLD_NO_LOCK,
            )
            return

        lock = self._tracker.effective_lock(now)

        if lock is EffectiveLock.LOST:
            # Drop the lock entirely; require a fresh operator designate. The
            # held setpoint goes with it, so a re-lock recomputes from scratch.
            self._tracker.drop()
            self._last_command = None
            await self._publish_state(
                now=now,
                active=True,
                lock_state=LOCK_LOST,
                commanding=False,
                hold_reason=HOLD_LOCK_LOST,
                force=True,
            )
            return

        target = self._tracker.locked_target(now)
        if target is None:
            # uncertain (or coasting): hold, stop commanding.
            await self._publish_state(
                now=now,
                active=True,
                lock_state=lock.value,
                commanding=False,
                hold_reason=HOLD_LOCK_UNCERTAIN,
            )
            return

        if not self._pose.ready(now):
            # The subject is locked and the write path may well still work,
            # but the pose the projection measures from is either absent or
            # no longer current. Commanding here would fly the aircraft to a
            # point computed from where it used to be, with the error growing
            # for as long as the telemetry stays stalled. Hold, and say why —
            # silence here reads identical to a healthy pre-arm hold.
            await self._publish_state(
                now=now,
                active=True,
                lock_state=LOCK_LOCKED,
                commanding=False,
                hold_reason=HOLD_POSE_STALE,
            )
            return

        if not self._fc.is_fresh(now):
            # The HEARTBEAT stopped. Whatever the FC was doing when it last
            # reported, nothing confirms it is still armed or still in a mode
            # that accepts setpoints.
            await self._publish_state(
                now=now,
                active=True,
                lock_state=LOCK_LOCKED,
                commanding=False,
                hold_reason=HOLD_FC_STALE,
            )
            return

        if not (self._fc.armed and self._fc.guided):
            # Locked with a usable pose and a live heartbeat, but the FC is not
            # armed and in a guided/offboard mode, so a position setpoint would
            # be ignored. Report honestly and do not command.
            await self._publish_state(
                now=now,
                active=True,
                lock_state=LOCK_LOCKED,
                commanding=False,
                hold_reason=(
                    HOLD_FC_DISARMED if not self._fc.armed else HOLD_FC_NOT_GUIDED
                ),
            )
            return

        await self._command_follow(cfg, target.bbox, now)

    async def _command_follow(
        self, cfg: FollowConfig, bbox: BoundingBox, now: float
    ) -> None:
        # Coast hold: while the tracker holds LOCKED through the coast window it
        # returns the SAME frozen bbox object from the last sighting. Re-
        # projecting that stale bbox through fresh vehicle attitude each tick
        # would drift the world setpoint on stale image data, so hold (re-send)
        # the last commanded setpoint until a fresh detection replaces the bbox.
        if self._last_command is not None and bbox is self._last_command.bbox:
            await self._resend_last_command(now)
            return

        frame_w, frame_h = self._frame_size()

        # Which pitch/yaw the camera boresight is actually at. A reported or
        # commanded gimbal angle is the TOTAL orientation relative to the
        # vehicle, so it replaces the fixed mount tilt rather than adding to it.
        gimbal = self._gimbal_angles_deg(cfg, now)
        if gimbal is None:
            mount_pitch_deg = cfg.mount_pitch_deg
            gimbal_pitch_deg = 0.0
            gimbal_yaw_deg = 0.0
        else:
            mount_pitch_deg = 0.0
            gimbal_pitch_deg, gimbal_yaw_deg = gimbal

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
            mount_pitch_deg=mount_pitch_deg,
            gimbal_pitch_deg=gimbal_pitch_deg,
            gimbal_yaw_deg=gimbal_yaw_deg,
        )
        if setpoint is None:
            # No ground intersection (subject above the horizon / no AGL).
            await self._publish_state(
                now=now,
                active=True,
                lock_state=LOCK_LOCKED,
                commanding=False,
                hold_reason=HOLD_NO_GROUND_FIX,
            )
            return

        position_setpoint = mavlink_frames.build_position_setpoint(
            lat_deg=setpoint.lat_deg,
            lon_deg=setpoint.lon_deg,
            alt_rel_m=setpoint.alt_rel_m,
            yaw_rad=setpoint.yaw_rad,
        )
        await self._ctx.flight.guided_setpoint(**position_setpoint)

        gimbal_frame: bytes | None = None
        if cfg.gimbal_point:
            target = setpoint.target
            pitch_deg, yaw_deg = mavlink_frames.gimbal_angles_for_target(
                slant_range_m=target.slant_range_m,
                agl_m=self._pose.rel_alt_m,
                bearing_rad=target.bearing_rad,
                vehicle_yaw_rad=self._pose.yaw,
            )
            # Cache the commanded angle (projection convention: pitch positive =
            # down, so negate the negative-down gimbal command) as the boresight
            # estimate until the FC reports a real attitude.
            self._commanded_gimbal = (-pitch_deg, yaw_deg)
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
                gimbal_frame = None

        self._last_command = _LastCommand(
            bbox=bbox,
            position_setpoint=position_setpoint,
            gimbal_frame=gimbal_frame,
            range_m=setpoint.target.ground_range_m,
            distance_setpoint_m=cfg.follow_distance_m,
            height_setpoint_m=cfg.follow_height_m,
        )
        await self._publish_state(
            now=now,
            active=True,
            lock_state=LOCK_LOCKED,
            commanding=True,
            range_m=setpoint.target.ground_range_m,
            distance_setpoint_m=cfg.follow_distance_m,
            height_setpoint_m=cfg.follow_height_m,
        )

    async def _resend_last_command(self, now: float) -> None:
        """Re-send the last commanded setpoint (and gimbal frame) unchanged,
        holding position while coasting on a frozen bbox."""
        cmd = self._last_command
        if cmd is None:
            return
        await self._ctx.flight.guided_setpoint(**cmd.position_setpoint)
        if cmd.gimbal_frame is not None:
            try:
                await self._ctx.mavlink.send(cmd.gimbal_frame)
            except Exception as exc:  # noqa: BLE001
                self._ctx.log.info("follow_me_gimbal_skipped", error=str(exc))
        await self._publish_state(
            now=now,
            active=True,
            lock_state=LOCK_LOCKED,
            commanding=True,
            range_m=cmd.range_m,
            distance_setpoint_m=cmd.distance_setpoint_m,
            height_setpoint_m=cmd.height_setpoint_m,
        )

    def _frame_size(self) -> tuple[int, int]:
        """The source-frame size the bbox pixels live in.

        Uses the real dimensions the detection batch carried (cached in
        _on_batch); falls back to the common UVC 640x480 frame only when a
        batch arrived without them (a pre-frame-size agent).
        """
        if self._last_frame_wh is not None:
            return self._last_frame_wh
        return (_DEFAULT_FRAME_W, _DEFAULT_FRAME_H)

    def _gimbal_angles_deg(
        self, cfg: FollowConfig, now: float
    ) -> tuple[float, float] | None:
        """The camera boresight pitch/yaw (projection convention: pitch
        positive = down, yaw positive = right of the nose), or ``None`` for a
        fixed camera whose tilt is the configured mount pitch.

        Preference: the FC's CURRENT reported gimbal attitude, then the last
        angle the plugin commanded the gimbal to (only when gimbal pointing is
        enabled), then ``None`` (no gimbal in play). A report that has aged out
        drops to the commanded angle rather than being believed indefinitely:
        once MOUNT_ORIENTATION stops arriving, the angle the plugin asked for
        tracks the gimbal's real aim better than a report that predates it.
        """
        if self._gimbal.report.is_fresh(now):
            return (self._gimbal.pitch_deg, self._gimbal.yaw_deg)
        if cfg.gimbal_point and self._commanded_gimbal is not None:
            return self._commanded_gimbal
        return None

    # -- state read-back ---------------------------------------------

    async def _publish_state(
        self,
        *,
        force: bool = False,
        now: float | None = None,
        active: bool | None = None,
        lock_state: str | None = None,
        commanding: bool = False,
        hold_reason: str | None = None,
        range_m: float | None = None,
        distance_setpoint_m: float | None = None,
        height_setpoint_m: float | None = None,
    ) -> None:
        # Callers inside a tick pass that tick's clock so the published FC
        # flags are aged against the same instant the command gate used.
        if now is None:
            now = _monotonic()
        new = FollowState(
            active=bool(active) if active is not None else False,
            lock_state=lock_state,
            target_id=self._tracker.track_id,
            range_m=range_m,
            distance_setpoint_m=distance_setpoint_m,
            height_setpoint_m=height_setpoint_m,
            commanding=commanding,
            # Aged, not remembered: once the HEARTBEAT these came from is stale
            # the read-back stops asserting an arm/mode state nothing has
            # confirmed. hold_reason carries "fc-stale" so the operator can
            # tell a lost heartbeat from a genuinely disarmed aircraft.
            fc_armed=self._fc.armed_at(now),
            fc_guided=self._fc.guided_at(now),
            hold_reason=None if commanding else hold_reason,
        )
        heartbeat_due = (now - self._last_heartbeat) >= _STATE_HEARTBEAT_S
        if not force and not new.changed_from(self._published) and not heartbeat_due:
            return
        self._published = new
        self._last_heartbeat = now
        if self._ctx is None:
            return
        await self._ctx.events.publish(FOLLOW_STATE_TOPIC, new.to_dict())


__all__ = ["FollowMePlugin", "get_manifest", "manifest", "PLUGIN_ID"]
