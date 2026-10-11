"""The follow loop: the command gate, the setpoint projection and the
``follow.state`` read-back. A mixin of :class:`follow_me.FollowMePlugin`."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Any

from ados.sdk.tracking import EffectiveLock, LockedTargetTracker
from ados.sdk.vision import BoundingBox

from follow_me import fc_state, mavlink_frames, projection
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

# The fixed cadence the follow loop commands at, decoupled from the
# inference rate so the setpoint stream is smooth regardless of detector
# jitter.
_LOOP_HZ = 6.0
_LOOP_PERIOD = 1.0 / _LOOP_HZ

# A slow heartbeat for the follow.state read-back even when nothing
# changed, so the GCS knows the agent is alive.
_STATE_HEARTBEAT_S = 1.0

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


class FollowLoopMixin:
    """The follow-loop half of the plugin."""

    _ctx: Any
    _pose: fc_state.Pose
    _fc: fc_state.FcState
    _gimbal: fc_state.GimbalState
    _tracker: LockedTargetTracker
    _published: FollowState
    _last_heartbeat: float
    _last_command: _LastCommand | None
    _commanded_gimbal: tuple[float, float] | None
    _last_frame_wh: tuple[int, int] | None

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
        now = fc_state.monotonic()
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
            now = fc_state.monotonic()
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
