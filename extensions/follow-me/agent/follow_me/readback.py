"""Geometry helpers bound to the plugin's cached state, and the
``follow.state`` read-back. A mixin of :class:`follow_me.FollowMePlugin`."""

from __future__ import annotations

import math
from typing import Any

from follow_me import fc_state, projection
from follow_me.camera import CameraGeometry, configured_geometry, geometry_of
from follow_me.loop import SETPOINT_RATE_WINDOW_S
from follow_me.projection import CameraPose, TargetFix
from follow_me.ranging import Scan
from follow_me.state import (
    FOLLOW_STATE_TOPIC,
    PHASE_COASTING,
    PHASE_SEARCHING,
    FollowConfig,
    FollowState,
)
from follow_me.target_filter import position_sigma

# Fallback frame size when no detection batch carried source-frame dimensions.
# The common UVC capture size.
_DEFAULT_FRAME_W = 640
_DEFAULT_FRAME_H = 480

# A planar scan is a range cue only while younger than this (three periods of
# the 10 Hz scan topic).
_SCAN_MAX_AGE_S = 0.3

# A slow heartbeat for the read-back even when nothing changed, so the GCS
# knows the agent is alive; and the fastest it is re-published when only the
# continuous values (range, speed, prediction) moved. A change of phase,
# hold reason, lock or commanding flag is published at once.
_STATE_HEARTBEAT_S = 1.0
_STATE_MIN_PERIOD_S = 0.1
_DISCRETE_KEYS = (
    "active",
    "phase",
    "lock_state",
    "commanding",
    "hold_reason",
    "target_id",
    "reacquire_mode",
    "agl_source",
    "follow_mode",
    "orbit_direction",
    "lease_id",
    "nudged",
    "camera_geometry_source",
)


def _r(v: float | None, nd: int = 2) -> float | None:
    return None if v is None or not math.isfinite(v) else round(v, nd)


class ReadbackMixin:
    """Camera pose, AGL, measurement and read-back for the follow loop."""

    _ctx: Any
    _pose: fc_state.Pose
    _fc: fc_state.FcState
    _gimbal: fc_state.GimbalState
    _rangefinder: fc_state.Rangefinder
    _commanded_gimbal: tuple[float, float] | None
    _last_frame_wh: tuple[int, int] | None
    _scan: Scan | None
    _scan_at: float
    _published: FollowState
    _last_heartbeat: float
    _last_publish_s: float
    _phase: str
    _tracker: Any
    _filter: Any
    _last_fix: TargetFix | None
    _sighting: Any
    _latency_ms: int | None
    _active_camera: str | None
    _setpoint_times: Any
    _lease_id: str | None
    _nudges: Any
    _orbit_phase: float | None
    _geometry: CameraGeometry
    _latest_batch: Any

    def _setpoint_rate_hz(self, now: float) -> float:
        """Setpoints sent per second over the last SETPOINT_RATE_WINDOW_S."""
        times = self._setpoint_times
        while times and (now - times[0]) > SETPOINT_RATE_WINDOW_S:
            times.popleft()
        return round(len(times) / SETPOINT_RATE_WINDOW_S, 1)

    def _frame_size(self) -> tuple[int, int]:
        """The source-frame size the bbox pixels live in: the real dimensions
        the newest sized batch carried, else the common UVC 640x480."""
        if self._last_frame_wh is not None:
            return self._last_frame_wh
        return (_DEFAULT_FRAME_W, _DEFAULT_FRAME_H)

    def _gimbal_angles_deg(
        self, cfg: FollowConfig, now: float
    ) -> tuple[float, float] | None:
        """The camera boresight pitch/yaw (pitch positive = down, yaw positive
        = right of the nose), or ``None`` for a fixed camera whose tilt is the
        configured mount pitch.

        Preference: the FC's CURRENT reported gimbal attitude, then the last
        angle the plugin commanded (only when gimbal pointing is enabled and
        a gimbal has reported at least once), then ``None``. A report that has
        aged out drops to the commanded angle rather than being believed
        indefinitely. With no gimbal the camera is fixed: projecting a box
        through the angle the plugin asked for would aim each measurement at
        the previous estimate, and the estimate then runs away from the
        subject.
        """
        if self._gimbal.report.is_fresh(now):
            return (self._gimbal.pitch_deg, self._gimbal.yaw_deg)
        if cfg.gimbal_point and self._gimbal.present and self._commanded_gimbal is not None:
            return self._commanded_gimbal
        return None

    def _camera_pose(
        self,
        cfg: FollowConfig,
        now: float,
        geometry: CameraGeometry,
        exposed_s: float | None = None,
    ) -> CameraPose:
        """The camera's pose, with the vehicle attitude at ``exposed_s`` (a
        frame's exposure on the flight controller's clock) when given, else
        the latest. A box is a picture of where the subject was when its
        frame was exposed; the vehicle has pitched and yawed since, and
        projecting the box through the newer attitude moves the subject by
        metres."""
        roll, pitch, yaw = (
            self._pose.attitude_at(exposed_s)
            if exposed_s is not None
            else (self._pose.roll, self._pose.pitch, self._pose.yaw)
        )
        # A reported or commanded gimbal angle is the TOTAL orientation
        # relative to the vehicle, so it replaces the fixed mount tilt.
        gimbal = self._gimbal_angles_deg(cfg, now)
        if gimbal is None:
            return CameraPose(
                roll_rad=roll,
                pitch_rad=pitch,
                yaw_rad=yaw,
                mount_pitch_deg=geometry.mount_down_deg,
            )
        return CameraPose(
            roll_rad=roll,
            pitch_rad=pitch,
            yaw_rad=yaw,
            gimbal_pitch_deg=gimbal[0],
            gimbal_yaw_deg=gimbal[1],
        )

    def _agl(self, now: float) -> tuple[float, str]:
        return projection.select_agl(
            rangefinder_usable=self._rangefinder.usable(now),
            rangefinder_m=self._rangefinder.distance_m,
            roll_rad=self._pose.roll,
            pitch_rad=self._pose.pitch,
            baro_agl_m=self._pose.baro_agl_m(now),
        )

    def _fresh_scan(self, now: float) -> Scan | None:
        if self._scan is None or (now - self._scan_at) > _SCAN_MAX_AGE_S:
            return None
        return self._scan

    def _measure_box(
        self,
        cfg: FollowConfig,
        now: float,
        bbox: Any,
        frame_w: int,
        frame_h: int,
        geometry: CameraGeometry,
        exposed_s: float | None = None,
    ) -> TargetFix | None:
        agl, source = self._agl(now)
        return projection.measure_target(
            bbox_x=float(bbox.x),
            bbox_y=float(bbox.y),
            bbox_w=float(bbox.width),
            bbox_h=float(bbox.height),
            frame_width=frame_w,
            frame_height=frame_h,
            horizontal_fov_deg=geometry.hfov_deg,
            cam=self._camera_pose(cfg, now, geometry, exposed_s),
            agl_m=agl,
            agl_source=source,
            scan=self._fresh_scan(now),
        )

    def _predicted_bbox(
        self, cfg: FollowConfig, now: float, x: list[float]
    ) -> dict[str, object] | None:
        sighting = self._sighting
        frame_w, frame_h = (
            sighting.frame_wh
            if sighting is not None and sighting.frame_wh
            else self._frame_size()
        )
        geometry = sighting.geometry if sighting is not None else self._geometry
        box = projection.predict_bbox(
            offset_north_m=x[0] - self._pose.north_m,
            offset_east_m=x[1] - self._pose.east_m,
            agl_m=self._agl(now)[0],
            cam=self._camera_pose(cfg, now, geometry),
            frame_width=frame_w,
            frame_height=frame_h,
            horizontal_fov_deg=geometry.hfov_deg,
        )
        if box is None:
            return None
        camera_id = (
            self._sighting.camera_id
            if self._sighting is not None
            else (self._active_camera or "")
        )
        return {
            "camera_id": camera_id,
            "frame_w": frame_w,
            "frame_h": frame_h,
            "x": round(box[0], 1),
            "y": round(box[1], 1),
            "w": round(box[2], 1),
            "h": round(box[3], 1),
        }

    async def _publish_state(
        self,
        cfg: FollowConfig | None,
        *,
        now: float | None = None,
        active: bool = False,
        phase: str,
        lock_state: str | None = None,
        commanding: bool = False,
        hold_reason: str | None = None,
        coast_remaining_s: float = 0.0,
        predicted_bbox: dict[str, object] | None = None,
        force: bool = False,
    ) -> None:
        # Callers inside a tick pass that tick's clock so the published FC
        # flags are aged against the same instant the command gate used.
        if now is None:
            now = fc_state.monotonic()
        self._phase = phase
        new = self._build_state(
            cfg,
            now,
            active=active,
            phase=phase,
            lock_state=lock_state,
            commanding=commanding,
            hold_reason=hold_reason,
            coast_remaining_s=coast_remaining_s,
            predicted_bbox=predicted_bbox,
        )
        prev = self._published.to_dict()
        cur = new.to_dict()
        discrete_changed = any(prev[k] != cur[k] for k in _DISCRETE_KEYS)
        heartbeat_due = (now - self._last_heartbeat) >= _STATE_HEARTBEAT_S
        continuous_due = cur != prev and (now - self._last_publish_s) >= _STATE_MIN_PERIOD_S
        if not (force or discrete_changed or heartbeat_due or continuous_due):
            return
        self._published = new
        self._last_publish_s = now
        self._last_heartbeat = now
        if self._ctx is None:
            return
        await self._ctx.events.publish(FOLLOW_STATE_TOPIC, cur)

    def _build_state(
        self,
        cfg: FollowConfig | None,
        now: float,
        *,
        active: bool,
        phase: str,
        lock_state: str | None,
        commanding: bool,
        hold_reason: str | None,
        coast_remaining_s: float,
        predicted_bbox: dict[str, object] | None,
    ) -> FollowState:
        state = FollowState(
            active=active,
            phase=phase,
            lock_state=lock_state,
            target_id=self._tracker.track_id,
            track_id=self._tracker.track_id,
            commanding=commanding,
            # Aged, not remembered: once the HEARTBEAT is stale the read-back
            # stops asserting an arm/mode state nothing has confirmed.
            fc_armed=self._fc.armed_at(now),
            fc_guided=self._fc.guided_at(now),
            hold_reason=None if commanding else hold_reason,
            coast_remaining_s=round(coast_remaining_s, 2),
            predicted_bbox=predicted_bbox,
            pipeline_latency_ms=self._latency_ms,
            setpoint_rate_hz=self._setpoint_rate_hz(now),
        )
        state.lease_id = self._lease_id
        state.nudged = self._nudges.nudged
        if self._orbit_phase is not None:
            state.orbit_phase_deg = _r(math.degrees(self._orbit_phase) % 360.0, 1)
        # Before the first batch, and after a settings change, the settings
        # (or the default) are what the next frame will be projected with.
        geometry = (
            self._geometry
            if cfg is None
            else geometry_of(self._latest_batch, configured_geometry(cfg))
        )
        state.camera_hfov_deg = _r(geometry.hfov_deg, 1)
        state.camera_mount_pitch_deg = _r(geometry.mount_pitch_deg, 1)
        state.camera_geometry_source = geometry.source
        if cfg is not None:
            state.reacquire_mode = cfg.reacquire_mode
            state.follow_mode = cfg.follow_mode
            state.follow_angle_deg = _r(cfg.follow_angle_deg, 1)
            state.orbit_radius_m = _r(cfg.orbit_radius_m)
            state.orbit_rate_dps = _r(cfg.orbit_rate_dps, 1)
            state.orbit_direction = cfg.orbit_direction
            if commanding:
                # In orbit the flown geometry is the orbit radius (already
                # published); the follow distance is not a setpoint there.
                if cfg.follow_mode != "orbit":
                    state.distance_setpoint_m = cfg.follow_distance_m
                state.height_setpoint_m = cfg.follow_height_m
        if self._sighting is not None and self._tracker.has_lock:
            state.confidence = _r(float(self._sighting.confidence), 3)
        if self._pose.attitude.is_fresh(now):
            agl, source = self._agl(now)
            state.agl_m = _r(agl)
            state.agl_source = source
        pred = self._filter.predicted(now) if self._filter.initialized else None
        if pred is not None:
            x, p = pred
            state.target_ned = [round(x[0], 2), round(x[1], 2)]
            state.target_vel_ned = [round(x[2], 2), round(x[3], 2)]
            state.target_speed_mps = _r(math.hypot(x[2], x[3]))
        fix = self._last_fix
        if phase in (PHASE_COASTING, PHASE_SEARCHING) and pred is not None:
            x, p = pred
            dn = x[0] - self._pose.north_m
            de = x[1] - self._pose.east_m
            state.range_m = _r(math.hypot(dn, de))
            state.range_sigma_m = _r(position_sigma(p))
            state.bearing_deg = _r(math.degrees(math.atan2(de, dn)) % 360.0, 1)
        elif fix is not None:
            state.range_m = _r(fix.range_m)
            state.range_sigma_m = _r(fix.range_sigma_m)
            state.range_cues = [c.to_dict() for c in fix.cues]
            state.bearing_deg = _r(math.degrees(fix.bearing_rad) % 360.0, 1)
            state.depression_deg = _r(math.degrees(fix.depression_rad), 1)
        return state


__all__ = ["ReadbackMixin"]
