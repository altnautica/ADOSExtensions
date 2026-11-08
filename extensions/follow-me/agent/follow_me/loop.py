"""The follow loop: the command gate, lock loss and re-acquire, and the
goals sent to the pilot. A mixin of :class:`follow_me.FollowMePlugin`.

Every tick (20 Hz) decides one of three things:

* **follow**: a live engine lock, a current pose, a live armed flight
  controller, a recent range fix and the pilot's motion lease. The subject
  filter is updated from each new box and a ``follow`` or ``orbit`` goal
  (position, velocity feed-forward, yaw, the subject and the detour ring the
  pilot's planner searches) is sent to the pilot.
* **coast** (``smart`` re-acquire only): the lock was lost; the goal is the
  predicted one at half feed-forward for ``reacquire_window_s`` while looking
  for the subject in each new batch.
* **hold**: anything else. While the lease is held a ``position`` goal at the
  vehicle's current position keeps the pilot holding there; when the skill
  goes inactive the lease is released and the pilot brakes to its own hold.

The pilot revokes the lease on a pause, a stick takeover, land, RTH, an FC
mode change or a dead plugin; the follow then switches itself off
(``active=false``) and announces ``follow.stopped``.
"""

from __future__ import annotations

import asyncio
import math
from collections import deque
from dataclasses import dataclass, replace
from typing import Any

from ados.sdk.tracking import EffectiveLock, LockedTargetTracker

from follow_me import fc_state, mavlink_frames, setpoint
from follow_me.motion import LEASE_RETRY_S, SKILL_ID, Nudges, lease_error_reason
from follow_me.projection import TargetFix
from follow_me.reacquire import Coast, Embedding, blend_template
from follow_me.state import (
    FOLLOW_MODE_ORBIT,
    FOLLOW_STOPPED_TOPIC,
    HOLD_FC_DISARMED,
    HOLD_FC_STALE,
    HOLD_INACTIVE,
    HOLD_LEASE_BUSY,
    HOLD_LEASE_REFUSED,
    HOLD_LOCK_UNCERTAIN,
    HOLD_NO_LOCK,
    HOLD_NO_RANGE,
    HOLD_POSE_STALE,
    LOCK_LOCKED,
    PHASE_ACQUIRING,
    PHASE_FOLLOWING,
    PHASE_HOLDING,
    PHASE_IDLE,
    PHASE_LOST,
    FollowConfig,
)
from follow_me.target_filter import TargetFilter, subject_max_speed_mps

# The fixed cadence the follow loop commands at, decoupled from the
# inference rate so the setpoint stream is smooth regardless of detector
# jitter; the filter predicts between boxes.
_LOOP_HZ = 20.0
_LOOP_PERIOD = 1.0 / _LOOP_HZ

# With no new range fix for this long the prediction is no longer trusted
# enough to command on a live lock.
_MEASUREMENT_STALE_S = 1.0
# The camera is turned toward the latest sighting's measured bearing while
# that sighting is at most this old (through a short unsure spell too);
# after that the estimate's bearing takes over.
_SIGHTING_YAW_S = 3.0
# The follow ring the pilot's planner searches for detour goals that keep
# the subject in view: ±90° around the follow angle, ±60° around the orbit
# phase.
_FOLLOW_SPAN_DEG = 90.0
_ORBIT_SPAN_DEG = 60.0
# Gimbal pointing is re-sent at most this often.
_GIMBAL_PERIOD_S = 0.2
# Detection-to-setpoint latencies outside this are clock mismatches, not
# measurements, and are not reported.
_MAX_LATENCY_MS = 10_000.0
# Window the published goal send rate is measured over.
SETPOINT_RATE_WINDOW_S = 2.0


@dataclass(frozen=True)
class Sighting:
    """The engine-designated detection from the newest batch, with what the
    shared lock gate does not keep."""

    track_id: int
    bbox: Any
    camera_id: str
    frame_wh: tuple[int, int] | None
    ts_ms: int
    received_s: float
    confidence: float
    class_label: str
    embedding: Embedding | None


class FollowLoopMixin:
    """The follow-loop half of the plugin."""

    _ctx: Any
    _pose: fc_state.Pose
    _fc: fc_state.FcState
    _rangefinder: fc_state.Rangefinder
    _tracker: LockedTargetTracker
    _filter: TargetFilter
    _filter_track_id: int | None
    _template: Embedding | None
    _sighting: Sighting | None
    _measured_bbox: Any
    _last_fix: TargetFix | None
    _last_fix_s: float
    _coast: Coast | None
    _phase: str
    # The pilot's lease id as it sent it (pilot.v1: an unsigned integer),
    # passed back unchanged to goal_send and lease_release.
    _lease_id: int | str | None
    _lease_tried_s: float
    _lease_refusal: str | None
    _revoked_reason: str | None
    _nudges: Nudges
    _ref_heading: float | None
    _orbit_phase: float | None
    _geom_mode: str | None
    _geom_tick_s: float | None
    _pending_latency_ts_ms: int | None
    _latency_ms: int | None
    _last_gimbal_s: float
    _setpoint_times: deque[float]
    _last_lost_track_id: int | None
    _latest_batch: Any
    _latest_batch_s: float
    _commanded_gimbal: tuple[float, float] | None

    async def _follow_loop(self) -> None:
        while True:
            await asyncio.sleep(_LOOP_PERIOD)
            try:
                await self._housekeeping(fc_state.monotonic())
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
        if self._revoked_reason is not None:
            await self._on_revoked_stop(now)
        cfg = await self._config_for_tick()
        if not cfg.active:
            self._coast = None
            await self._release_lease()
            self._nudges.reset_session()
            await self._hold(
                cfg, now, phase=PHASE_IDLE, hold_reason=HOLD_INACTIVE, active=False
            )
            return
        await self._nudges.write_back(self._ctx, now)
        if self._coast is not None:
            await self._tick_coast(cfg, now)
            return
        await self._tick_follow(cfg, now)

    def _gate(self, now: float) -> str | None:
        """The hold reason the vehicle side imposes, or ``None`` to command."""
        if not self._pose.ready(now):
            # A pose the aircraft has since flown away from would place every
            # setpoint relative to where it used to be.
            return HOLD_POSE_STALE
        if not self._fc.is_fresh(now):
            return HOLD_FC_STALE
        if not self._fc.armed:
            return HOLD_FC_DISARMED
        return None

    async def _tick_follow(self, cfg: FollowConfig, now: float) -> None:
        if not self._tracker.has_lock:
            phase = PHASE_LOST if self._phase == PHASE_LOST else PHASE_ACQUIRING
            await self._hold(cfg, now, phase=phase, hold_reason=HOLD_NO_LOCK)
            return
        lock = self._tracker.effective_lock(now)
        if lock is EffectiveLock.LOST:
            await self._on_lock_lost(cfg, now)
            return
        target = self._tracker.locked_target(now)
        if target is None:
            # Unsure: hold, keep the camera on the latest sighting, and stop
            # the estimate where it is rather than carry its velocity on.
            self._filter.stop(now)
            await self._hold(
                cfg,
                now,
                phase=PHASE_HOLDING,
                hold_reason=HOLD_LOCK_UNCERTAIN,
                lock_state=lock.value,
                yaw_rad=self._sighted_bearing(now),
            )
            return
        if self._filter_track_id != target.track_id:
            # A different subject (a fresh operator designate): new track.
            self._reset_track(target.track_id)
        gate = self._gate(now)
        if gate is not None:
            await self._hold(
                cfg, now, phase=PHASE_HOLDING, hold_reason=gate, lock_state=LOCK_LOCKED
            )
            return
        # While the gate coasts on a frozen box (no new sighting) the box is
        # the same object; it is never re-projected through fresh attitude.
        if target.bbox is not self._measured_bbox:
            self._measure_sighting(cfg, now, target.bbox)
        if (
            not self._filter.initialized
            or (now - self._filter.last_update_s) > _MEASUREMENT_STALE_S
        ):
            await self._hold(
                cfg,
                now,
                phase=PHASE_HOLDING if self._filter.initialized else PHASE_ACQUIRING,
                hold_reason=HOLD_NO_RANGE,
                lock_state=LOCK_LOCKED,
            )
            return
        await self._command(
            cfg, now, phase=PHASE_FOLLOWING, ff_scale=1.0, lock_state=LOCK_LOCKED
        )

    def _reset_track(self, track_id: int | None) -> None:
        self._filter.reset()
        self._filter_track_id = track_id
        self._template = None
        self._measured_bbox = None
        self._last_fix = None
        self._last_lost_track_id = None
        self._latency_ms = None
        self._pending_latency_ts_ms = None
        self._ref_heading = None
        self._orbit_phase = None

    def _measure_sighting(self, cfg: FollowConfig, now: float, bbox: Any) -> None:
        self._measured_bbox = bbox
        s = self._sighting if (self._sighting and self._sighting.bbox is bbox) else None
        frame_w, frame_h = s.frame_wh if (s and s.frame_wh) else self._frame_size()
        # The box is projected through the attitude its frame was exposed at.
        exposed_s = s.ts_ms / 1000.0 if (s and s.ts_ms > 0) else None
        fix = self._measure_box(cfg, now, bbox, frame_w, frame_h, exposed_s)
        if fix is None:
            return
        if s is not None:
            self._filter.max_speed_mps = subject_max_speed_mps(s.class_label)
        self._filter.update(
            s.received_s if s else now,
            self._pose.north_m + fix.offset_north_m,
            self._pose.east_m + fix.offset_east_m,
            fix.covariance(),
        )
        self._last_fix = fix
        self._last_fix_s = now
        if s is not None:
            if s.ts_ms > 0:
                self._pending_latency_ts_ms = s.ts_ms
            if s.embedding is not None:
                self._template = blend_template(self._template, s.embedding)

    def _sighted_bearing(self, now: float) -> float | None:
        """The bearing the latest sighting measured (through the attitude its
        frame was exposed at), while it is recent enough to aim at.

        The camera is aimed along the measured line of sight, not at the
        filtered position: the bearing comes straight from the box and the
        attitude, while the position also carries the range's noise, so a
        wrong range or velocity cannot turn the subject out of the frame."""
        fix = self._last_fix
        if fix is None or (now - self._last_fix_s) > _SIGHTING_YAW_S:
            return None
        return fix.bearing_rad

    # -- the pilot motion lease -----------------------------------------

    async def _ensure_lease(self, now: float) -> str | None:
        """Hold the pilot's motion lease; the hold reason when it is refused."""
        if self._lease_id is not None:
            return None
        if self._lease_refusal is not None and (now - self._lease_tried_s) < LEASE_RETRY_S:
            return self._lease_refusal
        self._lease_tried_s = now
        try:
            reply = await self._ctx.flight.lease_acquire(kind="motion", skill_id=SKILL_ID)
        except Exception as exc:  # noqa: BLE001
            self._ctx.log.warning("follow_me_lease_error", error=str(exc))
            reply = None
        if isinstance(reply, dict) and reply.get("ok") and reply.get("lease_id"):
            self._lease_id = reply["lease_id"]
            self._lease_refusal = None
            self._ctx.log.info("follow_me_lease_acquired", lease_id=self._lease_id)
            return None
        err = lease_error_reason(reply)
        self._lease_refusal = HOLD_LEASE_BUSY if err == "busy" else HOLD_LEASE_REFUSED
        self._ctx.log.info("follow_me_lease_refused", err=err)
        return self._lease_refusal

    async def _release_lease(self) -> None:
        lease_id = self._lease_id
        self._lease_refusal = None
        if lease_id is None:
            return
        self._lease_id = None
        try:
            await self._ctx.flight.lease_release(lease_id)
        except Exception as exc:  # noqa: BLE001
            self._ctx.log.warning("follow_me_lease_release_error", error=str(exc))

    def _on_lease_revoked(self, payload: dict[str, Any]) -> None:
        if self._lease_id is None or str(payload.get("lease_id")) != str(self._lease_id):
            return
        self._lease_id = None
        self._revoked_reason = str(payload.get("reason") or "revoked")

    def _on_sticks(self, payload: dict[str, Any]) -> None:
        if self._lease_id is None or str(payload.get("lease_id")) != str(self._lease_id):
            return
        if self._cfg is None:
            return
        self._nudges.on_sticks(
            self._nudges.apply_to(self._cfg), payload.get("axes"), fc_state.monotonic()
        )

    async def _on_revoked_stop(self, now: float) -> None:
        """The pilot took the vehicle back: switch the skill off so every
        surface's toggle follows, and say why."""
        reason = self._revoked_reason
        self._revoked_reason = None
        self._ctx.log.info("follow_me_lease_revoked", reason=reason)
        setter = getattr(self._ctx.config_kv, "set", None)
        if setter is not None:
            await setter("active", False)
        if self._cfg is not None:
            self._cfg = replace(self._cfg, active=False)
        self._coast = None
        await self._ctx.events.publish(FOLLOW_STOPPED_TOPIC, {"reason": reason})

    # -- commanding and holding ----------------------------------------

    def _geometry_dt(self, cfg: FollowConfig, now: float) -> float:
        """Seconds since the previous goal, resetting the follow reference
        and orbit phase on a mode change."""
        if self._geom_mode != cfg.follow_mode:
            self._geom_mode = cfg.follow_mode
            self._ref_heading = None
            self._orbit_phase = None
            self._geom_tick_s = None
        dt = 0.0 if self._geom_tick_s is None else now - self._geom_tick_s
        self._geom_tick_s = now
        return max(0.0, min(dt, 0.5))

    def _goal(self, cfg: FollowConfig, now: float, x: list[float], ff_scale: float) -> dict[str, Any]:
        agl, _src = self._agl(now)
        dt = self._geometry_dt(cfg, now)
        tn, te, tvn, tve = x[0], x[1], x[2], x[3]
        dn, de = self._pose.north_m, self._pose.east_m
        here = setpoint.bearing(tn, te, dn, de)
        if cfg.follow_mode == FOLLOW_MODE_ORBIT:
            if self._orbit_phase is None:
                self._orbit_phase = here if here is not None else self._pose.yaw + math.pi
            else:
                self._orbit_phase = setpoint.advance_orbit_phase(
                    self._orbit_phase,
                    center_n=tn,
                    center_e=te,
                    radius_m=cfg.orbit_radius_m,
                    rate_dps=cfg.orbit_rate_dps,
                    direction=cfg.orbit_direction,
                    drone_n=dn,
                    drone_e=de,
                    dt_s=dt,
                )
            g = setpoint.orbit_goal(
                center_n=tn,
                center_e=te,
                center_vn=tvn,
                center_ve=tve,
                phase_rad=self._orbit_phase,
                radius_m=cfg.orbit_radius_m,
                rate_dps=cfg.orbit_rate_dps,
                direction=cfg.orbit_direction,
                max_speed_mps=cfg.max_speed_mps,
                ff_scale=ff_scale,
            )
            kind = "orbit"
            ring = {
                "ring_center_ne": [tn, te],
                "radius_m": cfg.orbit_radius_m,
                "angle_deg": math.degrees(self._orbit_phase) % 360.0,
                "angle_span_deg": _ORBIT_SPAN_DEG,
            }
        else:
            if self._ref_heading is None:
                # Engaging causes no swing: the reference starts where the
                # vehicle already is relative to the subject.
                base = here if here is not None else self._pose.yaw + math.pi
                self._ref_heading = setpoint.wrap_pi(base - math.radians(cfg.follow_angle_deg))
            else:
                self._ref_heading = setpoint.slew_ref_heading(self._ref_heading, tvn, tve, dt)
            g = setpoint.follow_goal(
                target_n=tn,
                target_e=te,
                target_vn=tvn,
                target_ve=tve,
                ref_heading_rad=self._ref_heading,
                follow_distance_m=cfg.follow_distance_m,
                follow_angle_deg=cfg.follow_angle_deg,
                max_speed_mps=cfg.max_speed_mps,
                ff_scale=ff_scale,
                drone_n=dn,
                drone_e=de,
                drone_yaw_rad=self._pose.yaw,
            )
            kind = "follow"
            ring = {
                "ring_center_ne": [tn, te],
                "radius_m": cfg.follow_distance_m,
                "angle_deg": math.degrees(self._ref_heading + math.radians(cfg.follow_angle_deg)) % 360.0,
                "angle_span_deg": _FOLLOW_SPAN_DEG,
            }
        # The camera looks along the latest sighting's line of sight; the
        # position goal stays on the estimate.
        sighted = self._sighted_bearing(now)
        return {
            "kind": kind,
            "n": g.north_m,
            "e": g.east_m,
            "d": setpoint.height_down(self._pose.down_m, agl, cfg.follow_height_m),
            "vn": g.vn_mps,
            "ve": g.ve_mps,
            "vd": 0.0,
            "yaw": g.yaw_rad if sighted is None else sighted,
            "yaw_rate": None,
            "subject_ne": [tn, te],
            "max_speed_mps": cfg.max_speed_mps,
            "candidates": ring,
            "margin_m": cfg.obstacle_margin_m,
        }

    async def _command(
        self,
        cfg: FollowConfig,
        now: float,
        *,
        phase: str,
        ff_scale: float,
        lock_state: str,
        coast_remaining_s: float = 0.0,
        predicted_bbox: dict[str, object] | None = None,
    ) -> None:
        pred = self._filter.predicted(now)
        if pred is None:
            await self._hold(
                cfg, now, phase=PHASE_HOLDING, hold_reason=HOLD_NO_RANGE, lock_state=lock_state
            )
            return
        refusal = await self._ensure_lease(now)
        if refusal is not None:
            await self._hold(
                cfg, now, phase=PHASE_HOLDING, hold_reason=refusal, lock_state=lock_state
            )
            return
        x = pred[0]
        agl, _src = self._agl(now)
        await self._send_goal(self._goal(cfg, now, x, ff_scale), now)
        if self._pending_latency_ts_ms is not None:
            # Both clocks are the host's monotonic clock (the engine stamps
            # frames with CLOCK_MONOTONIC).
            latency = now * 1000.0 - self._pending_latency_ts_ms
            self._latency_ms = (
                int(round(latency)) if 0.0 <= latency <= _MAX_LATENCY_MS else None
            )
            self._pending_latency_ts_ms = None
        await self._point_gimbal(cfg, now, x[0], x[1], agl)
        await self._publish_state(
            cfg,
            now=now,
            active=True,
            phase=phase,
            lock_state=lock_state,
            commanding=True,
            coast_remaining_s=coast_remaining_s,
            predicted_bbox=predicted_bbox,
        )

    async def _hold(
        self,
        cfg: FollowConfig,
        now: float,
        *,
        phase: str,
        hold_reason: str,
        lock_state: str | None = None,
        active: bool = True,
        coast_remaining_s: float = 0.0,
        force: bool = False,
        yaw_rad: float | None = None,
    ) -> None:
        if self._lease_id is not None and active and self._pose.ready(now):
            # Keep the lease alive and the vehicle where it is: the pilot
            # holds a position goal at the current position, at the current
            # heading or the one the caller gives.
            await self._send_goal(
                {
                    "kind": "position",
                    "n": self._pose.north_m,
                    "e": self._pose.east_m,
                    "d": self._pose.down_m,
                    "vn": 0.0,
                    "ve": 0.0,
                    "vd": 0.0,
                    "yaw": self._pose.yaw if yaw_rad is None else yaw_rad,
                    "yaw_rate": None,
                    "subject_ne": None,
                    "max_speed_mps": cfg.max_speed_mps,
                    "candidates": None,
                    "margin_m": cfg.obstacle_margin_m,
                },
                now,
            )
        await self._publish_state(
            cfg,
            now=now,
            active=active,
            phase=phase,
            lock_state=lock_state,
            commanding=False,
            hold_reason=hold_reason,
            coast_remaining_s=coast_remaining_s,
            force=force,
        )

    async def _send_goal(self, goal: dict[str, Any], now: float) -> None:
        """Send one goal under the lease and count it toward the published
        send rate (the read-back is where a bench reads the rate)."""
        lease_id = self._lease_id
        if lease_id is None:
            return
        await self._ctx.flight.goal_send(lease_id, goal)
        self._setpoint_times.append(now)

    async def _point_gimbal(
        self, cfg: FollowConfig, now: float, north: float, east: float, agl: float
    ) -> None:
        if not cfg.gimbal_point or (now - self._last_gimbal_s) < _GIMBAL_PERIOD_S:
            return
        dn = north - self._pose.north_m
        de = east - self._pose.east_m
        pitch_deg, yaw_deg = mavlink_frames.gimbal_angles_for_target(
            slant_range_m=math.hypot(math.hypot(dn, de), agl),
            agl_m=agl,
            bearing_rad=math.atan2(de, dn),
            vehicle_yaw_rad=self._pose.yaw,
        )
        # Cached in the projection convention (pitch positive = down) as the
        # boresight estimate until the FC reports a real gimbal attitude.
        self._commanded_gimbal = (-pitch_deg, yaw_deg)
        self._last_gimbal_s = now
        try:
            await self._ctx.mavlink.send(
                mavlink_frames.build_gimbal_pitchyaw(pitch_deg=pitch_deg, yaw_deg=yaw_deg)
            )
        except Exception as exc:  # noqa: BLE001
            # A missing gimbal component is non-fatal: the body-only follow
            # continues.
            self._ctx.log.info("follow_me_gimbal_skipped", error=str(exc))


__all__ = ["FollowLoopMixin", "Sighting"]
