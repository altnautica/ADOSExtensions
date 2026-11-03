"""The follow loop: the command gate, lock loss and re-acquire, and the
setpoints. A mixin of :class:`follow_me.FollowMePlugin`.

Every tick (20 Hz) decides one of three things:

* **follow**: a live engine lock, a current pose, a live armed+guided flight
  controller and a recent range fix. The subject filter is updated from each
  new box and the standoff setpoint (position, velocity feed-forward, yaw) is
  sent as SET_POSITION_TARGET_LOCAL_NED.
* **coast** (``smart`` re-acquire only): the lock was lost; fly toward the
  predicted standoff at half feed-forward for ``reacquire_window_s`` while
  looking for the subject in each new batch.
* **hold**: anything else. If the loop was commanding, a few zero-velocity
  setpoints at the current position are sent so the vehicle stops where it is
  instead of flying on along the last feed-forward; then nothing more is sent,
  so the operator's own guided commands are not fought.
"""

from __future__ import annotations

import asyncio
import math
from collections import deque
from dataclasses import asdict, dataclass
from typing import Any

from ados.sdk.tracking import EffectiveLock, LockedTargetTracker

from follow_me import fc_state, mavlink_frames, setpoint
from follow_me.projection import TargetFix
from follow_me.reacquire import Coast, Embedding, blend_template
from follow_me.state import (
    HOLD_FC_DISARMED,
    HOLD_FC_NOT_GUIDED,
    HOLD_FC_STALE,
    HOLD_INACTIVE,
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
from follow_me.target_filter import TargetFilter

# The fixed cadence the follow loop commands at, decoupled from the
# inference rate so the setpoint stream is smooth regardless of detector
# jitter; the filter predicts between boxes.
_LOOP_HZ = 20.0
_LOOP_PERIOD = 1.0 / _LOOP_HZ

# With no new range fix for this long the prediction is no longer trusted
# enough to command on a live lock.
_MEASUREMENT_STALE_S = 1.0
# Zero-velocity hold setpoints sent after the loop stops commanding (0.25 s
# at 20 Hz), so one dropped frame cannot leave the last feed-forward active.
_HOLD_REPEATS = 5
# Gimbal pointing is re-sent at most this often.
_GIMBAL_PERIOD_S = 0.2
# Detection-to-setpoint latencies outside this are clock mismatches, not
# measurements, and are not reported.
_MAX_LATENCY_MS = 10_000.0
# Window the published setpoint send rate is measured over.
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
    _coast: Coast | None
    _phase: str
    _hold_sends_remaining: int
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
        cfg = await self._config_for_tick()
        if not cfg.active:
            self._coast = None
            await self._hold(
                cfg, now, phase=PHASE_IDLE, hold_reason=HOLD_INACTIVE, active=False
            )
            return
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
        if not self._fc.guided:
            return HOLD_FC_NOT_GUIDED
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
            await self._hold(
                cfg,
                now,
                phase=PHASE_HOLDING,
                hold_reason=HOLD_LOCK_UNCERTAIN,
                lock_state=lock.value,
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

    def _measure_sighting(self, cfg: FollowConfig, now: float, bbox: Any) -> None:
        self._measured_bbox = bbox
        s = self._sighting if (self._sighting and self._sighting.bbox is bbox) else None
        frame_w, frame_h = s.frame_wh if (s and s.frame_wh) else self._frame_size()
        fix = self._measure_box(cfg, now, bbox, frame_w, frame_h)
        if fix is None:
            return
        self._filter.update(
            s.received_s if s else now,
            self._pose.north_m + fix.offset_north_m,
            self._pose.east_m + fix.offset_east_m,
            fix.covariance(),
        )
        self._last_fix = fix
        if s is not None:
            if s.ts_ms > 0:
                self._pending_latency_ts_ms = s.ts_ms
            if s.embedding is not None:
                self._template = blend_template(self._template, s.embedding)

    # -- commanding and holding ----------------------------------------

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
        x = pred[0]
        agl, _src = self._agl(now)
        sp = setpoint.compute_setpoint(
            drone_north_m=self._pose.north_m,
            drone_east_m=self._pose.east_m,
            drone_down_m=self._pose.down_m,
            drone_yaw_rad=self._pose.yaw,
            agl_m=agl,
            target_north_m=x[0],
            target_east_m=x[1],
            target_vn_mps=x[2],
            target_ve_mps=x[3],
            follow_distance_m=cfg.follow_distance_m,
            follow_height_m=cfg.follow_height_m,
            max_speed_mps=cfg.max_speed_mps,
            ff_scale=ff_scale,
        )
        await self._send_setpoint(sp, now)
        if self._pending_latency_ts_ms is not None:
            # Both clocks are the host's monotonic clock (the engine stamps
            # frames with CLOCK_MONOTONIC).
            latency = now * 1000.0 - self._pending_latency_ts_ms
            self._latency_ms = (
                int(round(latency)) if 0.0 <= latency <= _MAX_LATENCY_MS else None
            )
            self._pending_latency_ts_ms = None
        self._hold_sends_remaining = _HOLD_REPEATS
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
    ) -> None:
        if (
            self._hold_sends_remaining > 0
            and self._pose.ready(now)
            and self._fc.armed_at(now)
            and self._fc.guided_at(now)
        ):
            hold = setpoint.hold_setpoint(
                north_m=self._pose.north_m,
                east_m=self._pose.east_m,
                down_m=self._pose.down_m,
                yaw_rad=self._pose.yaw,
            )
            await self._send_setpoint(hold, now)
            self._hold_sends_remaining -= 1
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

    async def _send_setpoint(self, sp: setpoint.LocalSetpoint, now: float) -> None:
        """Send one SET_POSITION_TARGET_LOCAL_NED and count it toward the
        published send rate (the agent's own setpoints never reach a ground
        tlog, so the read-back is where a bench reads the rate)."""
        await self._ctx.flight.guided_setpoint(
            **mavlink_frames.build_local_setpoint(**asdict(sp))
        )
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
