"""The follow loop: the command gate, the uncertain lock, and the goals sent
to the pilot. A mixin of :class:`follow_me.FollowMePlugin`.

Every tick (20 Hz) decides one of three things:

* **follow**: a live engine lock, a current pose, a live armed flight
  controller, a recent range fix and the pilot's motion lease. The subject
  filter is updated from each new box and a ``follow`` or ``orbit`` goal
  (position, velocity feed-forward, yaw, the subject and the detour ring the
  pilot's planner searches) is sent to the pilot. While the lock is unsure
  and ``uncertain_mode`` is ``follow``, the goal is the prediction at reduced
  feed-forward.
* **recover**: the lock was lost; :mod:`follow_me.recovery` predicts,
  repositions and searches, or holds for the operator.
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

from ados.sdk import clock
from ados.sdk.tracking import EffectiveLock, LockedTargetTracker

from follow_me import fc_state, intent, mavlink_frames, setpoint
from follow_me.camera import DEFAULT_GEOMETRY, CameraGeometry, exposure_fc_s
from follow_me.intent import VEL_MODE_COMMAND
from follow_me.motion import LEASE_RETRY_S, SKILL_ID, Nudges, lease_error_reason
from follow_me.projection import TargetFix
from follow_me.ranging import range_bearing_covariance
from follow_me.reacquire import (
    OVERLAP_IOU,
    PERSON,
    Embedding,
    blend_template,
    box_iou,
    template_may_update,
)
from follow_me.recovery import SEARCH_FF_SCALE, Recovery
from follow_me.standoff import (
    SEPARATION_SIGMAS,
    StandoffController,
    StillSubject,
    pilot_subject_sigma,
)
from follow_me.state import (
    FOLLOW_MODE_ORBIT,
    FOLLOW_STOPPED_TOPIC,
    HOLD_CAMERA_LOST,
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
    LOCK_LOST,
    LOCK_UNCERTAIN,
    PHASE_ACQUIRING,
    PHASE_AWAIT_OPERATOR,
    PHASE_FOLLOWING,
    PHASE_HOLDING,
    PHASE_IDLE,
    REACQUIRE_SMART,
    UNCERTAIN_FOLLOW,
    UNCERTAIN_FOLLOW_MAX_S,
    FollowConfig,
)
from follow_me.subject import SubjectFeed, SubjectView
from follow_me.target_filter import Cov2, TargetFilter, subject_max_speed_mps

# The fixed cadence the follow loop commands at, decoupled from the
# inference rate so the setpoint stream is smooth regardless of detector
# jitter; the filter predicts between boxes.
_LOOP_HZ = 20.0
_LOOP_PERIOD = 1.0 / _LOOP_HZ

# With no fused range fix for this long the prediction is no longer trusted
# enough to command on a live lock, and an unsure lock's feed-forward is
# reduced. Shorter gaps (a missed frame, a full occlusion of up to 2 s, fixes
# the outlier gate dropped) are ridden through on the prediction at full
# speed: a stop there is what tilts the camera.
_MEASUREMENT_STALE_S = 2.0
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
# The engine publishes a batch for every inferred frame (6 Hz or more on the
# boards this plugin supports), an empty one too. With none from the follow
# camera for this long the camera is lost; the first seconds after
# activation are exempt while the engine starts detecting on the camera the
# follow just leased.
_CAMERA_LOST_S = 2.0
_CAMERA_START_GRACE_S = 5.0
# A frame stamp more than this before its batch arrived is a clock mismatch:
# the fix is placed at the batch's arrival instead.
_MAX_EXPOSURE_AGE_S = 1.0
# The range sigma a fix is fused with at the least. A recorded forest follow
# measured 0.7-1.5 m (median) between consecutive fixes of a subject 7-30 m
# away where the cues claimed about 0.4 m; fused at the cues' own figure that
# noise went straight into the estimate and the goal.
_MIN_RANGE_SIGMA_M = 0.75
_MIN_RANGE_SIGMA_FRAC = 0.06
# A goal point lies at least the standoff (or orbit radius) less this from
# the subject's worst case (the nearest of its estimate as flown, its
# undecayed prediction and a standing subject's centre).
_GOAL_INSIDE_M = 1.0


def fix_range_sigma(fix: TargetFix) -> float:
    """The range sigma a fix is fused with: the part of its own that changes
    from frame to frame, held to the measured floor."""
    own = fix.range_sigma_m if fix.range_noise_m is None else fix.range_noise_m
    return max(own, _MIN_RANGE_SIGMA_M, _MIN_RANGE_SIGMA_FRAC * fix.range_m)


def fix_covariance(fix: TargetFix) -> Cov2:
    """The north/east covariance a fix is fused with: its own, with the range
    sigma held to the measured floor."""
    return range_bearing_covariance(fix.bearing_rad, fix_range_sigma(fix), fix.cross_sigma_m)


def _estimate_error(
    x: list[float], p: list[list[float]], drone_n: float, drone_e: float, scatter_ratio: float
) -> tuple[float, float]:
    """The 1-sigma error the subject estimate really carries: along the line
    of sight from the drone, and in its velocity. The filter's own spread,
    scaled by how far the fixes actually scatter against what it fuses them
    with (:attr:`TargetFilter.scatter_ratio`)."""
    dn, de = x[0] - drone_n, x[1] - drone_e
    d = math.hypot(dn, de)
    un, ue = (dn / d, de / d) if d > 1e-6 else (1.0, 0.0)
    pos = un * (p[0][0] * un + p[0][1] * ue) + ue * (p[1][0] * un + p[1][1] * ue)
    vel = 0.5 * (p[2][2] + p[3][3])
    return (
        math.sqrt(max(0.0, pos)) * scatter_ratio,
        math.sqrt(max(0.0, vel)) * scatter_ratio,
    )


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
    # The frame's sensor-clock stamp (ms), when the camera has one.
    ts_sensor_ms: int | None = None
    # The camera geometry the batch carried.
    geometry: CameraGeometry = DEFAULT_GEOMETRY


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
    _recovery: Recovery | None
    # When the current unsure spell began, or None while the lock is sure.
    _uncertain_since_s: float | None
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
    _goal_kind: str | None
    _goal_refusal: tuple[str | None, str] | None
    _latest_batch: Any
    _latest_batch_s: float
    _commanded_gimbal: tuple[float, float] | None
    _fc_clock: fc_state.FcClock
    _geometry: CameraGeometry
    _active_since_s: float | None
    _camera_lost: bool
    _subject_motion: Any
    _range_tier: Any
    _outpaced_since_s: float | None
    _still: StillSubject
    _standoff: StandoffController
    # Where the current hold began (n, e, d, yaw), or None while not holding.
    _hold_point: tuple[float, float, float, float] | None
    # When the last fix was fused (its arrival on the loop's clock).
    _fused_s: float
    # The filter's restart count the follow last smoothed from.
    _restarts_seen: int
    # The engine's state of the designated subject (see follow_me.subject).
    _subject: SubjectFeed

    async def _follow_loop(self) -> None:
        while True:
            await clock.sleep(_LOOP_PERIOD)
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
            # A STOP or a lease revoke (which switches the follow off first)
            # ends here. The drone may be flown elsewhere before the follow is
            # switched on again on the same track, so it restarts its geometry
            # where the drone is then, as engaging does.
            self._reset_geometry()
            self._range_tier.reset()
            self._recovery = None
            self._uncertain_since_s = None
            self._active_since_s = None
            self._camera_lost = False
            await self._release_lease()
            self._nudges.reset_session()
            await self._hold(
                cfg, now, phase=PHASE_IDLE, hold_reason=HOLD_INACTIVE, active=False
            )
            return
        await self._nudges.write_back(self._ctx, now)
        if self._active_since_s is None:
            self._active_since_s = now
        if (
            now - self._active_since_s > _CAMERA_START_GRACE_S
            and now - self._latest_batch_s > _CAMERA_LOST_S
        ):
            await self._on_camera_lost(cfg, now)
            return
        self._camera_lost = False
        if self._recovery is not None:
            await self._tick_recovery(cfg, now)
            return
        await self._tick_follow(cfg, now)

    async def _on_camera_lost(self, cfg: FollowConfig, now: float) -> None:
        """The follow camera's batches stopped: hold, and the estimate stops
        where it is rather than carry its velocity on. A recovery is kept, so
        the tracks it gave up on stay excluded and a wait for the operator
        still waits when the frames return; while it waits, the hold stays at
        its latched point and the phase stays the wait, so the operator's
        prompt stays up."""
        if not self._camera_lost:
            self._camera_lost = True
            self._ctx.log.warning(
                "follow_me_camera_lost", camera=self._follow_camera()
            )
        self._uncertain_since_s = None
        self._filter.stop(now)
        rec = self._recovery
        if rec is not None and rec.phase == PHASE_AWAIT_OPERATOR:
            await self._hold(
                cfg,
                now,
                phase=PHASE_AWAIT_OPERATOR,
                hold_reason=HOLD_CAMERA_LOST,
                lock_state=LOCK_LOST,
                hold_ne=rec.hold_ne,
                hold_d=rec.hold_d,
            )
            return
        locked_before = self._tracker.has_lock or self._filter.initialized
        await self._hold(
            cfg,
            now,
            phase=PHASE_HOLDING if locked_before else PHASE_ACQUIRING,
            hold_reason=HOLD_CAMERA_LOST,
        )

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
            await self._hold(cfg, now, phase=PHASE_ACQUIRING, hold_reason=HOLD_NO_LOCK)
            return
        if self._subject.take_release(self._tracker.track_id, self._follow_camera()):
            # The engine released the subject: the same loss as a lost lock.
            await self._on_lock_lost(cfg, now)
            return
        lock = self._tracker.effective_lock(now)
        if lock is EffectiveLock.LOST:
            if await self._follow_beacon(cfg, now, LOCK_LOST):
                return
            await self._on_lock_lost(cfg, now)
            return
        target = self._tracker.locked_target(now)
        if target is None:
            await self._tick_uncertain(cfg, now)
            return
        self._uncertain_since_s = None
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
        stale = not self._filter.initialized or (now - self._fused_s) > _MEASUREMENT_STALE_S
        if stale and not self._subject_measured(now, _MEASUREMENT_STALE_S):
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
        self._uncertain_since_s = None
        self._latency_ms = None
        self._pending_latency_ts_ms = None
        self._ref_heading = None
        self._orbit_phase = None
        self._still.reset()
        self._standoff.reset()
        self._subject_motion.clear()
        self._outpaced_since_s = None
        self._fused_s = -math.inf

    async def _tick_uncertain(self, cfg: FollowConfig, now: float) -> None:
        """An unsure lock: keep following the prediction (``uncertain_mode``
        follow) or hold (``hold``). An unsure spell longer than
        UNCERTAIN_FOLLOW_MAX_S is handled as a loss, so a hold never outlasts
        it.

        The tracker calls every missed frame unsure. While the last fix is
        recent the prediction is as good as on a sure lock, so the follow
        flies on unchanged: a missed frame neither stops nor slows the
        vehicle. Past _MEASUREMENT_STALE_S without a fix the subject's
        feed-forward is reduced, as for a prediction after a loss."""
        if self._uncertain_since_s is None:
            self._uncertain_since_s = now
        if await self._follow_beacon(cfg, now, LOCK_UNCERTAIN):
            return
        if now - self._uncertain_since_s > UNCERTAIN_FOLLOW_MAX_S:
            await self._on_lock_lost(cfg, now)
            return
        # Strict keeps its contract on an unsure lock too: hold at once.
        if cfg.uncertain_mode == UNCERTAIN_FOLLOW and cfg.reacquire_mode == REACQUIRE_SMART:
            gate = self._gate(now)
            if gate is not None:
                await self._hold(
                    cfg, now, phase=PHASE_HOLDING, hold_reason=gate, lock_state=LOCK_UNCERTAIN
                )
                return
            fresh = (now - self._fused_s) <= _MEASUREMENT_STALE_S or self._subject_measured(
                now, _MEASUREMENT_STALE_S
            )
            await self._command(
                cfg,
                now,
                phase=PHASE_FOLLOWING,
                ff_scale=1.0 if fresh else SEARCH_FF_SCALE,
                lock_state=LOCK_UNCERTAIN,
            )
            return
        # Hold, keep the camera on the latest sighting, and stop the estimate
        # where it is rather than carry its velocity on.
        self._filter.stop(now)
        await self._hold(
            cfg,
            now,
            phase=PHASE_HOLDING,
            hold_reason=HOLD_LOCK_UNCERTAIN,
            lock_state=LOCK_UNCERTAIN,
            yaw_rad=self._sighted_bearing(now),
        )

    def _measure_sighting(self, cfg: FollowConfig, now: float, bbox: Any) -> None:
        self._measured_bbox = bbox
        s = self._sighting if (self._sighting and self._sighting.bbox is bbox) else None
        frame_w, frame_h = s.frame_wh if (s and s.frame_wh) else self._frame_size()
        # The box is projected through the attitude its frame was exposed at,
        # with the geometry of the camera that took it.
        exposed_s = self._exposure_s(s.ts_ms, s.ts_sensor_ms) if s else None
        geometry = s.geometry if s else self._geometry
        fix = self._measure_box(cfg, now, bbox, frame_w, frame_h, geometry, exposed_s)
        if fix is None:
            return
        if s is not None:
            self._filter.max_speed_mps = subject_max_speed_mps(s.class_label)
        # The fix is placed from where the vehicle was when the frame was
        # exposed, and at that time; the filter carries it on to now, which
        # also takes the detector's latency out of the estimate.
        t_fix = s.received_s if s else now
        t_exposed = self._host_exposure_s(s, exposed_s) if s else None
        if t_exposed is not None:
            t_fix = t_exposed
        back = max(0.0, now - t_fix)
        north = self._pose.north_m - self._pose.vn_mps * back + fix.offset_north_m
        east = self._pose.east_m - self._pose.ve_mps * back + fix.offset_east_m
        cov = fix_covariance(fix)
        fused = self._filter.update(t_fix, north, east, cov)
        if fused:
            self._fused_s = now
        scatter = self._filter.scatter_measured
        if scatter is not None:
            # The range scatter in metres: how far the fixes really scatter
            # along the line of sight (see follow_me.range_tier).
            self._range_tier.update(scatter * fix_range_sigma(fix), settled=self._filter.scatter_settled)
        # Every fix, fused or gated out: a subject breaking into a run puts
        # its first fixes outside the gate.
        est = self._filter.last_estimate()
        speed = math.hypot(est[2], est[3]) if est is not None else 0.0
        self._still.add_fix(now, north, east, cov, speed, fused=fused)
        # The measured motion, unclamped by the filter's speed cap, for the
        # faster-than-the-limit check.
        self._subject_motion.record(t_fix, north, east, fix.range_m)
        self._last_fix = fix
        self._last_fix_s = now
        if s is not None:
            if s.ts_ms > 0:
                self._pending_latency_ts_ms = s.ts_ms
            if s.embedding is not None and template_may_update(
                lock_state=LOCK_LOCKED,
                confidence=s.confidence,
                occlusion_hint=None,
                others_near=self._others_near(bbox),
            ):
                self._template = blend_template(self._template, s.embedding)

    def _host_exposure_s(self, s: Sighting, exposed_fc_s: float | None) -> float | None:
        """A sighting's exposure on the host clock, or None when no stamp
        places it within _MAX_EXPOSURE_AGE_S before its receipt."""
        offset = self._fc_clock.offset_s
        if exposed_fc_s is not None and offset is not None:
            t = exposed_fc_s + offset
        elif s.ts_ms > 0:
            t = s.ts_ms / 1000.0
        else:
            return None
        return t if 0.0 <= s.received_s - t <= _MAX_EXPOSURE_AGE_S else None

    def _exposure_s(self, ts_ms: int, ts_sensor_ms: int | None) -> float | None:
        """A frame's exposure on the flight-controller clock, or None (the
        latest attitude) while that clock is not yet known."""
        return exposure_fc_s(ts_ms, ts_sensor_ms, self._fc_clock.offset_s)

    def _others_near(self, bbox: Any) -> int:
        """People in the newest batch whose box overlaps the subject's."""
        batch = self._latest_batch
        n = 0
        for det in getattr(batch, "detections", None) or ():
            other = getattr(det, "bbox", None)
            if other is None or other is bbox or getattr(det, "class_label", None) != PERSON:
                continue
            if box_iou(bbox, other) > OVERLAP_IOU:
                n += 1
        return n
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
        # A distance or radius is nudged from the one flown, never below the
        # range's minimum.
        self._nudges.on_sticks(
            self._nudges.apply_to(self._cfg),
            payload.get("axes"),
            fc_state.monotonic(),
            least_standoff_m=self._range_tier.min_standoff_m,
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
        self._recovery = None
        self._uncertain_since_s = None
        await self._ctx.events.publish(FOLLOW_STOPPED_TOPIC, {"reason": reason})

    # -- commanding and holding ----------------------------------------

    def _reset_geometry(self) -> None:
        """Forget the follow reference, the orbit phase and the standing
        subject: the next goal seeds them from where the drone is."""
        self._ref_heading = None
        self._orbit_phase = None
        self._geom_tick_s = None
        self._still.reset()
        self._standoff.reset()
        self._hold_point = None

    def _geometry_dt(self, cfg: FollowConfig, now: float) -> float:
        """Seconds since the previous goal, resetting the follow reference
        and orbit phase on a mode change."""
        if self._geom_mode != cfg.follow_mode:
            self._geom_mode = cfg.follow_mode
            self._reset_geometry()
        dt = 0.0 if self._geom_tick_s is None else now - self._geom_tick_s
        self._geom_tick_s = now
        return max(0.0, min(dt, 0.5))

    def _goal(
        self,
        cfg: FollowConfig,
        now: float,
        x: list[float],
        p: list[list[float]],
        ff_scale: float,
        subject: SubjectView | None = None,
    ) -> dict[str, Any]:
        """The goal round the estimate ``x``/``p``: the engine's subject state
        when ``subject`` is given (its spread as it states it), else the
        follow's own filter."""
        agl, _src = self._agl(now)
        dt = self._geometry_dt(cfg, now)
        if self._filter.restarts != self._restarts_seen:
            # The estimate started over: the pace and the command smoothed
            # from the old one (a phantom's speed) no longer apply.
            self._restarts_seen = self._filter.restarts
            self._standoff.reset()
        # A standing subject is followed round a fixed centre, with no
        # velocity: range noise no longer moves the goal.
        ref = self._still.reference((x[0], x[1], x[2], x[3]))
        tn, te = ref.north_m, ref.east_m
        dn, de = self._pose.north_m, self._pose.east_m
        scatter = 1.0 if subject is not None else self._filter.scatter_ratio
        range_sigma, pace_sigma = _estimate_error(x, p, dn, de, scatter)
        # The subject's velocity fed forward, smoothed by how uncertain it
        # is; the reference heading still turns on the filter's own.
        tvn, tve = self._standoff.pace(
            now=now, vn=ref.vn_mps, ve=ref.ve_mps, sigma_mps=pace_sigma, still=ref.still
        )
        here = setpoint.bearing(tn, te, dn, de)
        if cfg.follow_mode == FOLLOW_MODE_ORBIT:
            if self._orbit_phase is None:
                self._orbit_phase = here if here is not None else self._pose.yaw + math.pi
            else:
                self._orbit_phase = setpoint.advance_orbit_phase(
                    self._orbit_phase,
                    center_n=tn,
                    center_e=te,
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
                self._ref_heading = setpoint.slew_ref_heading(
                    self._ref_heading, ref.vn_mps, ref.ve_mps, dt
                )
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
        # The camera turns toward the latest sighting's line of sight (the
        # estimate's bearing without one) through the bearing control; the
        # goal flies the speed law toward the geometry's point.
        sighted = self._sighted_bearing(now)
        # Kept clear of: a standing subject's centre (the mean of many fixes:
        # a run of range outliers that drags the estimate and the newest
        # fixes' range in for a moment does not move it, and a step toward
        # the vehicle ends standing within STILL_STEP_FIXES fixes); a moving
        # one's estimate and the newest fixes' range.
        # The worst case the hard guard keeps the safe distance from: the
        # nearer of the estimate flown on and its undecayed prediction (on a
        # coast a subject coming toward the vehicle keeps coming), or a
        # standing subject's centre. (The newest fixes' range, which a run
        # of range outliers drags in, is the soft guard's.)
        standoff_m = cfg.orbit_radius_m if kind == "orbit" else cfg.follow_distance_m
        raw = subject.at(now) if subject is not None else self._filter.predicted(now)
        coast = subject.coast_spread_m(now) if subject is not None else self._filter.coast_spread_m(now)
        # A standing subject's centre: its estimate swings a metre on range
        # noise, and a step toward the vehicle shows in the newest fixes'
        # range first. On a coast after a walk up to the vehicle (see
        # TargetFilter.approached) it is guarded as a moving one is.
        moving = not ref.still or (coast > 0.0 and self._filter.approached((dn, de), both=True))
        cands = [((tn, te), (0.0, 0.0))] if ref.still else [((x[0], x[1]), (x[2], x[3]))]
        if raw is not None and moving:
            cands.append(((raw[0][0], raw[0][1]), (raw[0][2], raw[0][3])))
        guard_ne, guard_v = min(cands, key=lambda c: math.hypot(c[0][0] - dn, c[0][1] - de))
        nearest_v = guard_v
        guard_near = self._subject_motion.near_range(now)
        guard_pace: tuple[float, float] = (tvn, tve)
        # On a coast, also a sigma of the velocity's own error faster toward
        # the vehicle (a walk the estimate puts 10 % slow closes a metre in
        # five seconds; not for a standing subject's walk up, whose noise is
        # that error), and no credit for the pace fed forward.
        coast_sigma = coast if moving else 0.0
        gr = math.hypot(guard_ne[0] - dn, guard_ne[1] - de)
        if coast_sigma > 0.0 and raw is not None and gr > 1e-6:
            un, ue = (guard_ne[0] - dn) / gr, (guard_ne[1] - de) / gr
            toward = 0.0 if ref.still else math.sqrt(max(0.0, 0.5 * (raw[1][2][2] + raw[1][3][3])))
            guard_v = (guard_v[0] - toward * un, guard_v[1] - toward * ue)
            guard_pace = guard_v
        if ref.still:
            near = None
            keep_ne, keep_v = (tn, te), (0.0, 0.0)
        else:
            near = guard_near
            keep_ne, keep_v = guard_ne, nearest_v
        cmd = self._standoff.command(
            now=now,
            goal_ne=(g.north_m, g.east_m),
            goal_v_ne=(g.vn_mps, g.ve_mps),
            drone_ne=(dn, de),
            drone_v_ne=(self._pose.vn_mps, self._pose.ve_mps),
            drone_yaw=self._pose.yaw,
            yaw_target=g.yaw_rad if sighted is None else sighted,
            still=ref.still,
            latch_goal=ref.still and kind == "follow",
            max_speed_mps=cfg.max_speed_mps,
            subject_ne=keep_ne,
            subject_v_ne=keep_v,
            standoff_m=standoff_m,
            near_m=near,
            range_sigma_m=range_sigma,
            scatter_weight=self._filter.scatter_weight,
            coast_sigma_m=coast_sigma,
            guard_ne=guard_ne,
            guard_v_ne=guard_v,
            guard_pace_v_ne=guard_pace,
        )
        # The pilot keeps the subject's body clear by the goal's subject
        # point and this error: the estimate's own, or what the newest fixes
        # put it short by when they put the subject nearer, bounded so the
        # distance the pilot keeps stays inside the standoff.
        short = 0.0 if near is None else math.hypot(keep_ne[0] - dn, keep_ne[1] - de) - near
        subject_sigma = pilot_subject_sigma(standoff_m, max(range_sigma, short / SEPARATION_SIGMAS))
        # The goal point never lies nearer the subject's worst case than the
        # standoff less _GOAL_INSIDE_M: the pilot's planner flies detours to it.
        gn, ge = cmd.north_m, cmd.east_m
        least = max(0.0, standoff_m - _GOAL_INSIDE_M)
        on, oe = gn - guard_ne[0], ge - guard_ne[1]
        gd = math.hypot(on, oe)
        if gd < least:
            if gd <= 1e-6:
                on, oe = dn - guard_ne[0], de - guard_ne[1]
                gd = max(1e-6, math.hypot(on, oe))
            gn, ge = guard_ne[0] + on / gd * least, guard_ne[1] + oe / gd * least
        goal = intent.ring_goal(
            kind,
            (gn, ge, setpoint.height_down(self._pose.down_m, agl, cfg.follow_height_m)),
            cmd,
            cfg,
            ring=ring,
            subject_ne=(keep_ne[0], keep_ne[1]),
            subject_sigma_m=subject_sigma,
        )
        return intent.follow_goal(
            goal,
            cfg=cfg,
            pace_ne=(tvn, tve),
            standoff_m=standoff_m,
            subject_ref=None if subject is None else subject.state.id,
            drone=(dn, de, self._pose.vn_mps, self._pose.ve_mps),
            guard=(guard_ne, guard_v, coast_sigma),
        )

    async def _command(
        self,
        cfg: FollowConfig,
        now: float,
        *,
        phase: str,
        ff_scale: float,
        lock_state: str,
        predicted_bbox: dict[str, object] | None = None,
    ) -> None:
        # The engine's subject state while it is fresh; else the own filter,
        # flown on the coast: a velocity no fix has confirmed for a while
        # decays (see TargetFilter.coasted).
        subject = self._subject_view(now)
        pred = subject.at(now) if subject is not None else self._filter.coasted(now)
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
        x, p = pred
        if not self._range_tier.engaged:
            # The first goal: the minimum standoff starts from where the
            # vehicle is (nothing was raised before it).
            orbit = cfg.follow_mode == FOLLOW_MODE_ORBIT
            self._range_tier.engage(
                math.hypot(x[0] - self._pose.north_m, x[1] - self._pose.east_m),
                cfg.orbit_radius_m if orbit else cfg.follow_distance_m,
            )
            cfg = self._range_tier.apply_to(cfg)
        agl, _src = self._agl(now)
        await self._send_goal(self._goal(cfg, now, x, p, ff_scale, subject), now)
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
        force: bool = False,
        yaw_rad: float | None = None,
        hold_ne: tuple[float, float] | None = None,
        hold_d: float | None = None,
    ) -> None:
        if self._lease_id is not None and active and self._pose.ready(now):
            # Keep the lease alive and the vehicle still: the pilot brings
            # the commanded velocity to zero on its own profile and holds
            # where it stops. The goal names the point the hold began at (or
            # the latched hover point the caller gives, which the drifting
            # pose estimate cannot move) and the heading then (or the one the
            # caller gives); it is not the current pose, so a hold never
            # chases the vehicle round.
            if self._hold_point is None:
                self._hold_point = (
                    self._pose.north_m,
                    self._pose.east_m,
                    self._pose.down_m,
                    self._pose.yaw,
                )
            hn, he, hd, hyaw = self._hold_point
            n, e = (hn, he) if hold_ne is None else hold_ne
            d = hd if hold_d is None else hold_d
            await self._send_goal(
                {
                    "kind": "position",
                    "n": n,
                    "e": e,
                    "d": d,
                    "vn": 0.0,
                    "ve": 0.0,
                    "vd": 0.0,
                    "vel_mode": VEL_MODE_COMMAND,
                    "yaw": hyaw if yaw_rad is None else yaw_rad,
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
            force=force,
        )

    async def _send_goal(self, goal: dict[str, Any], now: float) -> None:
        """Send one goal under the lease and count it toward the published
        send rate (the read-back is where a bench reads the rate). Any goal
        but a hold ends the hold, so the next one latches where it begins.
        A refused goal is logged once per kind and reason and leaves no
        `goal_kind`: the pilot is not flying it."""
        lease_id = self._lease_id
        if lease_id is None:
            return
        kind = goal.get("kind")
        if kind != "position":
            self._hold_point = None
        try:
            reply = await self._ctx.flight.goal_send(lease_id, goal)
        except Exception as exc:  # noqa: BLE001 - the host rejected it at decode
            reply = {"ok": False, "err": str(exc)}
        self._setpoint_times.append(now)
        if isinstance(reply, dict) and not reply.get("ok", True):
            refusal = (kind, lease_error_reason(reply))
            if refusal != self._goal_refusal:
                self._ctx.log.warning("follow_me_goal_refused", kind=kind, err=refusal[1])
            self._goal_refusal = refusal
            self._goal_kind = None
            return
        if self._goal_refusal is not None:
            self._ctx.log.info("follow_me_goal_accepted", kind=kind)
        self._goal_refusal = None
        self._goal_kind = kind

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
        await self._send_gimbal(now, pitch_deg, yaw_deg)

    async def _send_gimbal(self, now: float, pitch_deg: float, yaw_deg: float) -> None:
        """Point the gimbal (pitch negative = down, yaw relative to the
        vehicle), at most every _GIMBAL_PERIOD_S."""
        if (now - self._last_gimbal_s) < _GIMBAL_PERIOD_S:
            return
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
