"""Lock loss: strict give-up, and the smart coast + re-acquire window.

A mixin of :class:`follow_me.FollowMePlugin`, beside
:class:`follow_me.loop.FollowLoopMixin` (whose ``_hold``, ``_command``,
``_gate`` and ``_tick_follow`` it drives). The pure matching rules live in
:mod:`follow_me.reacquire`.
"""

from __future__ import annotations

from typing import Any

from ados.sdk.tracking import EffectiveLock

from follow_me.camera import configured_geometry, geometry_of, sensor_ts_ms
from follow_me.reacquire import PERSON, Coast, best_candidate
from follow_me.state import (
    FOLLOW_LOST_TOPIC,
    FOLLOW_REACQUIRED_TOPIC,
    HOLD_LOCK_LOST,
    LOCK_LOST,
    PHASE_COASTING,
    PHASE_LOST,
    PHASE_SEARCHING,
    REACQUIRE_SMART,
    FollowConfig,
)
from follow_me.target_filter import position_sigma

# Feed-forward scale while coasting on a prediction.
_COAST_FF_SCALE = 0.5
# Prediction spread past which the coast is "searching": no box can be drawn.
_SEARCH_SIGMA_M = 1.5


class CoastMixin:
    """Lock-loss handling for the follow loop."""

    _ctx: Any
    _coast: Coast | None
    _tracker: Any
    _filter: Any
    _template: Any
    _last_fix: Any
    _measured_bbox: Any
    _filter_track_id: int | None
    _last_lost_track_id: int | None
    _latest_batch: Any
    _latest_batch_s: float
    _pose: Any

    async def _on_lock_lost(self, cfg: FollowConfig, now: float) -> None:
        track_id = self._tracker.track_id
        self._tracker.drop()
        self._measured_bbox = None
        if cfg.reacquire_mode == REACQUIRE_SMART and self._filter.initialized:
            self._coast = Coast(
                lost_track_id=track_id,
                started_s=now,
                deadline_s=now + cfg.reacquire_window_s,
            )
            self._ctx.log.info(
                "follow_me_coasting", track_id=track_id, window_s=cfg.reacquire_window_s
            )
            await self._tick_coast(cfg, now)
            return
        await self._give_up(cfg, now, track_id, reason="lock_lost")

    async def _give_up(
        self, cfg: FollowConfig, now: float, track_id: int | None, *, reason: str
    ) -> None:
        """Strict loss, or a smart window that expired: hold and wait for the
        operator to designate again."""
        self._coast = None
        self._tracker.drop()
        self._filter.reset()
        self._filter_track_id = None
        self._template = None
        self._last_fix = None
        if track_id is not None and track_id != self._last_lost_track_id:
            self._last_lost_track_id = track_id
            await self._ctx.events.publish(
                FOLLOW_LOST_TOPIC, {"track_id": track_id, "reason": reason}
            )
        await self._hold(
            cfg,
            now,
            phase=PHASE_LOST,
            hold_reason=HOLD_LOCK_LOST,
            lock_state=LOCK_LOST,
            force=True,
        )

    async def _tick_coast(self, cfg: FollowConfig, now: float) -> None:
        coast = self._coast
        assert coast is not None
        if (
            coast.designated
            and self._tracker.has_lock
            and self._tracker.effective_lock(now) is EffectiveLock.LOCKED
        ):
            # The engine locked the re-acquire designate: follow it, keeping
            # the subject filter and template.
            self._coast = None
            self._filter_track_id = self._tracker.track_id
            self._measured_bbox = None
            await self._tick_follow(cfg, now)
            return
        if cfg.reacquire_mode != REACQUIRE_SMART:
            # Switched to strict mid-coast.
            await self._give_up(cfg, now, coast.lost_track_id, reason="lock_lost")
            return
        if now >= coast.deadline_s:
            await self._give_up(
                cfg, now, coast.lost_track_id, reason="reacquire_timeout"
            )
            return
        gate = self._gate(now)
        if gate is not None:
            await self._hold(
                cfg,
                now,
                phase=PHASE_COASTING,
                hold_reason=gate,
                lock_state=LOCK_LOST,
                coast_remaining_s=coast.remaining_s(now),
            )
            return
        if not coast.designated:
            await self._try_reacquire(cfg, now, coast)
        pred = self._filter.predicted(now)
        if pred is None:
            await self._give_up(
                cfg, now, coast.lost_track_id, reason="reacquire_timeout"
            )
            return
        box = self._predicted_bbox(cfg, now, pred[0])
        tight = box is not None and position_sigma(pred[1]) <= _SEARCH_SIGMA_M
        await self._command(
            cfg,
            now,
            phase=PHASE_COASTING if tight else PHASE_SEARCHING,
            ff_scale=_COAST_FF_SCALE,
            lock_state=LOCK_LOST,
            coast_remaining_s=coast.remaining_s(now),
            predicted_bbox=box,
        )

    async def _try_reacquire(self, cfg: FollowConfig, now: float, coast: Coast) -> None:
        batch = self._latest_batch
        if batch is None or self._latest_batch_s <= coast.last_batch_s:
            return
        coast.last_batch_s = self._latest_batch_s
        fw = getattr(batch, "frame_width", None)
        fh = getattr(batch, "frame_height", None)
        frame_w, frame_h = (int(fw), int(fh)) if fw and fh else self._frame_size()
        # The boxes are projected through the attitude their frame was
        # exposed at, with the geometry of the camera that took them.
        ts_ms = int(getattr(batch, "ts_ms", 0) or 0)
        exposed_s = self._exposure_s(ts_ms, sensor_ts_ms(batch))
        geometry = geometry_of(batch, configured_geometry(cfg))

        def measure(bbox: Any) -> tuple[float, float, Any] | None:
            fix = self._measure_box(cfg, now, bbox, frame_w, frame_h, geometry, exposed_s)
            if fix is None:
                return None
            return (
                self._pose.north_m + fix.offset_north_m,
                self._pose.east_m + fix.offset_east_m,
                fix.covariance(),
            )

        cand = best_candidate(
            getattr(batch, "detections", None) or (),
            template=self._template,
            min_similarity=cfg.reacquire_min_similarity,
            measure=measure,
            gate=lambda n, e, cov: self._filter.mahalanobis(now, n, e, cov),
        )
        if cand is None:
            return
        det = cand.detection
        try:
            result = await self._ctx.vision.designate_track(
                batch.camera_id,
                det.bbox,
                class_label=PERSON,
                confidence=float(det.confidence),
            )
        except Exception as exc:  # noqa: BLE001
            self._ctx.log.warning("follow_me_reacquire_designate_failed", error=str(exc))
            return
        if not (isinstance(result, dict) and result.get("designated")):
            return
        track = result.get("track_id")
        coast.designated = True
        coast.designated_track = int(track) if isinstance(track, int) else None
        self._filter.update(now, cand.north_m, cand.east_m, cand.covariance)
        await self._ctx.events.publish(
            FOLLOW_REACQUIRED_TOPIC,
            {
                "track_id": coast.designated_track,
                "similarity": round(cand.similarity, 3),
                "gap_s": round(now - coast.started_s, 2),
            },
        )
        self._ctx.log.info(
            "follow_me_reacquired",
            track_id=coast.designated_track,
            similarity=round(cand.similarity, 3),
        )


__all__ = ["CoastMixin"]
