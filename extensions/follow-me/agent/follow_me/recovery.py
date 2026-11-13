"""Lock loss: the predictive recovery, and holding for the operator.

A mixin of :class:`follow_me.FollowMePlugin`, beside
:class:`follow_me.loop.FollowLoopMixin` (whose ``_hold``, ``_command``,
``_gate``, ``_send_goal`` and ``_tick_follow`` it drives). The pure matching
rules live in :mod:`follow_me.reacquire`, the search steps in
:mod:`follow_me.search`.

On a loss in ``smart`` mode the follow

1. **predicts**: flies toward the predicted standoff at reduced feed-forward
   while the prediction is young and has not spread far (``PREDICT_MAX_S``,
   ``PREDICT_SIGMA_M``);
2. **repositions**: asks the pilot for a ``vantage`` goal, a point within
   ``search_radius_m`` of where the loss happened that sees the predicted
   position's spread past the obstacles;
3. **searches**: at the vantage point, sweeps the camera across the
   predicted position, then asks for a wider vantage, step by step;

and checks every new detection batch for the same person (appearance and
position) in each of the three. The whole recovery shares one budget
(``search_budget_s``). When it runs out, in ``strict`` mode at once, and with
``predictive_search`` off after the prediction phase, the vehicle **holds for
the operator** at a latched hover point and never re-locks on its own: the
operator resumes the search, designates a subject, or stops.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from ados.sdk.tracking import EffectiveLock

from follow_me import setpoint
from follow_me.camera import configured_geometry, geometry_of, sensor_ts_ms
from follow_me.reacquire import PERSON, best_candidate
from follow_me.search import (
    SearchBudget,
    SearchStep,
    search_steps,
    sweep_pitch,
    sweep_yaw,
)
from follow_me.state import (
    FOLLOW_LOST_TOPIC,
    FOLLOW_REACQUIRED_TOPIC,
    HOLD_AWAIT_OPERATOR,
    HOLD_NO_RANGE,
    HOLD_PREDICTIVE_OFF,
    HOLD_SEARCH_EXHAUSTED,
    LOCK_LOST,
    PHASE_AWAIT_OPERATOR,
    PHASE_PREDICTING,
    PHASE_REPOSITIONING,
    PHASE_SEARCHING,
    REACQUIRE_SMART,
    FollowConfig,
)
from follow_me.target_filter import position_sigma

# The prediction phase lasts at most this long, and ends early once the
# prediction has spread by more than PREDICT_SIGMA_M (1 sigma) since the
# recovery began: the spread at the loss is the measurement's, not the
# prediction's.
PREDICT_MAX_S = 2.0
PREDICT_SIGMA_M = 1.5
# Feed-forward scale while flying on a prediction rather than a sighting.
SEARCH_FF_SCALE = 0.5
# The goal kind that asks the pilot to choose a vantage point.
VANTAGE_GOAL = "vantage"
# An occluder narrower than this (a trunk, a column) is ``narrow``; anything
# wider (a wall, a corner) is ``wide``.
NARROW_OCCLUDER_M = 1.5

# Vantage states as the pilot reports them.
VANTAGE_ENROUTE = "enroute"
VANTAGE_ARRIVED = "arrived"
VANTAGE_NONE = "no-vantage"
_VANTAGE_STATES = (VANTAGE_ENROUTE, VANTAGE_ARRIVED, VANTAGE_NONE)

# Why the follow is holding for the operator: the ``follow.lost`` reason and
# the published hold reason.
REASON_LOCK_LOST = "lock_lost"
REASON_SEARCH_EXHAUSTED = "search_exhausted"
REASON_PREDICTIVE_OFF = "predictive_search_off"
_HOLD_FOR_REASON = {
    REASON_LOCK_LOST: HOLD_AWAIT_OPERATOR,
    REASON_SEARCH_EXHAUSTED: HOLD_SEARCH_EXHAUSTED,
    REASON_PREDICTIVE_OFF: HOLD_PREDICTIVE_OFF,
}


@dataclass
class Recovery:
    """One recovery, from the loss to a re-lock or the operator."""

    lost_track_id: int | None
    started_s: float
    deadline_s: float
    # Where the recovery started: the hard bound of the search.
    anchor_ne: tuple[float, float]
    # The prediction's 1-sigma spread when the recovery began.
    sigma_at_start_m: float = 0.0
    phase: str = PHASE_PREDICTING
    steps: tuple[SearchStep, ...] = ()
    # The search step being flown to or swept, from the first vantage request.
    step_index: int | None = None
    step_started_s: float = 0.0
    vantage_state: str | None = None
    vantage_ne: tuple[float, float] | None = None
    occluder: dict[str, object] | None = None
    coverage: float | None = None
    # Whether a re-acquire designate was accepted by the engine, and the
    # track it returned (the engine may omit the id).
    designated: bool = False
    designated_track: int | None = None
    # Receive time of the last batch already examined, so a batch is matched
    # once.
    last_batch_s: float = -math.inf
    # Holding for the operator: the latched hover point and the hold reason.
    hold_ne: tuple[float, float] | None = None
    hold_reason: str | None = None
    # The operator_resume_seq already acted on.
    resume_seq_seen: int = 0
    lost_announced: bool = False

    def remaining_s(self, now: float) -> float:
        return max(0.0, self.deadline_s - now)

    @property
    def sigma_k(self) -> float:
        if self.step_index is None or not self.steps:
            return 1.0
        return self.steps[min(self.step_index, len(self.steps) - 1)].sigma_k


class RecoveryMixin:
    """Lock-loss handling for the follow loop."""

    _ctx: Any
    _recovery: Recovery | None
    _tracker: Any
    _filter: Any
    _template: Any
    _measured_bbox: Any
    _filter_track_id: int | None
    _latest_batch: Any
    _latest_batch_s: float
    _pose: Any
    _lease_id: Any
    _uncertain_since_s: float | None

    async def _on_lock_lost(self, cfg: FollowConfig, now: float) -> None:
        track_id = self._tracker.track_id
        self._tracker.drop()
        self._measured_bbox = None
        self._uncertain_since_s = None
        if cfg.reacquire_mode == REACQUIRE_SMART and self._filter.initialized:
            self._start_recovery(cfg, now, track_id)
            self._ctx.log.info("follow_me_recovering", track_id=track_id)
            await self._tick_recovery(cfg, now)
            return
        rec = self._new_recovery(cfg, now, track_id)
        self._recovery = rec
        await self._await_operator(cfg, now, rec, reason=REASON_LOCK_LOST)

    def _new_recovery(self, cfg: FollowConfig, now: float, track_id: int | None) -> Recovery:
        budget = SearchBudget(cfg.search_budget_s, cfg.search_radius_m, cfg.search_speed_mps)
        return Recovery(
            lost_track_id=track_id,
            started_s=now,
            deadline_s=now + cfg.search_budget_s,
            anchor_ne=(self._pose.north_m, self._pose.east_m),
            sigma_at_start_m=_sigma_now(self._filter, now),
            steps=tuple(search_steps(budget)),
            resume_seq_seen=cfg.operator_resume_seq,
        )

    def _start_recovery(self, cfg: FollowConfig, now: float, track_id: int | None) -> Recovery:
        """A fresh recovery with a full budget, from the filter as it stands."""
        rec = self._new_recovery(cfg, now, track_id)
        self._recovery = rec
        return rec

    async def _await_operator(
        self, cfg: FollowConfig, now: float, rec: Recovery, *, reason: str
    ) -> None:
        """Stop at a latched hover point and wait for the operator. The lease
        is kept: a hold goal goes out every tick, so the pilot's goal timeout
        never fires and the vehicle stays where it stopped."""
        rec.phase = PHASE_AWAIT_OPERATOR
        rec.hold_reason = _HOLD_FOR_REASON[reason]
        rec.hold_ne = (self._pose.north_m, self._pose.east_m)
        rec.resume_seq_seen = cfg.operator_resume_seq
        rec.designated = False
        if not rec.lost_announced:
            rec.lost_announced = True
            await self._ctx.events.publish(
                FOLLOW_LOST_TOPIC,
                {
                    "track_id": rec.lost_track_id,
                    "reason": reason,
                    "awaiting_operator": True,
                    "hold_ne": [rec.hold_ne[0], rec.hold_ne[1]],
                },
            )
        self._ctx.log.info("follow_me_awaiting_operator", reason=reason)
        await self._hold_for_operator(cfg, now, rec, force=True)

    async def _hold_for_operator(
        self, cfg: FollowConfig, now: float, rec: Recovery, *, force: bool = False
    ) -> None:
        await self._hold(
            cfg,
            now,
            phase=PHASE_AWAIT_OPERATOR,
            hold_reason=rec.hold_reason or HOLD_AWAIT_OPERATOR,
            lock_state=LOCK_LOST,
            hold_ne=rec.hold_ne,
            force=force,
        )

    async def _tick_recovery(self, cfg: FollowConfig, now: float) -> None:
        rec = self._recovery
        assert rec is not None
        if rec.phase == PHASE_AWAIT_OPERATOR:
            await self._tick_await_operator(cfg, now, rec)
            return
        if (
            rec.designated
            and self._tracker.has_lock
            and self._tracker.effective_lock(now) is EffectiveLock.LOCKED
        ):
            # The engine locked the re-acquire designate: follow it, keeping
            # the subject filter and template.
            self._recovery = None
            self._filter_track_id = self._tracker.track_id
            self._measured_bbox = None
            await self._tick_follow(cfg, now)
            return
        if cfg.reacquire_mode != REACQUIRE_SMART:
            # Switched to strict mid-recovery.
            await self._await_operator(cfg, now, rec, reason=REASON_LOCK_LOST)
            return
        if now >= rec.deadline_s:
            await self._await_operator(cfg, now, rec, reason=REASON_SEARCH_EXHAUSTED)
            return
        gate = self._gate(now)
        if gate is not None:
            await self._hold(cfg, now, phase=rec.phase, hold_reason=gate, lock_state=LOCK_LOST)
            return
        if not rec.designated:
            await self._try_reacquire(cfg, now, rec)
        pred = self._filter.predicted(now)
        if pred is None:
            await self._await_operator(cfg, now, rec, reason=REASON_SEARCH_EXHAUSTED)
            return
        x, p = pred
        if rec.phase == PHASE_PREDICTING:
            young = (now - rec.started_s) < PREDICT_MAX_S
            spread = position_sigma(p) - rec.sigma_at_start_m
            if young and spread <= PREDICT_SIGMA_M:
                await self._command(
                    cfg,
                    now,
                    phase=PHASE_PREDICTING,
                    ff_scale=SEARCH_FF_SCALE,
                    lock_state=LOCK_LOST,
                    predicted_bbox=self._predicted_bbox(cfg, now, x),
                )
                return
            if not cfg.predictive_search:
                await self._await_operator(cfg, now, rec, reason=REASON_PREDICTIVE_OFF)
                return
            if not rec.steps:
                await self._await_operator(cfg, now, rec, reason=REASON_SEARCH_EXHAUSTED)
                return
            self._next_vantage(rec, 0)
        if rec.phase == PHASE_REPOSITIONING and rec.vantage_state in (VANTAGE_ARRIVED, VANTAGE_NONE):
            rec.phase = PHASE_SEARCHING
            rec.step_started_s = now
        if rec.phase == PHASE_SEARCHING:
            assert rec.step_index is not None
            if (now - rec.step_started_s) >= rec.steps[rec.step_index].dwell_s:
                if rec.step_index + 1 >= len(rec.steps):
                    await self._await_operator(cfg, now, rec, reason=REASON_SEARCH_EXHAUSTED)
                    return
                self._next_vantage(rec, rec.step_index + 1)
        await self._send_vantage(cfg, now, rec, x, p)

    @staticmethod
    def _next_vantage(rec: Recovery, index: int) -> None:
        """Ask for the vantage of step ``index``: what the pilot said about
        the previous goal no longer applies."""
        rec.phase = PHASE_REPOSITIONING
        rec.step_index = index
        rec.vantage_state = None
        rec.vantage_ne = None
        rec.occluder = None
        rec.coverage = None

    async def _tick_await_operator(self, cfg: FollowConfig, now: float, rec: Recovery) -> None:
        if self._tracker.has_lock and self._tracker.effective_lock(now) is EffectiveLock.LOCKED:
            # The operator designated a subject: follow it from scratch.
            self._recovery = None
            self._filter_track_id = None
            await self._tick_follow(cfg, now)
            return
        if cfg.operator_resume_seq > rec.resume_seq_seen:
            rec.resume_seq_seen = cfg.operator_resume_seq
            if self._filter.predicted(now) is None:
                # Nothing left to predict from: keep holding, keep the prompt.
                rec.hold_reason = HOLD_NO_RANGE
            else:
                self._start_recovery(cfg, now, rec.lost_track_id)
                self._ctx.log.info("follow_me_search_resumed", seq=cfg.operator_resume_seq)
                await self._tick_recovery(cfg, now)
                return
        await self._hold_for_operator(cfg, now, rec)

    async def _send_vantage(
        self, cfg: FollowConfig, now: float, rec: Recovery, x: list[float], p: Any
    ) -> None:
        refusal = await self._ensure_lease(now)
        if refusal is not None:
            await self._hold(cfg, now, phase=rec.phase, hold_reason=refusal, lock_state=LOCK_LOST)
            return
        assert rec.step_index is not None
        step = rec.steps[rec.step_index]
        pn, pe = x[0], x[1]
        yaw: float | None = None
        if rec.phase == PHASE_SEARCHING:
            t = now - rec.step_started_s
            base = math.atan2(pe - self._pose.east_m, pn - self._pose.north_m)
            yaw = setpoint.wrap_pi(sweep_yaw(step, base, t))
            if cfg.gimbal_point:
                # Straight ahead of the vehicle, at the step's search pitch.
                await self._send_gimbal(now, sweep_pitch(step, t), 0.0)
        agl, _src = self._agl(now)
        await self._send_goal(
            {
                "kind": VANTAGE_GOAL,
                # The pilot replaces the position with the vantage point it
                # chooses; until then the vehicle holds where it is.
                "n": self._pose.north_m,
                "e": self._pose.east_m,
                "d": setpoint.height_down(self._pose.down_m, agl, cfg.follow_height_m),
                "vn": 0.0,
                "ve": 0.0,
                "vd": 0.0,
                "yaw": yaw,
                "yaw_rate": None,
                "subject_ne": [pn, pe],
                "max_speed_mps": min(cfg.search_speed_mps, cfg.max_speed_mps),
                "candidates": None,
                "margin_m": cfg.obstacle_margin_m,
                "vantage": {
                    "predicted_ne": [pn, pe],
                    "cov_ne": [p[0][0], p[0][1], p[1][1]],
                    "sigma_k": step.sigma_k,
                    "standoff_m": cfg.follow_distance_m,
                    "max_radius_m": cfg.search_radius_m,
                    "anchor_ne": [rec.anchor_ne[0], rec.anchor_ne[1]],
                },
            },
            now,
        )
        await self._publish_state(
            cfg, now=now, active=True, phase=rec.phase, lock_state=LOCK_LOST, commanding=True
        )

    def _on_goal_status(self, payload: dict[str, Any]) -> None:
        """The pilot's report on the vantage goal (``flight.goal.status``)."""
        rec = self._recovery
        if rec is None or self._lease_id is None:
            return
        if str(payload.get("lease_id")) != str(self._lease_id):
            return
        state = payload.get("state")
        if state in _VANTAGE_STATES:
            rec.vantage_state = str(state)
        rec.vantage_ne = _pair(payload.get("vantage_ne"))
        cov = payload.get("coverage")
        rec.coverage = float(cov) if isinstance(cov, (int, float)) else None
        occ = _pair(payload.get("occluder_ne"))
        width = payload.get("occluder_width_m")
        if occ is None or not isinstance(width, (int, float)):
            rec.occluder = None
        else:
            kind = "narrow" if float(width) < NARROW_OCCLUDER_M else "wide"
            rec.occluder = {"ne": [occ[0], occ[1]], "kind": kind}

    async def _try_reacquire(self, cfg: FollowConfig, now: float, rec: Recovery) -> None:
        batch = self._latest_batch
        if batch is None or self._latest_batch_s <= rec.last_batch_s:
            return
        rec.last_batch_s = self._latest_batch_s
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
        rec.designated = True
        rec.designated_track = int(track) if isinstance(track, int) else None
        self._filter.update(now, cand.north_m, cand.east_m, cand.covariance)
        await self._ctx.events.publish(
            FOLLOW_REACQUIRED_TOPIC,
            {
                "track_id": rec.designated_track,
                "similarity": round(cand.similarity, 3),
                "gap_s": round(now - rec.started_s, 2),
                "mahalanobis": round(cand.mahalanobis, 2),
                "phase": rec.phase,
            },
        )
        self._ctx.log.info(
            "follow_me_reacquired",
            track_id=rec.designated_track,
            similarity=round(cand.similarity, 3),
            phase=rec.phase,
        )


def _sigma_now(filt: Any, now: float) -> float:
    pred = filt.predicted(now)
    return 0.0 if pred is None else position_sigma(pred[1])


def _pair(raw: Any) -> tuple[float, float] | None:
    if not isinstance(raw, (list, tuple)) or len(raw) != 2:
        return None
    try:
        a, b = float(raw[0]), float(raw[1])
    except (TypeError, ValueError):
        return None
    return (a, b) if math.isfinite(a) and math.isfinite(b) else None


__all__ = [
    "NARROW_OCCLUDER_M",
    "PREDICT_MAX_S",
    "PREDICT_SIGMA_M",
    "SEARCH_FF_SCALE",
    "VANTAGE_GOAL",
    "Recovery",
    "RecoveryMixin",
]
