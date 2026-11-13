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
from dataclasses import dataclass, field, replace
from typing import Any

from ados.sdk.tracking import EffectiveLock

from follow_me import fc_state, setpoint
from follow_me.camera import configured_geometry, geometry_of, sensor_ts_ms
from follow_me.reacquire import PERSON, best_candidate
from follow_me.search import (
    SearchBudget,
    SearchStep,
    search_hypotheses,
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
# ``goal.status`` names no goal, so for this long after a new vantage request
# a status may still be the pilot's report on the previous one (it replans at
# 5 Hz) and is ignored.
STATUS_SETTLE_S = 0.3
# With no status from the pilot for this long while repositioning (a host
# without ``flight.goal.status``, or a pilot that stopped reporting), the
# search starts from where the vehicle is.
STATUS_TIMEOUT_S = 2.0
# A re-acquire designate the engine reports lost is abandoned at once; one it
# has not locked within this long (still unsure, or never reported) is
# abandoned too, and matching resumes.
DESIGNATE_LOCK_S = 3.0
# A vantage the vehicle cannot settle at (gusts, a fence, a pilot that keeps
# replanning) still gets its sweep: once the vehicle has been within
# NEAR_VANTAGE_M of it for NEAR_VANTAGE_S, or the flight there has taken
# REPOSITION_SLACK_S longer than the distance at the search speed.
NEAR_VANTAGE_M = 1.5
NEAR_VANTAGE_S = 2.0
REPOSITION_SLACK_S = 3.0

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
    # When the subject was lost; a resumed search keeps the original loss.
    lost_s: float
    # Where the recovery started: the hard bound of the search.
    anchor_ne: tuple[float, float]
    # The prediction's 1-sigma spread when the recovery began.
    sigma_at_start_m: float = 0.0
    phase: str = PHASE_PREDICTING
    steps: tuple[SearchStep, ...] = ()
    # The search step being flown to or swept, from the first vantage request.
    step_index: int | None = None
    step_started_s: float = 0.0
    # When the current vantage was asked for, and when the pilot last
    # reported on it.
    requested_s: float = 0.0
    status_s: float = -math.inf
    vantage_state: str | None = None
    vantage_ne: tuple[float, float] | None = None
    # When the pilot first named the vantage point for this request, and
    # when the vehicle came within NEAR_VANTAGE_M of it.
    vantage_seen_s: float | None = None
    near_since_s: float | None = None
    occluder: dict[str, object] | None = None
    coverage: float | None = None
    # Whether a re-acquire designate was accepted by the engine, and the
    # track it returned (the engine may omit the id).
    designated: bool = False
    designated_track: int | None = None
    designated_s: float = 0.0
    # Engine tracks this recovery must never adopt: the lost one and every
    # re-acquire designate it abandoned. The engine gives every designate a
    # fresh id, so an operator designate is never among them.
    excluded_tracks: set[int] = field(default_factory=set)
    # Receive time of the last batch already examined, so a batch is matched
    # once.
    last_batch_s: float = -math.inf
    # Holding for the operator: the latched hover point (and height) and the
    # hold reason.
    hold_ne: tuple[float, float] | None = None
    hold_d: float | None = None
    hold_reason: str | None = None
    # The operator_resume_seq already acted on.
    resume_seq_seen: int = 0
    lost_announced: bool = False
    # The appearance and position scores of the candidate behind the
    # current re-acquire designate, announced when the engine locks it.
    designated_similarity: float = 0.0
    designated_mahalanobis: float = 0.0

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
            self._start_recovery(cfg, now, track_id, lost_s=now)
            self._ctx.log.info("follow_me_recovering", track_id=track_id)
            await self._tick_recovery(cfg, now)
            return
        rec = self._start_recovery(cfg, now, track_id, lost_s=now)
        await self._await_operator(cfg, now, rec, reason=REASON_LOCK_LOST)

    def _start_recovery(
        self, cfg: FollowConfig, now: float, track_id: int | None, *, lost_s: float
    ) -> Recovery:
        """A fresh recovery with a full budget, from the filter as it stands."""
        budget = SearchBudget(cfg.search_budget_s, cfg.search_radius_m, cfg.search_speed_mps)
        rec = Recovery(
            lost_track_id=track_id,
            started_s=now,
            deadline_s=now + cfg.search_budget_s,
            lost_s=lost_s,
            anchor_ne=(self._pose.north_m, self._pose.east_m),
            sigma_at_start_m=_sigma_now(self._filter, now),
            steps=tuple(search_steps(budget)),
            resume_seq_seen=cfg.operator_resume_seq,
        )
        if track_id is not None:
            rec.excluded_tracks.add(track_id)
        self._recovery = rec
        return rec

    def _abandon_designate(self, rec: Recovery) -> None:
        """Give up on a re-acquire designate: never adopt its track again."""
        if rec.designated_track is not None:
            rec.excluded_tracks.add(rec.designated_track)
        rec.designated = False
        rec.designated_track = None
        self._tracker.drop()

    async def _await_operator(
        self, cfg: FollowConfig, now: float, rec: Recovery, *, reason: str
    ) -> None:
        """Stop at a latched hover point and wait for the operator. The lease
        is kept: a hold goal goes out every tick, so the pilot's goal timeout
        never fires and the vehicle stays where it stopped."""
        rec.phase = PHASE_AWAIT_OPERATOR
        rec.hold_reason = _HOLD_FOR_REASON[reason]
        rec.hold_ne = (self._pose.north_m, self._pose.east_m)
        rec.hold_d = self._pose.down_m
        rec.resume_seq_seen = cfg.operator_resume_seq
        # Only an operator designate ends the wait: the engine's own lost
        # track and any re-acquire designate still in flight are dropped and
        # never adopted.
        self._abandon_designate(rec)
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
            hold_d=rec.hold_d,
            force=force,
        )

    async def _tick_recovery(self, cfg: FollowConfig, now: float) -> None:
        rec = self._recovery
        assert rec is not None
        if rec.phase == PHASE_AWAIT_OPERATOR:
            await self._tick_await_operator(cfg, now, rec)
            return
        if self._operator_designated(now, rec):
            # The operator picked a subject mid-search: follow it from
            # scratch, with a new filter and template.
            self._recovery = None
            self._filter_track_id = None
            await self._tick_follow(cfg, now)
            return
        if rec.designated:
            lock = self._tracker.effective_lock(now) if self._tracker.has_lock else None
            if lock is EffectiveLock.LOCKED:
                # The engine locked the re-acquire designate: announce the
                # re-lock on the track it locked, and follow it, keeping the
                # subject filter and template.
                track = self._tracker.track_id
                await self._ctx.events.publish(
                    FOLLOW_REACQUIRED_TOPIC,
                    {
                        "track_id": track,
                        "similarity": round(rec.designated_similarity, 3),
                        "gap_s": round(now - rec.lost_s, 2),
                        "mahalanobis": round(rec.designated_mahalanobis, 2),
                        "phase": rec.phase,
                    },
                )
                self._ctx.log.info("follow_me_reacquired", track_id=track, phase=rec.phase)
                self._recovery = None
                self._filter_track_id = track
                self._measured_bbox = None
                await self._tick_follow(cfg, now)
                return
            if lock is EffectiveLock.LOST or (now - rec.designated_s) > DESIGNATE_LOCK_S:
                self._ctx.log.info("follow_me_reacquire_abandoned", track_id=rec.designated_track)
                self._abandon_designate(rec)
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
        # Past the filter's horizon the search goes on around the last
        # estimate, with the spread the subject's speed allows.
        pred = self._filter.search_predicted(now)
        if pred is None:
            await self._await_operator(cfg, now, rec, reason=REASON_SEARCH_EXHAUSTED)
            return
        x, p = pred
        if rec.phase == PHASE_PREDICTING:
            young = (now - rec.started_s) < PREDICT_MAX_S
            spread = position_sigma(p) - rec.sigma_at_start_m
            # With nothing left to extrapolate there is no leg to fly.
            moving = self._filter.predicted(now) is not None
            if young and moving and spread <= PREDICT_SIGMA_M:
                # The prediction leg is held to the search speed, so it cannot
                # carry the vehicle past the search radius before the first
                # vantage request bounds it.
                await self._command(
                    replace(cfg, max_speed_mps=min(cfg.search_speed_mps, cfg.max_speed_mps)),
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
            self._next_vantage(rec, 0, now)
        if rec.phase == PHASE_REPOSITIONING and self._reposition_done(cfg, now, rec):
            rec.phase = PHASE_SEARCHING
            rec.step_started_s = now
        if rec.phase == PHASE_SEARCHING:
            assert rec.step_index is not None
            if (now - rec.step_started_s) >= rec.steps[rec.step_index].dwell_s:
                if rec.step_index + 1 >= len(rec.steps):
                    # The widest step goes round again until the budget is
                    # spent: the whole configured time is searched.
                    rec.step_started_s = now
                else:
                    self._next_vantage(rec, rec.step_index + 1, now)
        await self._send_vantage(cfg, now, rec, x, p)

    def _operator_designated(self, now: float, rec: Recovery) -> bool:
        """A lock on a track that is neither one this recovery gave up on nor
        its own re-acquire designate: the operator designated a subject."""
        if not self._tracker.has_lock:
            return False
        tracked = self._tracker.track_id
        if tracked in rec.excluded_tracks:
            return False
        if rec.designated and (rec.designated_track is None or tracked == rec.designated_track):
            return False
        return self._tracker.effective_lock(now) is EffectiveLock.LOCKED

    def _reposition_done(self, cfg: FollowConfig, now: float, rec: Recovery) -> bool:
        """Whether to sweep from here: the pilot says it arrived or found no
        vantage, it has stopped reporting, the vehicle has hovered near the
        vantage for a while, or the flight there is overdue."""
        if rec.vantage_state in (VANTAGE_ARRIVED, VANTAGE_NONE):
            return True
        if now - max(rec.requested_s, rec.status_s) > STATUS_TIMEOUT_S:
            return True
        if rec.vantage_ne is None or rec.vantage_seen_s is None:
            return False
        dist = math.hypot(
            rec.vantage_ne[0] - self._pose.north_m, rec.vantage_ne[1] - self._pose.east_m
        )
        if dist <= NEAR_VANTAGE_M:
            if rec.near_since_s is None:
                rec.near_since_s = now
            if now - rec.near_since_s >= NEAR_VANTAGE_S:
                return True
        else:
            rec.near_since_s = None
        speed = max(0.1, min(cfg.search_speed_mps, cfg.max_speed_mps))
        return now - rec.vantage_seen_s > dist / speed + REPOSITION_SLACK_S

    def _on_frame_shift(self, dn: float, de: float, dd: float) -> None:
        """The local frame jumped (an EKF position reset): move every point
        held in it by the same step, so each stays put in the world."""
        self._filter.shift(dn, de)
        rec = self._recovery
        if rec is None:
            return
        rec.anchor_ne = (rec.anchor_ne[0] + dn, rec.anchor_ne[1] + de)
        if rec.hold_ne is not None:
            rec.hold_ne = (rec.hold_ne[0] + dn, rec.hold_ne[1] + de)
        if rec.hold_d is not None:
            rec.hold_d += dd
        if rec.vantage_ne is not None:
            rec.vantage_ne = (rec.vantage_ne[0] + dn, rec.vantage_ne[1] + de)

    @staticmethod
    def _next_vantage(rec: Recovery, index: int, now: float) -> None:
        """Ask for the vantage of step ``index``: what the pilot said about
        the previous goal no longer applies."""
        rec.phase = PHASE_REPOSITIONING
        rec.step_index = index
        rec.requested_s = now
        rec.status_s = -math.inf
        rec.vantage_state = None
        rec.vantage_ne = None
        rec.vantage_seen_s = None
        rec.near_since_s = None
        rec.occluder = None
        rec.coverage = None

    async def _tick_await_operator(self, cfg: FollowConfig, now: float, rec: Recovery) -> None:
        if self._operator_designated(now, rec):
            # The operator designated a subject: follow it from scratch.
            self._recovery = None
            self._filter_track_id = None
            await self._tick_follow(cfg, now)
            return
        if cfg.operator_resume_seq > rec.resume_seq_seen:
            rec.resume_seq_seen = cfg.operator_resume_seq
            if self._filter.search_predicted(now) is None:
                # Nothing to search around (no estimate was ever made): keep
                # holding, keep the prompt.
                rec.hold_reason = HOLD_NO_RANGE
            else:
                fresh = self._start_recovery(cfg, now, rec.lost_track_id, lost_s=rec.lost_s)
                # The tracks the earlier search gave up on stay given up on,
                # and the batches it already examined are not matched again.
                fresh.excluded_tracks |= rec.excluded_tracks
                fresh.last_batch_s = self._latest_batch_s
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
                    # Besides the prediction: where the subject was last seen
                    # (it stopped) and a little back along its track (it
                    # turned back). The pilot counts them like ellipse samples.
                    "hypotheses_ne": [[n, e] for n, e in self._hypotheses()],
                },
            },
            now,
        )
        await self._publish_state(
            cfg, now=now, active=True, phase=rec.phase, lock_state=LOCK_LOST, commanding=True
        )

    def _hypotheses(self) -> list[tuple[float, float]]:
        last = self._filter.last_estimate()
        return [] if last is None else search_hypotheses(last)

    def _on_goal_status(self, payload: dict[str, Any]) -> None:
        """The pilot's report on the vantage goal (``flight.goal.status``)."""
        rec = self._recovery
        if rec is None or self._lease_id is None:
            return
        if str(payload.get("lease_id")) != str(self._lease_id):
            return
        now = fc_state.monotonic()
        if rec.phase != PHASE_REPOSITIONING and rec.phase != PHASE_SEARCHING:
            return
        if now - rec.requested_s < STATUS_SETTLE_S:
            return
        rec.status_s = now
        state = payload.get("state")
        if state in _VANTAGE_STATES:
            rec.vantage_state = str(state)
        rec.vantage_ne = _pair(payload.get("vantage_ne"))
        if rec.vantage_ne is not None and rec.vantage_seen_s is None:
            rec.vantage_seen_s = now
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
            # The engine may have applied it before the request failed (a
            # timeout): treat it as a designate of our own whose track is
            # unknown, so its lock is a re-acquire, announced, and not taken
            # for an operator designate.
            self._ctx.log.warning("follow_me_reacquire_designate_failed", error=str(exc))
            self._mark_designated(rec, None, cand, now)
            return
        if not (isinstance(result, dict) and result.get("designated")):
            return
        track = result.get("track_id")
        self._mark_designated(rec, int(track) if isinstance(track, int) else None, cand, now)
        self._filter.update(now, cand.north_m, cand.east_m, cand.covariance)
        self._ctx.log.info(
            "follow_me_reacquire_designated",
            track_id=rec.designated_track,
            similarity=round(cand.similarity, 3),
            phase=rec.phase,
        )

    @staticmethod
    def _mark_designated(rec: Recovery, track: int | None, cand: Any, now: float) -> None:
        rec.designated = True
        rec.designated_track = track
        rec.designated_s = now
        rec.designated_similarity = cand.similarity
        rec.designated_mahalanobis = cand.mahalanobis


def _sigma_now(filt: Any, now: float) -> float:
    pred = filt.search_predicted(now)
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
