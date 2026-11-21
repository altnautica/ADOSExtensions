"""The vision engine's estimate of the designated subject, as the follow flies
it. A mixin of :class:`follow_me.FollowMePlugin`.

The agent publishes the subject the operator designated as
``vision.subject.v1`` (read with ``event.subscribe`` and
``vision.subject.read``): its ground-contact point in local NED, velocity,
covariance, predictions 0.5-3 s ahead (kept out of space a map reports
occupied) and the cues that fed it (``box``, ``depth``, ``ground``,
``beacon``). While a state is fresh (stamped within :data:`SUBJECT_FRESH_S`)
and names the track the follow is locked on, the goals, the gimbal and the
recovery's prediction are built on it; otherwise on the follow's own
estimate from the boxes, as without the topic.

A state fed by the operator's phone alone (``sources == {beacon}``, vision
has lost the subject) keeps the follow flying on an unsure or lost lock,
within the same separation guard, sized by the beacon's spread; the
read-back says so (``subject_source: beacon``). A strict re-acquire, or an
unsure lock set to hold, flies on vision alone and holds as before. A
release of the locked track (the designation ended or the engine lost the
subject past its coast) is a loss: the recovery or the wait for the
operator starts at once.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from follow_me.state import (
    LOCK_LOST,
    LOCK_UNCERTAIN,
    PHASE_FOLLOWING,
    PHASE_HOLDING,
    REACQUIRE_SMART,
    UNCERTAIN_FOLLOW,
    FollowConfig,
)

# A state older than this (by its stamp, on the frame clock the engine
# steps on: ADOS time) is not flown on.
SUBJECT_FRESH_S = 0.5
# A stamp this far ahead of the plugin's clock is a clock mismatch.
_FUTURE_SLACK_S = 0.1
# The velocity spread taken when a state carries no prediction to read it
# from, and the least read from one.
_NO_PREDICTION_VEL_SIGMA_MPS = 1.0
_MIN_VEL_SIGMA_MPS = 0.2
# Fix age past which the estimate is coasting (the follow's own filter
# starts its coast as late at the soonest).
_COAST_GRACE_S = 0.5

SOURCE_BEACON = "beacon"
# What ``subject_source`` names in the read-back: vision cues fed the
# state (a beacon may have too), the beacon alone, or no cue within the
# last second (the engine's prediction).
SUBJECT_SOURCE_VISION = "vision"
SUBJECT_SOURCE_BEACON = "beacon"
SUBJECT_SOURCE_PREDICTED = "predicted"

Mat = list[list[float]]


@dataclass(frozen=True)
class SubjectView:
    """One fresh state of the locked subject, read at any time near it."""

    state: Any
    stamp_s: float

    @property
    def beacon_only(self) -> bool:
        return frozenset(self.state.sources) == frozenset({SOURCE_BEACON})

    @property
    def source(self) -> str:
        if self.beacon_only:
            return SUBJECT_SOURCE_BEACON
        return SUBJECT_SOURCE_VISION if self.state.sources else SUBJECT_SOURCE_PREDICTED

    def fix_age_s(self, t: float) -> float:
        """Seconds since the last measurement of any kind, at ``t``."""
        return self.state.fix_age_ms / 1000.0 + max(0.0, t - self.stamp_s)

    def _knots(self) -> list[tuple[float, tuple[float, ...], tuple[float, ...]]]:
        s = self.state
        out = [(0.0, tuple(s.pos_ned), tuple(s.cov))]
        out += sorted(
            (
                (p.dt_s, tuple(p.pos_ned), tuple(p.cov))
                for p in s.predictions
                if p.dt_s > 0.0
            ),
            key=lambda k: k[0],
        )
        return out

    def _vel_var(self) -> tuple[float, float]:
        """North and east velocity variances: how fast the first prediction's
        spread grows on the state's."""
        knots = self._knots()
        floor = _MIN_VEL_SIGMA_MPS * _MIN_VEL_SIGMA_MPS
        if len(knots) < 2:
            v = _NO_PREDICTION_VEL_SIGMA_MPS * _NO_PREDICTION_VEL_SIGMA_MPS
            return (v, v)
        (_t0, _p0, c0), (t1, _p1, c1) = knots[0], knots[1]
        return (
            max(floor, (c1[0] - c0[0]) / (t1 * t1)),
            max(floor, (c1[4] - c0[4]) / (t1 * t1)),
        )

    def at(self, t: float) -> tuple[list[float], Mat]:
        """``(n, e, vn, ve)`` and its 4 x 4 covariance at ``t``: the state and
        its predictions interpolated, and run on at the velocity past the
        last one."""
        s = self.state
        dt = t - self.stamp_s
        vn, ve = s.vel_ned[0], s.vel_ned[1]
        vvn, vve = self._vel_var()
        knots = self._knots()
        if dt <= 0.0:
            _k, pos, cov = knots[0]
            n, e = pos[0] + vn * dt, pos[1] + ve * dt
            cnn, cne, cee = cov[0], cov[1], cov[4]
        else:
            lo = knots[0]
            for hi in knots[1:]:
                if dt <= hi[0]:
                    w = (dt - lo[0]) / (hi[0] - lo[0])
                    n = lo[1][0] + w * (hi[1][0] - lo[1][0])
                    e = lo[1][1] + w * (hi[1][1] - lo[1][1])
                    cnn = lo[2][0] + w * (hi[2][0] - lo[2][0])
                    cne = lo[2][1] + w * (hi[2][1] - lo[2][1])
                    cee = lo[2][4] + w * (hi[2][4] - lo[2][4])
                    break
                lo = hi
            else:
                run = dt - lo[0]
                n, e = lo[1][0] + vn * run, lo[1][1] + ve * run
                cnn, cne, cee = (
                    lo[2][0] + vvn * run * run,
                    lo[2][1],
                    lo[2][4] + vve * run * run,
                )
        p = [
            [cnn, cne, 0.0, 0.0],
            [cne, cee, 0.0, 0.0],
            [0.0, 0.0, vvn, 0.0],
            [0.0, 0.0, 0.0, vve],
        ]
        return [n, e, vn, ve], p

    def coast_spread_m(self, t: float) -> float:
        """The spread the estimate gained since its last measurement, past
        the coast's grace: 0 while measurements keep coming."""
        coast = self.fix_age_s(t) - _COAST_GRACE_S
        if coast <= 0.0:
            return 0.0
        vvn, vve = self._vel_var()
        return math.sqrt(0.5 * (vvn + vve)) * coast

    def path_ne(self, t: float, *, step_s: float = 0.5) -> list[list[float]]:
        """The predicted position (north, east) at ``t`` and every ``step_s``
        after it, as far as the state predicts."""
        last = self._knots()[-1][0]
        out: list[list[float]] = []
        k = 0
        while (t + k * step_s) - self.stamp_s <= last + 1e-9:
            x, _p = self.at(t + k * step_s)
            out.append([round(x[0], 2), round(x[1], 2)])
            k += 1
        return out


class SubjectFeed:
    """The newest ``vision.subject.v1`` record, and a release not yet acted
    on."""

    def __init__(self) -> None:
        self._state: Any = None
        self._released: tuple[str, int] | None = None

    def on_record(self, record: Any) -> None:
        """A decoded record: a state (it has a position) or a release."""
        if hasattr(record, "pos_ned"):
            self._state = record
            self._released = None
            return
        s = self._state
        if s is not None and s.id == record.id and s.camera_id == record.camera_id:
            self._state = None
        self._released = (str(record.camera_id), int(record.id))

    def clear(self) -> None:
        """Forget the state (the local frame jumped under it)."""
        self._state = None

    def view(
        self, now: float, track_id: int | None, camera_id: str | None
    ) -> SubjectView | None:
        """The state while it is fresh and names ``track_id`` (on
        ``camera_id`` when one is given)."""
        s = self._state
        if s is None or track_id is None or s.id != track_id:
            return None
        if camera_id is not None and s.camera_id != camera_id:
            return None
        stamp_s = s.stamp_ns / 1e9
        age = now - stamp_s
        if age > SUBJECT_FRESH_S or age < -_FUTURE_SLACK_S:
            return None
        return SubjectView(s, stamp_s)

    def take_release(self, track_id: int | None, camera_id: str | None) -> bool:
        """Whether ``track_id`` was released; a release is taken once."""
        r = self._released
        if r is None or track_id is None or r[1] != track_id:
            return False
        if camera_id is not None and r[0] != camera_id:
            return False
        self._released = None
        return True


class SubjectMixin:
    """The subject state's subscription, and following on the beacon."""

    _ctx: Any
    _subject: SubjectFeed
    _filter_track_id: int | None
    _still: Any

    async def _subscribe_subject(self, ctx: Any) -> None:
        """Read the subject state. Optional: an agent that does not publish
        it, or refuses ``vision.subject.read``, leaves the follow on its own
        estimate."""
        subscribe = getattr(ctx.vision, "subscribe_subject", None)
        if subscribe is None:
            ctx.log.info(
                "follow_me_subject_unavailable", reason="host has no subscribe_subject"
            )
            return
        try:
            await subscribe(self._on_subject, camera_id=None)
        except Exception as exc:  # noqa: BLE001
            ctx.log.info("follow_me_subject_unavailable", reason=str(exc))

    def _on_subject(self, record: Any) -> None:
        self._subject.on_record(record)

    def _subject_view(self, now: float) -> SubjectView | None:
        """The fresh state of the subject the follow is locked on."""
        return self._subject.view(now, self._filter_track_id, self._follow_camera())

    def _subject_measured(self, now: float, within_s: float) -> bool:
        """Whether a fresh state carries a measurement within ``within_s``."""
        view = self._subject_view(now)
        return view is not None and view.fix_age_s(now) <= within_s

    async def _follow_beacon(
        self, cfg: FollowConfig, now: float, lock_state: str
    ) -> bool:
        """Follow on the phone beacon alone while vision has lost the subject
        (smart re-acquire, an unsure lock set to follow): whether this tick
        was handled."""
        assert lock_state in (LOCK_UNCERTAIN, LOCK_LOST)
        if (
            cfg.reacquire_mode != REACQUIRE_SMART
            or cfg.uncertain_mode != UNCERTAIN_FOLLOW
        ):
            return False
        view = self._subject_view(now)
        if view is None or not view.beacon_only:
            return False
        # No sighting tells a standing subject from a moving one: the goals
        # are built round the beacon's estimate.
        self._still.reset()
        gate = self._gate(now)
        if gate is not None:
            await self._hold(
                cfg, now, phase=PHASE_HOLDING, hold_reason=gate, lock_state=lock_state
            )
            return True
        await self._command(
            cfg, now, phase=PHASE_FOLLOWING, ff_scale=1.0, lock_state=lock_state
        )
        return True


__all__ = [
    "SUBJECT_FRESH_S",
    "SUBJECT_SOURCE_BEACON",
    "SUBJECT_SOURCE_PREDICTED",
    "SUBJECT_SOURCE_VISION",
    "SubjectFeed",
    "SubjectMixin",
    "SubjectView",
]
