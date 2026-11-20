"""Constant-velocity Kalman filter for the followed subject.

State ``[n, e, vn, ve]`` in the flight controller's local NED frame (metres,
metres per second). The process model is a constant velocity driven by white
acceleration noise of 1.0 m/s^2 (a walking or running person changing pace
and direction); the measurement is the subject's north/east position from one
box (vehicle LOCAL_POSITION_NED plus the projected offset) with the 2x2
range/bearing covariance from :mod:`follow_me.projection`.

Pure Python, no numpy: four states do not need it and the plugin stays free
of a heavy dependency on small companion boards.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Any

Mat = list[list[float]]

# Process noise, the 1-sigma white acceleration, m/s^2: a walker or runner
# changing pace and direction. 1.0 rather than more, so range noise between
# sightings is not read as acceleration.
ACCEL_SIGMA_MPS2 = 1.0
# Velocity sigma a fresh track starts with: a person can be moving at up to
# a run when first seen.
_INIT_VEL_SIGMA_MPS = 3.0
# The constant-velocity extrapolation is refused this far past the last
# update: long enough for the longest search (60 s) plus the tracker's coast
# and an unsure spell before the loss. Past it, a search still has the last
# estimate with a spread bounded by how far the subject can have gone (see
# :meth:`TargetFilter.search_predicted`).
MAX_PREDICT_S = 70.0
# A coast: with no fix fused for COAST_GRACE_FIXES of the track's usual
# intervals between fixes, and at least COAST_GRACE_S (a few missed frames
# ride on the prediction as they are: 0.5 s at 10 Hz, about 1.3 s at 3 Hz
# missing half its frames), the velocity the follow flies on decays, on
# COAST_TAU_S for a velocity known to COAST_VEL_SIGMA_MPS and shorter the
# less it is known (see :meth:`TargetFilter.coasted`). A walk measured by
# five fixes is known to 1.5-2 m/s; flown on for three unsure seconds it
# carried the estimate 7 m past the walker and the vehicle over it.
COAST_GRACE_S = 0.5
COAST_GRACE_FIXES = 3.0
COAST_TAU_S = 2.0
COAST_VEL_SIGMA_MPS = 0.5
# The usual interval is a running mean over this many intervals of up to
# _INTERVAL_MAX_S (longer ones are gaps, not the cadence).
_INTERVAL_FIXES = 10
_INTERVAL_MAX_S = 1.0
# The innovation gate: χ² with two degrees of freedom at 99.73 % (3 sigma).
# A recorded forest follow put 5 % of its range fixes 3.5-7.6 m off the
# subject, outliers no constant-velocity person makes.
GATE_CHI2 = 11.83
# Inside the gate, a fix beyond 2 sigma (χ² 4.0) is fused with its noise
# scaled by how far out it lies.
HUBER_CHI2 = 4.0
# The outlier in a row that is fused anyway when it and the ones before it
# agree (a subject that really did turn or sprint is never gated out): at
# 3 Hz the third fix, 0.7 s on. Scattered outliers (range errors either side
# of the subject) never agree and stay out.
GATE_FORCE_AFTER = 3
# A fix that comes REACQUIRE_GAP_S or more after the last one fused is a
# re-acquisition: the tracker missed the subject for a while, or locked onto
# it again. Over such a gap the filter's own spread admits a jump that only
# a velocity the track never measured explains. A recorded forest follow
# re-locked onto a tree 4 m short of a walker after a 0.95 s gap; the fix
# passed the gate on the velocity spread four fixes had left, the estimate
# came out walking toward the vehicle at 2.4 m/s, and the gate then kept
# the walker's own next fix out. Such a fix is therefore measured against
# what it may move the estimate by on its own: its noise, the estimate's
# position spread and the subject's acceleration (ACCEL_SIGMA_MPS2) over
# the gap, whatever velocity spread the track has. Beyond REACQUIRE_CHI2 of
# that it jumped and is held out; the estimate predicts on. A fix that
# agrees with the estimate again is fused (the jumped one was a false
# lock); REACQUIRE_CONFIRM_FIXES jumped fixes in a row that agree with each
# other start the estimate over on them (the subject is there, and the
# estimate's velocity was wrong).
REACQUIRE_GAP_S = 0.5
REACQUIRE_CHI2 = GATE_CHI2
REACQUIRE_CONFIRM_FIXES = 2
# Rejected fixes agree when each lies within this many of its own sigmas of
# the one before, plus the distance the subject can cover in between.
_AGREE_SIGMAS = 3.0
# The reach used for a subject class with no speed cap, m/s.
_UNCAPPED_REACH_MPS = 5.0
# The range scatter (see :attr:`TargetFilter.scatter_ratio`) averages about
# this many fixes (5 s at 10 Hz): long enough that the innovations of a
# second's manoeuvre barely move it, short enough to follow the cues as the
# range or the light changes. It reads no more than SCATTER_RATIO_MAX. A
# track's first SCATTER_MIN_FIXES fixes read 0 and the next SCATTER_MIN_FIXES
# take it in gradually (about 4 s at 10 Hz): a fresh track's first
# innovations say more about its start than about the cues, and a smoothing
# switched on at once would itself step the command.
SCATTER_FIXES = 50
SCATTER_MIN_FIXES = 20
SCATTER_RATIO_MAX = 2.0

# The fastest a followed subject of each detector class is taken to move,
# m/s. A person's cap is a run on a trail: a velocity estimate above it comes
# from range noise between sightings, not from the subject, and
# extrapolating it carries the estimate away from the subject within a
# second (the innovation gate keeps the range outliers that caused most of it
# out). A faster subject is still followed: each sighting still moves the
# estimate. A class with no entry is not capped here (the goal's
# feed-forward is still clamped to the configured max_speed_mps).
SUBJECT_MAX_SPEED_MPS: dict[str, float] = {"person": 5.0}


def subject_max_speed_mps(class_label: str | None) -> float | None:
    """The speed cap for a subject of ``class_label``, or ``None``."""
    if not class_label:
        return None
    return SUBJECT_MAX_SPEED_MPS.get(class_label.strip().lower())


def _zeros(n: int, m: int) -> Mat:
    return [[0.0] * m for _ in range(n)]


def _mul(a: Mat, b: Mat) -> Mat:
    n, k, m = len(a), len(b), len(b[0])
    out = _zeros(n, m)
    for i in range(n):
        ai = a[i]
        oi = out[i]
        for p in range(k):
            aip = ai[p]
            if aip == 0.0:
                continue
            bp = b[p]
            for j in range(m):
                oi[j] += aip * bp[j]
    return out


def _t(a: Mat) -> Mat:
    return [list(row) for row in zip(*a)]


def _add(a: Mat, b: Mat) -> Mat:
    return [[x + y for x, y in zip(ra, rb)] for ra, rb in zip(a, b)]


def _inv2(m: Mat) -> Mat | None:
    det = m[0][0] * m[1][1] - m[0][1] * m[1][0]
    if abs(det) < 1e-12:
        return None
    return [[m[1][1] / det, -m[0][1] / det], [-m[1][0] / det, m[0][0] / det]]


def _transition(dt: float) -> Mat:
    return [
        [1.0, 0.0, dt, 0.0],
        [0.0, 1.0, 0.0, dt],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _process_noise(dt: float) -> Mat:
    # Discrete white-noise-acceleration model, per axis
    # q^2 * [[dt^4/4, dt^3/2], [dt^3/2, dt^2]].
    q = ACCEL_SIGMA_MPS2 * ACCEL_SIGMA_MPS2
    a = q * dt**4 / 4.0
    b = q * dt**3 / 2.0
    c = q * dt * dt
    return [
        [a, 0.0, b, 0.0],
        [0.0, a, 0.0, b],
        [b, 0.0, c, 0.0],
        [0.0, b, 0.0, c],
    ]


_H = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]]
_HT = _t(_H)

Cov2 = tuple[tuple[float, float], tuple[float, float]]


class TargetFilter:
    """The subject track. Call :meth:`update` with each new measurement and
    :meth:`predicted` to read the state at any later time; prediction never
    mutates the filter, so a coast never accumulates drift into the state.

    ``max_speed_mps`` caps the velocity estimate (see
    :data:`SUBJECT_MAX_SPEED_MPS`); ``None`` leaves it free."""

    def __init__(self) -> None:
        self._x: list[float] | None = None
        self._p: Mat = _zeros(4, 4)
        # The time the state is propagated to, and the last measurement.
        self._t: float = 0.0
        self._t_meas: float = 0.0
        # The 1-sigma position spread just after the last measurement.
        self._sigma_meas: float = 0.0
        # The measurements gated out in a row (t, n, e, sigma), newest last.
        self._rejected: list[tuple[float, float, float, float]] = []
        # The fixes held out in a row as jumped (t, n, e, sigma, covariance),
        # newest last.
        self._jumps: list[tuple[float, float, float, float, Cov2]] = []
        # The running means of the squared innovation along the line of
        # sight and of its predicted variance, over the fixes fused, and how
        # many fixes they average (see :attr:`scatter_ratio`).
        self._scatter_y2 = 0.0
        self._scatter_s = 0.0
        self._scatter_n = 0
        self.max_speed_mps: float | None = None
        # The running mean interval between consecutive fixes fused (the
        # detector's cadence, misses included), or None before two.
        self._interval_s: float | None = None

    @property
    def initialized(self) -> bool:
        return self._x is not None

    @property
    def last_update_s(self) -> float:
        """When the last measurement was fused."""
        return self._t_meas

    @property
    def last_sigma_m(self) -> float:
        """The 1-sigma position spread just after the last measurement."""
        return self._sigma_meas

    @property
    def scatter_ratio(self) -> float:
        """How far the fixes actually scatter about the estimate along the
        line of sight, as a share of what the filter takes them to: the root
        of the mean squared innovation over its predicted variance, over the
        last SCATTER_FIXES or so fixes fused. 1 when the range cues' noise is
        what the fixes are fused with, near 0 on clean ranges (a lidar, a
        depth camera; a subject's own sharp manoeuvre still reads as a
        little scatter). Scaling the filter's own spread by it gives the error
        the estimate really carries. It is :attr:`scatter_measured` taken in
        by :attr:`scatter_weight`: a track's first SCATTER_MIN_FIXES fixes
        read 0 and the next SCATTER_MIN_FIXES take it in gradually."""
        measured = self.scatter_measured
        return 0.0 if measured is None else measured * self.scatter_weight

    @property
    def scatter_measured(self) -> float | None:
        """The scatter ratio the fixes so far measure, or ``None`` for a
        track's first SCATTER_MIN_FIXES fixes."""
        if self._scatter_n <= SCATTER_MIN_FIXES or self._scatter_s <= 0.0:
            return None
        return min(SCATTER_RATIO_MAX, math.sqrt(self._scatter_y2 / self._scatter_s))

    @property
    def scatter_weight(self) -> float:
        """How much of the measured scatter is taken in: 0 for a track's
        first SCATTER_MIN_FIXES fixes, rising to 1 over the next
        SCATTER_MIN_FIXES."""
        if self._scatter_s <= 0.0:
            return 0.0
        return max(0.0, min(1.0, self._scatter_n / SCATTER_MIN_FIXES - 1.0))

    @property
    def scatter_settled(self) -> bool:
        """Whether :attr:`scatter_ratio` reads the measured scatter in full."""
        return self.scatter_weight >= 1.0

    def last_estimate(self) -> tuple[float, float, float, float] | None:
        """The state ``(n, e, vn, ve)`` as last fused (or stopped), not
        propagated."""
        if self._x is None:
            return None
        return (self._x[0], self._x[1], self._x[2], self._x[3])

    def shift(self, dn: float, de: float) -> None:
        """Move the estimate by a jump of the local frame (an EKF position
        reset), so it stays where the subject is in the world."""
        if self._x is not None:
            self._x[0] += dn
            self._x[1] += de

    def reset(self) -> None:
        self._x = None
        self._p = _zeros(4, 4)
        self._t = 0.0
        self._t_meas = 0.0
        self._sigma_meas = 0.0
        self._rejected.clear()
        self._jumps.clear()
        self._scatter_y2 = 0.0
        self._scatter_s = 0.0
        self._scatter_n = 0
        self._interval_s = None

    def stop(self, t: float) -> None:
        """Stop the estimate where it is predicted at ``t``, with no velocity.

        While the lock is unsure no sighting corrects the estimate, and the
        last velocity (often range noise) would carry it metres from the
        subject before the next one. The velocity spread restarts at a fresh
        track's, so the next sightings estimate the motion again. Calling it
        again while the hold lasts leaves the estimate where it is."""
        if self._x is None:
            return
        x, p = self._propagate(t)
        v = _INIT_VEL_SIGMA_MPS * _INIT_VEL_SIGMA_MPS
        self._x = [x[0], x[1], 0.0, 0.0]
        self._p = [
            [p[0][0], p[0][1], 0.0, 0.0],
            [p[1][0], p[1][1], 0.0, 0.0],
            [0.0, 0.0, v, 0.0],
            [0.0, 0.0, 0.0, v],
        ]
        self._t = max(self._t, t)

    def _rejected_agree(self) -> bool:
        """Whether the last GATE_FORCE_AFTER gated-out fixes agree with each
        other: a subject that moved, not scattered range errors."""
        return len(self._rejected) >= GATE_FORCE_AFTER and self._agree(self._rejected)

    def _agree(self, fixes: Sequence[tuple[Any, ...]]) -> bool:
        """Whether each of ``fixes`` (t, n, e, sigma, ...) lies within
        _AGREE_SIGMAS of the two fixes' sigmas of the one before, plus the
        distance the subject can cover in between."""
        reach = self.max_speed_mps or _UNCAPPED_REACH_MPS
        for a, b in zip(fixes, fixes[1:]):
            bound = _AGREE_SIGMAS * math.hypot(a[3], b[3]) + reach * max(0.0, b[0] - a[0])
            if math.hypot(b[1] - a[1], b[2] - a[2]) > bound:
                return False
        return True

    def _cap_speed(self) -> None:
        cap = self.max_speed_mps
        if self._x is None or cap is None:
            return
        speed = math.hypot(self._x[2], self._x[3])
        if speed > cap > 0.0:
            k = cap / speed
            self._x[2] *= k
            self._x[3] *= k

    def _propagate(self, t: float) -> tuple[list[float], Mat]:
        assert self._x is not None
        dt = max(0.0, t - self._t)
        if dt == 0.0:
            return list(self._x), [list(r) for r in self._p]
        f = _transition(dt)
        x = [sum(f[i][j] * self._x[j] for j in range(4)) for i in range(4)]
        p = _add(_mul(_mul(f, self._p), _t(f)), _process_noise(dt))
        return x, p

    def _reach_m(self, t: float) -> float | None:
        """How far from the last estimate the subject can be at ``t`` (3
        sigma): the class speed cap times the time since the last sighting,
        plus that sighting's own 3 sigma. ``None`` without a cap."""
        cap = self.max_speed_mps
        if cap is None or cap <= 0.0:
            return None
        return cap * max(0.0, t - self._t_meas) + 3.0 * self._sigma_meas

    def _bounded(self, t: float, p: Mat) -> Mat:
        """``p`` with its position spread no wider than the reachable
        distance. A constant-velocity spread grows with the cube of time;
        past a few seconds it admits anyone in a wide circle, which a subject
        moving at its speed cap cannot have left."""
        reach = self._reach_m(t)
        s3 = 3.0 * position_sigma(p)
        if reach is None or s3 <= reach or s3 <= 0.0:
            return p
        k = reach / s3
        d = (k, k, 1.0, 1.0)
        return [[p[i][j] * d[i] * d[j] for j in range(4)] for i in range(4)]

    def predicted(self, t: float) -> tuple[list[float], Mat] | None:
        """``(state, covariance)`` propagated to ``t``, or ``None`` before the
        first update or past :data:`MAX_PREDICT_S`. The position spread is
        bounded by the distance the subject can have covered."""
        if self._x is None or (t - self._t) > MAX_PREDICT_S:
            return None
        x, p = self._propagate(t)
        return x, self._bounded(t, p)

    def coasted(self, t: float) -> tuple[list[float], Mat] | None:
        """What the follow flies on at ``t``: :meth:`predicted`, its velocity
        decaying once the coast starts (:meth:`coast_start_s`), on
        :meth:`coast_tau_s`. The position runs on by the decaying velocity's
        integral, so a coast on a velocity few fixes measured does not carry
        the estimate (and the goal) metres past the subject. The covariance
        is the prediction's."""
        pred = self.predicted(t)
        if pred is None or self._x is None:
            return pred
        x, p = pred
        age = t - self.coast_start_s()
        if age <= 0.0:
            return pred
        tau = self.coast_tau_s()
        decay = math.exp(-age / tau)
        # The prediction ran the last velocity on for ``age``; it runs on
        # for tau·(1 − decay) instead.
        run = tau * (1.0 - decay) - age
        vn, ve = self._x[2], self._x[3]
        return [x[0] + vn * run, x[1] + ve * run, vn * decay, ve * decay], p

    def coast_start_s(self) -> float:
        """When a coast starts: COAST_GRACE_FIXES usual fix intervals, and at
        least COAST_GRACE_S, after the last fix fused."""
        grace = COAST_GRACE_S
        if self._interval_s is not None:
            grace = max(grace, COAST_GRACE_FIXES * self._interval_s)
        return max(self._t, self._t_meas + grace)

    def coast_tau_s(self) -> float:
        """How fast a coast lets the velocity go: COAST_TAU_S on a velocity
        known to COAST_VEL_SIGMA_MPS, shorter the less the fixes so far have
        pinned it (a fresh track's few fixes leave it metres a second out)."""
        sv2 = 0.5 * (self._p[2][2] + self._p[3][3])
        return COAST_TAU_S / (1.0 + sv2 / (COAST_VEL_SIGMA_MPS * COAST_VEL_SIGMA_MPS))

    def coast_spread_m(self, t: float) -> float:
        """The position spread the prediction at ``t`` gained over the coast
        (since :meth:`coast_start_s`): 0 while fixes keep coming."""
        if self._x is None:
            return 0.0
        start = self.coast_start_s()
        now, then = self.predicted(t), self.predicted(start)
        if t <= start or now is None or then is None:
            return 0.0
        return math.sqrt(max(0.0, position_sigma(now[1]) ** 2 - position_sigma(then[1]) ** 2))

    def search_predicted(self, t: float) -> tuple[list[float], Mat] | None:
        """What a search uses at ``t``: the prediction while there is one;
        past :data:`MAX_PREDICT_S`, the last estimate standing still with a
        spread of the reachable distance. ``None`` before the first update,
        or past the horizon for a subject with no speed cap."""
        pred = self.predicted(t)
        if pred is not None or self._x is None:
            return pred
        reach = self._reach_m(t)
        if reach is None:
            return None
        var = (reach / 3.0) ** 2
        cap2 = (self.max_speed_mps or 0.0) ** 2
        return (
            [self._x[0], self._x[1], 0.0, 0.0],
            [
                [var, 0.0, 0.0, 0.0],
                [0.0, var, 0.0, 0.0],
                [0.0, 0.0, cap2, 0.0],
                [0.0, 0.0, 0.0, cap2],
            ],
        )

    def mahalanobis(self, t: float, north: float, east: float, r: Cov2) -> float | None:
        """Mahalanobis distance of a position measurement from the search
        prediction at ``t``, including the measurement's own covariance. A
        point further from the last estimate than the subject can have gone
        (plus the measurement's own 3 sigma) is infinitely far: the bounded
        spread is centred on the prediction, which runs along the track, so
        on its own it would still admit points beyond reach ahead of it."""
        pred = self.search_predicted(t)
        if pred is None or self._x is None:
            return None
        reach = self._reach_m(t)
        if reach is not None:
            r_sigma = position_sigma([list(r[0]), list(r[1])])
            gone = math.hypot(north - self._x[0], east - self._x[1])
            if gone > reach + 3.0 * r_sigma:
                return math.inf
        x, p = pred
        s = [
            [p[0][0] + r[0][0], p[0][1] + r[0][1]],
            [p[1][0] + r[1][0], p[1][1] + r[1][1]],
        ]
        s_inv = _inv2(s)
        if s_inv is None:
            return None
        dn = north - x[0]
        de = east - x[1]
        d2 = (
            dn * (s_inv[0][0] * dn + s_inv[0][1] * de)
            + de * (s_inv[1][0] * dn + s_inv[1][1] * de)
        )
        return math.sqrt(max(0.0, d2))

    def update(
        self, t: float, north: float, east: float, r: Cov2, *, gated: bool = True
    ) -> bool:
        """Fuse one north/east position measurement taken at ``t``; whether
        it was fused.

        A measurement whose innovation lies outside the χ² gate
        (:data:`GATE_CHI2`, 3 sigma in two dimensions) is an outlier of the
        range cues (a lidar return off a trunk, a box cut by foliage) and is
        dropped, so a single bad fix cannot move the estimate metres. Gating
        must never lock a real manoeuvre out: once :data:`GATE_FORCE_AFTER`
        outliers in a row agree with each other, the newest is fused with
        the position spread widened by the innovation, which pulls the
        estimate onto the measurements again. A fix after a gap (see
        :data:`REACQUIRE_GAP_S`) that jumped is held out until the next
        ones show which was right. ``gated=False`` fuses a measurement
        already checked elsewhere (a re-acquire) as it is, or starts the
        estimate over on it when it jumped."""
        if self._x is None:
            self._restart(t, north, east, r)
            return True
        x, p = self._propagate(t)
        y = [north - x[0], east - x[1]]
        surprise = self._reacquire_chi2(t, y, r)
        if surprise is not None and surprise > REACQUIRE_CHI2:
            return self._on_jump(t, north, east, r, gated=gated)
        s_inv = _innovation_inverse(p, r)
        if s_inv is None:
            return False
        d2 = y[0] * (s_inv[0][0] * y[0] + s_inv[0][1] * y[1]) + y[1] * (
            s_inv[1][0] * y[0] + s_inv[1][1] * y[1]
        )
        if gated and d2 > GATE_CHI2:
            self._rejected.append((t, north, east, position_sigma([list(r[0]), list(r[1])])))
            del self._rejected[:-GATE_FORCE_AFTER]
            if not self._rejected_agree():
                return False
            p = [list(row) for row in p]
            p[0][0] += y[0] * y[0]
            p[0][1] += y[0] * y[1]
            p[1][0] += y[1] * y[0]
            p[1][1] += y[1] * y[1]
            s_inv = _innovation_inverse(p, r)
            if s_inv is None:
                return False
        elif gated and d2 > HUBER_CHI2:
            # Inside the gate but far out: weighed down (a Huber update), so
            # a fix the gate lets through cannot swing the velocity.
            w = d2 / HUBER_CHI2
            s_inv = _innovation_inverse(p, ((r[0][0] * w, r[0][1] * w), (r[1][0] * w, r[1][1] * w)))
            if s_inv is None:
                return False
        if gated and d2 <= GATE_CHI2:
            # A fix forced through the gate (a manoeuvre) or fused unchecked
            # (a re-acquire) is not range noise.
            self._note_scatter(p, r, y)
        self._rejected.clear()
        self._jumps.clear()
        k = _mul(_mul(p, _HT), s_inv)
        self._x = [x[i] + k[i][0] * y[0] + k[i][1] * y[1] for i in range(4)]
        # Joseph-free form; P stays symmetric enough at this size, and is
        # re-symmetrised to keep rounding from accumulating.
        kh = _mul(k, _H)
        i_kh = [[(1.0 if i == j else 0.0) - kh[i][j] for j in range(4)] for i in range(4)]
        p_new = _mul(i_kh, p)
        self._p = [[(p_new[i][j] + p_new[j][i]) / 2.0 for j in range(4)] for i in range(4)]
        self._cap_speed()
        self._t = max(self._t, t)
        gap = t - self._t_meas
        if 0.0 < gap <= _INTERVAL_MAX_S:
            prev = gap if self._interval_s is None else self._interval_s
            self._interval_s = prev + (gap - prev) / _INTERVAL_FIXES
        self._t_meas = max(self._t_meas, t)
        self._sigma_meas = position_sigma(self._p)
        return True

    def _restart(self, t: float, north: float, east: float, r: Cov2) -> None:
        """Start the estimate on one fix taken at ``t``, with no velocity and
        a fresh track's velocity spread. The speed cap and the range scatter
        measured so far are kept: they belong to the subject's class and to
        the range cues, not to the track."""
        v = _INIT_VEL_SIGMA_MPS * _INIT_VEL_SIGMA_MPS
        self._x = [north, east, 0.0, 0.0]
        self._p = [
            [r[0][0], r[0][1], 0.0, 0.0],
            [r[1][0], r[1][1], 0.0, 0.0],
            [0.0, 0.0, v, 0.0],
            [0.0, 0.0, 0.0, v],
        ]
        self._t = t
        self._t_meas = t
        self._sigma_meas = position_sigma(self._p)
        self._rejected.clear()
        self._jumps.clear()

    def _reacquire_chi2(self, t: float, y: list[float], r: Cov2) -> float | None:
        """For a fix at ``t``, ``y`` off the prediction, REACQUIRE_GAP_S or
        more after the last fix fused: its squared Mahalanobis distance
        against the fix's own covariance ``r``, the estimate's position
        spread and the subject's acceleration over the gap, the velocity
        spread left out (a jump only it explains is a velocity no fix has
        shown yet). ``None`` for a fix with no gap before it."""
        if t - self._t_meas < REACQUIRE_GAP_S:
            return None
        a = _process_noise(max(0.0, t - self._t))[0][0]
        p = self._p
        s_inv = _inv2(
            [
                [p[0][0] + a + r[0][0], p[0][1] + r[0][1]],
                [p[1][0] + r[1][0], p[1][1] + a + r[1][1]],
            ]
        )
        if s_inv is None:
            return None
        return y[0] * (s_inv[0][0] * y[0] + s_inv[0][1] * y[1]) + y[1] * (
            s_inv[1][0] * y[0] + s_inv[1][1] * y[1]
        )

    def _on_jump(self, t: float, north: float, east: float, r: Cov2, *, gated: bool) -> bool:
        """A fix that jumped; whether it was fused. It is held out until
        REACQUIRE_CONFIRM_FIXES jumped fixes in a row agree, which then
        start the estimate over (the oldest) and are fused in turn. A
        re-acquire checked elsewhere (``gated=False``) starts the estimate
        over on its own."""
        if not gated:
            self._restart(t, north, east, r)
            return True
        self._jumps.append((t, north, east, position_sigma([list(r[0]), list(r[1])]), r))
        del self._jumps[:-REACQUIRE_CONFIRM_FIXES]
        if len(self._jumps) < REACQUIRE_CONFIRM_FIXES or not self._agree(self._jumps):
            return False
        held = list(self._jumps)
        t0, n0, e0, _s0, r0 = held[0]
        self._restart(t0, n0, e0, r0)
        for tk, nk, ek, _sk, rk in held[1:]:
            self.update(tk, nk, ek, rk, gated=False)
        return True

    def _note_scatter(self, p: Mat, r: Cov2, y: list[float]) -> None:
        """Add one innovation ``y`` along the line of sight (the major axis
        of the fix's covariance ``r``) and its predicted variance to the
        running means behind :attr:`scatter_ratio`."""
        a, b, c = r[0][0], r[0][1], r[1][1]
        major = (a + c) / 2.0 + math.sqrt(max(0.0, (a - c) ** 2 / 4.0 + b * b))
        un, ue = (b, major - a) if abs(b) > 1e-12 else ((1.0, 0.0) if a >= c else (0.0, 1.0))
        n = math.hypot(un, ue)
        un, ue = un / n, ue / n
        y_los = y[0] * un + y[1] * ue
        s_los = (
            un * ((p[0][0] + a) * un + (p[0][1] + b) * ue)
            + ue * ((p[1][0] + b) * un + (p[1][1] + c) * ue)
        )
        self._scatter_n = min(self._scatter_n + 1, SCATTER_FIXES)
        k = 1.0 / self._scatter_n
        self._scatter_y2 += (y_los * y_los - self._scatter_y2) * k
        self._scatter_s += (s_los - self._scatter_s) * k


def _innovation_inverse(p: Mat, r: Cov2) -> Mat | None:
    """The inverse of the innovation covariance ``H P Hᵀ + R``."""
    return _inv2(
        [
            [p[0][0] + r[0][0], p[0][1] + r[0][1]],
            [p[1][0] + r[1][0], p[1][1] + r[1][1]],
        ]
    )


def position_sigma(p: Mat) -> float:
    """A scalar 1-sigma position spread: the root of the larger eigenvalue of
    the 2x2 position block."""
    a, b, c = p[0][0], p[0][1], p[1][1]
    tr = a + c
    disc = math.sqrt(max(0.0, (a - c) ** 2 / 4.0 + b * b))
    return math.sqrt(max(0.0, tr / 2.0 + disc))


__all__ = [
    "ACCEL_SIGMA_MPS2",
    "GATE_CHI2",
    "GATE_FORCE_AFTER",
    "MAX_PREDICT_S",
    "SCATTER_FIXES",
    "SCATTER_MIN_FIXES",
    "SCATTER_RATIO_MAX",
    "SUBJECT_MAX_SPEED_MPS",
    "TargetFilter",
    "position_sigma",
    "subject_max_speed_mps",
]
