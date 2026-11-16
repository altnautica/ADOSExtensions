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
# Rejected fixes agree when each lies within this many of its own sigmas of
# the one before, plus the distance the subject can cover in between.
_AGREE_SIGMAS = 3.0
# The reach used for a subject class with no speed cap, m/s.
_UNCAPPED_REACH_MPS = 5.0
# The range scatter (see :attr:`TargetFilter.scatter_ratio`) averages about
# this many fixes (5 s at 10 Hz): long enough that the innovations of a
# second's manoeuvre barely move it, short enough to follow the cues as the
# range or the light changes. It reads no more than SCATTER_RATIO_MAX, and is
# taken in gradually over the first SCATTER_MIN_FIXES fixes (2 s at 10 Hz):
# a fresh track's first innovations say more about its start than about the
# cues, and a smoothing switched on at once would itself step the command.
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
        # The running means of the squared innovation along the line of
        # sight and of its predicted variance, over the fixes fused, and how
        # many fixes they average (see :attr:`scatter_ratio`).
        self._scatter_y2 = 0.0
        self._scatter_s = 0.0
        self._scatter_n = 0
        self.max_speed_mps: float | None = None

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
        depth camera). Scaling the filter's own spread by it gives the error
        the estimate really carries. A track's first SCATTER_MIN_FIXES fixes
        read 0 and the next SCATTER_MIN_FIXES take it in gradually."""
        if self._scatter_n <= SCATTER_MIN_FIXES or self._scatter_s <= 0.0:
            return 0.0
        ratio = min(SCATTER_RATIO_MAX, math.sqrt(self._scatter_y2 / self._scatter_s))
        return ratio * min(1.0, self._scatter_n / SCATTER_MIN_FIXES - 1.0)

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
        self._scatter_y2 = 0.0
        self._scatter_s = 0.0
        self._scatter_n = 0

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
        rej = self._rejected
        if len(rej) < GATE_FORCE_AFTER:
            return False
        reach = self.max_speed_mps or _UNCAPPED_REACH_MPS
        for (t0, n0, e0, s0), (t1, n1, e1, s1) in zip(rej, rej[1:]):
            bound = _AGREE_SIGMAS * math.hypot(s0, s1) + reach * max(0.0, t1 - t0)
            if math.hypot(n1 - n0, e1 - e0) > bound:
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
        estimate onto the measurements again. ``gated=False`` fuses a
        measurement already checked elsewhere (a re-acquire) as it is."""
        if self._x is None:
            self._x = [north, east, 0.0, 0.0]
            v = _INIT_VEL_SIGMA_MPS * _INIT_VEL_SIGMA_MPS
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
            return True
        x, p = self._propagate(t)
        y = [north - x[0], east - x[1]]
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
        self._t_meas = max(self._t_meas, t)
        self._sigma_meas = position_sigma(self._p)
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
