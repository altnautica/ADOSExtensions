"""Constant-velocity Kalman filter for the followed subject.

State ``[n, e, vn, ve]`` in the flight controller's local NED frame (metres,
metres per second). The process model is a constant velocity driven by white
acceleration noise of 1.5 m/s^2 (a walking or running person changing pace
and direction); the measurement is the subject's north/east position from one
box (vehicle LOCAL_POSITION_NED plus the projected offset) with the 2x2
range/bearing covariance from :mod:`follow_me.projection`.

Pure Python, no numpy: four states do not need it and the plugin stays free
of a heavy dependency on small companion boards.
"""

from __future__ import annotations

import math

Mat = list[list[float]]

# Process noise, the 1-sigma white acceleration, m/s^2.
ACCEL_SIGMA_MPS2 = 1.5
# Velocity sigma a fresh track starts with: a person can be moving at up to
# a run when first seen.
_INIT_VEL_SIGMA_MPS = 3.0
# Predictions further than this past the last update are refused; the caller
# treats the track as gone.
MAX_PREDICT_S = 30.0

# The fastest a followed subject of each detector class is taken to move,
# m/s. A person's cap is a steady run: a velocity estimate above it comes from
# range noise between sightings, not from the subject, and extrapolating it
# carries the estimate away from the subject within a second. A faster
# subject is still followed: each sighting still moves the estimate, which
# then trails a 4 m/s runner by under a metre. A class with no entry is not
# capped here (the goal's feed-forward is still clamped to the configured
# max_speed_mps).
SUBJECT_MAX_SPEED_MPS: dict[str, float] = {"person": 3.0}


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
        self.max_speed_mps: float | None = None

    @property
    def initialized(self) -> bool:
        return self._x is not None

    @property
    def last_update_s(self) -> float:
        """When the last measurement was fused."""
        return self._t_meas

    def reset(self) -> None:
        self._x = None
        self._p = _zeros(4, 4)
        self._t = 0.0
        self._t_meas = 0.0

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

    def predicted(self, t: float) -> tuple[list[float], Mat] | None:
        """``(state, covariance)`` propagated to ``t``, or ``None`` before the
        first update or past :data:`MAX_PREDICT_S`."""
        if self._x is None or (t - self._t) > MAX_PREDICT_S:
            return None
        return self._propagate(t)

    def mahalanobis(self, t: float, north: float, east: float, r: Cov2) -> float | None:
        """Mahalanobis distance of a position measurement from the prediction
        at ``t``, including the measurement's own covariance."""
        pred = self.predicted(t)
        if pred is None:
            return None
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

    def update(self, t: float, north: float, east: float, r: Cov2) -> None:
        """Fuse one north/east position measurement taken at ``t``."""
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
            return
        x, p = self._propagate(t)
        hp = _mul(_H, p)
        s = _add(_mul(hp, _HT), [list(r[0]), list(r[1])])
        s_inv = _inv2(s)
        if s_inv is None:
            return
        k = _mul(_mul(p, _HT), s_inv)
        y = [north - x[0], east - x[1]]
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


def position_sigma(p: Mat) -> float:
    """A scalar 1-sigma position spread: the root of the larger eigenvalue of
    the 2x2 position block."""
    a, b, c = p[0][0], p[0][1], p[1][1]
    tr = a + c
    disc = math.sqrt(max(0.0, (a - c) ** 2 / 4.0 + b * b))
    return math.sqrt(max(0.0, tr / 2.0 + disc))


__all__ = [
    "ACCEL_SIGMA_MPS2",
    "MAX_PREDICT_S",
    "SUBJECT_MAX_SPEED_MPS",
    "TargetFilter",
    "position_sigma",
    "subject_max_speed_mps",
]
