"""The standoff controller: how fast and where the follow asks the pilot to
fly, and where it points the camera.

Follow-me sends the pilot its complete desired velocity (``vel_mode``
``command``); the pilot shapes it within the vehicle's jerk and acceleration
limits, caps it for obstacles and the fence, and adds no position loop of its
own. This module turns the subject estimate and the follow or orbit geometry
into that velocity:

* **Still subject.** The subject stands when the median of its fixes over
  the last ``STILL_SPAN_S`` lies within ``STILL_ENTER_DRIFT_M`` of the median
  over the span before it (under 0.4 m/s). Its reference is then a fixed
  centre with no velocity, refined by every fix with its pull clipped to
  ``STILL_PULL_M``, so range noise on a standing person no longer moves the
  goal. It walks again once the median of the last span's fixes leaves
  ``STILL_EXIT_RADIUS_M`` of the centre, or once ``STILL_STEP_FIXES`` fixes
  in a row (gated out by the subject filter or not) each land well off it on
  the same side: a subject breaking into a run. Medians of fixes, not
  estimates: a single range outlier moves an estimate (and its velocity) by
  a metre, and a median of the fixes not at all.
* **Deadband.** While the subject stands, the follow goal moves only when the
  geometry puts it more than ``GOAL_DEADBAND_M`` from the latched goal, and
  the vehicle holds once it is within ``HOLD_DEADBAND_M`` of it.
* **Speed law.** ``v = v_goal + û·s(e)``: the goal's own motion (the
  subject's, plus the ring's for an orbit) plus a closing speed along the
  error ``e`` to the goal. ``s`` is linear (``APPROACH_GAIN_PER_S``) near the
  goal and a constant-deceleration square root (``APPROACH_DECEL_MPS2``)
  further out, capped at the speed limit: full speed when far, a ramp that
  decelerates at no more than ``APPROACH_DECEL_MPS2`` into the standoff, and
  nothing at it. The commanded velocity changes by at most
  ``COMMAND_ACCEL_MPS2`` per second, so it never steps, except to keep
  clear of the subject.
* **Separation.** Whatever the geometry asks, the velocity toward the
  subject is held so the vehicle stops short of the clear distance
  (:func:`clear_distance`: the safe distance plus two sigmas of the range
  error, kept inside the standoff) or passes the subject beside it, and
  within it the vehicle backs away (:func:`keep_clear`). The cut along the
  line of sight is not slewed: the pilot shapes it at its profile's
  limits.
* **Range noise.** The subject velocity fed forward, the range error and
  the command are smoothed, and the range error is corrected only past a
  deadband, each by the error the subject estimate really carries; on clean
  ranges next to none of it applies (a subject's own sharp manoeuvre reads
  as a little scatter), and under monocular range noise the command
  holds the subject's pace instead of swinging round it.
* **Bearing.** The yaw turns toward the subject at a rate from the same
  square-root law (``YAW_GAIN_PER_S``, ``YAW_ACCEL_DPS2``, at most
  ``YAW_RATE_MAX_DPS``), with a deadband of ``YAW_DEADBAND_STILL_DEG``
  round a standing subject and ``YAW_DEADBAND_DEG`` round a moving one.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

Cov2 = tuple[tuple[float, float], tuple[float, float]]

# Still-subject detection: spans of fixes, at least this many in each.
STILL_SPAN_S = 2.0
_STILL_MIN_FIXES = 3
STILL_ENTER_DRIFT_M = 0.6
# ...and while the subject filter also reads it slower than this.
STILL_ENTER_SPEED_MPS = 0.6
# The subject walks off once the median of the last span lies this many
# sigmas (of that median, from the fixes' own covariance) and at least
# STILL_EXIT_MIN_M from the centre, or STILL_EXIT_RADIUS_M whatever the
# sigmas. Monocular range is poor and bearing good, so a step across the
# line of sight is seen within a fix or two and one along it takes longer.
STILL_EXIT_SIGMA = 4.0
STILL_EXIT_MIN_M = 0.5
STILL_EXIT_RADIUS_M = 1.5
# A median of n fixes spreads about 1.57/n of one fix's variance.
_MEDIAN_VAR_FACTOR = 1.57
# ...over the fixes of this last span, on this many fixes in a row.
STILL_EXIT_SPAN_S = 1.5
STILL_EXIT_FIXES = 2
# Breaking into a run: STILL_STEP_FIXES fixes in a row, each beyond
# STILL_STEP_SIGMA of its own sigma and STILL_STEP_MIN_M off the centre, on
# the same side (within 45°).
STILL_STEP_SIGMA = 2.5
STILL_STEP_MIN_M = 1.0
STILL_STEP_FIXES = 4
_SAME_SIDE_COS = math.cos(math.radians(45.0))
STILL_PULL_M = 1.0
# The centre averages at most this many fixes (20 s at 3 Hz), so it still
# settles onto a subject that shuffles.
_STILL_MEAN_FIXES = 60

# Deadbands round a standing subject.
GOAL_DEADBAND_M = 0.5
HOLD_DEADBAND_M = 0.2

# The speed law. The pilot shapes every goal velocity within its profile's
# limits (Normal: 1.0 m/s², 2.0 m/s³, a 0.5 s jerk ramp) and is the
# authority on them: the plugin is not told the profile, and a Cine profile
# simply shapes the command more slowly. The command slews at the Normal
# profile's acceleration, so a catch-up or a stop is never slower than the
# vehicle allows; the approach decelerates at 0.6 of it, looking ahead by
# the shaper's lag, so the vehicle follows the ramp into the standoff
# without overshooting it.
APPROACH_GAIN_PER_S = 0.6
APPROACH_DECEL_MPS2 = 0.6
COMMAND_ACCEL_MPS2 = 1.0
# The lag from a command to the vehicle's motion: the shaper's 0.5 s jerk
# ramp and the command's own slew.
APPROACH_LEAD_S = 0.7

# Range noise. Every term below scales with the error the subject estimate
# really carries (the filter's spread times how far the fixes actually
# scatter, see :func:`follow_me.loop._estimate_error`), so on clean ranges
# (a lidar, a depth camera) the law is next to the one above, and under
# monocular range noise it no longer turns that noise into speed swings:
# each swing at the profile's acceleration tilts the vehicle 5.8°.
#
# * The subject's velocity is fed forward smoothed on PACE_SMOOTH_S_PER_MPS
#   seconds per m/s of its error; any change beyond PACE_GATE_SIGMAS of that
#   error passes at once (a stop, a sprint).
# * The range error is corrected only beyond RANGE_DEADBAND_SIGMAS of the
#   estimate's error along the line of sight (at most RANGE_DEADBAND_MAX_M),
#   by the speed law above shifted out by that band, so the catch-up and the
#   ramp into the standoff keep their shape.
# * That correction is smoothed on CORRECTION_SMOOTH_S_PER_M seconds per
#   metre of the error: the estimate's range moves in a step at every fix,
#   and the correction follows it as a ramp, not a step.
# * Their sum is smoothed again on COMMAND_SMOOTH_S_PER_M seconds per metre,
#   so a fix that bends either ramp bends the command's acceleration
#   gradually: the command's jerk stays a fraction of the profile's.
# No smoothing exceeds SMOOTH_MAX_S (a fresh track's spread is large).
PACE_GATE_SIGMAS = 2.0
PACE_SMOOTH_S_PER_MPS = 2.0
RANGE_DEADBAND_SIGMAS = 2.0
RANGE_DEADBAND_MAX_M = 1.0
CORRECTION_SMOOTH_S_PER_M = 2.0
COMMAND_SMOOTH_S_PER_M = 0.4
SMOOTH_MAX_S = 2.0

# Separation from the subject. The vehicle never flies into or over the
# person: whatever the follow geometry asks, the commanded velocity relative
# to the subject may not carry the vehicle inside the safe distance.
#
# * The safe distance is SAFE_DISTANCE_M horizontal (a person's reach and
#   the airframe's, about a metre, plus two sigmas of a monocular range
#   fix), held at least SAFE_STANDOFF_MARGIN_M inside a shorter standoff.
# * Uncertain range is taken short: the distance kept clear adds
#   SEPARATION_SIGMAS of the error the estimate carries along the line of
#   sight, at most SEPARATION_MARGIN_MAX_M, and that margin too stays
#   SAFE_STANDOFF_MARGIN_M inside the standoff (never past the safe
#   distance). Under monocular range noise (σ 0.75 m and its outliers) the
#   subject estimate puts a subject at the standoff up to 1.5 m nearer for
#   a moment; 2 m inside, the guard does not answer that noise at any
#   standoff allowed. Over a short standoff the
#   height floor (MIN_FOLLOW_HEIGHT_M) keeps the rotors a metre clear of
#   the subject whatever the horizontal distance.
# * The closing speed toward the subject (relative to the subject's own
#   motion) is limited so the vehicle can still stop, decelerating at
#   SEPARATION_DECEL_MPS2 after the shaper's lag, short of the clear
#   distance, and so it does not reach it inside SEPARATION_HORIZON_S
#   (the predicted conflict); within it the vehicle backs away at
#   SEPARATION_PUSH_PER_S per metre.
# * Only velocities whose path past the subject reaches the clear distance
#   are limited: of the closest admissible ones, a sidestep that passes
#   the subject on one side is taken when it asks less change than
#   stopping, and the side is held (SEPARATION_SIDE_HOLD_MPS) while the
#   conflict lasts.
SAFE_DISTANCE_M = 2.5
SAFE_STANDOFF_MARGIN_M = 2.0
SEPARATION_SIGMAS = 2.0
SEPARATION_MARGIN_MAX_M = 1.5
SEPARATION_DECEL_MPS2 = 0.8
SEPARATION_GAIN_PER_S = 1.0
SEPARATION_HORIZON_S = 3.0
SEPARATION_PUSH_PER_S = 1.0
SEPARATION_SIDE_HOLD_MPS = 0.5
SEPARATION_STALE_M = 2.5

# Bearing control.
YAW_GAIN_PER_S = 2.0
YAW_ACCEL_DPS2 = 60.0
YAW_RATE_MAX_DPS = 30.0
YAW_DEADBAND_DEG = 2.0
YAW_DEADBAND_STILL_DEG = 6.0

# A command older than this is not continued: the next one starts from the
# vehicle's own velocity and heading (after a hold, a search or a re-engage).
_RESUME_AFTER_S = 0.25


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def sqrt_law(error: float, gain: float, decel: float) -> float:
    """The closing speed for ``error`` (≥ 0): ``gain·error`` near zero and
    ``√(2·decel·error − decel²/gain²)`` beyond ``decel/gain²``, the two
    meeting with the same value and slope. Flying it never decelerates
    faster than ``decel``."""
    if error <= 0.0:
        return 0.0
    if gain <= 0.0:
        return math.sqrt(2.0 * decel * error)
    linear_to = decel / (gain * gain)
    if error <= linear_to:
        return gain * error
    return math.sqrt(2.0 * decel * error - decel * decel / (gain * gain))


def _clamp_len(vn: float, ve: float, limit: float) -> tuple[float, float]:
    n = math.hypot(vn, ve)
    if n <= limit or n <= 1e-9:
        return vn, ve
    k = max(0.0, limit) / n
    return vn * k, ve * k


def _smoothing(dt: float, tau: float) -> float:
    """The share of the way a first-order smoothing on ``tau`` seconds
    (capped at SMOOTH_MAX_S) moves in ``dt``; all of it for no smoothing."""
    tau = min(tau, SMOOTH_MAX_S)
    return 1.0 if tau <= 1e-6 else 1.0 - math.exp(-dt / tau)


def safe_distance(standoff_m: float) -> float:
    """The horizontal distance the vehicle never closes inside, for a
    follow standoff (or orbit radius) of ``standoff_m``."""
    return max(0.0, min(SAFE_DISTANCE_M, standoff_m - SAFE_STANDOFF_MARGIN_M))


def clear_distance(standoff_m: float, range_sigma_m: float) -> float:
    """The horizontal distance kept clear of the subject: the safe distance
    plus SEPARATION_SIGMAS of the range error, the margin held
    SAFE_STANDOFF_MARGIN_M inside the standoff (the safe distance itself
    is)."""
    safe = safe_distance(standoff_m)
    margin = min(SEPARATION_MARGIN_MAX_M, SEPARATION_SIGMAS * max(0.0, range_sigma_m))
    return min(safe + margin, max(safe, standoff_m - SAFE_STANDOFF_MARGIN_M))


def keep_clear(
    *,
    rel_ne: tuple[float, float],
    subject_v_ne: tuple[float, float],
    pace_v_ne: tuple[float, float],
    want_ne: tuple[float, float],
    command_v_ne: tuple[float, float],
    range_sigma_m: float,
    standoff_m: float,
    side: int,
    near_m: float | None = None,
) -> tuple[tuple[float, float], int, bool]:
    """The velocity nearest ``want_ne`` that keeps the vehicle clear of the
    subject, the side it passes on (0: none) and whether it was limited.

    ``rel_ne`` is the subject's offset from the vehicle, ``subject_v_ne`` its
    velocity and ``pace_v_ne`` the one fed forward (:meth:`StandoffController.pace`),
    ``command_v_ne`` the velocity commanded now (what the vehicle is turning
    toward), ``range_sigma_m`` the error the estimate carries along the line
    of sight, ``standoff_m`` the follow standoff (or orbit radius) the clear
    distance comes from (:func:`clear_distance`), ``side`` the side
    returned last tick and ``near_m`` the range of the newest fixes (see
    :meth:`follow_me.speed_limit.SubjectMotion.near_range`), kept clear in
    place of the estimate's when it is nearer."""
    r = math.hypot(rel_ne[0], rel_ne[1])
    sn, se = subject_v_ne
    if r > 1e-6:
        un, ue = rel_ne[0] / r, rel_ne[1] / r
    else:
        # Over the subject: away from where the command was carrying it.
        cn, ce = command_v_ne[0] - sn, command_v_ne[1] - se
        c = math.hypot(cn, ce)
        un, ue = (cn / c, ce / c) if c > 1e-6 else (1.0, 0.0)
    margin = min(SEPARATION_MARGIN_MAX_M, SEPARATION_SIGMAS * max(0.0, range_sigma_m))
    # The subject's motion along the line of sight: the faster away of the
    # estimate's and the pace fed forward (a range outlier swings the first
    # for a fix or two), and none away when the newest fixes put the
    # subject well nearer than the estimate (it trails a turn back).
    away = sn * un + se * ue
    credit = max(away, pace_v_ne[0] * un + pace_v_ne[1] * ue)
    if near_m is not None:
        if near_m < r - max(margin, SEPARATION_STALE_M):
            credit = min(credit, 0.0)
        r = min(r, near_m)
    sn, se = sn + (credit - away) * un, se + (credit - away) * ue
    qn, qe = want_ne[0] - sn, want_ne[1] - se
    clear = clear_distance(standoff_m, range_sigma_m)
    q_u = qn * un + qe * ue
    if r <= clear:
        # Inside: every velocity that does not back away is refused.
        cap = -SEPARATION_PUSH_PER_S * (clear - r)
        if q_u <= cap:
            return want_ne, 0, False
        half = 0.5 * math.pi
    else:
        closing = (command_v_ne[0] - sn) * un + (command_v_ne[1] - se) * ue
        ahead = r - clear - max(0.0, closing) * APPROACH_LEAD_S
        cap = min(
            sqrt_law(ahead, SEPARATION_GAIN_PER_S, SEPARATION_DECEL_MPS2),
            (r - clear) / SEPARATION_HORIZON_S,
        )
        half = math.asin(clear / r)
        # A velocity outside the cone of paths that reach the clear
        # distance passes the subject, however fast.
        if q_u <= cap or q_u < math.hypot(qn, qe) * math.cos(half):
            return want_ne, 0, False
    # Stop closing: the closing part cut to the cap, the rest kept.
    best = (qn - (q_u - cap) * un, qe - (q_u - cap) * ue)
    best_d = abs(q_u - cap)
    best_side = side
    if half < 0.5 * math.pi:
        # Or pass on one side: the nearest velocity along that side's edge.
        edges = []
        for s in (1, -1):
            c, w = math.cos(half), s * math.sin(half)
            en, ee = un * c - ue * w, ue * c + un * w
            k = max(0.0, qn * en + qe * ee)
            pn, pe = en * k, ee * k
            edges.append((math.hypot(qn - pn, qe - pe), s, (pn, pe)))
        edges.sort(key=lambda x: x[0])
        d, s, p = edges[0]
        held = next((x for x in edges if x[1] == side), None)
        if held is not None and held[0] <= d + SEPARATION_SIDE_HOLD_MPS:
            d, s, p = held
        if d < best_d:
            best, best_d, best_side = p, d, s
    return (sn + best[0], se + best[1]), best_side, True


@dataclass(frozen=True)
class SubjectRef:
    """What the goal geometry is built round."""

    north_m: float
    east_m: float
    vn_mps: float
    ve_mps: float
    still: bool


def _median(values: list[float]) -> float:
    s = sorted(values)
    m = len(s) // 2
    return s[m] if len(s) % 2 else 0.5 * (s[m - 1] + s[m])


class StillSubject:
    """Decides whether the subject stands, and its reference while it does."""

    def __init__(self) -> None:
        self._fixes: deque[tuple[float, float, float, Cov2]] = deque()
        self._still = False
        self._center: tuple[float, float] = (0.0, 0.0)
        self._count = 0
        # Fixes in a row that put the subject off its centre.
        self._off = 0
        # Fixes in a row each well off the centre on the same side, and that
        # side.
        self._step = 0
        self._step_dir: tuple[float, float] = (0.0, 0.0)

    @property
    def still(self) -> bool:
        return self._still

    def reset(self) -> None:
        self._fixes.clear()
        self._still = False
        self._count = 0
        self._off = 0
        self._step = 0

    def shift(self, dn: float, de: float) -> None:
        """Move with a jump of the local frame."""
        self._fixes = deque((t, n + dn, e + de, r) for t, n, e, r in self._fixes)
        self._center = (self._center[0] + dn, self._center[1] + de)

    def _span(self, t0: float, t1: float) -> list[tuple[float, float, float, Cov2]]:
        return [f for f in self._fixes if t0 < f[0] <= t1]

    @staticmethod
    def _median_ne(pts: list[tuple[float, float, float, Cov2]]) -> tuple[float, float] | None:
        if len(pts) < _STILL_MIN_FIXES:
            return None
        return (_median([p[1] for p in pts]), _median([p[2] for p in pts]))

    def _walked_off(self, pts: list[tuple[float, float, float, Cov2]], recent: tuple[float, float]) -> bool:
        dn, de = recent[0] - self._center[0], recent[1] - self._center[1]
        dist = math.hypot(dn, de)
        if dist > STILL_EXIT_RADIUS_M:
            return True
        if dist <= STILL_EXIT_MIN_M:
            return False
        k = _MEDIAN_VAR_FACTOR / (len(pts) * len(pts))
        a = k * sum(p[3][0][0] for p in pts)
        b = k * sum(p[3][0][1] for p in pts)
        c = k * sum(p[3][1][1] for p in pts)
        det = a * c - b * b
        if det <= 1e-12:
            return False
        d2 = (c * dn * dn - 2.0 * b * dn * de + a * de * de) / det
        return d2 > STILL_EXIT_SIGMA * STILL_EXIT_SIGMA

    def _stepped_off(self, n: float, e: float, cov: Cov2) -> bool:
        """Whether this fix and the ones just before it all lie well off the
        centre on the same side: a subject that broke into a run is past
        any fix's noise within a few fixes, long before a median of the span
        moves. One range outlier, or two, never makes a run of them."""
        dn, de = n - self._center[0], e - self._center[1]
        d = math.hypot(dn, de)
        if d <= STILL_STEP_MIN_M:
            self._step = 0
            return False
        un, ue = dn / d, de / d
        sigma = math.sqrt(max(0.0, un * (cov[0][0] * un + cov[0][1] * ue) + ue * (cov[1][0] * un + cov[1][1] * ue)))
        if d <= STILL_STEP_SIGMA * sigma:
            self._step = 0
            return False
        same = self._step > 0 and un * self._step_dir[0] + ue * self._step_dir[1] >= _SAME_SIDE_COS
        self._step = self._step + 1 if same else 1
        self._step_dir = (un, ue)
        return self._step >= STILL_STEP_FIXES

    def _walk_on(self, t: float, n: float, e: float, cov: Cov2) -> None:
        """Walking again: standing is judged afresh from here."""
        self._still = False
        self._off = 0
        self._step = 0
        self._fixes.clear()
        self._fixes.append((t, n, e, cov))

    def add_fix(
        self, t: float, n: float, e: float, cov: Cov2, speed_mps: float, *, fused: bool = True
    ) -> None:
        """A fix placed at ``t`` with its covariance, and the subject
        filter's speed estimate after it. ``fused``: whether the filter took
        it; one its gate refused counts only toward breaking into a run."""
        stepped = self._still and self._stepped_off(n, e, cov)
        if not fused:
            if stepped:
                self._walk_on(t, n, e, cov)
            return
        fixes = self._fixes
        fixes.append((t, n, e, cov))
        while fixes and t - fixes[0][0] > 2.0 * STILL_SPAN_S:
            fixes.popleft()
        pts = self._span(t - STILL_SPAN_S, t)
        recent = self._median_ne(pts)
        if recent is None:
            return
        if self._still:
            near = self._span(t - STILL_EXIT_SPAN_S, t)
            off_now = self._median_ne(near)
            walked = off_now is not None and self._walked_off(near, off_now)
            self._off = self._off + 1 if walked else 0
            if self._off >= STILL_EXIT_FIXES or stepped:
                self._walk_on(t, n, e, cov)
                return
            self._count = min(self._count + 1, _STILL_MEAN_FIXES)
            dn, de = n - self._center[0], e - self._center[1]
            d = math.hypot(dn, de)
            k = min(1.0, STILL_PULL_M / d) / self._count if d > 1e-9 else 0.0
            self._center = (self._center[0] + dn * k, self._center[1] + de * k)
            return
        earlier = self._median_ne(self._span(t - 2.0 * STILL_SPAN_S, t - STILL_SPAN_S))
        if (
            earlier is not None
            and speed_mps <= STILL_ENTER_SPEED_MPS
            and math.hypot(recent[0] - earlier[0], recent[1] - earlier[1]) <= STILL_ENTER_DRIFT_M
        ):
            whole = self._median_ne(list(fixes))
            assert whole is not None
            self._still = True
            self._center = whole
            self._count = len(fixes)

    def reference(self, x: tuple[float, float, float, float]) -> SubjectRef:
        """What to build the goal round, given the estimate ``x`` (north,
        east and the velocity) now."""
        if self._still:
            return SubjectRef(self._center[0], self._center[1], 0.0, 0.0, True)
        return SubjectRef(x[0], x[1], x[2], x[3], False)


@dataclass(frozen=True)
class Command:
    """One goal's position, commanded velocity and yaw."""

    north_m: float
    east_m: float
    vn_mps: float
    ve_mps: float
    yaw_rad: float


class StandoffController:
    """The commanded velocity and yaw, continuous from tick to tick."""

    def __init__(self) -> None:
        self._t: float | None = None
        self._v: tuple[float, float] = (0.0, 0.0)
        self._yaw = 0.0
        self._yaw_rate = 0.0
        self._goal: tuple[float, float] | None = None
        # The range correction and the velocity the command steers toward,
        # as last smoothed (see CORRECTION_SMOOTH_S_PER_M and
        # COMMAND_SMOOTH_S_PER_M).
        self._corr: tuple[float, float] = (0.0, 0.0)
        self._want: tuple[float, float] = (0.0, 0.0)
        # The subject velocity fed forward, when it was, and whether the
        # subject stood then.
        self._pace: tuple[float, float] = (0.0, 0.0)
        self._pace_t: float | None = None
        self._pace_still = False
        # The side the vehicle passes the subject on, and whether the last
        # command was limited to keep clear of it (see keep_clear).
        self._side = 0
        self.limited = False

    def reset(self) -> None:
        self._t = None
        self._goal = None
        self._pace_t = None

    def pace(
        self, *, now: float, vn: float, ve: float, sigma_mps: float, still: bool
    ) -> tuple[float, float]:
        """The subject velocity to feed forward, from the subject filter's
        ``(vn, ve)`` and ``sigma_mps``, the error it really carries. Within
        PACE_GATE_SIGMAS of that error a change is taken in on
        PACE_SMOOTH_S_PER_MPS seconds per m/s of it; any part beyond, at
        once. A standing subject's velocity (none), and the first after it
        stands or walks on, are taken as they are: the standing decision is
        the evidence a range change alone does not give."""
        changed = still != self._pace_still
        self._pace_still = still
        if still or changed or self._pace_t is None or now - self._pace_t > _RESUME_AFTER_S:
            self._pace_t = now
            self._pace = (vn, ve)
            return self._pace
        dt = max(0.0, now - self._pace_t)
        self._pace_t = now
        sigma = max(0.0, sigma_mps)
        pn, pe = self._pace
        dn, de = vn - pn, ve - pe
        d = math.hypot(dn, de)
        gate = PACE_GATE_SIGMAS * sigma
        if d > gate:
            k = 1.0 - gate / d
            pn, pe = pn + dn * k, pe + de * k
        k = _smoothing(dt, PACE_SMOOTH_S_PER_MPS * sigma)
        self._pace = (pn + (vn - pn) * k, pe + (ve - pe) * k)
        return self._pace

    def shift(self, dn: float, de: float) -> None:
        """Move with a jump of the local frame."""
        if self._goal is not None:
            self._goal = (self._goal[0] + dn, self._goal[1] + de)

    def command(
        self,
        *,
        now: float,
        goal_ne: tuple[float, float],
        goal_v_ne: tuple[float, float],
        drone_ne: tuple[float, float],
        drone_v_ne: tuple[float, float],
        drone_yaw: float,
        yaw_target: float,
        still: bool,
        latch_goal: bool,
        max_speed_mps: float,
        subject_ne: tuple[float, float],
        subject_v_ne: tuple[float, float],
        standoff_m: float,
        near_m: float | None = None,
        range_sigma_m: float = 0.0,
    ) -> Command:
        """``latch_goal``: hold the goal through moves inside the deadband (a
        standing subject followed at a fixed angle; an orbit's goal moves on
        purpose). ``goal_v_ne`` carries the subject's velocity from
        :meth:`pace`; ``range_sigma_m`` is the error the subject estimate
        really carries along the line of sight (0 on clean ranges).
        ``subject_ne`` and ``subject_v_ne`` are the subject estimate's own
        position and velocity, unsmoothed, which the vehicle keeps clear of
        by the distance ``standoff_m`` (the follow standoff or orbit radius)
        sets, with ``near_m`` the newest fixes' range (see
        :func:`keep_clear`)."""
        resume = self._t is None or now - self._t > _RESUME_AFTER_S
        if resume:
            self._v = drone_v_ne
            self._yaw = drone_yaw
            self._yaw_rate = 0.0
            self._goal = None
            self._side = 0
            dt = 0.0
        else:
            dt = max(0.0, now - self._t)
        self._t = now

        gn, ge = goal_ne
        if latch_goal:
            if self._goal is None or math.hypot(gn - self._goal[0], ge - self._goal[1]) > GOAL_DEADBAND_M:
                self._goal = (gn, ge)
            gn, ge = self._goal
        else:
            self._goal = None
        en, ee = gn - drone_ne[0], ge - drone_ne[1]
        dist = math.hypot(en, ee)
        un, ue = (en / dist, ee / dist) if dist > 1e-6 else (0.0, 0.0)
        # The error the vehicle will have once the shaper has turned the
        # current command into motion: closing faster than the goal moves,
        # it covers that much more before a slower command bites.
        rel = (self._v[0] - goal_v_ne[0]) * un + (self._v[1] - goal_v_ne[1]) * ue
        sigma = max(0.0, range_sigma_m)
        band = HOLD_DEADBAND_M if still else min(RANGE_DEADBAND_MAX_M, RANGE_DEADBAND_SIGMAS * sigma)
        ahead = dist - max(0.0, rel) * APPROACH_LEAD_S - band
        closing = sqrt_law(ahead, APPROACH_GAIN_PER_S, APPROACH_DECEL_MPS2)
        closing = min(closing, max(0.0, max_speed_mps))
        cn, ce = un * closing, ue * closing
        want = _clamp_len(goal_v_ne[0] + cn, goal_v_ne[1] + ce, max_speed_mps)
        if resume or still:
            # A standing subject's centre is the mean of many fixes, not the
            # estimate: no fix steps it.
            self._corr = (cn, ce)
            self._want = want
        else:
            k = _smoothing(dt, CORRECTION_SMOOTH_S_PER_M * sigma)
            self._corr = (self._corr[0] + (cn - self._corr[0]) * k, self._corr[1] + (ce - self._corr[1]) * k)
            want = _clamp_len(goal_v_ne[0] + self._corr[0], goal_v_ne[1] + self._corr[1], max_speed_mps)
            k = _smoothing(dt, COMMAND_SMOOTH_S_PER_M * sigma)
            self._want = (self._want[0] + (want[0] - self._want[0]) * k, self._want[1] + (want[1] - self._want[1]) * k)
        # Clear of the subject, from its estimate as it is: no smoothing
        # delays backing away.
        want, self._side, limited = keep_clear(
            rel_ne=(subject_ne[0] - drone_ne[0], subject_ne[1] - drone_ne[1]),
            subject_v_ne=subject_v_ne,
            pace_v_ne=goal_v_ne,
            want_ne=self._want,
            command_v_ne=self._v,
            range_sigma_m=sigma,
            standoff_m=standoff_m,
            side=self._side,
            near_m=near_m,
        )
        want = _clamp_len(want[0], want[1], max_speed_mps)
        dvn, dve = _clamp_len(want[0] - self._v[0], want[1] - self._v[1], COMMAND_ACCEL_MPS2 * dt)
        slewed = (self._v[0] + dvn, self._v[1] + dve)
        rel = (subject_ne[0] - drone_ne[0], subject_ne[1] - drone_ne[1])
        r = math.hypot(rel[0], rel[1])
        un, ue = (rel[0] / r, rel[1] / r) if r > 1e-6 else (0.0, 0.0)
        along = (want[0] - slewed[0]) * un + (want[1] - slewed[1]) * ue
        if limited and along < -1e-9:
            # Safety outranks smoothness: a cut in the closing speed is not
            # slewed (the pilot shapes it at its profile's limits). Only the
            # part along the line of sight steps; the rest, a sidestep
            # included, is still slewed.
            self._v = (slewed[0] + along * un, slewed[1] + along * ue)
        else:
            self._v = slewed
        self.limited = limited

        err = wrap_pi(yaw_target - self._yaw)
        band = math.radians(YAW_DEADBAND_STILL_DEG if still else YAW_DEADBAND_DEG)
        rate = math.copysign(
            min(
                sqrt_law(abs(err) - band, YAW_GAIN_PER_S, math.radians(YAW_ACCEL_DPS2)),
                math.radians(YAW_RATE_MAX_DPS),
            ),
            err,
        )
        step = math.radians(YAW_ACCEL_DPS2) * dt
        self._yaw_rate += max(-step, min(step, rate - self._yaw_rate))
        self._yaw = wrap_pi(self._yaw + self._yaw_rate * dt)
        return Command(gn, ge, self._v[0], self._v[1], self._yaw)


__all__ = [
    "APPROACH_DECEL_MPS2",
    "APPROACH_GAIN_PER_S",
    "COMMAND_ACCEL_MPS2",
    "COMMAND_SMOOTH_S_PER_M",
    "CORRECTION_SMOOTH_S_PER_M",
    "GOAL_DEADBAND_M",
    "HOLD_DEADBAND_M",
    "PACE_GATE_SIGMAS",
    "PACE_SMOOTH_S_PER_MPS",
    "RANGE_DEADBAND_MAX_M",
    "RANGE_DEADBAND_SIGMAS",
    "SAFE_DISTANCE_M",
    "SAFE_STANDOFF_MARGIN_M",
    "SEPARATION_SIGMAS",
    "SMOOTH_MAX_S",
    "STILL_ENTER_DRIFT_M",
    "STILL_ENTER_SPEED_MPS",
    "STILL_EXIT_MIN_M",
    "STILL_EXIT_RADIUS_M",
    "STILL_EXIT_SIGMA",
    "STILL_PULL_M",
    "STILL_SPAN_S",
    "YAW_RATE_MAX_DPS",
    "Command",
    "StandoffController",
    "StillSubject",
    "SubjectRef",
    "clear_distance",
    "keep_clear",
    "safe_distance",
    "sqrt_law",
]
