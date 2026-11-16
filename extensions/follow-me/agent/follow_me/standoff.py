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
  ``COMMAND_ACCEL_MPS2`` per second, so it never steps.
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

    def reset(self) -> None:
        self._t = None
        self._goal = None

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
    ) -> Command:
        """``latch_goal``: hold the goal through moves inside the deadband (a
        standing subject followed at a fixed angle; an orbit's goal moves on
        purpose)."""
        if self._t is None or now - self._t > _RESUME_AFTER_S:
            self._v = drone_v_ne
            self._yaw = drone_yaw
            self._yaw_rate = 0.0
            self._goal = None
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
        ahead = dist - max(0.0, rel) * APPROACH_LEAD_S - (HOLD_DEADBAND_M if still else 0.0)
        closing = sqrt_law(ahead, APPROACH_GAIN_PER_S, APPROACH_DECEL_MPS2)
        closing = min(closing, max(0.0, max_speed_mps))
        want = _clamp_len(goal_v_ne[0] + un * closing, goal_v_ne[1] + ue * closing, max_speed_mps)
        dvn, dve = _clamp_len(want[0] - self._v[0], want[1] - self._v[1], COMMAND_ACCEL_MPS2 * dt)
        self._v = (self._v[0] + dvn, self._v[1] + dve)

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
    "GOAL_DEADBAND_M",
    "HOLD_DEADBAND_M",
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
    "sqrt_law",
]
