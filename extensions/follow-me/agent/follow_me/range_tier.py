"""The shortest standoff the follow flies, from how well the subject's range
is measured.

A camera alone puts the subject's range out by 0.5-1 m (σ) and more on an
outlier, so the estimate dips toward the vehicle by a metre or two for a
moment. At a 3 or 4 m standoff such a dip reaches the distance the
follow's guard and the pilot's subject brake keep, and the vehicle brakes
on noise: a tilt and a reversal along the line of sight every few seconds.
A measured range (a lidar or a depth camera, σ 0.2 m or better) dips by
centimetres, and 3 m flies smoothly. So the shortest standoff, for the
follow distance and the orbit radius alike, is MEASURED_MIN_STANDOFF_M on
measured range and MONOCULAR_MIN_STANDOFF_M otherwise. A shorter setting is
flown at that minimum and read back as raised.

Which range the follow has is read from the fixes themselves, not from the
sensor's name: the range scatter, how far the fixes actually scatter about
the subject estimate along the line of sight (the filter's measured scatter
ratio times the range sigma the fixes are fused with). That scatter also
carries the estimate's own error and the subject's manoeuvres: a measured
range (σ MEASURED_RANGE_SIGMA_M) reads 0.1-0.3 m, a monocular one (σ 0.5 m
or more) 0.45-0.9 m. Once the filter has averaged enough fixes to tell
(:attr:`follow_me.target_filter.TargetFilter.scatter_settled`), the follow
takes the measured tier when the scatter is MEASURED_ENTER_M or less and
leaves it only above MEASURED_LEAVE_M, so the minimum does not flip on a
noisy stretch. A camera's scatter shows sooner: past MEASURED_LEAVE_M on
the first fixes the filter measures at all, the tier is the monocular one
at once.

The minimum flown moves with the tier, never toward a standing subject:

* Until the tier is known the follow does not close in from the distance
  it engaged at to a shorter setting, nor open it out: the minimum is that
  distance less the goal deadband round a standing subject
  (``GOAL_DEADBAND_M``: the goal stays where the vehicle is), held between
  the two tiers', when the setting is shorter. A vehicle that engaged 3 m
  from a subject is not sent off to 5 m on a fresh track's first fixes,
  nor brought in from 5 m to a 3 m setting.
* A larger minimum (the tier found, or turned, monocular) applies at once.
* A smaller one is approached at INWARD_RATE_MPS, and only while the
  subject moves: the vehicle is never repositioned toward a person who
  stands.
"""

from __future__ import annotations

from dataclasses import replace

from follow_me.standoff import GOAL_DEADBAND_M
from follow_me.state import FOLLOW_MODE_ORBIT, FollowConfig

MEASURED_RANGE_SIGMA_M = 0.2
MEASURED_ENTER_M = 1.5 * MEASURED_RANGE_SIGMA_M
MEASURED_LEAVE_M = 0.45
MEASURED_MIN_STANDOFF_M = 3.0
MONOCULAR_MIN_STANDOFF_M = 5.0
# How fast the minimum flown comes in to a smaller tier's: the goal moves
# toward the subject no faster than this, on top of the subject's own pace.
INWARD_RATE_MPS = 0.2

# Why a shorter setting is flown farther (read back as `standoff_reason`):
# the range is a camera's alone; it is not known yet; the measured range's
# minimum is nearer, but the subject stands; or the vehicle is closing in to
# it while the subject moves.
STANDOFF_CAMERA_RANGE = "camera-range"
STANDOFF_RANGE_UNKNOWN = "range-unknown"
STANDOFF_SUBJECT_STANDING = "subject-standing"
STANDOFF_CLOSING_IN = "closing-in"


class RangeTier:
    """Which range tier the follow is in, from the range scatter it
    measures, and the minimum standoff flown on it."""

    __slots__ = ("_least", "_still", "_t", "measured")

    def __init__(self) -> None:
        # True: measured range; False: monocular; None: not known yet.
        self.measured: bool | None = None
        # The minimum standoff flown, or None before the follow engages.
        self._least: float | None = None
        self._t: float | None = None
        # Whether the subject stood at the last step.
        self._still = False

    def reset(self) -> None:
        self.measured = None
        self._least = None
        self._t = None
        self._still = False

    def update(self, scatter_m: float, *, settled: bool) -> None:
        """Take a range scatter (m) the filter measures; ``settled``: whether
        it has averaged enough fixes to read in full."""
        if settled:
            leave = MEASURED_LEAVE_M if self.measured else MEASURED_ENTER_M
            self.measured = scatter_m <= leave
        elif scatter_m > MEASURED_LEAVE_M:
            self.measured = False

    @property
    def engaged(self) -> bool:
        """Whether a minimum is flown (see :meth:`engage`)."""
        return self._least is not None

    def engage(self, range_m: float, set_m: float) -> None:
        """Start flying a minimum. ``range_m`` is the vehicle's horizontal
        distance from the subject and ``set_m`` the standoff set: one set
        shorter than ``range_m`` less GOAL_DEADBAND_M is held at that,
        between the tiers' minima."""
        if self._least is None:
            hold = range_m - GOAL_DEADBAND_M
            least = hold if set_m < hold else MEASURED_MIN_STANDOFF_M
            self._least = min(MONOCULAR_MIN_STANDOFF_M, max(MEASURED_MIN_STANDOFF_M, least))

    def step(self, now: float, *, still: bool) -> None:
        """Move the minimum flown toward the tier's at ``now``; ``still``:
        whether the subject stands."""
        dt = 0.0 if self._t is None else max(0.0, now - self._t)
        self._t = now
        self._still = still
        if self._least is None or self.measured is None:
            return
        target = MEASURED_MIN_STANDOFF_M if self.measured else MONOCULAR_MIN_STANDOFF_M
        if target >= self._least:
            self._least = target
        elif not still:
            self._least = max(target, self._least - INWARD_RATE_MPS * dt)

    @property
    def min_standoff_m(self) -> float:
        """The minimum standoff flown; the measured tier's before the
        follow engages (nothing is raised then)."""
        return MEASURED_MIN_STANDOFF_M if self._least is None else self._least

    @property
    def reason(self) -> str:
        """Why the minimum flown is what it is (see STANDOFF_CAMERA_RANGE
        and the rest)."""
        if self.measured is None:
            return STANDOFF_RANGE_UNKNOWN
        if not self.measured:
            return STANDOFF_CAMERA_RANGE
        return STANDOFF_SUBJECT_STANDING if self._still else STANDOFF_CLOSING_IN

    def apply_to(self, cfg: FollowConfig) -> FollowConfig:
        """``cfg`` with the follow distance and the orbit radius at least the
        minimum flown, whether the one flown was raised to it, and why."""
        least = self.min_standoff_m
        flown = cfg.orbit_radius_m if cfg.follow_mode == FOLLOW_MODE_ORBIT else cfg.follow_distance_m
        raised = flown < least
        return replace(
            cfg,
            follow_distance_m=max(cfg.follow_distance_m, least),
            orbit_radius_m=max(cfg.orbit_radius_m, least),
            standoff_min_m=least,
            standoff_raised=raised,
            standoff_reason=self.reason if raised else None,
        )


__all__ = [
    "INWARD_RATE_MPS",
    "MEASURED_ENTER_M",
    "MEASURED_LEAVE_M",
    "MEASURED_MIN_STANDOFF_M",
    "MEASURED_RANGE_SIGMA_M",
    "MONOCULAR_MIN_STANDOFF_M",
    "STANDOFF_CAMERA_RANGE",
    "STANDOFF_CLOSING_IN",
    "STANDOFF_RANGE_UNKNOWN",
    "STANDOFF_SUBJECT_STANDING",
    "RangeTier",
]
