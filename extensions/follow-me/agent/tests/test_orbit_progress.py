"""Closed-loop orbit progress: the plugin's orbit geometry flown by a small
kinematic stand-in for the pilot and the flight controller.

Each 20 Hz tick advances the phase and builds the goal with the plugin's own
``setpoint.advance_orbit_phase`` and ``setpoint.orbit_goal``. The stand-in:

* flies a clear goal as the pilot does, feed-forward plus the position error
  (the FC's position gain is 1), capped at the goal's speed limit;
* replaces a goal inside the obstacle margin of a trunk with the pilot's
  orbit detour candidate: the ring points ±10…60° around the phase at the
  radius and ±2 m, cheapest first (|Δφ|/60 + |Δr|/2), ties to the nearest,
  and flies there without feed-forward;
* cuts any step toward the trunk inside the margin, so the vehicle slides
  round it as the pilot's brake leaves it only a sideways push;
* lags the commanded velocity by 0.5 s, as the FC does.

The subject stands still. Its estimate is wrong along the line of sight only,
as a monocular range is, and is refreshed at 3 Hz.

The orbit has to cover at least 0.8 and at most 1.1 of the commanded turns,
hold the radius, and keep clear of the trunk.
"""

from __future__ import annotations

import math
import random

import pytest

from follow_me import setpoint

_R = 8.0
_RATE_DPS = 10.0
_MAX_SPEED = 6.0
_MARGIN = 1.5
_TRUNK_RADIUS = 0.4
_DT = 0.05
_FC_LAG_S = 0.5
_FIX_PERIOD_S = 1.0 / 3.0
_FLIGHT_S = 60.0
_COMMANDED_TURNS = _RATE_DPS * _FLIGHT_S / 360.0
# A recorded forest follow measured the subject's range off by 2.6 m (median).
_RANGE_SIGMA_M = 2.6

_DETOUR_DPHI = (10.0, -10.0, 20.0, -20.0, 30.0, -30.0, 40.0, -40.0, 50.0, -50.0, 60.0, -60.0)
_DETOUR_DR = (0.0, 2.0, -2.0)

Point = tuple[float, float]


def _clamp(v: Point, limit: float) -> Point:
    n = math.hypot(*v)
    return v if n <= limit else (v[0] * limit / n, v[1] * limit / n)


def _fly(
    *, direction: str, trunk_bearing_deg: float | None, range_sigma_m: float, seed: int
) -> tuple[float, float, float]:
    """Signed turns flown round the true subject (+ in the commanded
    direction), the median true range, and the least clearance to the trunk."""
    rnd = random.Random(seed)
    s = 1.0 if direction == "cw" else -1.0
    trunk: Point | None = None
    if trunk_bearing_deg is not None:
        tb = math.radians(trunk_bearing_deg)
        trunk = (_R * math.cos(tb), _R * math.sin(tb))

    def clearance(p: Point) -> float:
        if trunk is None:
            return math.inf
        return math.hypot(p[0] - trunk[0], p[1] - trunk[1]) - _TRUNK_RADIUS

    pos: Point = (-_R, 0.0)
    vel: Point = (0.0, 0.0)
    est: Point = (0.0, 0.0)
    phase: float | None = None
    next_fix = 0.0
    prev_bearing = math.atan2(pos[1], pos[0])
    turned = 0.0
    ranges: list[float] = []
    least = math.inf
    t = 0.0
    while t < _FLIGHT_S:
        if t >= next_fix:
            next_fix += _FIX_PERIOD_S
            d = math.hypot(*pos)
            err = rnd.gauss(0.0, range_sigma_m)
            est = (-pos[0] / d * err, -pos[1] / d * err)
        if phase is None:
            here = setpoint.bearing(est[0], est[1], pos[0], pos[1])
            assert here is not None
            phase = here
        else:
            phase = setpoint.advance_orbit_phase(
                phase,
                center_n=est[0],
                center_e=est[1],
                rate_dps=_RATE_DPS,
                direction=direction,
                drone_n=pos[0],
                drone_e=pos[1],
                dt_s=_DT,
            )
        g = setpoint.orbit_goal(
            center_n=est[0],
            center_e=est[1],
            center_vn=0.0,
            center_ve=0.0,
            phase_rad=phase,
            radius_m=_R,
            rate_dps=_RATE_DPS,
            direction=direction,
            max_speed_mps=_MAX_SPEED,
            ff_scale=1.0,
        )
        goal: Point = (g.north_m, g.east_m)
        ff: Point = (g.vn_mps, g.ve_mps)
        if clearance(goal) < _MARGIN:
            candidates = []
            for dphi in _DETOUR_DPHI:
                for dr in _DETOUR_DR:
                    b = phase + math.radians(dphi)
                    p = (est[0] + (_R + dr) * math.cos(b), est[1] + (_R + dr) * math.sin(b))
                    cost = abs(dphi) / 60.0 + abs(dr) / 2.0
                    candidates.append((cost, math.hypot(p[0] - pos[0], p[1] - pos[1]), p))
            candidates.sort(key=lambda c: (c[0], c[1]))
            pick = next((c[2] for c in candidates if clearance(c[2]) >= _MARGIN), None)
            if pick is None:
                ask: Point = (0.0, 0.0)
            else:
                to = (pick[0] - pos[0], pick[1] - pos[1])
                v = _clamp(to, _MAX_SPEED)
                ask = (v[0] + to[0], v[1] + to[1])
        else:
            to = (goal[0] - pos[0], goal[1] - pos[1])
            ask = (ff[0] + to[0], ff[1] + to[1])
        cmd = _clamp(ask, _MAX_SPEED)
        k = min(1.0, _DT / _FC_LAG_S)
        vel = (vel[0] + (cmd[0] - vel[0]) * k, vel[1] + (cmd[1] - vel[1]) * k)
        nxt = (pos[0] + vel[0] * _DT, pos[1] + vel[1] * _DT)
        if trunk is not None and clearance(nxt) < _MARGIN and clearance(nxt) < clearance(pos):
            w = (trunk[0] - pos[0], trunk[1] - pos[1])
            wn = math.hypot(*w)
            inward = (vel[0] * w[0] + vel[1] * w[1]) / wn
            vel = (vel[0] - inward * w[0] / wn, vel[1] - inward * w[1] / wn)
            nxt = (pos[0] + vel[0] * _DT, pos[1] + vel[1] * _DT)
        pos = nxt
        least = min(least, clearance(pos))
        b = math.atan2(pos[1], pos[0])
        turned += setpoint.wrap_pi(b - prev_bearing)
        prev_bearing = b
        ranges.append(math.hypot(*pos))
        t += _DT
    ranges.sort()
    return s * math.degrees(turned) / 360.0, ranges[len(ranges) // 2], least


def _assert_orbit(turns: float, median_range: float, least_clearance: float) -> None:
    assert 0.8 * _COMMANDED_TURNS <= turns <= 1.1 * _COMMANDED_TURNS, (
        f"{turns:.2f} turns flown, {_COMMANDED_TURNS:.2f} commanded"
    )
    assert abs(median_range - _R) < 1.5
    assert least_clearance >= 1.0


@pytest.mark.parametrize(
    ("direction", "trunk_bearing_deg"), [("cw", 225.0), ("ccw", 135.0)], ids=["cw", "ccw"]
)
def test_the_orbit_circles_past_a_trunk_on_the_ring(direction: str, trunk_bearing_deg: float) -> None:
    # The trunk sits 45° along the orbit from the start.
    _assert_orbit(
        *_fly(direction=direction, trunk_bearing_deg=trunk_bearing_deg, range_sigma_m=0.0, seed=1)
    )


@pytest.mark.parametrize("seed", range(1, 6))
def test_the_orbit_keeps_its_rate_through_range_noise(seed: int) -> None:
    _assert_orbit(
        *_fly(direction="cw", trunk_bearing_deg=None, range_sigma_m=_RANGE_SIGMA_M, seed=seed)
    )


@pytest.mark.parametrize("seed", range(1, 6))
def test_the_orbit_circles_past_a_trunk_through_range_noise(seed: int) -> None:
    _assert_orbit(
        *_fly(direction="cw", trunk_bearing_deg=225.0, range_sigma_m=_RANGE_SIGMA_M, seed=seed)
    )


@pytest.mark.parametrize("direction", ["cw", "ccw"])
def test_a_fully_blocked_drone_holds_the_phase_at_the_max_lead(direction: str) -> None:
    # The pilot cannot move the drone at all for two minutes: the phase stops
    # within one step past the max lead and never runs away round the ring.
    s = 1.0 if direction == "cw" else -1.0
    step = math.radians(_RATE_DPS) * _DT
    drone = (-_R, 0.0)
    here = math.atan2(drone[1], drone[0])
    phase = here
    for _ in range(int(120.0 / _DT)):
        phase = setpoint.advance_orbit_phase(
            phase,
            center_n=0.0,
            center_e=0.0,
            rate_dps=_RATE_DPS,
            direction=direction,
            drone_n=drone[0],
            drone_e=drone[1],
            dt_s=_DT,
        )
    lead = s * setpoint.wrap_pi(phase - here)
    limit = math.radians(setpoint.ORBIT_MAX_LEAD_DEG)
    assert limit <= lead <= limit + step + 1e-9
