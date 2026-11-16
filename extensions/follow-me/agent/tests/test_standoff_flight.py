"""The follow flown closed-loop (see :mod:`follow_sim`): a standing subject
under recorded range noise holds the vehicle still, a far subject is caught
up with fast and approached without overshooting the standoff, an orbit
turns at its commanded rate, and a missed detection neither stops nor slows
the vehicle.

The targets are the forest-follow audit's: with a standing subject the
recorded flight wandered at 0.27 m/s (median) and 0.95 m/s (95th
percentile), its range spread 1.5 m and its yaw turned at up to 10°/s
(95th percentile); its orbit turned at 0.04°/s of a commanded 10°/s.
"""

from __future__ import annotations

import math
from typing import Any

import pytest
from follow_sim import DT, Flight, fly, percentile


def _still(_t: float) -> tuple[float, float]:
    return (0.0, 0.0)


def _yaw_rates_dps(f: Flight, idx: range) -> list[float]:
    out = []
    prev = None
    for i in idx:
        yaw = f.goals[i].get("yaw")
        if yaw is None:
            prev = None
            continue
        if prev is not None:
            d = (yaw - prev + math.pi) % (2.0 * math.pi) - math.pi
            out.append(abs(math.degrees(d)) / DT)
        prev = yaw
    return out


def _sign_flips(f: Flight, idx: range) -> int:
    """Reversals of the commanded velocity (either axis, |v| > 0.05)."""
    flips = 0
    last = [0.0, 0.0]
    for i in idx:
        g = f.goals[i]
        for k, key in enumerate(("vn", "ve")):
            v = float(g.get(key, 0.0))
            if abs(v) <= 0.05:
                continue
            if last[k] * v < 0.0:
                flips += 1
            last[k] = v
    return flips


# The detector rate the agent runs at (10 Hz) and the one the recorded
# flights ran at (3 Hz), with what each must hold a standing subject to: the
# recorded 0.95 m/s (95th percentile) down to 0.1 at 10 Hz and 0.2 at 3 Hz.
@pytest.mark.asyncio
@pytest.mark.parametrize(("fix_hz", "speed_p95"), [(10.0, 0.1), (3.0, 0.2)])
async def test_a_standing_subject_under_range_noise_holds_the_vehicle_still(
    monkeypatch: Any, fix_hz: float, speed_p95: float
) -> None:
    for seed in range(1, 9):
        f = await fly(
            monkeypatch,
            subject=_still,
            start=(-6.0, 0.0),
            duration_s=60.0,
            miss=0.3,
            seed=seed,
            fix_hz=fix_hz,
        )
        idx = f.after(10.0)
        speeds = [f.speed(i) for i in idx]
        ranges = [f.range(i) for i in idx]
        mean = sum(ranges) / len(ranges)
        sd = math.sqrt(sum((r - mean) ** 2 for r in ranges) / len(ranges))
        yaw_p95 = percentile(_yaw_rates_dps(f, idx), 0.95)
        print(
            f"{fix_hz} Hz seed {seed}: speed p50 {percentile(speeds, 0.5):.3f}"
            f" p95 {percentile(speeds, 0.95):.3f} range {mean:.2f}±{sd:.2f}"
            f" yaw p95 {yaw_p95:.2f} flips {_sign_flips(f, idx)}"
        )
        assert percentile(speeds, 0.95) <= speed_p95
        assert sd <= 0.5
        assert abs(mean - 6.0) <= 1.0
        assert yaw_p95 <= 5.0
        # 50 s of standing: no velocity reversals at the detector's rate (the
        # recorded flight reversed 54 times a minute).
        assert _sign_flips(f, idx) <= 2
        assert all(p == "following" for p in (f.phases[i] for i in idx))


def _walker(t: float) -> tuple[float, float]:
    return (30.0 + 1.4 * t, 0.0)


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("label", "subject"), [("walking", _walker), ("standing", lambda _t: (30.0, 0.0))]
)
async def test_a_far_subject_is_caught_up_fast_and_approached_without_overshoot(
    monkeypatch: Any, label: str, subject: Any
) -> None:
    # The speed law itself, on clean fixes: range noise is the next test's.
    f = await fly(
        monkeypatch,
        subject=subject,
        start=(0.0, 0.0),
        duration_s=60.0,
        seed=4,
        range_sigma_m=0.0,
        outlier_share=0.0,
    )
    all_idx = range(len(f.t))
    speeds = [f.speed(i) for i in all_idx]
    ranges = [f.range(i) for i in all_idx]
    print(
        f"{label}: peak {max(speeds):.2f} m/s, least range {min(ranges):.2f} m,"
        f" final {ranges[-1]:.2f} m"
    )
    # Fast when far: well over the subject's pace while beyond 2x the
    # standoff (the recorded flights crawled at 1.7-1.9 m/s at 8-20 m). From
    # rest 24 m short, accelerating at 0.8 and braking at 0.6 m/s² peaks at
    # about 4 m/s; a walker running ahead lets it reach more.
    assert max(speeds[i] for i in all_idx if ranges[i] > 12.0) >= 3.5
    # No overshoot into the standoff.
    assert min(ranges) >= 6.0 - 0.5
    # Settled at the standoff.
    assert all(abs(ranges[i] - 6.0) <= 0.5 for i in f.after(45.0))
    # Inside 2x the standoff the closing speed only falls: a ramp, not a
    # brake and a surge.
    closing = [(ranges[i] - ranges[i + 1]) / DT for i in range(len(ranges) - 1)]
    inside = next(i for i, r in enumerate(ranges) if r <= 12.0)
    settled = next(i for i, r in enumerate(ranges) if r <= 6.5)
    for a, b in zip(closing[inside:settled], closing[inside + 1 : settled]):
        assert b <= a + 0.05


@pytest.mark.asyncio
@pytest.mark.parametrize(("fix_hz", "rms_mean"), [(10.0, 0.5), (3.0, 1.0)])
async def test_a_walking_subject_under_range_noise_is_held_at_the_standoff(
    monkeypatch: Any, fix_hz: float, rms_mean: float
) -> None:
    # Recorded range noise and 30 % missed frames, ten flights: the standoff
    # error is within 1 m RMS on average (0.5 m at the agent's 10 Hz) and
    # inside ±2 m at least 80 % of every flight (the recorded walks were
    # never inside ±2 m).
    rms, bands = [], []
    for seed in range(1, 11):
        f = await fly(
            monkeypatch,
            subject=lambda t: (8.0 + 1.4 * t, 0.0),
            start=(2.0, 0.0),
            duration_s=50.0,
            miss=0.3,
            seed=seed,
            fix_hz=fix_hz,
        )
        ranges = [f.range(i) for i in f.after(10.0)]
        rms.append(math.sqrt(sum((r - 6.0) ** 2 for r in ranges) / len(ranges)))
        bands.append(sum(abs(r - 6.0) <= 2.0 for r in ranges) / len(ranges))
    print(f"standoff rms {[round(r, 2) for r in rms]}, inside ±2 m {[round(b, 2) for b in bands]}")
    assert sum(rms) / len(rms) <= rms_mean
    assert min(bands) >= 0.8


@pytest.mark.asyncio
@pytest.mark.parametrize("seed", [1, 2, 3])
async def test_a_walker_that_stops_is_held_still_and_followed_again(
    monkeypatch: Any, seed: int
) -> None:
    # Walks 20 s, stands 15 s, walks on; the agent's 10 Hz detector.
    def subject(t: float) -> tuple[float, float]:
        return (8.0 + 1.4 * min(t, 20.0) + 1.4 * max(0.0, t - 35.0), 0.0)

    f = await fly(
        monkeypatch,
        subject=subject,
        start=(2.0, 0.0),
        duration_s=60.0,
        miss=0.3,
        seed=seed,
        fix_hz=10.0,
    )
    # Once the vehicle has come to rest after the stop (it overshoots and
    # comes back to the standoff first), until the walk on: it stays at rest.
    second = int(round(1.0 / DT))
    rest = next(
        i for i in f.after(21.0) if all(f.speed(k) < 0.1 for k in range(i, i + second))
    )
    idx = range(rest, f.after(35.0)[0])
    speeds = [f.speed(i) for i in idx]
    print(f"seed {seed}: at rest {f.t[rest] - 20.0:.1f} s after the stop, speed p95 {percentile(speeds, 0.95):.3f}")
    assert f.t[rest] - 20.0 <= 10.0
    assert percentile(speeds, 0.95) <= 0.1
    assert _sign_flips(f, idx) == 0
    assert percentile(_yaw_rates_dps(f, idx), 0.95) <= 5.0
    # Following again: back inside ±1 m of the standoff within 15 s.
    assert any(abs(f.range(i) - 6.0) <= 1.0 for i in f.after(45.0) if f.t[i] < 50.0)
    assert all(p == "following" for p in f.phases)


@pytest.mark.asyncio
@pytest.mark.parametrize("direction", ["cw", "ccw"])
async def test_an_orbit_turns_at_its_commanded_rate_under_range_noise(
    monkeypatch: Any, direction: str
) -> None:
    f = await fly(
        monkeypatch,
        subject=_still,
        start=(-8.0, 0.0),
        duration_s=70.0,
        config={
            "follow_mode": "orbit",
            "orbit_radius_m": 8.0,
            "orbit_rate_dps": 10.0,
            "orbit_direction": direction,
        },
        miss=0.3,
        seed=5,
    )
    idx = f.after(10.0)
    s = 1.0 if direction == "cw" else -1.0
    turned = 0.0
    prev = None
    for i in idx:
        b = math.atan2(f.pos[i][1], f.pos[i][0])
        if prev is not None:
            turned += (b - prev + math.pi) % (2.0 * math.pi) - math.pi
        prev = b
    seconds = f.t[idx[-1]] - f.t[idx[0]]
    rate = s * math.degrees(turned) / seconds
    ranges = [f.range(i) for i in idx]
    print(f"{direction}: {rate:.2f} deg/s, radius p50 {percentile(ranges, 0.5):.2f}")
    assert 9.0 <= rate <= 11.0
    assert abs(percentile(ranges, 0.5) - 8.0) <= 0.75


@pytest.mark.asyncio
async def test_missed_detections_neither_stop_nor_slow_the_follow(monkeypatch: Any) -> None:
    # Every other batch a miss (the recorded walk had 59 % unsure batches),
    # clean fixes otherwise: the follow neither holds nor slows.
    f = await fly(
        monkeypatch,
        subject=lambda t: (8.0 + 1.4 * t, 0.0),
        start=(2.0, 0.0),
        duration_s=40.0,
        miss=0.5,
        seed=6,
        range_sigma_m=0.0,
        outlier_share=0.0,
    )
    idx = f.after(15.0)
    assert all(f.phases[i] == "following" for i in idx)
    assert all(f.goals[i].get("kind") == "follow" for i in idx)
    cmd = [math.hypot(f.goals[i]["vn"], f.goals[i]["ve"]) for i in idx]
    # The command never steps: at most the 0.8 m/s² slew per tick.
    assert max(abs(b - a) for a, b in zip(cmd, cmd[1:])) <= 0.8 * DT + 1e-6
    # Never more than 30 % off the subject's pace.
    assert all(abs(c - 1.4) <= 0.3 * 1.4 for c in cmd)


@pytest.mark.asyncio
async def test_a_single_uncertain_batch_does_not_zero_the_commanded_velocity(
    monkeypatch: Any,
) -> None:
    # Follow a walker on clean fixes, then one batch reports the subject
    # unsure: the next goal still follows at the same speed.
    f = await fly(
        monkeypatch,
        subject=lambda t: (8.0 + 1.4 * t, 0.0),
        start=(2.0, 0.0),
        duration_s=20.0,
        range_sigma_m=0.0,
        outlier_share=0.0,
        miss_frames=(46,),
    )
    # Frame k is the batch at (k - 1) / 3 s.
    i = next(k for k, t in enumerate(f.t) if t >= 45 / 3.0 - 1e-9)
    before = math.hypot(f.goals[i - 1]["vn"], f.goals[i - 1]["ve"])
    for k in range(i, i + 6):
        g = f.goals[k]
        assert g["kind"] == "follow" and f.phases[k] == "following"
        assert math.hypot(g["vn"], g["ve"]) >= 0.9 * before > 1.0
