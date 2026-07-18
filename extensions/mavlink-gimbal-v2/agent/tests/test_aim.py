"""Unit tests for the pure visual-servo aim controller.

No I/O, no MAVLink, no SDK: the controller takes a duck-typed bounding
box and returns absolute gimbal setpoints. These tests pin the sign
conventions, the dead-band hold, axis clamping, closed-loop convergence,
and the degenerate-frame guard.
"""

from __future__ import annotations

from types import SimpleNamespace

from altnautica_gimbal_v2.aim import AimConfig, GimbalAimController


def _bbox(cx: float, cy: float, w: float = 40.0, h: float = 40.0) -> SimpleNamespace:
    """A box centred at ``(cx, cy)`` in frame pixels."""
    return SimpleNamespace(x=cx - w / 2.0, y=cy - h / 2.0, width=w, height=h)


def _cfg(**overrides) -> AimConfig:
    base = dict(
        frame_width=1280.0,
        frame_height=720.0,
        hfov_deg=66.0,
        vfov_deg=41.0,
        gain=0.3,
        deadband_frac=0.04,
    )
    base.update(overrides)
    return AimConfig(**base)


def test_centred_target_is_held_within_deadband() -> None:
    ctrl = GimbalAimController(_cfg())
    # Dead-centre: cx=640, cy=360.
    assert ctrl.update(_bbox(640.0, 360.0)) is None
    # Slightly off but inside the 0.04 dead-band on both axes.
    assert ctrl.update(_bbox(640.0 + 0.02 * 640.0, 360.0 + 0.02 * 360.0)) is None
    # The controller commanded nothing, so its model stays at zero.
    assert ctrl.cmd_pitch == 0.0
    assert ctrl.cmd_yaw == 0.0


def test_target_to_the_right_yields_positive_yaw_delta() -> None:
    ctrl = GimbalAimController(_cfg())
    # nx = +0.5 (right of centre), ny = 0.
    result = ctrl.update(_bbox(640.0 + 0.5 * 640.0, 360.0))
    assert result is not None
    pitch, yaw = result
    assert yaw > 0.0  # right target drives yaw positive with invert_yaw=False
    assert pitch == 0.0  # no vertical error, so pitch is untouched


def test_target_below_centre_yields_positive_pitch_delta() -> None:
    ctrl = GimbalAimController(_cfg())
    # ny = +0.5 (below centre), nx = 0.
    result = ctrl.update(_bbox(640.0, 360.0 + 0.5 * 360.0))
    assert result is not None
    pitch, yaw = result
    assert pitch > 0.0
    assert yaw == 0.0


def test_invert_yaw_flips_the_sign() -> None:
    plain = GimbalAimController(_cfg())
    inverted = GimbalAimController(_cfg(invert_yaw=True))
    box = _bbox(640.0 + 0.5 * 640.0, 360.0)
    _, yaw_plain = plain.update(box)  # type: ignore[misc]
    _, yaw_inv = inverted.update(box)  # type: ignore[misc]
    assert yaw_plain > 0.0
    assert yaw_inv < 0.0
    assert yaw_inv == -yaw_plain


def test_invert_pitch_flips_the_sign() -> None:
    plain = GimbalAimController(_cfg())
    inverted = GimbalAimController(_cfg(invert_pitch=True))
    box = _bbox(640.0, 360.0 + 0.5 * 360.0)
    pitch_plain, _ = plain.update(box)  # type: ignore[misc]
    pitch_inv, _ = inverted.update(box)  # type: ignore[misc]
    assert pitch_plain > 0.0
    assert pitch_inv < 0.0
    assert pitch_inv == -pitch_plain


def test_command_is_clamped_to_yaw_limits() -> None:
    ctrl = GimbalAimController(
        _cfg(gain=1.0, hfov_deg=90.0, yaw_min_deg=-5.0, yaw_max_deg=5.0)
    )
    # Full-right target with unit gain would command +45 deg; the clamp
    # must cap it at the +5 deg maximum.
    result = ctrl.update(_bbox(1280.0, 360.0))
    assert result is not None
    assert result[1] == 5.0
    assert ctrl.cmd_yaw == 5.0


def test_command_is_clamped_to_pitch_limits() -> None:
    ctrl = GimbalAimController(
        _cfg(gain=1.0, vfov_deg=90.0, pitch_min_deg=-4.0, pitch_max_deg=6.0)
    )
    # Target far above centre drives pitch negative; clamp at pitch_min.
    result = ctrl.update(_bbox(640.0, 0.0))
    assert result is not None
    assert result[0] == -4.0
    assert ctrl.cmd_pitch == -4.0


def test_degenerate_frame_returns_none() -> None:
    assert GimbalAimController(_cfg(frame_width=0.0)).update(_bbox(10.0, 10.0)) is None
    assert GimbalAimController(_cfg(frame_height=0.0)).update(_bbox(10.0, 10.0)) is None
    assert (
        GimbalAimController(_cfg(frame_width=-1280.0)).update(_bbox(10.0, 10.0))
        is None
    )


def test_reset_zeroes_the_model() -> None:
    ctrl = GimbalAimController(_cfg())
    ctrl.update(_bbox(640.0 + 0.5 * 640.0, 360.0 + 0.5 * 360.0))
    assert ctrl.cmd_yaw != 0.0 or ctrl.cmd_pitch != 0.0
    ctrl.reset()
    assert ctrl.cmd_pitch == 0.0
    assert ctrl.cmd_yaw == 0.0


def test_closed_loop_converges_with_shrinking_steps() -> None:
    """A world-fixed off-centre target: as the gimbal yaw approaches it,
    the target's pixel offset shrinks, so the commanded yaw converges and
    the per-step delta decays. This models the physical feedback the pure
    controller does not itself simulate."""
    cfg = _cfg(hfov_deg=60.0, gain=0.3)
    ctrl = GimbalAimController(cfg)
    target_yaw = 20.0  # degrees, inside the yaw range and the half-FOV band
    half = cfg.hfov_deg / 2.0

    commanded: list[float] = []
    held = False
    for _ in range(60):
        nx = (target_yaw - ctrl.cmd_yaw) / half
        nx = max(-1.0, min(1.0, nx))
        cx = cfg.frame_width / 2.0 + nx * (cfg.frame_width / 2.0)
        res = ctrl.update(_bbox(cx, cfg.frame_height / 2.0))
        if res is None:
            held = True
            break
        commanded.append(res[1])

    assert len(commanded) >= 3
    # Monotonic approach toward the target.
    assert all(commanded[i] < commanded[i + 1] for i in range(len(commanded) - 1))
    assert commanded[-1] <= target_yaw  # never overshoots
    # Each step is smaller than the previous (converging, not ramping).
    deltas = [commanded[i + 1] - commanded[i] for i in range(len(commanded) - 1)]
    assert all(deltas[i] > deltas[i + 1] for i in range(len(deltas) - 1))
    assert deltas[-1] > 0.0
    # The loop settles inside the dead-band and then holds.
    assert held
    assert abs(target_yaw - ctrl.cmd_yaw) <= cfg.deadband_frac * half + 1e-9


def test_rate_holds_inside_deadband() -> None:
    ctrl = GimbalAimController(_cfg())
    assert ctrl.rate(_bbox(640.0, 360.0)) is None


def test_rate_right_target_yields_positive_yaw_rate() -> None:
    ctrl = GimbalAimController(_cfg())
    result = ctrl.rate(_bbox(640.0 + 0.5 * 640.0, 360.0))
    assert result is not None
    pitch_rate, yaw_rate = result
    assert yaw_rate > 0.0  # right target slews yaw positive
    assert pitch_rate == 0.0  # no vertical error


def test_rate_invert_yaw_flips_the_sign() -> None:
    box = _bbox(640.0 + 0.5 * 640.0, 360.0)
    plain = GimbalAimController(_cfg()).rate(box)
    inverted = GimbalAimController(_cfg(invert_yaw=True)).rate(box)
    assert plain is not None and inverted is not None
    assert plain[1] > 0.0
    assert inverted[1] == -plain[1]


def test_rate_does_not_integrate_the_model() -> None:
    # Rate commands do not touch the controller's integrated-angle model, so a
    # switch back to position mode resumes from the held angle.
    ctrl = GimbalAimController(_cfg())
    ctrl.rate(_bbox(640.0 + 0.5 * 640.0, 360.0))
    assert ctrl.cmd_pitch == 0.0
    assert ctrl.cmd_yaw == 0.0
