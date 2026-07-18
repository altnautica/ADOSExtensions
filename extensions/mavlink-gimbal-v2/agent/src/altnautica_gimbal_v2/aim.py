"""Visual-servo aim controller for the gimbal (pure, no I/O).

Given a detected target's bounding box in the camera frame, the
controller nudges an absolute gimbal pitch/yaw setpoint so the target
drifts toward the frame centre. It is a small incremental controller,
not a full PID: each call moves the commanded angle by a gain-scaled
fraction of the current angular error, and the commanded angle is the
running integral of those nudges. In closed loop (the gimbal moves, the
camera re-centres the target) the pixel error shrinks each step, so the
commanded angle converges and the per-step nudge decays to zero.

FAIL-SAFE / bench-tunable design:

* A dead-band around centre returns ``None`` (no command) so a
  well-centred target does not produce jitter commands.
* ``gain`` defaults to a conservative 0.3 so the loop is stable and does
  not overshoot when the closed-loop model does not perfectly match the
  physical optics.
* Every commanded angle is clamped to the configured pitch/yaw limits.

Sign conventions (which way a positive pixel offset drives the gimbal)
are UNVALIDATED against real hardware. The ``invert_pitch`` /
``invert_yaw`` config flags let a bench operator correct a reversed axis
with NO code change: if the gimbal slews the wrong way, flip the flag.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class AimConfig:
    """Static configuration for :class:`GimbalAimController`.

    ``frame_width`` / ``frame_height`` are the pixel dimensions the
    bounding boxes are expressed in (the detection batch does not carry
    them, so they come from the plugin config). ``hfov_deg`` /
    ``vfov_deg`` are the camera's horizontal / vertical fields of view;
    they map a normalized pixel offset to an angular error. The pitch /
    yaw limits bound the commanded angle and normally mirror the driver's
    own clamp so the controller model and the driver agree.
    """

    frame_width: float
    frame_height: float
    hfov_deg: float
    vfov_deg: float
    gain: float = 0.3
    deadband_frac: float = 0.04
    invert_pitch: bool = False
    invert_yaw: bool = False
    pitch_min_deg: float = -90.0
    pitch_max_deg: float = 30.0
    yaw_min_deg: float = -180.0
    yaw_max_deg: float = 180.0


# Scale that turns the per-step angular error (degrees) into a rate command
# (degrees/second) for the rate-control mode. A conservative constant so the
# rate servo is stable; the gimbal integrates the rate against its own clock.
_RATE_SCALE_HZ = 5.0


def _clamp(value: float, lo: float, hi: float) -> float:
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


class GimbalAimController:
    """Incremental absolute-angle visual servo. Pure and unit-testable.

    Holds the running commanded pitch/yaw. :meth:`update` advances them
    toward centring the given box; :meth:`reset` zeros the model.
    """

    def __init__(self, config: AimConfig) -> None:
        self._cfg = config
        self.cmd_pitch: float = 0.0
        self.cmd_yaw: float = 0.0

    def _normalized_error(self, bbox: Any) -> tuple[float, float] | None:
        """The target's normalized offset from frame centre, in [-1, 1].

        Positive nx is right of centre, positive ny is below centre. Returns
        ``None`` when no command should be sent this step: the frame is
        degenerate (``width``/``height`` <= 0) or the target is inside the
        dead-band (already centred). Shared by :meth:`update` (position servo)
        and :meth:`rate` (rate servo) so both use one error model.
        """

        cfg = self._cfg
        w = float(cfg.frame_width)
        h = float(cfg.frame_height)
        if w <= 0.0 or h <= 0.0:
            return None

        # Box centre, clamped into the frame so a partially-out-of-frame
        # box cannot drive the normalized offset past the frame edge.
        cx = _clamp(float(bbox.x) + float(bbox.width) / 2.0, 0.0, w)
        cy = _clamp(float(bbox.y) + float(bbox.height) / 2.0, 0.0, h)

        nx = _clamp((cx - w / 2.0) / (w / 2.0), -1.0, 1.0)
        ny = _clamp((cy - h / 2.0) / (h / 2.0), -1.0, 1.0)

        if abs(nx) < cfg.deadband_frac and abs(ny) < cfg.deadband_frac:
            # On target: hold, emit no command (avoids centre jitter).
            return None
        return (nx, ny)

    def update(self, bbox: Any) -> tuple[float, float] | None:
        """Advance the commanded angle toward centring ``bbox``.

        ``bbox`` is any object exposing ``x`` / ``y`` / ``width`` /
        ``height`` (pixels, origin top-left, in the configured frame
        resolution). Returns the new ``(cmd_pitch, cmd_yaw)`` in degrees,
        or ``None`` when no command should be sent this step: the frame
        is degenerate (``width``/``height`` <= 0) or the target is inside
        the dead-band (already centred), in which case the gimbal holds.
        """

        err = self._normalized_error(bbox)
        if err is None:
            return None
        cfg = self._cfg
        nx, ny = err

        # Angular error: the normalized offset scaled by the half-FOV.
        yaw_err = nx * (cfg.hfov_deg / 2.0)
        pitch_err = ny * (cfg.vfov_deg / 2.0)

        d_yaw = cfg.gain * yaw_err * (-1.0 if cfg.invert_yaw else 1.0)
        d_pitch = cfg.gain * pitch_err * (-1.0 if cfg.invert_pitch else 1.0)

        self.cmd_yaw = _clamp(
            self.cmd_yaw + d_yaw, cfg.yaw_min_deg, cfg.yaw_max_deg
        )
        self.cmd_pitch = _clamp(
            self.cmd_pitch + d_pitch, cfg.pitch_min_deg, cfg.pitch_max_deg
        )
        return (self.cmd_pitch, self.cmd_yaw)

    def rate(self, bbox: Any) -> tuple[float, float] | None:
        """A proportional angular rate (deg/s) that drives ``bbox`` toward
        the frame centre, for the rate-control mode.

        Same error model and sign conventions as :meth:`update`, but returns a
        rate command instead of an integrated absolute angle, so the gimbal
        slews continuously toward the target rather than to a computed
        setpoint. Returns ``(pitch_rate, yaw_rate)`` in degrees/second, or
        ``None`` inside the dead-band (hold). The rate is not integrated here;
        the gimbal device integrates it against its own clock.
        """

        err = self._normalized_error(bbox)
        if err is None:
            return None
        cfg = self._cfg
        nx, ny = err
        yaw_err = nx * (cfg.hfov_deg / 2.0)
        pitch_err = ny * (cfg.vfov_deg / 2.0)
        yaw_rate = (
            cfg.gain * yaw_err * (-1.0 if cfg.invert_yaw else 1.0) * _RATE_SCALE_HZ
        )
        pitch_rate = (
            cfg.gain * pitch_err * (-1.0 if cfg.invert_pitch else 1.0) * _RATE_SCALE_HZ
        )
        return (pitch_rate, yaw_rate)

    def reset(self) -> None:
        """Zero the controller's internal commanded-angle model.

        This only resets the controller's *model*; it sends no command.
        It is NOT called on a lock loss -- the aim loop persists the model
        across a loss so a re-lock resumes smoothly from the held angle.
        Kept as an explicit recenter primitive.
        """

        self.cmd_pitch = 0.0
        self.cmd_yaw = 0.0


__all__ = ["AimConfig", "GimbalAimController"]
