"""Suggested-mode derivation.

The plugin advertises a ``suggestedMode`` field on the heartbeat so
the GCS can default the ModeCard selection when the operator has
not picked one explicitly. The suggestion is a pure function of the
detected hardware: a camera plus a rangefinder unlocks optical flow,
a camera alone unlocks the rangefinder-free degraded variant, a
forward camera on a higher-end SBC unlocks one of the VIO modes, and
absent any camera the suggestion is ``off``.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SuggestedMode:
    """The derived suggestion.

    ``mode`` is one of the six ``ESTIMATOR_REGISTRY`` keys.
    ``reason`` explains the choice in one short sentence for the
    heartbeat's diagnostic field; the GCS surfaces it on the ModeCard
    tooltip.
    """

    mode: str
    reason: str


def derive_suggested_mode(
    *,
    has_camera: bool,
    has_rangefinder: bool,
    has_forward_camera: bool = False,
    has_npu_board: bool = False,
) -> SuggestedMode:
    """Pick the best default mode for the detected hardware.

    The decision tree:

    * No camera anywhere -> ``off`` (the plugin has nothing to do).
    * Forward camera plus an NPU-capable SBC -> ``vio_openvins``
      (the lighter of the two VIO engines fits low-cost NPU boards
      better; the operator can switch to ``vio_vins_fusion`` if they
      have the headroom).
    * Camera plus rangefinder -> ``optical_flow``.
    * Camera, no rangefinder -> ``optical_flow_degraded`` (uses the
      baro/GPS scale ladder; the GCS marks this state explicitly).
    """

    if not has_camera and not has_forward_camera:
        return SuggestedMode(
            mode="off",
            reason="No camera detected; plugin idles.",
        )
    if has_forward_camera and has_npu_board:
        return SuggestedMode(
            mode="vio_openvins",
            reason="Forward camera plus an NPU-capable SBC.",
        )
    if has_camera and has_rangefinder:
        return SuggestedMode(
            mode="optical_flow",
            reason="Downward camera plus a rangefinder.",
        )
    if has_camera and not has_rangefinder:
        return SuggestedMode(
            mode="optical_flow_degraded",
            reason="Downward camera, no rangefinder. Scale from baro/GPS ladder.",
        )
    return SuggestedMode(
        mode="off",
        reason="No usable hardware combination detected.",
    )
