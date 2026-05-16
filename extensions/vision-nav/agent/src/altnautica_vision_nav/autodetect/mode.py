"""Suggested-mode derivation.

The plugin advertises a ``suggestedMode`` field on the heartbeat so
the GCS can default the ModeCard selection when the operator has
not picked one explicitly. The suggestion is a pure function of the
detected hardware: a camera plus a rangefinder unlocks optical flow,
a camera alone unlocks the rangefinder-free degraded variant, a
forward camera on a higher-end SBC unlocks one of the VIO modes, and
absent any camera the suggestion is ``off``.

Orientation is part of the suggestion. Operators flying over ground
(agriculture, survey, SAR, pipeline patrol) want VIO bound to the
downward camera so the ground texture dominates. Indoor and corridor
operators want VIO bound to the forward camera so depth parallax
during forward translation is rich. The detector cannot read the
operator's intent, so the suggestion exposes ``recommended_orientation``
and the wizard surfaces it as a follow-up choice.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

OrientationHint = Literal["forward", "downward", "auto"]


@dataclass(frozen=True)
class SuggestedMode:
    """The derived suggestion.

    ``mode`` is one of the six ``ESTIMATOR_REGISTRY`` keys.
    ``reason`` explains the choice in one short sentence for the
    heartbeat's diagnostic field; the GCS surfaces it on the ModeCard
    tooltip.
    ``recommended_orientation`` is the camera direction the operator
    should pick when the suggested mode is a VIO mode. It is ``"auto"``
    for optical-flow modes (which are always downward) and for ``off``.
    """

    mode: str
    reason: str
    recommended_orientation: OrientationHint = "auto"


def derive_suggested_mode(
    *,
    has_camera: bool,
    has_rangefinder: bool,
    has_forward_camera: bool = False,
    has_downward_camera: bool = False,
    has_npu_board: bool = False,
    prefers_over_ground: bool = False,
) -> SuggestedMode:
    """Pick the best default mode for the detected hardware.

    The decision tree:

    * No camera anywhere -> ``off`` (the plugin has nothing to do).
    * Forward AND downward camera + NPU board -> ``hybrid_of_plus_vio``
      (both estimators, EKF fuses).
    * Camera + rangefinder -> ``optical_flow``.
    * Forward camera + NPU + no rangefinder -> ``vio_openvins`` forward
      (corridor / indoor / inspection default).
    * Downward camera + NPU + no rangefinder + over-ground intent ->
      ``vio_vins_fusion`` downward (agriculture / survey / SAR /
      pipeline default).
    * Camera + no rangefinder + no NPU -> ``optical_flow_degraded``
      (baro/GPS scale ladder).

    ``prefers_over_ground`` is a wizard-supplied hint that biases the
    suggestion toward downward VIO when the operator's selected suite
    (agriculture, survey, SAR, inspection of horizontal infrastructure)
    favors over-ground flight. The hint is informational; the operator
    still confirms the orientation in the wizard.
    """

    if not has_camera and not has_forward_camera and not has_downward_camera:
        return SuggestedMode(
            mode="off",
            reason="No camera detected; plugin idles.",
        )

    if has_forward_camera and has_downward_camera and has_npu_board:
        return SuggestedMode(
            mode="hybrid_of_plus_vio",
            reason=(
                "Forward and downward cameras both present on an "
                "NPU-capable SBC; hybrid runs both estimators."
            ),
            recommended_orientation="auto",
        )

    if has_camera and has_rangefinder:
        return SuggestedMode(
            mode="optical_flow",
            reason="Downward camera plus a rangefinder.",
            recommended_orientation="downward",
        )

    if has_forward_camera and has_npu_board and not prefers_over_ground:
        return SuggestedMode(
            mode="vio_openvins",
            reason=(
                "Forward camera plus an NPU-capable SBC. Indoor and "
                "corridor flight."
            ),
            recommended_orientation="forward",
        )

    if has_downward_camera and has_npu_board and prefers_over_ground:
        return SuggestedMode(
            mode="vio_vins_fusion",
            reason=(
                "Downward camera plus an NPU-capable SBC. Over-ground "
                "flight where ground texture dominates."
            ),
            recommended_orientation="downward",
        )

    if has_forward_camera and has_npu_board and prefers_over_ground:
        return SuggestedMode(
            mode="vio_vins_fusion",
            reason=(
                "Forward camera plus an NPU-capable SBC. No downward "
                "camera detected; VIO will run on the forward camera "
                "but accuracy depends on scene parallax."
            ),
            recommended_orientation="forward",
        )

    if has_camera and not has_rangefinder:
        return SuggestedMode(
            mode="optical_flow_degraded",
            reason="Downward camera, no rangefinder. Scale from baro/GPS ladder.",
            recommended_orientation="downward",
        )

    return SuggestedMode(
        mode="off",
        reason="No usable hardware combination detected.",
    )
