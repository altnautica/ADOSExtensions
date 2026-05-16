"""Camera enumeration.

Lists every ``/dev/video*`` node on the host and tags each with a
best-guess camera type (UVC, CSI, or unknown). The plugin uses the
result to pick a default ``config.camera.device_path`` when the
operator has not set one, and to populate the heartbeat's
``recommendedCameraId`` + ``detectedCameraCount`` fields.

The probe is intentionally cheap: a ``glob`` + a stat. No V4L2 ioctls,
no libcamera handshake. The detailed capabilities the plugin needs
later (resolution, codec) come from the live capture handshake, not
from this enumeration pass. Tests mock ``glob`` to drive the
selection logic in isolation.
"""

from __future__ import annotations

import glob
import logging
from dataclasses import dataclass
from pathlib import Path

_LOG = logging.getLogger(__name__)


@dataclass(frozen=True)
class DetectedCamera:
    """One camera node the host exposes.

    ``device_path`` is the absolute path the V4L2 layer talks to.
    ``kind`` is a coarse hint (``"uvc"``, ``"csi"``, ``"unknown"``).
    """

    device_path: str
    kind: str


def detect_cameras(
    video_glob: str = "/dev/video*",
    csi_paths: tuple[str, ...] = ("/dev/media0", "/dev/media1"),
) -> list[DetectedCamera]:
    """Enumerate every video device the host exposes.

    The result is sorted by device path so ``/dev/video0`` comes
    before ``/dev/video1`` deterministically.
    """

    nodes = sorted(glob.glob(video_glob))
    csi_present = any(Path(p).exists() for p in csi_paths)
    cameras: list[DetectedCamera] = []
    for node in nodes:
        # Heuristic: the first node when a CSI media controller is
        # also present is the CSI camera; subsequent nodes are usually
        # UVC. This matches the way Raspberry Pi BSPs lay things out.
        kind = "csi" if (csi_present and node.endswith("0")) else "uvc"
        cameras.append(DetectedCamera(device_path=node, kind=kind))
    return cameras


def pick_camera_for_mode(
    cameras: list[DetectedCamera], mode: str
) -> DetectedCamera | None:
    """Return the best camera for a given estimator mode.

    Heuristics:
    * Optical flow modes prefer the first UVC node (typically the
      downward-facing USB camera).
    * VIO modes prefer the first CSI node (typically the
      forward-facing global-shutter camera). Falls back to the first
      UVC node if no CSI is present.
    * Hybrid modes return the first camera and let the operator wire
      the second by config (we cannot guess two roles from one probe).
    * Off mode returns ``None``.
    """

    if not cameras:
        return None
    if mode == "off":
        return None
    if mode.startswith("vio_") or mode == "hybrid_of_plus_vio":
        csi = next((c for c in cameras if c.kind == "csi"), None)
        if csi is not None:
            return csi
    return cameras[0]
