"""Camera intrinsics loader.

Accepts the ``cam0`` block of a Kalibr ``camchain.yaml`` file. v1
supports the monocular pinhole projection only — covers UVC sensors,
Pi HQ camera, IMX cameras with stock lenses, and any de-fished feed.
The other Kalibr projection models (``omni``, ``ds``, ``eucm``) are
deferred; the loader rejects them with a clear "unsupported in v1"
message rather than silently mis-projecting.

The schema mirrors the Kalibr wiki:

* ``camera_model``: ``pinhole``
* ``intrinsics``: ``[fx, fy, cx, cy]`` in pixels
* ``distortion_model``: ``radtan`` (Brown-Conrady), ``equidistant``
  (Kannala-Brandt), or ``none``
* ``distortion_coeffs``: 4 floats for radtan / equidistant, 0 for none
* ``resolution``: ``[width, height]`` in pixels
* ``rostopic``: accepted and ignored
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Literal, Optional

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

CameraModel = Literal["pinhole"]
DistortionModel = Literal["radtan", "equidistant", "none"]


class CameraIntrinsicsError(ValueError):
    """Raised when an intrinsics file fails to parse or validate."""


# A few sanity bounds. The wiki examples never exceed |k| ≈ 0.5 for a
# real lens; coefficients beyond ±2 are almost certainly a bad
# calibration. We refuse to load those rather than silently project
# with nonsense.
RADTAN_COEFF_LIMIT = 2.0
TIMESHIFT_WARN_LIMIT_S = 0.2  # surfaced as a warning, not an error
TIMESHIFT_ERROR_LIMIT_S = 0.5


class CameraIntrinsics(BaseModel):
    """Kalibr-compatible ``cam0`` intrinsics block (v1 pinhole-only)."""

    model_config = ConfigDict(extra="ignore")

    camera_model: CameraModel
    intrinsics: list[float] = Field(min_length=4, max_length=4)
    distortion_model: DistortionModel
    distortion_coeffs: list[float] = Field(default_factory=list)
    resolution: list[int] = Field(min_length=2, max_length=2)
    rostopic: Optional[str] = None

    @property
    def fx(self) -> float:
        return self.intrinsics[0]

    @property
    def fy(self) -> float:
        return self.intrinsics[1]

    @property
    def cx(self) -> float:
        return self.intrinsics[2]

    @property
    def cy(self) -> float:
        return self.intrinsics[3]

    @property
    def width(self) -> int:
        return self.resolution[0]

    @property
    def height(self) -> int:
        return self.resolution[1]

    @property
    def has_suspicious_focal_length(self) -> bool:
        """Heuristic: f / width < 0.6 on a non-fisheye distortion looks fishy.

        Surfaced as a warning, never an error. The operator may have
        deliberately loaded pinhole intrinsics for a pre-rectified
        fisheye feed.
        """

        if self.distortion_model == "equidistant":
            return False
        return (self.fx + self.fy) / 2.0 < 0.6 * float(self.width)

    @model_validator(mode="after")
    def _validate(self) -> "CameraIntrinsics":
        if self.fx <= 0 or self.fy <= 0:
            raise CameraIntrinsicsError(
                f"fx and fy must be positive, got fx={self.fx}, fy={self.fy}"
            )
        if not (0 <= self.cx <= self.width):
            raise CameraIntrinsicsError(
                f"cx={self.cx} outside frame width [0, {self.width}]"
            )
        if not (0 <= self.cy <= self.height):
            raise CameraIntrinsicsError(
                f"cy={self.cy} outside frame height [0, {self.height}]"
            )
        if self.width <= 0 or self.height <= 0:
            raise CameraIntrinsicsError(
                f"resolution must be positive, got [{self.width}, {self.height}]"
            )

        expected_n = {"radtan": 4, "equidistant": 4, "none": 0}[
            self.distortion_model
        ]
        if len(self.distortion_coeffs) != expected_n:
            raise CameraIntrinsicsError(
                f"{self.distortion_model} expects {expected_n} coeffs, "
                f"got {len(self.distortion_coeffs)}"
            )

        if self.distortion_model == "radtan":
            k1, k2 = self.distortion_coeffs[0], self.distortion_coeffs[1]
            if abs(k1) > RADTAN_COEFF_LIMIT or abs(k2) > RADTAN_COEFF_LIMIT:
                raise CameraIntrinsicsError(
                    f"radtan k1/k2 out of plausible range: k1={k1}, k2={k2}"
                )
        return self


def load_intrinsics(path: str | Path) -> CameraIntrinsics:
    """Load and validate a Kalibr ``camchain.yaml`` file.

    Accepts both the top-level ``cam0`` block style (``{cam0: {...}}``)
    and the bare-block style (just the inner dict). Multi-camera files
    (``cam1``, ``cam2``, ...) are accepted; only ``cam0`` is read and
    the rest are silently ignored so a stereo calibration file is not
    forced through hand-editing for a monocular pipeline.
    """

    path = Path(path)
    if not path.exists():
        raise CameraIntrinsicsError(f"calibration file not found: {path}")

    try:
        text = path.read_text(encoding="utf-8")
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise CameraIntrinsicsError(f"YAML parse failed: {exc}") from exc

    block = _select_cam_block(data)
    if not isinstance(block, dict):
        raise CameraIntrinsicsError(
            "expected a mapping at the top level (or under 'cam0')"
        )

    try:
        return CameraIntrinsics.model_validate(block)
    except CameraIntrinsicsError:
        raise
    except Exception as exc:  # noqa: BLE001 - Pydantic raises a different type
        raise CameraIntrinsicsError(str(exc)) from exc


def _select_cam_block(data: Any) -> Any:
    """Pick the right block from a Kalibr file regardless of style.

    Three shapes accepted:

    1. ``{cam0: {...}}`` — canonical Kalibr camchain layout
    2. ``{camera_model: ..., intrinsics: [...], ...}`` — bare inner block
    3. ``None`` / empty — surfaces a clear error
    """

    if data is None:
        raise CameraIntrinsicsError("calibration file is empty")
    if isinstance(data, dict) and "cam0" in data:
        return data["cam0"]
    return data
