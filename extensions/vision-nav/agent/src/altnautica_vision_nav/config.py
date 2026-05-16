"""Per-drone configuration models for the vision-nav plugin.

Mirrors the JSON Schema at ``config-schema.json`` at the extension
root. The schema is the contract the GCS half consumes; this module
is the runtime validator the agent half uses when the host hands it
a config dict via ``on_configure``.

Pydantic v2 is intentionally chosen over a hand-rolled validator
because the upstream agent already depends on it and because
``ValidationError`` carries enough field detail to surface bad
configs as actionable log messages without bespoke error code.
"""

from __future__ import annotations

from typing import Literal, Optional

from pydantic import BaseModel, ConfigDict, Field, model_validator


CameraOrientation = Literal["forward", "downward", "side", "auto"]


class CameraConfig(BaseModel):
    """Camera capture settings.

    The plugin opens the first source it can: when ``bus_type`` is
    ``csi`` and a libcamera binding is available, the libcamera path
    runs; otherwise the V4L2 path runs against ``device_path``.

    ``orientation`` records which way the lens points. Downward is the
    default for over-ground flight where ground texture dominates;
    forward is the default for indoor or corridor flight. ``auto``
    defers to the bound HAL camera role and is only safe when the
    board profile declares the orientation; the wizard refuses to
    finalize an ``auto`` choice on boards without unambiguous metadata.
    """

    model_config = ConfigDict(extra="ignore")

    device_path: str = Field(
        default="/dev/video0",
        pattern=r"^/dev/video[0-9]+$",
        description="V4L2 or CSI video node. Constrained so a malformed "
        "config cannot point the capture path at an arbitrary device.",
    )
    bus_type: Literal["uvc", "csi"] = "uvc"
    orientation: CameraOrientation = "auto"
    width: int = Field(default=640, ge=64, le=4096)
    height: int = Field(default=480, ge=64, le=4096)
    fps: int = Field(default=30, ge=1, le=240)


class RangefinderConfig(BaseModel):
    """Distance source used to scale optical flow into metric velocity.

    ``topology = companion`` means the plugin owns a sensor it reads
    directly. ``topology = fc`` means the flight controller already
    publishes ``DISTANCE_SENSOR`` and the plugin relays that stream.
    ``topology = none`` disables the metric velocity branch.
    """

    model_config = ConfigDict(extra="ignore")

    topology: Literal["companion", "fc", "none"] = "fc"
    driver: Literal[
        "tfluna_uart",
        "garmin_lidarlite_i2c",
        "vl53l1x_i2c",
        "fc_relay",
    ] = "fc_relay"
    device: Optional[str] = Field(
        default=None,
        pattern=r"^/dev/(ttyS|ttyUSB|ttyAMA|ttyACM|serial)[0-9]+$|"
        r"^[0-9]+$|^/dev/i2c-[0-9]+$",
        description="UART device path for serial drivers or an I2C bus "
        "identifier (digit or /dev/i2c-N path) for I2C drivers. "
        "Constrained so an operator-supplied config cannot open an "
        "unrelated file as a tty or SMBus.",
    )
    baud: Optional[int] = Field(default=None, ge=1200, le=1_000_000)


class FirmwareConfig(BaseModel):
    """Flight firmware identification.

    Optical flow is supported on ArduPilot, PX4, and iNav (7.0+). VIO
    modes are supported on ArduPilot and PX4; on iNav they are
    experimental and the plugin disables them at install time with a
    clear reason. Betaflight is intentionally absent: the firmware has
    no position estimator, so optical-flow injection has no consumer.
    Operators on Betaflight hardware who need GPS-denied flight should
    cross-flash iNav or ArduPilot Copter on the same FC.
    """

    model_config = ConfigDict(extra="ignore")

    type: Literal["ardupilot", "px4", "inav"] = "ardupilot"
    ekf_source_set_index: Optional[int] = Field(default=None, ge=1, le=3)


class PreArmConfig(BaseModel):
    """Optional pre-arm helper that pushes a synthetic EKF origin.

    Used when the firmware refuses to arm without a global origin and
    the operator is flying purely on optical flow.
    """

    model_config = ConfigDict(extra="ignore")

    auto_set_origin: bool = False
    origin_lat: float = Field(default=0.0, ge=-90.0, le=90.0)
    origin_lon: float = Field(default=0.0, ge=-180.0, le=180.0)
    origin_alt_m: float = 0.0


class VisionNavConfig(BaseModel):
    """Top-level plugin configuration.

    ``flow_quality_min`` is the gate the pipeline applies before
    emitting MAVLink frames. Quality is the integer output of the
    optical-flow processor (0..255).
    """

    model_config = ConfigDict(extra="ignore")

    mode: Literal[
        "off",
        "optical_flow",
        "optical_flow_degraded",
        "vio_openvins",
        "vio_vins_fusion",
        "hybrid_of_plus_vio",
    ] = "optical_flow"
    camera: CameraConfig = Field(default_factory=CameraConfig)
    secondary_camera: Optional[CameraConfig] = Field(
        default=None,
        description="Only used by hybrid_of_plus_vio. The primary "
        "camera carries the downward optical-flow stream; the "
        "secondary carries the forward VIO stream. Both orientations "
        "must be set explicitly when this field is populated.",
    )
    rangefinder: RangefinderConfig = Field(default_factory=RangefinderConfig)
    firmware: FirmwareConfig = Field(default_factory=FirmwareConfig)
    pre_arm: PreArmConfig = Field(default_factory=PreArmConfig)
    flow_quality_min: int = Field(default=50, ge=0, le=255)

    @model_validator(mode="after")
    def _validate_mode_and_cameras(self) -> "VisionNavConfig":
        # iNav rejection runs first because it is a fundamental
        # firmware-capability mismatch; further mode-specific checks
        # would produce confusing secondary errors on top.
        if self.firmware.type == "inav" and self.mode in {
            "vio_openvins",
            "vio_vins_fusion",
            "hybrid_of_plus_vio",
        }:
            raise ValueError(
                "VIO modes are not supported on iNav in this release. "
                "Use mode='optical_flow' with a downward camera + "
                "rangefinder, or cross-flash ArduPilot Copter or PX4 "
                "for VIO."
            )

        # Hybrid requires both cameras with explicit, opposed orientations.
        if self.mode == "hybrid_of_plus_vio":
            if self.secondary_camera is None:
                raise ValueError(
                    "hybrid_of_plus_vio requires both camera and "
                    "secondary_camera; the primary holds the downward "
                    "optical-flow stream and the secondary holds the "
                    "forward VIO stream."
                )
            orientations = {
                self.camera.orientation,
                self.secondary_camera.orientation,
            }
            if orientations != {"forward", "downward"}:
                raise ValueError(
                    "hybrid_of_plus_vio requires one camera with "
                    "orientation='downward' and one with "
                    "orientation='forward'; got "
                    f"{sorted(orientations)}."
                )
            if self.camera.device_path == self.secondary_camera.device_path:
                raise ValueError(
                    "camera and secondary_camera must point at distinct "
                    "device_path values."
                )

        # Optical-flow modes require a downward-facing camera. ``auto``
        # is allowed so the wizard can defer to HAL board metadata;
        # explicit ``forward`` or ``side`` is rejected.
        if self.mode in {"optical_flow", "optical_flow_degraded"}:
            if self.camera.orientation in {"forward", "side"}:
                raise ValueError(
                    f"Mode {self.mode!r} needs a downward-facing "
                    "camera; got orientation="
                    f"{self.camera.orientation!r}."
                )

        return self


def load_config(raw: dict) -> VisionNavConfig:
    """Parse and validate a raw config dict.

    Raises ``pydantic.ValidationError`` on bad input. Plugin lifecycle
    converts that exception into a logged error and keeps the pipeline
    paused; the host re-invokes ``on_configure`` when a corrected
    config lands.
    """

    return VisionNavConfig.model_validate(raw or {})
