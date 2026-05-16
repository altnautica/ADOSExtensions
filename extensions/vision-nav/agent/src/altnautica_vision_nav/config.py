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

from pydantic import BaseModel, ConfigDict, Field


class CameraConfig(BaseModel):
    """Camera capture settings.

    The plugin opens the first source it can: when ``bus_type`` is
    ``csi`` and a libcamera binding is available, the libcamera path
    runs; otherwise the V4L2 path runs against ``device_path``.
    """

    model_config = ConfigDict(extra="ignore")

    device_path: str = Field(
        default="/dev/video0",
        pattern=r"^/dev/video[0-9]+$",
        description="V4L2 or CSI video node. Constrained so a malformed "
        "config cannot point the capture path at an arbitrary device.",
    )
    bus_type: Literal["uvc", "csi"] = "uvc"
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
    """Flight firmware identification."""

    model_config = ConfigDict(extra="ignore")

    type: Literal["ardupilot", "px4"] = "ardupilot"
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
    ] = "optical_flow"
    camera: CameraConfig = Field(default_factory=CameraConfig)
    rangefinder: RangefinderConfig = Field(default_factory=RangefinderConfig)
    firmware: FirmwareConfig = Field(default_factory=FirmwareConfig)
    pre_arm: PreArmConfig = Field(default_factory=PreArmConfig)
    flow_quality_min: int = Field(default=50, ge=0, le=255)


def load_config(raw: dict) -> VisionNavConfig:
    """Parse and validate a raw config dict.

    Raises ``pydantic.ValidationError`` on bad input. Plugin lifecycle
    converts that exception into a logged error and keeps the pipeline
    paused; the host re-invokes ``on_configure`` when a corrected
    config lands.
    """

    return VisionNavConfig.model_validate(raw or {})
