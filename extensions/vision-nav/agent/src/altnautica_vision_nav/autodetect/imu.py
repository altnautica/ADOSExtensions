"""IMU source selector.

The agent has two MAVLink IMU sources today, ``RAW_IMU`` and
``SCALED_IMU2``. ``RAW_IMU`` is the canonical ArduPilot output at the
FC's primary IMU rate. ``SCALED_IMU2`` is the secondary IMU and on
many builds runs at half the primary's rate. The v1 preference is
``RAW_IMU`` so this selector starts there. The interface is shaped to
support a future direct-bus IMU (BMI088 over I2C, DroneCAN) without
touching the call site.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PreferredImuSource:
    """The chosen IMU source identity.

    ``source_id`` matches the value the plugin records on the
    heartbeat's ``imuSource`` field. The plugin instantiates the
    matching :class:`BaseImuSource` subclass; this dataclass only
    carries the identity decision.
    """

    source_id: str
    reason: str


def detect_preferred_imu_source(
    has_direct_i2c: bool = False,
    has_dronecan: bool = False,
) -> PreferredImuSource:
    """Return the preferred IMU source for the current host.

    The two flags model the future direct-IMU paths. When both are
    ``False`` (the current v1 reality) the selector returns
    ``mavlink-raw-imu``. The order favours higher-rate paths when
    they are available so the future direct-IMU rollout flips the
    plugin's behaviour automatically without a config change.
    """

    if has_direct_i2c:
        return PreferredImuSource(
            source_id="direct-i2c-bmi088",
            reason="bench-IMU bus answered the chip-id probe",
        )
    if has_dronecan:
        return PreferredImuSource(
            source_id="direct-dronecan",
            reason="DroneCAN bus reports a RawIMU node",
        )
    return PreferredImuSource(
        source_id="mavlink-raw-imu",
        reason="default; FC publishes RAW_IMU on the MAVLink stream",
    )
