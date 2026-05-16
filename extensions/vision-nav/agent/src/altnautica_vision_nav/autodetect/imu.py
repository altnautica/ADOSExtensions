"""IMU source selector.

The agent's IMU sources, in preference order:

1. **Direct BMI088 over I2C** at ~400 Hz. Bypasses the FC; the highest
   sample rate path the plugin supports today.
2. **DroneCAN RawIMU** at the bus's native rate (typically 400 Hz on
   a healthy bus). Currently a placeholder; the pyuavcan dependency
   is not yet vendored.
3. **MAVLink RAW_IMU** at the FC's stream rate (50-200 Hz). Universal
   fallback that works on every supported FC.

The selector probes the I2C buses for a BMI088 chip-id handshake and
returns the highest-rate source that answers. The plugin instantiates
the matching :class:`BaseImuSource` subclass.
"""

from __future__ import annotations

import glob
from dataclasses import dataclass
from typing import Callable, Optional

from altnautica_vision_nav.imu.direct_i2c import (
    BMI088_ACCEL_CHIP_ID_EXPECTED,
    BMI088_ACCEL_CHIP_ID_REG,
    BMI088_ACCEL_DEFAULT_ADDR,
    BMI088_GYRO_CHIP_ID_EXPECTED,
    BMI088_GYRO_CHIP_ID_REG,
    BMI088_GYRO_DEFAULT_ADDR,
)

I2C_BUS_GLOB = "/dev/i2c-*"


@dataclass(frozen=True)
class PreferredImuSource:
    """The chosen IMU source identity.

    ``source_id`` matches the value the plugin records on the
    heartbeat's ``imuSource`` field. ``bus_num`` is non-None when the
    chosen source is a direct-I2C path; the plugin instantiates the
    matching driver against that bus.
    """

    source_id: str
    reason: str
    bus_num: Optional[int] = None


def detect_preferred_imu_source(
    has_direct_i2c: Optional[bool] = None,
    has_dronecan: bool = False,
    i2c_probe: Optional[Callable[[int, int, int], int]] = None,
    i2c_glob: str = I2C_BUS_GLOB,
) -> PreferredImuSource:
    """Pick the preferred IMU source for the current host.

    The selector probes I2C buses for a BMI088 chip-id match. The
    ``has_direct_i2c`` arg forces the result when set (used by tests
    that already know the answer). The ``i2c_probe`` callable lets
    tests inject a fake reader; the default uses smbus2.

    Returns the highest-rate source that answered.
    """

    if has_direct_i2c is True:
        return PreferredImuSource(
            source_id="direct-i2c-bmi088",
            reason="bench-IMU bus answered the chip-id probe",
        )

    if has_direct_i2c is None:
        bus_num = _probe_bmi088(
            probe=i2c_probe or _default_chip_id_probe,
            i2c_glob=i2c_glob,
        )
        if bus_num is not None:
            return PreferredImuSource(
                source_id="direct-i2c-bmi088",
                reason=f"BMI088 chip-id matched on /dev/i2c-{bus_num}",
                bus_num=bus_num,
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


def _probe_bmi088(
    *,
    probe: Callable[[int, int, int], int],
    i2c_glob: str,
) -> Optional[int]:
    """Scan every I2C bus for the BMI088 accel + gyro chip IDs."""

    for node in sorted(glob.glob(i2c_glob)):
        bus_num = _bus_number(node)
        if bus_num is None:
            continue
        try:
            accel_id = probe(bus_num, BMI088_ACCEL_DEFAULT_ADDR, BMI088_ACCEL_CHIP_ID_REG)
            if accel_id != BMI088_ACCEL_CHIP_ID_EXPECTED:
                continue
            gyro_id = probe(bus_num, BMI088_GYRO_DEFAULT_ADDR, BMI088_GYRO_CHIP_ID_REG)
            if gyro_id != BMI088_GYRO_CHIP_ID_EXPECTED:
                continue
            return bus_num
        except Exception:  # noqa: BLE001 - probe must not raise
            continue
    return None


def _bus_number(node: str) -> Optional[int]:
    tail = node.rsplit("-", 1)[-1]
    try:
        return int(tail)
    except ValueError:
        return None


def _default_chip_id_probe(bus_num: int, address: int, register: int) -> int:
    """Read one register byte via smbus2.

    Returns ``-1`` on any failure so the caller treats it as a non-match.
    """

    try:
        import smbus2  # type: ignore[import-untyped]
    except ImportError:
        return -1
    bus = None
    try:
        bus = smbus2.SMBus(bus_num)
        return int(bus.read_byte_data(address, register))
    except Exception:  # noqa: BLE001
        return -1
    finally:
        if bus is not None:
            try:
                bus.close()
            except Exception:  # noqa: BLE001
                pass
