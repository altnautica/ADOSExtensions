"""Rangefinder probe.

Scans the I2C buses for the two common companion-side rangefinders
(LIDAR-Lite v3 at 0x62, VL53L1X at 0x29) and the standard UART node
for the TF-Luna. If none of those answer the plugin falls back to
the FC-relayed ``DISTANCE_SENSOR`` MAVLink message (no probe
necessary; the relay topology is config-only).

The probe stays cheap and side-effect-free in the absence of the
hardware: a missing bus node returns "not present" without raising,
and the I2C handshake uses a single quick-read to confirm the chip
responds. Hardware presence does not implicitly start the driver;
the plugin's existing :class:`RangefinderDriver` constructors do that.
"""

from __future__ import annotations

import glob
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

_LOG = logging.getLogger(__name__)


@dataclass(frozen=True)
class DetectedRangefinder:
    """One detected rangefinder.

    ``driver`` matches the plugin config's driver name so the plugin
    can pass it straight into the driver factory. ``bus`` is the
    underlying device path or address for diagnostics.
    """

    driver: str
    topology: str  # "companion" | "fc" | "none"
    bus: str


I2C_BUS_GLOB = "/dev/i2c-*"
UART_NODE_GLOBS = ("/dev/ttyUSB*", "/dev/ttyAMA*", "/dev/ttyACM*")
LIDAR_LITE_ADDR = 0x62
VL53L1X_ADDR = 0x29


def detect_rangefinder(
    i2c_probe: Callable[[int, int], bool] | None = None,
    uart_probe: Callable[[str], bool] | None = None,
    i2c_glob: str = I2C_BUS_GLOB,
    uart_globs: tuple[str, ...] = UART_NODE_GLOBS,
) -> DetectedRangefinder | None:
    """Probe the host for a companion-side rangefinder.

    Returns the first match in priority order: LIDAR-Lite, VL53L1X,
    TF-Luna over UART. Returns ``None`` when no companion device is
    detected; the caller falls back to FC relay (which does not need
    a probe) or to a baro/GPS scale ladder.

    The two callbacks let tests inject fake probes. The default
    implementations attempt a bare-minimum read via :mod:`smbus2` and
    :mod:`serial`; when either library is absent the probe returns
    ``False`` for that bus type so the absence does not raise.
    """

    if i2c_probe is None:
        i2c_probe = _default_i2c_probe
    if uart_probe is None:
        uart_probe = _default_uart_probe

    i2c_nodes = sorted(glob.glob(i2c_glob))
    for node in i2c_nodes:
        bus_num = _bus_number(node)
        if bus_num is None:
            continue
        if i2c_probe(bus_num, LIDAR_LITE_ADDR):
            return DetectedRangefinder(
                driver="garmin_lidarlite_i2c",
                topology="companion",
                bus=node,
            )
        if i2c_probe(bus_num, VL53L1X_ADDR):
            return DetectedRangefinder(
                driver="vl53l1x_i2c",
                topology="companion",
                bus=node,
            )

    for pattern in uart_globs:
        for node in sorted(glob.glob(pattern)):
            if uart_probe(node):
                return DetectedRangefinder(
                    driver="tfluna_uart",
                    topology="companion",
                    bus=node,
                )
    return None


def _bus_number(node: str) -> int | None:
    suffix = node.rsplit("/", 1)[-1]
    tail = suffix.rsplit("-", 1)[-1]
    try:
        return int(tail)
    except ValueError:
        return None


def _default_i2c_probe(bus_num: int, address: int) -> bool:
    try:
        import smbus2  # type: ignore[import-untyped]
    except ImportError:
        return False
    try:
        bus = smbus2.SMBus(bus_num)
    except (OSError, PermissionError):
        return False
    try:
        bus.read_byte(address)
        return True
    except OSError:
        return False
    finally:
        try:
            bus.close()
        except OSError:
            pass


def _default_uart_probe(device: str) -> bool:
    try:
        import serial  # type: ignore[import-untyped]
    except ImportError:
        return False
    try:
        port = serial.Serial(device, baudrate=115200, timeout=0.1)
    except (OSError, serial.SerialException):  # type: ignore[attr-defined]
        return False
    try:
        port.write(b"\x5a\x05\x00\x01\x60")  # TF-Luna trigger
        data = port.read(9)
        return len(data) == 9 and data[0:2] == b"\x59\x59"
    except (OSError, serial.SerialException):  # type: ignore[attr-defined]
        return False
    finally:
        try:
            port.close()
        except OSError:
            pass
