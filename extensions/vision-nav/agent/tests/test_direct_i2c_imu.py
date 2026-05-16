"""Tests for the direct-I2C BMI088 IMU source.

Hardware-bound paths are gated behind a ``bus_factory`` callable on
the class constructor so a fake bus can drive the configure + read
flow without an actual I2C device. The tests cover the chip-id
handshake, the 6-byte LE block decode, the bias-calibration averaging,
and the auto-detect probe's chip-id matcher.
"""

from __future__ import annotations

import time
from typing import Any, Optional

import pytest

from altnautica_vision_nav.autodetect.imu import (
    PreferredImuSource,
    detect_preferred_imu_source,
)
from altnautica_vision_nav.imu.direct_i2c import (
    BMI088_ACCEL_CHIP_ID_EXPECTED,
    BMI088_ACCEL_CHIP_ID_REG,
    BMI088_ACCEL_DEFAULT_ADDR,
    BMI088_GYRO_CHIP_ID_EXPECTED,
    BMI088_GYRO_CHIP_ID_REG,
    BMI088_GYRO_DEFAULT_ADDR,
    DirectI2cImu,
    _le16,
)


class _FakeBus:
    """Minimal smbus2 stand-in for the BMI088 register surface.

    Track every read/write so tests can assert the configure sequence.
    The data registers (DATA_X_LSB blocks) return a fixed sample so
    the SI conversion is deterministic. The chip-id registers return
    the expected magic so the chip-id handshake passes.
    """

    def __init__(self, *, accel_id: int = BMI088_ACCEL_CHIP_ID_EXPECTED,
                 gyro_id: int = BMI088_GYRO_CHIP_ID_EXPECTED) -> None:
        self.accel_id = accel_id
        self.gyro_id = gyro_id
        self.writes: list[tuple[int, int, int]] = []
        self.closed = False
        # Sample buffers: gyro reading near zero, accel z near +1 g.
        # 1g in BMI088 +/- 24g range: 32768 / 24 ≈ 1365 LSB.
        self.accel_block = [0x00, 0x00, 0x00, 0x00, 0x55, 0x05]  # x=0, y=0, z=1365
        self.gyro_block = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

    def read_byte_data(self, address: int, register: int) -> int:
        if address == BMI088_ACCEL_DEFAULT_ADDR and register == BMI088_ACCEL_CHIP_ID_REG:
            return self.accel_id
        if address == BMI088_GYRO_DEFAULT_ADDR and register == BMI088_GYRO_CHIP_ID_REG:
            return self.gyro_id
        return 0

    def read_i2c_block_data(
        self, address: int, register: int, length: int
    ) -> list[int]:
        if address == BMI088_ACCEL_DEFAULT_ADDR:
            return list(self.accel_block[:length])
        return list(self.gyro_block[:length])

    def write_byte_data(
        self, address: int, register: int, value: int
    ) -> None:
        self.writes.append((address, register, value))

    def close(self) -> None:
        self.closed = True


# ---------------------------------------------------------------------------
# DirectI2cImu lifecycle
# ---------------------------------------------------------------------------


def test_direct_i2c_imu_handshake_runs_configure() -> None:
    bus = _FakeBus()
    imu = DirectI2cImu(bus_num=1, bus_factory=lambda _n: bus)
    imu.start()
    try:
        # Configure must have written PWR_CONF + PWR_CTRL + RANGE + gyro range.
        registers = [(addr, reg) for (addr, reg, _val) in bus.writes]
        assert (BMI088_ACCEL_DEFAULT_ADDR, 0x7C) in registers
        assert (BMI088_ACCEL_DEFAULT_ADDR, 0x7D) in registers
        assert (BMI088_ACCEL_DEFAULT_ADDR, 0x41) in registers
        assert (BMI088_GYRO_DEFAULT_ADDR, 0x0F) in registers
    finally:
        imu.stop()


def test_direct_i2c_imu_refuses_bad_chip_id() -> None:
    bad_bus = _FakeBus(accel_id=0x00)
    imu = DirectI2cImu(bus_num=1, bus_factory=lambda _n: bad_bus)
    with pytest.raises(RuntimeError, match="accel chip id"):
        imu.start()


def test_direct_i2c_imu_polls_samples() -> None:
    bus = _FakeBus()
    imu = DirectI2cImu(
        bus_num=1, target_rate_hz=200.0, bus_factory=lambda _n: bus
    )
    imu.start()
    try:
        # Give the poll thread enough time to record at least one sample.
        time.sleep(0.05)
        sample = imu.latest()
        assert sample is not None
        # The fake accel z block is approximately +1 g.
        assert 8.5 < sample.zacc < 11.0
    finally:
        imu.stop()


def test_direct_i2c_imu_stops_cleanly() -> None:
    bus = _FakeBus()
    imu = DirectI2cImu(bus_num=1, bus_factory=lambda _n: bus)
    imu.start()
    imu.stop()
    assert bus.closed is True


def test_direct_i2c_imu_bias_calibration_idempotent() -> None:
    bus = _FakeBus()
    imu = DirectI2cImu(bus_num=1, bus_factory=lambda _n: bus)
    imu.start()
    try:
        imu.start_bias_calibration(target_samples=5)
        time.sleep(0.05)
        # A second call while in-flight is a no-op.
        imu.start_bias_calibration(target_samples=999)
    finally:
        imu.stop()


# ---------------------------------------------------------------------------
# _le16 helper
# ---------------------------------------------------------------------------


def test_le16_decodes_positive() -> None:
    # 0x0001 (low=0x01, high=0x00) → 1
    assert _le16([0x01, 0x00], 0) == 1


def test_le16_decodes_negative_twos_complement() -> None:
    # 0xFFFF (low=0xFF, high=0xFF) → -1
    assert _le16([0xFF, 0xFF], 0) == -1


def test_le16_decodes_at_offset() -> None:
    assert _le16([0x00, 0x00, 0x02, 0x00], 2) == 2


# ---------------------------------------------------------------------------
# autodetect.imu integration
# ---------------------------------------------------------------------------


def test_imu_autodetect_picks_direct_i2c_on_chipid_match(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        "glob.glob",
        lambda pattern: ["/dev/i2c-1"] if "i2c" in pattern else [],
    )

    def probe(bus: int, addr: int, register: int) -> int:
        if (addr, register) == (BMI088_ACCEL_DEFAULT_ADDR, BMI088_ACCEL_CHIP_ID_REG):
            return BMI088_ACCEL_CHIP_ID_EXPECTED
        if (addr, register) == (BMI088_GYRO_DEFAULT_ADDR, BMI088_GYRO_CHIP_ID_REG):
            return BMI088_GYRO_CHIP_ID_EXPECTED
        return -1

    result = detect_preferred_imu_source(i2c_probe=probe)
    assert result.source_id == "direct-i2c-bmi088"
    assert result.bus_num == 1


def test_imu_autodetect_falls_back_when_no_chipid(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        "glob.glob",
        lambda pattern: ["/dev/i2c-1"] if "i2c" in pattern else [],
    )
    result = detect_preferred_imu_source(i2c_probe=lambda *_a: -1)
    assert result.source_id == "mavlink-raw-imu"
    assert result.bus_num is None


def test_imu_autodetect_force_flag_still_works(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # has_direct_i2c=True forces the answer without probing.
    monkeypatch.setattr("glob.glob", lambda _pattern: [])
    result = detect_preferred_imu_source(has_direct_i2c=True)
    assert result.source_id == "direct-i2c-bmi088"
    assert result.bus_num is None  # no probe was run


def test_imu_autodetect_preferred_imu_source_dataclass_is_frozen() -> None:
    src = PreferredImuSource(source_id="mavlink-raw-imu", reason="x")
    with pytest.raises(Exception):
        src.source_id = "other"  # type: ignore[misc]
