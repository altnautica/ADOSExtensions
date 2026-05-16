"""Direct I2C IMU subscriber for Bosch BMI088.

The MAVLink IMU path tops out at the firmware's wire rate, typically
50 to 200 Hz on a stock ArduPilot build. The two VIO engines we
support (OpenVINS, VINS-Fusion) want 400 Hz IMU samples for the
filter to stay convergent on aggressive motion. This module bypasses
the FC by reading the BMI088 directly off an I2C bus the carrier
board exposes.

The BMI088 is a split chip with separate accel and gyro dies. The two
dies live at different I2C addresses (0x18/0x19 for accel, 0x68/0x69
for gyro). The class probes both addresses at startup and refuses to
start if either die does not respond.

The polling loop reads at ~400 Hz on a background thread; the main
plugin loop reads SI samples through the same :class:`BaseImuSource`
buffer the MAVLink sources use, so the estimator does not care which
source produced the samples.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable, Optional

from altnautica_vision_nav.imu.base import BaseImuSource, ImuSample

log = logging.getLogger(__name__)

# Bosch BMI088 register map -- accel die.
BMI088_ACCEL_DEFAULT_ADDR = 0x18
BMI088_ACCEL_CHIP_ID_REG = 0x00
BMI088_ACCEL_CHIP_ID_EXPECTED = 0x1E
BMI088_ACCEL_PWR_CONF_REG = 0x7C
BMI088_ACCEL_PWR_CTRL_REG = 0x7D
BMI088_ACCEL_RANGE_REG = 0x41
BMI088_ACCEL_RANGE_24G = 0x03
BMI088_ACCEL_DATA_X_LSB_REG = 0x12

# Bosch BMI088 register map -- gyro die.
BMI088_GYRO_DEFAULT_ADDR = 0x68
BMI088_GYRO_CHIP_ID_REG = 0x00
BMI088_GYRO_CHIP_ID_EXPECTED = 0x0F
BMI088_GYRO_RANGE_REG = 0x0F
BMI088_GYRO_RANGE_2000DPS = 0x00
BMI088_GYRO_DATA_X_LSB_REG = 0x02

# Conversion constants. The accel range is +/- 24 g at 16-bit
# resolution; the gyro range is +/- 2000 dps at 16-bit resolution.
ACCEL_LSB_TO_G = 24.0 / 32768.0
G_TO_M_PER_S2 = 9.80665
GYRO_LSB_TO_DPS = 2000.0 / 32768.0
DEG_TO_RAD = 0.017453292519943295


class DirectI2cImu(BaseImuSource):
    """BMI088 IMU read directly off an I2C bus.

    Constructor parameters mirror the operator-facing config layout:
    a bus number plus optional address overrides. The class starts a
    background poller thread on :meth:`start`. The polling rate target
    is 400 Hz; the actual achieved rate is reported via
    :meth:`rate_hz` from the base class's EMA.

    The class does not depend on smbus2 at import time so the plugin
    can import the module on hosts without I2C. The ``bus_factory``
    parameter lets unit tests inject a fake bus.
    """

    source_id = "direct-i2c-bmi088"

    def __init__(
        self,
        bus_num: int,
        *,
        accel_addr: int = BMI088_ACCEL_DEFAULT_ADDR,
        gyro_addr: int = BMI088_GYRO_DEFAULT_ADDR,
        target_rate_hz: float = 400.0,
        buffer_capacity: int = 400,
        bus_factory: Optional[Callable[[int], Any]] = None,
    ) -> None:
        super().__init__(buffer_capacity=buffer_capacity)
        self._bus_num = int(bus_num)
        self._accel_addr = int(accel_addr)
        self._gyro_addr = int(gyro_addr)
        self._target_period_s = 1.0 / float(target_rate_hz)
        self._bus_factory = bus_factory
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._bus: Any = None
        # Bias offsets: applied to each sample after SI conversion. The
        # operator triggers a stationary calibration that overwrites
        # these. The defaults are zero so an uncalibrated source still
        # produces meaningful (just slightly biased) samples.
        self._gyro_bias = (0.0, 0.0, 0.0)
        self._accel_bias = (0.0, 0.0, 0.0)
        # Calibration state. Non-None while a stationary calibration
        # is in progress; the poll loop fills the buffer and the calc
        # runs at completion.
        self._cal_lock = threading.Lock()
        self._cal_samples: Optional[list[tuple[ImuSample]]] = None
        self._cal_target: int = 0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        if self._thread is not None:
            return
        bus = self._open_bus()
        self._configure(bus)
        self._bus = bus
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._poll_loop,
            name=f"DirectI2cImu-{self._bus_num}",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        thread = self._thread
        self._thread = None
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        bus = self._bus
        self._bus = None
        if bus is not None:
            close = getattr(bus, "close", None)
            if callable(close):
                try:
                    close()
                except Exception:  # noqa: BLE001 - teardown must not raise
                    log.warning(
                        "direct i2c imu bus close raised; ignoring",
                        exc_info=True,
                    )

    # ------------------------------------------------------------------
    # Operator-facing bias calibration
    # ------------------------------------------------------------------

    def start_bias_calibration(self, target_samples: int = 200) -> None:
        """Begin a stationary bias calibration.

        The operator places the drone on a level surface and triggers
        this from the GCS. The poll loop collects ``target_samples``
        and averages them; the average gyro reading becomes the gyro
        bias, and the accel reading minus gravity (z = 9.81 m/s^2)
        becomes the accel bias. Idempotent: a second call while a
        calibration is in flight is a no-op.
        """

        with self._cal_lock:
            if self._cal_samples is not None:
                return
            self._cal_samples = []
            self._cal_target = max(1, int(target_samples))

    def calibration_progress(self) -> tuple[int, int]:
        """Return ``(samples_collected, target_samples)``.

        Returns ``(0, 0)`` when no calibration is in flight so the GCS
        can render an idle state.
        """

        with self._cal_lock:
            if self._cal_samples is None:
                return (0, 0)
            return (len(self._cal_samples), self._cal_target)

    def bias(self) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        """Return the current ``(gyro_bias, accel_bias)`` tuple."""

        return self._gyro_bias, self._accel_bias

    # ------------------------------------------------------------------
    # Bus open + configure
    # ------------------------------------------------------------------

    def _open_bus(self) -> Any:
        if self._bus_factory is not None:
            return self._bus_factory(self._bus_num)
        try:
            import smbus2  # type: ignore[import-untyped]
        except ImportError as exc:
            raise RuntimeError(
                "smbus2 is required for the direct-I2C IMU path"
            ) from exc
        return smbus2.SMBus(self._bus_num)

    def _configure(self, bus: Any) -> None:
        # Verify chip IDs first; refuse to start if either die is silent.
        accel_id = bus.read_byte_data(
            self._accel_addr, BMI088_ACCEL_CHIP_ID_REG
        )
        if accel_id != BMI088_ACCEL_CHIP_ID_EXPECTED:
            raise RuntimeError(
                f"bmi088 accel chip id mismatch: expected "
                f"0x{BMI088_ACCEL_CHIP_ID_EXPECTED:02X}, got 0x{accel_id:02X}"
            )
        gyro_id = bus.read_byte_data(
            self._gyro_addr, BMI088_GYRO_CHIP_ID_REG
        )
        if gyro_id != BMI088_GYRO_CHIP_ID_EXPECTED:
            raise RuntimeError(
                f"bmi088 gyro chip id mismatch: expected "
                f"0x{BMI088_GYRO_CHIP_ID_EXPECTED:02X}, got 0x{gyro_id:02X}"
            )
        # Bring the accel out of suspend, set +/- 24 g full scale.
        bus.write_byte_data(self._accel_addr, BMI088_ACCEL_PWR_CONF_REG, 0x00)
        bus.write_byte_data(self._accel_addr, BMI088_ACCEL_PWR_CTRL_REG, 0x04)
        bus.write_byte_data(
            self._accel_addr, BMI088_ACCEL_RANGE_REG, BMI088_ACCEL_RANGE_24G
        )
        # Gyro defaults are 2000 dps already on cold boot; the explicit
        # write is belt-and-suspenders against a stale config.
        bus.write_byte_data(
            self._gyro_addr,
            BMI088_GYRO_RANGE_REG,
            BMI088_GYRO_RANGE_2000DPS,
        )

    # ------------------------------------------------------------------
    # Poll loop
    # ------------------------------------------------------------------

    def _poll_loop(self) -> None:
        bus = self._bus
        if bus is None:
            return
        # Wait on the stop event rather than sleeping unconditionally;
        # ``stop()`` can then unwind the thread immediately even if a
        # tick is mid-pause. ``wait(timeout)`` returns True when the
        # event fires before the timeout, so the loop exits cleanly.
        while not self._stop_event.is_set():
            tick_start = time.monotonic()
            try:
                sample = self._read_sample(bus)
            except Exception:  # noqa: BLE001 - one bad read must not kill the thread
                log.warning("direct i2c imu read failed", exc_info=True)
                if self._stop_event.wait(timeout=self._target_period_s):
                    return
                continue
            if sample is None:
                if self._stop_event.wait(timeout=self._target_period_s):
                    return
                continue
            self._maybe_record_calibration(sample)
            biased = self._apply_bias(sample)
            self._record(biased)
            elapsed = time.monotonic() - tick_start
            remainder = self._target_period_s - elapsed
            if remainder > 0 and self._stop_event.wait(timeout=remainder):
                return

    def _read_sample(self, bus: Any) -> Optional[ImuSample]:
        # Accel: read 6 bytes starting at DATA_X_LSB.
        accel_bytes = bus.read_i2c_block_data(
            self._accel_addr, BMI088_ACCEL_DATA_X_LSB_REG, 6
        )
        gyro_bytes = bus.read_i2c_block_data(
            self._gyro_addr, BMI088_GYRO_DATA_X_LSB_REG, 6
        )
        ax = _le16(accel_bytes, 0)
        ay = _le16(accel_bytes, 2)
        az = _le16(accel_bytes, 4)
        gx = _le16(gyro_bytes, 0)
        gy = _le16(gyro_bytes, 2)
        gz = _le16(gyro_bytes, 4)
        ts_ns = time.monotonic_ns()
        return ImuSample(
            ts_ns=ts_ns,
            xgyro=gx * GYRO_LSB_TO_DPS * DEG_TO_RAD,
            ygyro=gy * GYRO_LSB_TO_DPS * DEG_TO_RAD,
            zgyro=gz * GYRO_LSB_TO_DPS * DEG_TO_RAD,
            xacc=ax * ACCEL_LSB_TO_G * G_TO_M_PER_S2,
            yacc=ay * ACCEL_LSB_TO_G * G_TO_M_PER_S2,
            zacc=az * ACCEL_LSB_TO_G * G_TO_M_PER_S2,
        )

    # ------------------------------------------------------------------
    # Bias helpers
    # ------------------------------------------------------------------

    def _maybe_record_calibration(self, sample: ImuSample) -> None:
        with self._cal_lock:
            if self._cal_samples is None:
                return
            self._cal_samples.append((sample,))
            if len(self._cal_samples) < self._cal_target:
                return
            # Average the buffer; gravity gets subtracted from accel-z.
            samples = [s[0] for s in self._cal_samples]
            n = float(len(samples))
            gyro_b = (
                sum(s.xgyro for s in samples) / n,
                sum(s.ygyro for s in samples) / n,
                sum(s.zgyro for s in samples) / n,
            )
            accel_b = (
                sum(s.xacc for s in samples) / n,
                sum(s.yacc for s in samples) / n,
                sum(s.zacc for s in samples) / n - G_TO_M_PER_S2,
            )
            self._gyro_bias = gyro_b
            self._accel_bias = accel_b
            self._cal_samples = None
            self._cal_target = 0
            log.info(
                "direct i2c imu bias calibration finished gyro=%s accel=%s",
                gyro_b,
                accel_b,
            )

    def _apply_bias(self, sample: ImuSample) -> ImuSample:
        gx, gy, gz = self._gyro_bias
        ax, ay, az = self._accel_bias
        return ImuSample(
            ts_ns=sample.ts_ns,
            xgyro=sample.xgyro - gx,
            ygyro=sample.ygyro - gy,
            zgyro=sample.zgyro - gz,
            xacc=sample.xacc - ax,
            yacc=sample.yacc - ay,
            zacc=sample.zacc - az,
        )


def _le16(buf: list[int], offset: int) -> int:
    """Decode a little-endian signed 16-bit value from two consecutive bytes."""

    low = buf[offset]
    high = buf[offset + 1]
    raw = (high << 8) | low
    if raw & 0x8000:
        raw -= 0x10000
    return raw
