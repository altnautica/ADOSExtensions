//! Per-device packet pipeline: parse, CRC check, clock mapping, scan assembly
//! and IMU samples.

use crate::clock::{ClockConfig, ClockMapper};
use crate::packet::{CrcStatus, DataType, Packet, PacketError, TimeType};
use crate::scan::{Scan, ScanAssembler, DEFAULT_PERIOD_NS};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DecoderConfig {
    /// Scan period on the device clock.
    pub period_ns: u64,
    pub clock: ClockConfig,
}

impl Default for DecoderConfig {
    fn default() -> Self {
        Self {
            period_ns: DEFAULT_PERIOD_NS,
            clock: ClockConfig::default(),
        }
    }
}

/// One IMU sample on the host clock, sensor frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSample {
    /// Host monotonic ns.
    pub t_ns: u64,
    pub device_t_ns: u64,
    pub time_type: TimeType,
    /// rad/s.
    pub gyro: [f32; 3],
    /// m/s².
    pub accel: [f32; 3],
}

#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    Scan(Scan),
    Imu(ImuSample),
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct DecoderStats {
    pub packets: u64,
    pub parse_errors: u64,
    pub crc_errors: u64,
    /// Packets whose `crc32` field was zero (accepted unchecked).
    pub crc_absent: u64,
    /// Restarts of the point-stream clock filter.
    pub clock_resets: u64,
    /// Restarts of the IMU-stream clock filter.
    pub imu_clock_resets: u64,
    pub scans: u64,
    pub imu_samples: u64,
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum DecodeError {
    #[error(transparent)]
    Packet(#[from] PacketError),
    #[error("crc32 mismatch: field {field:#010x}, computed {computed:#010x}")]
    Crc { field: u32, computed: u32 },
}

/// Decoder for one device.
///
/// Point and IMU packets arrive on separate UDP ports with their own
/// transport delays, so each stream has its own clock filter: an IMU packet
/// stamped earlier than the last point (or later than the next one) never
/// touches the point filter or the open scan.
#[derive(Debug)]
pub struct Decoder {
    points_clock: ClockMapper,
    imu_clock: ClockMapper,
    scans: ScanAssembler,
    stats: DecoderStats,
}

impl Decoder {
    pub fn new(cfg: DecoderConfig) -> Self {
        Self {
            points_clock: ClockMapper::new(cfg.clock),
            imu_clock: ClockMapper::new(cfg.clock),
            scans: ScanAssembler::new(cfg.period_ns),
            stats: DecoderStats::default(),
        }
    }

    pub fn stats(&self) -> DecoderStats {
        self.stats
    }

    /// The point stream's device-to-host clock.
    pub fn points_clock(&self) -> &ClockMapper {
        &self.points_clock
    }

    /// The IMU stream's device-to-host clock.
    pub fn imu_clock(&self) -> &ClockMapper {
        &self.imu_clock
    }

    /// Decode one packet received at host monotonic `host_rx_ns`, appending
    /// every completed scan and IMU sample to `out`.
    pub fn push(
        &mut self,
        buf: &[u8],
        host_rx_ns: u64,
        out: &mut Vec<Event>,
    ) -> Result<(), DecodeError> {
        let pkt = match Packet::parse(buf) {
            Ok(p) => p,
            Err(e) => {
                self.stats.parse_errors += 1;
                return Err(e.into());
            }
        };
        match pkt.crc_status() {
            CrcStatus::Valid => {}
            CrcStatus::Absent => self.stats.crc_absent += 1,
            CrcStatus::Mismatch { field, computed } => {
                self.stats.crc_errors += 1;
                return Err(DecodeError::Crc { field, computed });
            }
        }
        self.stats.packets += 1;

        let h = pkt.header;
        if h.data_type == DataType::Imu {
            self.push_imu(&pkt, host_rx_ns, out);
            return Ok(());
        }
        let before = self.points_clock.offset_ns();
        if self
            .points_clock
            .observe(h.time_type, h.last_sample_ns(), host_rx_ns)
        {
            self.stats.clock_resets += 1;
            // The open scan is on the old device timebase: close it with the
            // offset that belonged to that timebase.
            match before {
                Some(o) => self.flush_into(o, out),
                None => self.scans.clear(),
            }
        }
        let offset = self
            .points_clock
            .offset_ns()
            .expect("observe leaves an estimate");
        let mut scans = Vec::new();
        self.scans.push(&pkt, offset, &mut scans);
        self.emit(scans, out);
        Ok(())
    }

    fn push_imu(&mut self, pkt: &Packet<'_>, host_rx_ns: u64, out: &mut Vec<Event>) {
        let h = pkt.header;
        if self
            .imu_clock
            .observe(h.time_type, h.last_sample_ns(), host_rx_ns)
        {
            self.stats.imu_clock_resets += 1;
        }
        let offset = self
            .imu_clock
            .offset_ns()
            .expect("observe leaves an estimate");
        for s in pkt.imu() {
            let dev = h.timestamp_ns.saturating_add(s.offset_ns);
            self.stats.imu_samples += 1;
            out.push(Event::Imu(ImuSample {
                t_ns: (dev as i128 + offset).clamp(0, u64::MAX as i128) as u64,
                device_t_ns: dev,
                time_type: h.time_type,
                gyro: s.gyro,
                accel: s.accel,
            }));
        }
    }

    /// Close the open scan with the current clock estimate (stream end).
    pub fn flush(&mut self, out: &mut Vec<Event>) {
        if let Some(o) = self.points_clock.offset_ns() {
            self.flush_into(o, out);
        }
    }

    fn flush_into(&mut self, offset: i128, out: &mut Vec<Event>) {
        let mut scans = Vec::new();
        self.scans.flush(offset, &mut scans);
        self.emit(scans, out);
    }

    fn emit(&mut self, scans: Vec<Scan>, out: &mut Vec<Event>) {
        self.stats.scans += scans.len() as u64;
        out.extend(scans.into_iter().map(Event::Scan));
    }
}
