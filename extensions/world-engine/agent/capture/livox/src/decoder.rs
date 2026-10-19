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
    pub clock_resets: u64,
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

/// Decoder for one device (one clock).
#[derive(Debug)]
pub struct Decoder {
    clock: ClockMapper,
    scans: ScanAssembler,
    stats: DecoderStats,
}

impl Decoder {
    pub fn new(cfg: DecoderConfig) -> Self {
        Self {
            clock: ClockMapper::new(cfg.clock),
            scans: ScanAssembler::new(cfg.period_ns),
            stats: DecoderStats::default(),
        }
    }

    pub fn stats(&self) -> DecoderStats {
        self.stats
    }

    pub fn clock(&self) -> &ClockMapper {
        &self.clock
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
        let before = self.clock.offset_ns();
        if self
            .clock
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
        let offset = self.clock.offset_ns().expect("observe leaves an estimate");

        if h.data_type == DataType::Imu {
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
            return Ok(());
        }
        let mut scans = Vec::new();
        self.scans.push(&pkt, offset, &mut scans);
        self.emit(scans, out);
        Ok(())
    }

    /// Close the open scan with the current clock estimate (stream end).
    pub fn flush(&mut self, out: &mut Vec<Event>) {
        if let Some(o) = self.clock.offset_ns() {
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
