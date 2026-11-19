//! `imu_noise`: Allan deviation over a long static IMU recording.
//!
//! Every `/imu/<id>` sample of a rig IMU is block-averaged to a base period of
//! about 5 ms (an Allan curve needs nothing shorter, and it keeps a two-hour
//! 1 kHz record small), then each axis gets an overlapping Allan curve and the
//! slope fits in [`super::allan`].

use std::collections::BTreeMap;

use ados_capture_contract::v1::ImuBatch;
use ados_capture_contract::{ImuNoise, Rig, RigSensorKind, SessionReader};
use serde_json::{json, Value};

use super::allan::{axes_noise, AxisNoise};
use crate::recon::StageError;

/// Shortest record accepted, seconds.
pub const MIN_DURATION_S: f64 = 20.0 * 60.0;

/// Samples used to estimate the rate before block averaging starts.
const RATE_PROBE: usize = 64;

/// Target base period after block averaging, nanoseconds.
const BASE_PERIOD_NS: f64 = 5.0e6;

/// One IMU's samples, block-averaged as they arrive.
#[derive(Debug, Default)]
pub struct ImuSeries {
    pending: Vec<[f64; 6]>,
    pending_t: Vec<u64>,
    block: usize,
    acc: [f64; 6],
    acc_n: usize,
    axes: [Vec<f64>; 6],
    first_t: Option<u64>,
    last_t: u64,
    samples: u64,
}

/// A finished series.
#[derive(Debug, Clone)]
pub struct Series {
    /// Period of one averaged value, seconds.
    pub tau0: f64,
    /// Gyro x, y, z then accel x, y, z.
    pub axes: [Vec<f64>; 6],
    pub duration_s: f64,
    pub samples: u64,
}

impl ImuSeries {
    /// Add one sample: gyro rad/s, accel m/s².
    pub fn push(&mut self, t_ns: u64, gyro: [f64; 3], accel: [f64; 3]) {
        let v = [gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]];
        self.first_t.get_or_insert(t_ns);
        self.last_t = self.last_t.max(t_ns);
        self.samples += 1;
        if self.block == 0 {
            self.pending.push(v);
            self.pending_t.push(t_ns);
            if self.pending.len() >= RATE_PROBE {
                self.start_blocks();
            }
        } else {
            self.accumulate(v);
        }
    }

    fn start_blocks(&mut self) {
        let mut dts: Vec<u64> = self
            .pending_t
            .windows(2)
            .filter_map(|w| w[1].checked_sub(w[0]).filter(|d| *d > 0))
            .collect();
        dts.sort_unstable();
        let dt = dts.get(dts.len() / 2).copied().unwrap_or(1) as f64;
        self.block = ((BASE_PERIOD_NS / dt).floor() as usize).max(1);
        for v in std::mem::take(&mut self.pending) {
            self.accumulate(v);
        }
        self.pending_t.clear();
    }

    fn accumulate(&mut self, v: [f64; 6]) {
        for (a, x) in self.acc.iter_mut().zip(v) {
            *a += x;
        }
        self.acc_n += 1;
        if self.acc_n == self.block {
            let n = self.block as f64;
            for (axis, a) in self.axes.iter_mut().zip(self.acc.iter_mut()) {
                axis.push(*a / n);
                *a = 0.0;
            }
            self.acc_n = 0;
        }
    }

    /// Close the series; `None` with fewer than two samples.
    pub fn finish(mut self) -> Option<Series> {
        if self.samples < 2 {
            return None;
        }
        if self.block == 0 {
            self.start_blocks();
        }
        let first = self.first_t?;
        let span_ns = self.last_t.saturating_sub(first) as f64;
        let mean_dt_ns = span_ns / (self.samples - 1) as f64;
        Some(Series {
            tau0: self.block as f64 * mean_dt_ns * 1e-9,
            axes: self.axes,
            duration_s: span_ns * 1e-9,
            samples: self.samples,
        })
    }
}

/// Fitted noise of one IMU.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuFit {
    pub gyro: AxisNoise,
    pub accel: AxisNoise,
    pub duration_s: f64,
}

/// Fit a finished series. `E_INPUT` below [`MIN_DURATION_S`].
pub fn fit(id: &str, s: &Series) -> Result<ImuFit, StageError> {
    if s.duration_s < MIN_DURATION_S {
        return Err(StageError::input(format!(
            "imu {id}: {:.0} s of data; the noise fit needs at least {:.0} s of static recording",
            s.duration_s, MIN_DURATION_S
        )));
    }
    let tau_max = s.duration_s / 10.0;
    let [gx, gy, gz, ax, ay, az] = &s.axes;
    let gyro = axes_noise([gx, gy, gz], s.tau0, tau_max);
    let accel = axes_noise([ax, ay, az], s.tau0, tau_max);
    match (gyro, accel) {
        (Some(gyro), Some(accel)) => Ok(ImuFit {
            gyro,
            accel,
            duration_s: s.duration_s,
        }),
        _ => Err(StageError::input(format!(
            "imu {id}: the Allan curve is empty (constant or missing samples)"
        ))),
    }
}

/// Read every rig IMU's samples, fit them, and return the rig with
/// `imu_noise` replaced plus the report body.
pub fn calibrate(
    reader: &SessionReader,
    progress: &mut dyn FnMut(f32),
) -> Result<(Rig, Value), StageError> {
    let mut rig = reader.rig().clone();
    let ids: Vec<String> = rig
        .sensors
        .iter()
        .filter(|s| s.kind == RigSensorKind::Imu)
        .map(|s| s.id.clone())
        .collect();
    if ids.is_empty() {
        return Err(StageError::input("the rig has no IMU"));
    }
    let mut series: BTreeMap<String, ImuSeries> = BTreeMap::new();
    reader.for_each_message(|m| {
        let Some(id) = m.topic.strip_prefix("/imu/") else {
            return Ok(());
        };
        if !ids.iter().any(|i| i == id) {
            return Ok(());
        }
        let batch: ImuBatch = m.decode()?;
        let s = series.entry(id.to_string()).or_default();
        for smp in batch.samples {
            if smp.gyro.len() == 3 && smp.accel.len() == 3 {
                let g = [smp.gyro[0], smp.gyro[1], smp.gyro[2]];
                let a = [smp.accel[0], smp.accel[1], smp.accel[2]];
                s.push(smp.t_ns, g, a);
            }
        }
        Ok(())
    })?;
    progress(0.4);
    let finished: Vec<(String, Series)> = series
        .into_iter()
        .filter_map(|(id, s)| s.finish().map(|f| (id, f)))
        .collect();
    if finished.is_empty() {
        return Err(StageError::input("the session holds no IMU samples"));
    }
    let mut fits: Vec<(String, ImuFit)> = Vec::new();
    for (i, (id, s)) in finished.iter().enumerate() {
        fits.push((id.clone(), fit(id, s)?));
        progress(0.4 + 0.5 * (i + 1) as f32 / finished.len() as f32);
    }
    for (id, f) in &fits {
        if let Some(sensor) = rig.sensors.iter_mut().find(|s| &s.id == id) {
            sensor.imu_noise = Some(ImuNoise {
                gyro_nd: f.gyro.nd,
                accel_nd: f.accel.nd,
                gyro_rw: f.gyro.rw,
                accel_rw: f.accel.rw,
            });
        }
    }
    // Rig order decides which IMU the top-level figures describe.
    let primary = ids
        .iter()
        .find_map(|id| fits.iter().find(|(f, _)| f == id))
        .expect("at least one fit");
    let fit_json = |id: &str, f: &ImuFit| {
        json!({
            "id": id,
            "gyro_nd": f.gyro.nd,
            "accel_nd": f.accel.nd,
            "gyro_rw": f.gyro.rw,
            "accel_rw": f.accel.rw,
            "duration_s": f.duration_s,
        })
    };
    let report = json!({
        "reprojection_rms_px": null,
        "tool": "ados-allan",
        "tool_version": env!("CARGO_PKG_VERSION"),
        "imu_id": primary.0,
        "gyro_nd": primary.1.gyro.nd,
        "accel_nd": primary.1.accel.nd,
        "gyro_rw": primary.1.gyro.rw,
        "accel_rw": primary.1.accel.rw,
        "duration_s": primary.1.duration_s,
        "imus": fits.iter().map(|(id, f)| fit_json(id, f)).collect::<Vec<_>>(),
    });
    Ok((rig, report))
}

#[cfg(test)]
mod tests {
    use super::super::allan::tests::{synth, Gauss};
    use super::*;

    /// 25 minutes at 400 Hz: block averaging to 5 ms keeps the fits.
    #[test]
    fn block_averaged_series_recovers_known_noise() {
        let dt = 0.0025;
        let n = 25 * 60 * 400;
        let mut g = Gauss::new(11);
        let (gnd, grw, and, arw) = (1.6e-4, 2.2e-5, 2.0e-3, 3.0e-3);
        let gyro: Vec<Vec<f64>> = (0..3).map(|_| synth(&mut g, n, dt, gnd, grw)).collect();
        let accel: Vec<Vec<f64>> = (0..3).map(|_| synth(&mut g, n, dt, and, arw)).collect();
        let mut s = ImuSeries::default();
        for k in 0..n {
            let t = 1_000_000_000 + k as u64 * 2_500_000;
            s.push(
                t,
                [gyro[0][k], gyro[1][k], gyro[2][k]],
                [accel[0][k], accel[1][k], accel[2][k]],
            );
        }
        let series = s.finish().unwrap();
        assert!((series.tau0 - 0.005).abs() < 1e-9);
        let f = fit("imu0", &series).unwrap();
        for (got, want) in [
            (f.gyro.nd, gnd),
            (f.gyro.rw, grw),
            (f.accel.nd, and),
            (f.accel.rw, arw),
        ] {
            assert!((got / want - 1.0).abs() < 0.1, "got {got}, want {want}");
        }
    }

    #[test]
    fn short_records_are_refused() {
        let mut s = ImuSeries::default();
        for k in 0..1000u64 {
            s.push(k * 5_000_000, [0.0; 3], [0.0, 0.0, 9.8]);
        }
        let err = fit("imu0", &s.finish().unwrap()).unwrap_err();
        assert_eq!(err.code, "E_INPUT");
    }
}
