//! Replay a recorded planar scan log through the live scan matcher
//! (`ados_scan2d::LocalSlam`) and print the wall time: a fixed CPU workload
//! for calibrating a bench's CPU quota against a companion board.
//!
//! ```text
//! cargo run --release --example scan2d_replay -- <lidar0.polar.zst> [--repeat N]
//! ```
//!
//! The log is a capture session's `lidar/<sensor>.polar.zst`. Scans are fed
//! back to back as fast as the matcher takes them, with a level IMU (the
//! polar form keeps no IMU), so the time measured is matching alone.

use std::fs::File;
use std::time::Instant;

use ados_scan2d::{ImuSample, LocalSlam, Scan2dConfig, ScanInput};
use world_engine_protocol::atlas::PolarScan;

fn scan_input(s: &PolarScan) -> ScanInput {
    let points: Vec<[f32; 3]> = (0..s.range_mm.len())
        .filter_map(|k| s.body_point(k))
        .map(|[x, y_frd]| [x, -y_frd, 0.0])
        .collect();
    ScanInput {
        t_ns: s.t_ns,
        t_offset_ns: vec![0; points.len()],
        points,
    }
}

fn main() -> anyhow::Result<()> {
    let mut args = std::env::args().skip(1);
    let path = args
        .next()
        .ok_or_else(|| anyhow::anyhow!("usage: scan2d_replay <scan log .polar.zst> [--repeat N]"))?;
    let repeat = match (args.next().as_deref(), args.next()) {
        (Some("--repeat"), Some(n)) => n.parse::<usize>()?.max(1),
        (None, _) => 1,
        (Some(other), _) => anyhow::bail!("unknown argument `{other}`"),
    };
    let raw = zstd::decode_all(File::open(&path)?)?;
    let scans = PolarScan::decode_all(&raw)?;
    anyhow::ensure!(!scans.is_empty(), "{path}: no scans");
    let inputs: Vec<ScanInput> = scans.iter().map(scan_input).collect();
    let span_s = (scans.last().map_or(0, |s| s.t_ns) - scans[0].t_ns) as f64 * 1e-9;

    let mut runs = Vec::with_capacity(repeat);
    for _ in 0..repeat {
        let mut slam = LocalSlam::new(Scan2dConfig::for_sensor(0.05, 12.0, &[]));
        let start = Instant::now();
        let mut last_t = None;
        for s in &inputs {
            // A level, still IMU every 5 ms up to the scan.
            let from = last_t.unwrap_or(s.t_ns.saturating_sub(100_000_000));
            let mut t = from;
            while t <= s.t_ns {
                slam.feed_imu(&ImuSample {
                    t_ns: t,
                    gyro: [0.0; 3],
                    accel: [0.0, 0.0, 9.81],
                });
                t += 5_000_000;
            }
            last_t = Some(t);
            std::hint::black_box(slam.feed_scan(s));
        }
        runs.push(start.elapsed().as_secs_f64());
    }
    let best = runs.iter().copied().fold(f64::INFINITY, f64::min);
    let mean = runs.iter().sum::<f64>() / runs.len() as f64;
    println!(
        "{{\"scans\":{},\"log_span_s\":{:.3},\"runs\":{},\"wall_s_best\":{:.4},\"wall_s_mean\":{:.4},\"ms_per_scan\":{:.3}}}",
        inputs.len(),
        span_s,
        runs.len(),
        best,
        mean,
        best * 1e3 / inputs.len() as f64
    );
    Ok(())
}
