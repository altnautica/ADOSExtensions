//! Replay a recorded planar scan stream through the companion's offline
//! planar-sweep reconstruction and write its products for benchmarking.
//!
//! ```text
//! planar_sweep_replay <lidar.polar.zst> --out <dir>
//! ```
//!
//! Decodes the `PolarScan` records, re-poses them with
//! `recon::planar_sweep::close_loops`, registers the trusted scans with
//! `recon::planar_sweep::sweep`, and writes into `<dir>`:
//!
//! - `map.ply`: binary little-endian float xyz, the swept cloud in `map_ned`;
//! - `poses.tum`: `t x y z qx qy qz qw` per scan after loop closure, `map_ned`
//!   (lost scans skipped, as the sweep skips them);
//! - `stats.json`: `{scans, points, reposed}`.

use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use world_engine_node::recon::planar_sweep::{close_loops, sweep};
use world_engine_protocol::atlas::{PolarScan, VioHealth};

/// The sweep's z-up world (`x`, `-y`, `-z`) back to `map_ned`.
fn flu_to_ned(p: [f64; 3]) -> [f32; 3] {
    [p[0] as f32, -p[1] as f32, -p[2] as f32]
}

fn read_scans(path: &Path) -> Result<Vec<PolarScan>, String> {
    let file = File::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let raw = zstd::decode_all(file).map_err(|e| format!("{}: {e}", path.display()))?;
    PolarScan::decode_all(&raw).map_err(|e| format!("{}: {e}", path.display()))
}

fn write_ply(path: &Path, points: &[[f32; 3]]) -> std::io::Result<()> {
    let mut w = BufWriter::new(File::create(path)?);
    write!(
        w,
        "ply\nformat binary_little_endian 1.0\nelement vertex {}\n\
         property float x\nproperty float y\nproperty float z\nend_header\n",
        points.len()
    )?;
    for p in points {
        for v in p {
            w.write_all(&v.to_le_bytes())?;
        }
    }
    w.flush()
}

fn write_tum(path: &Path, scans: &[PolarScan]) -> std::io::Result<()> {
    let mut w = BufWriter::new(File::create(path)?);
    for s in scans.iter().filter(|s| s.health != VioHealth::Lost) {
        let [qw, qx, qy, qz] = s.q;
        writeln!(
            w,
            "{:.9} {} {} {} {} {} {} {}",
            s.t_ns as f64 * 1e-9,
            s.t[0],
            s.t[1],
            s.t[2],
            qx,
            qy,
            qz,
            qw
        )?;
    }
    w.flush()
}

fn parse_args() -> Result<(PathBuf, PathBuf), String> {
    let mut input = None;
    let mut out = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--out" => out = Some(PathBuf::from(args.next().ok_or("--out needs a dir")?)),
            _ if input.is_none() && !a.starts_with("--") => input = Some(PathBuf::from(a)),
            _ => return Err(format!("unexpected argument {a:?}")),
        }
    }
    match (input, out) {
        (Some(i), Some(o)) => Ok((i, o)),
        _ => Err("usage: planar_sweep_replay <lidar.polar.zst> --out <dir>".to_string()),
    }
}

fn run() -> Result<(), String> {
    let (input, out) = parse_args()?;
    let mut scans = read_scans(&input)?;
    let summary = close_loops(&mut scans);
    let swept = sweep(&scans);
    let points: Vec<[f32; 3]> = swept.cloud.points.iter().copied().map(flu_to_ned).collect();

    fs::create_dir_all(&out).map_err(|e| format!("{}: {e}", out.display()))?;
    let io = |e: std::io::Error| e.to_string();
    write_ply(&out.join("map.ply"), &points).map_err(io)?;
    write_tum(&out.join("poses.tum"), &scans).map_err(io)?;
    let stats = serde_json::json!({
        "scans": summary.scans,
        "points": points.len(),
        "reposed": summary.reposed,
    });
    fs::write(out.join("stats.json"), format!("{stats}\n")).map_err(io)?;
    println!("{stats}");
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("planar_sweep_replay: {e}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn z_up_point_maps_back_to_ned() {
        // NED (1, 2, -3): north 1, east 2, 3 m up. z-up world: (1, -2, 3).
        assert_eq!(flu_to_ned([1.0, -2.0, 3.0]), [1.0, 2.0, -3.0]);
    }
}
