//! `dump <device|tcp:host:port|auto> [baud]`: print the device info and
//! health, then ten revolutions as `n_points, rev_hz`.

use std::process::ExitCode;
use std::time::Duration;

use ados_rplidar::{Rplidar, RplidarConfig, DEFAULT_BAUD};

const REVOLUTIONS: usize = 10;

fn main() -> ExitCode {
    let mut args = std::env::args().skip(1);
    let Some(device) = args.next() else {
        eprintln!("usage: dump <device|tcp:host:port|auto> [baud]");
        return ExitCode::from(2);
    };
    let baud = match args.next().map(|b| b.parse::<u32>()) {
        None => DEFAULT_BAUD,
        Some(Ok(b)) => b,
        Some(Err(e)) => {
            eprintln!("bad baud: {e}");
            return ExitCode::from(2);
        }
    };
    let cfg = RplidarConfig {
        device,
        baud,
        ..RplidarConfig::default()
    };
    let (lidar, rx) = match Rplidar::start(cfg) {
        Ok(started) => started,
        Err(e) => {
            eprintln!("start failed: {e}");
            return ExitCode::FAILURE;
        }
    };
    let info = lidar.info();
    println!(
        "info: path {} model 0x{:02X} firmware {} hardware {} serial {}",
        lidar.path(),
        info.model,
        info.firmware(),
        info.hardware,
        info.serial_hex()
    );
    let health = lidar.health();
    println!(
        "health: {:?} code 0x{:04X}",
        health.status, health.error_code
    );
    // The first revolution only anchors the rate of the second.
    let mut prev_t: Option<u64> = None;
    let mut printed = 0;
    while printed < REVOLUTIONS {
        let scan = match rx.recv_timeout(Duration::from_secs(5)) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("no revolution within 5 s: {e}");
                return ExitCode::FAILURE;
            }
        };
        if let Some(prev) = prev_t {
            let dt = scan.t_ns.saturating_sub(prev) as f64 * 1e-9;
            let hz = if dt > 0.0 { 1.0 / dt } else { f64::NAN };
            println!("{}, {hz:.2}", scan.len());
            printed += 1;
        }
        prev_t = Some(scan.t_ns);
    }
    let st = lidar.stats();
    println!(
        "stats: revolutions {} points {} no_return {} resync_bytes {} dropped {}",
        st.revolutions, st.points, st.no_return, st.resync_bytes, st.dropped_scans
    );
    ExitCode::SUCCESS
}
