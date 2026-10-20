//! The driver seam: dispatch on the rig model, the not-configured and
//! unsupported cases, and an RPLIDAR over TCP delivering scans end to end.

use std::io::{Read, Write};
use std::net::TcpListener;
use std::thread;
use std::time::Duration;

use ados_capture_contract::{LidarModel, LidarModelPattern};
use ados_rplidar::proto::{self, encode_descriptor, encode_standard_node};

use super::*;

fn model(name: &str, pattern: LidarModelPattern, rate_hz: f64) -> LidarModel {
    LidarModel {
        model: name.into(),
        pattern,
        h_fov_deg: 360.0,
        v_min_deg: 0.0,
        v_max_deg: 0.0,
        rate_hz,
        points_per_s: 5000.0,
        min_range_m: 0.05,
        max_range_m: 12.0,
        range_sigma_m: 0.03,
        blocked_azimuth_deg: vec![],
    }
}

fn c1() -> LidarModel {
    model(MODEL_RPLIDAR_C1, LidarModelPattern::Planar, 10.0)
}

fn start_err(m: &LidarModel, s: &LidarSettings) -> LidarSourceError {
    let dir = tempfile::tempdir().unwrap();
    match start(m, s, "lidar0", dir.path()) {
        Ok(_) => panic!("started"),
        Err(e) => e,
    }
}

#[test]
fn period_follows_the_model_rate() {
    assert_eq!(period_ns(&c1()).unwrap(), 100_000_000);
    let m = model(MODEL_LIVOX_MID360, LidarModelPattern::Solid, 8.0);
    assert_eq!(period_ns(&m).unwrap(), 125_000_000);
    let bad = model(MODEL_LIVOX_MID360, LidarModelPattern::Solid, 0.0);
    assert!(matches!(
        period_ns(&bad),
        Err(LidarSourceError::InvalidRate(_))
    ));
}

#[test]
fn an_unknown_model_is_refused_by_name() {
    let m = model("velodyne-vlp16", LidarModelPattern::Spinning, 10.0);
    let e = start_err(&m, &LidarSettings::default());
    assert!(matches!(&e, LidarSourceError::UnsupportedModel(n) if n == "velodyne-vlp16"));
    assert!(e.to_string().contains("velodyne-vlp16"));
}

#[test]
fn an_empty_device_setting_means_no_driver() {
    let mut s = LidarSettings::default();
    s.rplidar.device = "  ".into();
    let e = start_err(&c1(), &s);
    assert!(matches!(
        e,
        LidarSourceError::NotConfigured("rplidar.device")
    ));
    // The Livox default has no host address.
    let m = model(MODEL_LIVOX_MID360, LidarModelPattern::Solid, 10.0);
    let e = start_err(&m, &LidarSettings::default());
    assert!(matches!(
        e,
        LidarSourceError::NotConfigured("livox.host_ip")
    ));
}

/// A device on TCP that answers the handshake and streams 500-node
/// revolutions at 2 m.
fn fake_rplidar() -> String {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = format!("tcp:{}", listener.local_addr().unwrap());
    thread::spawn(move || {
        let Ok((mut s, _)) = listener.accept() else {
            return;
        };
        s.set_read_timeout(Some(Duration::from_millis(2))).unwrap();
        let (mut streaming, mut k, mut pending) = (false, 0usize, Vec::new());
        loop {
            let mut buf = [0u8; 64];
            match s.read(&mut buf) {
                Ok(0) => return,
                Ok(n) => pending.extend_from_slice(&buf[..n]),
                Err(e)
                    if matches!(
                        e.kind(),
                        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                    ) => {}
                Err(_) => return,
            }
            while pending.len() >= 2 {
                if pending[0] != proto::SYNC {
                    pending.remove(0);
                    continue;
                }
                let cmd = pending[1];
                pending.drain(..2);
                let reply: Vec<u8> = match cmd {
                    proto::CMD_STOP | proto::CMD_RESET => {
                        streaming = false;
                        Vec::new()
                    }
                    proto::CMD_GET_INFO => {
                        let mut r = encode_descriptor(&proto::INFO_DESCRIPTOR).to_vec();
                        r.extend_from_slice(&[0x41, 1, 1, 18]);
                        r.extend_from_slice(&[0xAB; 16]);
                        r
                    }
                    proto::CMD_GET_HEALTH => {
                        let mut r = encode_descriptor(&proto::HEALTH_DESCRIPTOR).to_vec();
                        r.extend_from_slice(&[0, 0, 0]);
                        r
                    }
                    proto::CMD_SCAN => {
                        streaming = true;
                        encode_descriptor(&proto::SCAN_DESCRIPTOR).to_vec()
                    }
                    _ => Vec::new(),
                };
                if s.write_all(&reply).is_err() {
                    return;
                }
            }
            if streaming {
                let mut burst = Vec::with_capacity(250);
                for _ in 0..50 {
                    let slot = k % 500;
                    let angle = slot as f32 * 0.72;
                    burst.extend_from_slice(&encode_standard_node(slot == 0, 40, angle, 2.0));
                    k += 1;
                }
                if s.write_all(&burst).is_err() {
                    return;
                }
                thread::sleep(Duration::from_millis(10));
            }
        }
    });
    addr
}

#[test]
fn an_rplidar_model_streams_planar_scans_through_the_seam() {
    let mut s = LidarSettings::default();
    s.rplidar.device = fake_rplidar();
    let dir = tempfile::tempdir().unwrap();
    let (driver, mut rx) = start(&c1(), &s, "lidar0", dir.path()).expect("start");
    let scan = (0..2)
        .map(|_| {
            let deadline = std::time::Instant::now() + Duration::from_secs(5);
            loop {
                match rx.try_recv() {
                    Ok(s) => break s,
                    Err(_) if std::time::Instant::now() < deadline => {
                        thread::sleep(Duration::from_millis(5))
                    }
                    Err(e) => panic!("no scan within 5 s: {e}"),
                }
            }
        })
        .last()
        .unwrap();
    assert_eq!(scan.sensor_id, "lidar0");
    assert_eq!(scan.len(), 500);
    assert_eq!(scan.t_offset_ns.len(), 500);
    assert!(scan.points.iter().all(|p| p[2] == 0.0));
    assert!(scan
        .points
        .iter()
        .all(|p| ((p[0] * p[0] + p[1] * p[1]).sqrt() - 2.0).abs() < 1e-3));
    let st = driver.stats();
    assert_eq!(st.connected, Some(true));
    assert!(st.scans >= 2, "{st:?}");
    drop(driver);
}
