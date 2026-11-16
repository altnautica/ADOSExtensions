//! The driver seam: dispatch on the rig model, the not-configured and
//! unsupported cases, an RPLIDAR over TCP delivering scans end to end, and
//! the run's LiDAR task (start retry, the stable scan channel, the status).

use std::io::{Read, Write};
use std::net::TcpListener;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::thread;
use std::time::Duration;

use ados_capture_contract::{LidarModel, LidarModelPattern};
use ados_rplidar::proto::{self, encode_descriptor, encode_standard_node};
use ados_rplidar::RplidarError;

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
    let (tx, _rx) = mpsc::channel(SCAN_CHANNEL_DEPTH);
    match start(m, s, "lidar0", dir.path(), tx) {
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
/// revolutions at `dist_m` (0: every node without a return).
fn fake_rplidar(dist_m: f32) -> String {
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
                    burst.extend_from_slice(&encode_standard_node(slot == 0, 40, angle, dist_m));
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
    let addr = fake_rplidar(2.0);
    s.rplidar.device = addr.clone();
    let dir = tempfile::tempdir().unwrap();
    let (tx, mut rx) = mpsc::channel(SCAN_CHANNEL_DEPTH);
    let driver = start(&c1(), &s, "lidar0", dir.path(), tx).expect("start");
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
    assert!(st.connected);
    assert!(st.scans >= 2, "{st:?}");
    assert!(st.revolutions >= st.scans, "{st:?}");
    assert_eq!(st.device.as_deref(), Some(addr.as_str()));
    assert_eq!(st.health, Some((LidarHealth::Good, 0)));
    assert_eq!((st.returns_last_scan, st.empty_scans), (Some(500), 0));
    drop(driver);
}

/// Scans that keep arriving with nothing in them are a live link that sees
/// nothing: the status says so instead of reading as a working sensor.
#[test]
fn an_rplidar_seeing_nothing_reports_its_empty_scans() {
    let mut s = LidarSettings::default();
    s.rplidar.device = fake_rplidar(0.0);
    let dir = tempfile::tempdir().unwrap();
    let (tx, mut rx) = mpsc::channel(SCAN_CHANNEL_DEPTH);
    let driver = start(&c1(), &s, "lidar0", dir.path(), tx).expect("start");
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    let mut got = 0;
    while got < 2 {
        match rx.try_recv() {
            Ok(scan) => {
                assert_eq!(scan.len(), 0, "no node returned");
                got += 1;
            }
            Err(_) if std::time::Instant::now() < deadline => {
                thread::sleep(Duration::from_millis(5))
            }
            Err(e) => panic!("no scan within 5 s: {e}"),
        }
    }
    let st = driver.stats();
    assert!(st.connected && st.scans >= 2, "{st:?}");
    assert_eq!(st.returns_last_scan, Some(0), "{st:?}");
    assert!(st.empty_scans >= 2, "{st:?}");
    let status = lidar_status(Some(&st), Some(10.0));
    assert_eq!(status.returns_last_scan, Some(0));
    assert_eq!(status.empty_scans, st.empty_scans);
    drop(driver);
}

#[test]
fn only_a_device_fault_is_worth_another_start() {
    for e in [
        LidarSourceError::Rplidar(RplidarError::NotFound),
        LidarSourceError::Rplidar(RplidarError::Health(0x10)),
        LidarSourceError::Rplidar(RplidarError::Timeout {
            cmd: "GET_INFO",
            ms: 1000,
        }),
    ] {
        assert!(e.is_retryable(), "{e}");
    }
    for e in [
        LidarSourceError::NotConfigured("rplidar.device"),
        LidarSourceError::UnsupportedModel("x".into()),
        LidarSourceError::InvalidRate(0.0),
        LidarSourceError::Unsupported,
    ] {
        assert!(!e.is_retryable(), "{e}");
    }
}

#[test]
fn the_status_reports_a_running_driver_and_nulls_without_one() {
    assert_eq!(lidar_status(None, Some(10.0)), LidarStatus::default());
    let up = LidarDriverStats {
        connected: true,
        device: Some("/dev/serial/by-id/usb-lidar".into()),
        health: Some((LidarHealth::Warning, 5)),
        revolutions: 40,
        scans: 39,
        dropped_scans: 1,
        reconnects: 2,
        returns_last_scan: Some(312),
        empty_scans: 4,
    };
    let st = lidar_status(Some(&up), Some(10.0));
    assert_eq!(
        st,
        LidarStatus {
            device: Some("/dev/serial/by-id/usb-lidar".into()),
            connected: true,
            health: Some(LidarHealth::Warning),
            health_code: Some(5),
            rate_hz: Some(10.0),
            scans: 39,
            dropped_scans: 1,
            reconnects: 2,
            returns_last_scan: Some(312),
            empty_scans: 4,
            ..LidarStatus::default()
        }
    );
    // A link that dropped keeps its port, last health and counters but has
    // no rate and no current returns.
    let down = LidarDriverStats {
        connected: false,
        ..up
    };
    let st = lidar_status(Some(&down), Some(10.0));
    assert!(!st.connected);
    assert_eq!((st.rate_hz, st.returns_last_scan), (None, None));
    assert_eq!(st.device.as_deref(), Some("/dev/serial/by-id/usb-lidar"));
    assert_eq!(st.health, Some(LidarHealth::Warning));
    assert_eq!(st.empty_scans, 4);
}

#[test]
fn the_rate_is_revolutions_per_second_between_samples() {
    let t0 = Instant::now();
    let at = |ms| t0 + Duration::from_millis(ms);
    let st = |revolutions, connected| LidarDriverStats {
        revolutions,
        connected,
        ..LidarDriverStats::default()
    };
    let mut m = RateMeter::default();
    assert_eq!(m.sample(at(0), &st(0, true)), None, "no previous sample");
    assert_eq!(m.sample(at(1000), &st(10, true)), Some(10.0));
    assert_eq!(m.sample(at(1500), &st(15, true)), Some(10.0));
    assert_eq!(m.sample(at(2500), &st(15, false)), None, "link down");
    m.restart(at(3000), &st(100, true));
    assert_eq!(m.sample(at(3500), &st(105, true)), Some(10.0));
}

/// A driver that is up on `tcp:fake` with a good health; dropping it sets
/// `dropped`.
struct FakeDriver {
    dropped: Arc<AtomicBool>,
}

impl LidarDriver for FakeDriver {
    fn stats(&self) -> LidarDriverStats {
        LidarDriverStats {
            connected: true,
            device: Some("tcp:fake".into()),
            health: Some((LidarHealth::Good, 0)),
            revolutions: 3,
            scans: 3,
            ..LidarDriverStats::default()
        }
    }
}

impl Drop for FakeDriver {
    fn drop(&mut self) {
        self.dropped.store(true, Ordering::SeqCst);
    }
}

fn quick(starter: Starter) -> LidarRunner {
    LidarRunner {
        retry: Duration::from_millis(20),
        status_period: Duration::from_millis(20),
        ..LidarRunner::new("lidar0", MODEL_RPLIDAR_C1, starter)
    }
}

type Started = Result<Box<dyn LidarDriver>, LidarSourceError>;

#[tokio::test]
async fn a_lidar_absent_at_start_is_retried_and_its_scans_reach_the_run_s_receiver() {
    let calls = Arc::new(AtomicUsize::new(0));
    let dropped = Arc::new(AtomicBool::new(false));
    let starter: Starter = {
        let (calls, dropped) = (calls.clone(), dropped.clone());
        Arc::new(move |tx: mpsc::Sender<LidarScan>| -> Started {
            // Absent twice, then it answers and streams.
            if calls.fetch_add(1, Ordering::SeqCst) < 2 {
                return Err(RplidarError::NotFound.into());
            }
            for t_ns in 1..=3 {
                tx.try_send(LidarScan {
                    sensor_id: "lidar0".into(),
                    t_ns,
                    ..LidarScan::default()
                })
                .unwrap();
            }
            Ok(Box::new(FakeDriver {
                dropped: dropped.clone(),
            }) as Box<dyn LidarDriver>)
        })
    };
    // The receiver exists before any start attempt, as the capture loop's.
    let (tx, mut rx) = mpsc::channel(SCAN_CHANNEL_DEPTH);
    let (status_tx, mut status_rx) = watch::channel(LidarStatus::default());
    let task = tokio::spawn(quick(starter).run(tx, status_tx));

    let mut got = Vec::new();
    for _ in 0..3 {
        let scan = tokio::time::timeout(Duration::from_secs(5), rx.recv())
            .await
            .expect("a scan within 5 s")
            .expect("the run's channel stays open");
        got.push(scan.t_ns);
    }
    assert_eq!(got, [1, 2, 3]);
    assert_eq!(calls.load(Ordering::SeqCst), 3);

    let st = tokio::time::timeout(Duration::from_secs(5), async {
        loop {
            status_rx.changed().await.unwrap();
            let st = status_rx.borrow_and_update().clone();
            if st.connected {
                break st;
            }
        }
    })
    .await
    .expect("a connected status within 5 s");
    assert_eq!(st.device.as_deref(), Some("tcp:fake"));
    assert_eq!(
        (st.health, st.health_code),
        (Some(LidarHealth::Good), Some(0))
    );
    assert_eq!(st.scans, 3);

    // The run ending drops the driver.
    task.abort();
    let _ = task.await;
    assert!(dropped.load(Ordering::SeqCst));
    assert_eq!(calls.load(Ordering::SeqCst), 3, "no start after success");
}

#[tokio::test]
async fn an_unconfigured_lidar_is_not_retried_and_reports_not_connected() {
    let calls = Arc::new(AtomicUsize::new(0));
    let starter: Starter = {
        let calls = calls.clone();
        Arc::new(move |_tx: mpsc::Sender<LidarScan>| -> Started {
            calls.fetch_add(1, Ordering::SeqCst);
            Err(LidarSourceError::NotConfigured("rplidar.device"))
        })
    };
    let (tx, _rx) = mpsc::channel(SCAN_CHANNEL_DEPTH);
    let (status_tx, mut status_rx) = watch::channel(LidarStatus {
        connected: true,
        ..LidarStatus::default()
    });
    let task = tokio::spawn(quick(starter).run(tx, status_tx));
    tokio::time::sleep(Duration::from_millis(200)).await;
    assert_eq!(calls.load(Ordering::SeqCst), 1);
    // Still reporting, as not running.
    assert!(status_rx.has_changed().unwrap());
    assert_eq!(*status_rx.borrow_and_update(), LidarStatus::default());
    task.abort();
    let _ = task.await;
}
