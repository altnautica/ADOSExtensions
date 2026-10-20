//! The driver against an in-process device speaking the protocol over TCP.

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use ados_rplidar::proto::{self, encode_descriptor, encode_standard_node};
use ados_rplidar::{HealthStatus, RawScan, Rplidar, RplidarConfig, RplidarError};

const NODES_PER_REV: usize = 500;
/// Slot with no return in every revolution.
const ZERO_SLOT: usize = 300;
/// Nodes written per 10 ms burst: 5 kHz.
const BURST: usize = 50;

#[derive(Clone)]
struct Fake {
    health: [u8; 3],
    /// Close the connection after this many nodes streamed.
    nodes_per_conn: Option<usize>,
    /// Every command byte received.
    commands: Arc<Mutex<Vec<u8>>>,
}

impl Fake {
    fn new() -> Self {
        Self {
            health: [0, 0, 0],
            nodes_per_conn: None,
            commands: Arc::default(),
        }
    }

    fn count(&self, cmd: u8) -> usize {
        self.commands
            .lock()
            .unwrap()
            .iter()
            .filter(|&&c| c == cmd)
            .count()
    }

    /// Listen on an ephemeral port, serving one connection after another.
    fn spawn(self) -> String {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = format!("tcp:{}", listener.local_addr().unwrap());
        thread::spawn(move || {
            for conn in listener.incoming() {
                let Ok(conn) = conn else { return };
                self.serve(conn);
            }
        });
        addr
    }

    fn serve(&self, mut s: TcpStream) {
        s.set_read_timeout(Some(Duration::from_millis(2))).unwrap();
        let mut streaming = false;
        let mut k = 0usize;
        let mut sent = 0usize;
        let mut pending = Vec::new();
        loop {
            let mut buf = [0u8; 64];
            match s.read(&mut buf) {
                Ok(0) => return,
                Ok(n) => pending.extend_from_slice(&buf[..n]),
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                Err(_) => return,
            }
            while pending.len() >= 2 {
                if pending[0] != proto::SYNC {
                    pending.remove(0);
                    continue;
                }
                let cmd = pending[1];
                pending.drain(..2);
                self.commands.lock().unwrap().push(cmd);
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
                        r.extend_from_slice(&self.health);
                        r
                    }
                    proto::CMD_SCAN => {
                        streaming = true;
                        k = 0;
                        encode_descriptor(&proto::SCAN_DESCRIPTOR).to_vec()
                    }
                    _ => Vec::new(),
                };
                if s.write_all(&reply).is_err() {
                    return;
                }
            }
            if streaming {
                let mut burst = Vec::with_capacity(BURST * 5);
                for _ in 0..BURST {
                    let slot = k % NODES_PER_REV;
                    let angle = slot as f32 * 360.0 / NODES_PER_REV as f32;
                    let dist = if slot == ZERO_SLOT { 0.0 } else { 2.0 };
                    burst.extend_from_slice(&encode_standard_node(slot == 0, 50, angle, dist));
                    k += 1;
                }
                if s.write_all(&burst).is_err() {
                    return;
                }
                sent += BURST;
                if self.nodes_per_conn.is_some_and(|n| sent >= n) {
                    return;
                }
                thread::sleep(Duration::from_millis(10));
            }
        }
    }
}

fn config(device: String) -> RplidarConfig {
    RplidarConfig {
        device,
        sensor_id: "lidar0".into(),
        ..RplidarConfig::default()
    }
}

fn recv(rx: &std::sync::mpsc::Receiver<RawScan>) -> RawScan {
    rx.recv_timeout(Duration::from_secs(5))
        .expect("revolution within 5 s")
}

fn assert_complete(s: &RawScan) {
    assert_eq!(s.len(), NODES_PER_REV - 1);
    assert_eq!(s.t_offset_ns.len(), s.len());
    assert_eq!(s.intensity.len(), s.len());
    assert!(s.points.iter().all(|p| p[2] == 0.0));
    assert!(s.t_offset_ns.windows(2).all(|w| w[0] <= w[1]));
    assert!(s.intensity.iter().all(|&q| q == 50));
}

#[test]
fn streams_complete_revolutions_with_monotonic_timestamps() {
    let addr = Fake::new().spawn();
    let (lidar, rx) = Rplidar::start(config(addr)).expect("start");
    assert_eq!(lidar.info().model, 0x41);
    assert_eq!(lidar.info().firmware(), "1.01");
    assert_eq!(lidar.health().status, HealthStatus::Good);

    let scans: Vec<RawScan> = (0..4).map(|_| recv(&rx)).collect();
    for s in &scans {
        assert_complete(s);
        // Slot 0 is device 0 deg: straight ahead. Slot 125 is device 90 deg
        // (clockwise, the right side): FLU -y.
        let p0 = s.points[0];
        assert!((p0[0] - 2.0).abs() < 1e-3 && p0[1].abs() < 1e-3, "{p0:?}");
        let p90 = s.points[125];
        assert!(
            p90[0].abs() < 1e-3 && (p90[1] + 2.0).abs() < 1e-3,
            "{p90:?}"
        );
        // A revolution spans well under a second.
        assert!(*s.t_offset_ns.last().unwrap() < 1_000_000_000);
    }
    for w in scans.windows(2) {
        assert!(w[0].t_ns < w[1].t_ns, "{} !< {}", w[0].t_ns, w[1].t_ns);
        let period = w[1].t_ns - w[0].t_ns;
        assert!(
            (40_000_000..400_000_000).contains(&period),
            "period {period} ns"
        );
    }
    let st = lidar.stats();
    assert!(st.connected);
    assert!(st.revolutions >= 4);
    assert_eq!(st.resync_bytes, 0);
}

#[test]
fn a_health_error_fails_start_with_its_code() {
    let mut fake = Fake::new();
    fake.health = [2, 0x34, 0x12];
    let addr = fake.spawn();
    match Rplidar::start(config(addr)) {
        Err(RplidarError::Health(code)) => assert_eq!(code, 0x1234),
        Err(e) => panic!("unexpected error {e}"),
        Ok(_) => panic!("started despite a health error"),
    }
}

#[test]
fn dropping_the_driver_stops_the_scan() {
    let fake = Fake::new();
    let addr = fake.clone().spawn();
    let (lidar, rx) = Rplidar::start(config(addr)).expect("start");
    recv(&rx);
    let stops_before = fake.count(proto::CMD_STOP);
    drop(lidar);
    let deadline = Instant::now() + Duration::from_secs(2);
    while fake.count(proto::CMD_STOP) == stops_before {
        assert!(Instant::now() < deadline, "no STOP after drop");
        thread::sleep(Duration::from_millis(10));
    }
}

#[test]
fn a_lost_link_is_reopened_and_scanning_resumes() {
    let mut fake = Fake::new();
    fake.nodes_per_conn = Some(3 * NODES_PER_REV + BURST);
    let addr = fake.clone().spawn();
    let (lidar, rx) = Rplidar::start(config(addr)).expect("start");
    let scans: Vec<RawScan> = (0..5).map(|_| recv(&rx)).collect();
    scans.iter().for_each(assert_complete);
    assert!(scans.windows(2).all(|w| w[0].t_ns < w[1].t_ns));
    // The fourth revolution comes from the second connection, at least the
    // reconnect delay after the third.
    assert!(scans[3].t_ns - scans[2].t_ns >= 1_900_000_000);
    assert!(lidar.stats().reconnects >= 1);
    assert!(fake.count(proto::CMD_SCAN) >= 2);
}
