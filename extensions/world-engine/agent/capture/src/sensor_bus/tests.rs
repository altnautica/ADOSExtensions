use super::*;
use ados_sensorbus::{encode, Origin};
use tokio::io::{AsyncReadExt, AsyncWriteExt};

fn imu(t0: u64, n: u64, step: u64) -> ImuBatch {
    ImuBatch {
        samples: (0..n)
            .map(|i| ImuSample {
                t_ns: t0 + i * step,
                seq: i,
                gyro: vec![0.0, 0.0, 0.1],
                accel: vec![0.0, 0.0, 9.81],
                temp_c: None,
            })
            .collect(),
    }
}

#[test]
fn the_imu_ring_keeps_ten_seconds() {
    let mut s = SensorState::default();
    // 12 s at 1 kHz in 1 s batches.
    for k in 0..12u64 {
        s.apply(&BusEvent::Imu {
            src: "imu0".into(),
            batch: imu(k * 1_000_000_000, 1000, 1_000_000),
        });
    }
    let n = s.imu_len("imu0");
    assert!((10_000..=10_001).contains(&n), "{n}");
    let w = s.imu_window(5_000_000_000, 5_010_000_000);
    assert_eq!(w.len(), 10);
    assert!(w.windows(2).all(|p| p[0].t_ns < p[1].t_ns));
}

#[test]
fn triggers_join_by_sequence_then_by_time() {
    let mut s = SensorState::default();
    for seq in 0..5u64 {
        s.apply(&BusEvent::Trigger {
            camera: "cam_fl".into(),
            rec: TriggerRecord {
                seq,
                t_mid_ns: 1_000_000_000 + seq * 50_000_000,
                exposure_ns: 2_000_000,
                gain: 1.0,
            },
        });
    }
    assert_eq!(s.join_trigger("cam_fl", Some(3), 0).unwrap().seq, 3);
    assert!(s.join_trigger("cam_fl", Some(9), 0).is_none());
    let near = s
        .join_trigger("cam_fl", None, 1_100_000_000 + 3_000_000)
        .unwrap();
    assert_eq!(near.seq, 2);
    assert!(s.join_trigger("cam_fl", None, 1_125_000_000).is_none());
    assert!(s.join_trigger("cam_fr", Some(1), 0).is_none());
}

#[tokio::test]
async fn ingest_subscribes_and_forwards_decoded_frames() {
    let dir = tempfile::tempdir().unwrap();
    let sock = dir.path().join("sensors.sock");
    let listener = tokio::net::UnixListener::bind(&sock).unwrap();
    let broker = tokio::spawn(async move {
        let (mut stream, _) = listener.accept().await.unwrap();
        let mut len = [0u8; 4];
        stream.read_exact(&mut len).await.unwrap();
        let mut body = vec![0u8; u32::from_be_bytes(len) as usize];
        stream.read_exact(&mut body).await.unwrap();
        let sub = ados_sensorbus::decode_body(&body).unwrap();
        let kinds: Vec<Kind> = sub.decode_msgpack().unwrap();
        for f in [
            BusFrame::proto(Kind::Imu, "imu0", Origin::Hub, &imu(10, 4, 1_000_000)),
            BusFrame::proto(
                Kind::Clock,
                "clock",
                Origin::Host,
                &ClockSample {
                    t_ns: 7,
                    host_mono_ns: 5,
                    utc_ns: None,
                    locked: true,
                    sigma_ns: 100,
                },
            ),
            BusFrame::new(Kind::Rtcm, "rest", Origin::Host, vec![1, 2]),
        ] {
            stream.write_all(&encode(&f).unwrap()).await.unwrap();
        }
        // Keep the stream open until the test is done reading.
        tokio::time::sleep(Duration::from_millis(500)).await;
        kinds
    });
    let state = Arc::new(Mutex::new(SensorState::default()));
    let (tx, mut rx) = mpsc::channel(16);
    let cancel = Shutdown::new();
    let task = tokio::spawn(run_ingest(sock, state.clone(), tx, cancel.clone()));
    let first = rx.recv().await.unwrap();
    assert!(
        matches!(&first, BusEvent::Imu { src, batch } if src == "imu0" && batch.samples.len() == 4)
    );
    assert!(matches!(rx.recv().await.unwrap(), BusEvent::Clock(c) if c.t_ns == 7));
    assert!(state.lock().connected);
    assert_eq!(state.lock().clock.to_session(6), 8);
    let kinds = broker.await.unwrap();
    assert_eq!(kinds, KINDS.to_vec());
    cancel.trigger();
    task.await.unwrap();
}

#[tokio::test]
async fn a_missing_socket_reports_disconnected_and_stops_on_cancel() {
    let dir = tempfile::tempdir().unwrap();
    let state = Arc::new(Mutex::new(SensorState::default()));
    let (tx, _rx) = mpsc::channel(1);
    let cancel = Shutdown::new();
    let task = tokio::spawn(run_ingest(
        dir.path().join("absent.sock"),
        state.clone(),
        tx,
        cancel.clone(),
    ));
    tokio::time::sleep(Duration::from_millis(100)).await;
    assert!(!state.lock().connected);
    cancel.trigger();
    tokio::time::timeout(Duration::from_secs(2), task)
        .await
        .unwrap()
        .unwrap();
}
