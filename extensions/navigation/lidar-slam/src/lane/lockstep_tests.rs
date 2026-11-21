//! The LiDAR lane's lockstep pacing: the pure state, and a whole lane driven
//! by a simulator stepping its grants faster than real time against a
//! barrier that waits for every registered participant.

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{sync_channel, SyncSender, TrySendError};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use ados_mapping_pose::lidar_scan::LidarScan;
use ados_mapping_pose::{diagonal_cov, se3, PoseEstimate, PoseProvider, PoseTrack};
use ados_mapping_protocol::capture::{PoseSource, VioHealth};
use ados_protocol::shutdown::Shutdown;
use ados_protocol::simclock::wire::{Done, Tick, DONE_LEN, FLAG_LOCKSTEP};
use ados_protocol::simclock::{Clock, SimFeed};
use tokio::io::AsyncReadExt;
use tokio::sync::mpsc;

use super::*;

const MS: u64 = 1_000_000;
const PERIOD: u64 = 100 * MS;
const GRANT: u64 = 20 * MS;

// ---- the pure state ----

#[test]
fn reports_from_the_second_scan_half_a_period_past_the_next_arrival() {
    let mut p = ScanPacing::default();
    p.fed(1_000 * MS, 0);
    p.answered(Some(1_000 * MS));
    assert_eq!(p.done_target(), None, "period unknown after one scan");
    p.fed(1_100 * MS, 1);
    assert_eq!(
        p.done_target(),
        None,
        "a scan with the provider holds the report"
    );
    p.answered(Some(1_099 * MS));
    assert_eq!(
        p.done_target(),
        None,
        "an older estimate does not answer it"
    );
    p.answered(Some(1_100 * MS));
    // The next scan (stamped 1200 ms) closes once the one after starts.
    assert_eq!(p.done_target(), Some(1_350 * MS));
}

#[test]
fn a_missed_scan_or_a_long_first_interval_is_no_period() {
    let mut p = ScanPacing::default();
    // A missed scan (100 -> 300) and a jittered stamp (404).
    for (i, t) in [0, 100, 300, 404, 500].into_iter().enumerate() {
        p.fed(t * MS, i as u64);
    }
    let period = p.period_ns().unwrap();
    assert!((95 * MS..=101 * MS).contains(&period), "{period}");
    // A driver's first interval a second long while its clock mapping
    // settles gives way at the next one.
    let mut p = ScanPacing::default();
    for t in [0, 1_021, 1_121] {
        p.fed(t * MS, 0);
    }
    assert_eq!(p.period_ns(), Some(100 * MS));
}

#[test]
fn a_provider_with_no_track_answers_every_scan() {
    let mut p = ScanPacing::default();
    p.fed(0, 0);
    p.fed(PERIOD, 0);
    assert_eq!(p.inflight(), 2);
    p.answered(None);
    assert_eq!(p.inflight(), 0);
    assert!(p.done_target().is_some());
}

#[test]
fn an_unanswered_scan_is_let_go_after_the_budget() {
    let mut p = ScanPacing::default();
    p.fed(0, 0);
    p.fed(PERIOD, 10);
    assert!(p.check(Some(PERIOD), SCAN_BUDGET_MS - 1).is_empty());
    assert_eq!(
        p.check(Some(PERIOD), SCAN_BUDGET_MS),
        vec![Paced::Overrun(0)]
    );
    assert_eq!(
        p.check(Some(PERIOD), SCAN_BUDGET_MS + 10),
        vec![Paced::Overrun(PERIOD)]
    );
    assert_eq!(p.done_target(), Some(PERIOD + 2 * PERIOD + PERIOD / 2));
}

#[test]
fn a_silent_source_deepens_then_leaves_and_keeps_its_depth() {
    let mut p = ScanPacing::default();
    p.fed(0, 0);
    p.fed(PERIOD, 0);
    p.answered(Some(PERIOD));
    // Next scan expected by 300 ms; the simulator holds at 350 ms.
    assert!(p.check(Some(300 * MS), 0).is_empty(), "not late yet");
    assert!(p.check(Some(350 * MS), 100).is_empty(), "late, probing");
    assert_eq!(
        p.check(Some(350 * MS), 100 + SCAN_PROBE_MS),
        vec![Paced::Deeper(2)]
    );
    assert_eq!(p.done_target(), Some(PERIOD + 3 * PERIOD + PERIOD / 2));
    let t = 1_000;
    assert!(p.check(Some(450 * MS), t).is_empty());
    assert_eq!(
        p.check(Some(450 * MS), t + SCAN_PROBE_MS),
        vec![Paced::Deeper(3)]
    );
    let t = 2_000;
    assert!(p.check(Some(550 * MS), t).is_empty());
    assert!(
        p.check(Some(550 * MS), t + SCAN_PROBE_MS).is_empty(),
        "deepest: waits"
    );
    assert_eq!(
        p.check(Some(550 * MS), t + SCAN_SILENCE_MS),
        vec![Paced::Gone]
    );
    assert_eq!(p.done_target(), None, "gone until scans return");
    assert_eq!(p.depth(), MAX_SOURCE_DEPTH_SCANS, "the learned depth stays");
}

#[test]
fn a_stamp_going_back_starts_over() {
    let mut p = ScanPacing::default();
    assert!(p.fed(5_000 * MS, 0));
    assert!(p.fed(5_100 * MS, 0));
    assert!(!p.fed(100 * MS, 0), "a new run");
    p.answered(Some(100 * MS));
    assert_eq!(p.done_target(), None, "the period is learned afresh");
}

// ---- a lane against a simulator and its barrier ----

/// The agent's barrier as the simulator sees it: every connection is a
/// participant holding each tick until it reports done past it.
#[derive(Clone, Default)]
struct FakeBarrier {
    parts: Arc<Mutex<BTreeMap<u64, Option<u64>>>>,
}

impl FakeBarrier {
    fn serve(&self, path: std::path::PathBuf) {
        let listener = tokio::net::UnixListener::bind(path).unwrap();
        let parts = self.parts.clone();
        tokio::spawn(async move {
            for id in 0.. {
                let Ok((mut s, _)) = listener.accept().await else {
                    return;
                };
                parts.lock().unwrap().insert(id, None);
                let parts = parts.clone();
                tokio::spawn(async move {
                    let mut b = [0u8; DONE_LEN];
                    while s.read_exact(&mut b).await.is_ok() {
                        let d = Done::decode(&b).unwrap();
                        parts.lock().unwrap().insert(id, Some(d.done_ns));
                    }
                    parts.lock().unwrap().remove(&id);
                });
            }
        });
    }

    fn parts(&self) -> usize {
        self.parts.lock().unwrap().len()
    }

    /// Whether the tick at `sim_ns` is acknowledged.
    fn ready(&self, sim_ns: u64) -> bool {
        self.parts
            .lock()
            .unwrap()
            .values()
            .all(|d| d.is_some_and(|d| d > sim_ns))
    }
}

/// A pose provider matching each scan on a worker thread after `work`,
/// with a two-scan queue that drops what does not fit (as the 3D one).
struct SlowPose {
    track: Arc<PoseTrack>,
    tx: Mutex<SyncSender<u64>>,
    dropped: AtomicU64,
    /// The sim time each pose was produced at, by scan stamp.
    produced: Arc<Mutex<Vec<(u64, u64)>>>,
}

impl SlowPose {
    fn new(clock: Clock, work: Duration) -> Arc<Self> {
        let (tx, rx) = sync_channel::<u64>(2);
        let track = Arc::new(PoseTrack::new());
        let produced = Arc::new(Mutex::new(Vec::new()));
        let (t, p) = (track.clone(), produced.clone());
        std::thread::spawn(move || {
            while let Ok(stamp) = rx.recv() {
                std::thread::sleep(work);
                p.lock().unwrap().push((stamp, clock.sim_ns().unwrap_or(0)));
                t.push(PoseEstimate {
                    t_ns: stamp,
                    r: se3::IDENTITY,
                    t: [0.0; 3],
                    cov: diagonal_cov(0.05, 0.01),
                    source: PoseSource::Lidar,
                    health: VioHealth::Good,
                    sigma_xy_m: None,
                    heading_ok: true,
                    health_reason: None,
                });
            }
        });
        Arc::new(Self {
            track,
            tx: Mutex::new(tx),
            dropped: AtomicU64::new(0),
            produced,
        })
    }
}

impl PoseProvider for SlowPose {
    fn source(&self) -> PoseSource {
        PoseSource::Lidar
    }
    fn track(&self) -> &PoseTrack {
        &self.track
    }
    fn feed_lidar(&self, scan: &LidarScan) {
        if let Err(TrySendError::Full(_)) = self.tx.lock().unwrap().try_send(scan.t_ns) {
            self.dropped.fetch_add(1, Ordering::Relaxed);
        }
    }
}

fn scan(t_ns: u64) -> LidarScan {
    LidarScan {
        sensor_id: "lidar0".into(),
        t_ns,
        points: vec![[5.0, 0.0, 0.0]],
        t_offset_ns: vec![0],
        intensity: vec![10],
    }
}

/// What one simulated run saw.
struct Take {
    /// Scans the simulator emitted.
    emitted: u64,
    /// Wall time of the scanning part.
    wall: Duration,
    /// The longest wall time a tick waited for the barrier.
    longest_hold: Duration,
    /// Wall time after the source stopped until no participant was left.
    left_after: Option<Duration>,
}

/// Step a lockstep simulator in `GRANT`s, `step_wall` of wall time each,
/// to `scan_until` with the LiDAR on (a scan handed over once the next one
/// starts, as both drivers do), then on to `run_until` with it silent.
/// Every tick waits for the barrier; a wait past 2 s (World Engine's
/// participant timeout) fails the run.
async fn simulate(
    feed: &SimFeed,
    barrier: &FakeBarrier,
    scans: mpsc::Sender<LidarScan>,
    step_wall: Duration,
    scan_until: u64,
    run_until: u64,
) -> Take {
    let (mut sim, mut seq, mut k) = (0u64, 0u32, 0u64);
    let mut take = Take {
        emitted: 0,
        wall: Duration::ZERO,
        longest_hold: Duration::ZERO,
        left_after: None,
    };
    let start = Instant::now();
    let mut stopped: Option<Instant> = None;
    while sim < run_until {
        sim += GRANT;
        if sim <= scan_until {
            while (k + 1) * PERIOD <= sim {
                scans.send(scan(k * PERIOD)).await.unwrap();
                k += 1;
            }
        } else if stopped.is_none() {
            take.wall = start.elapsed();
            stopped = Some(Instant::now());
        }
        if !step_wall.is_zero() {
            tokio::time::sleep(step_wall).await;
        }
        seq += 1;
        let tick = Tick {
            flags: FLAG_LOCKSTEP,
            seq,
            sim_ns: sim,
            grant_ns: sim + GRANT,
            speed: 0.0,
        };
        assert!(feed.ingest(&tick, Instant::now()));
        let held = Instant::now();
        while !barrier.ready(sim) {
            assert!(
                held.elapsed() < Duration::from_secs(2),
                "participant timeout at sim {} ms",
                sim / MS
            );
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
        take.longest_hold = take.longest_hold.max(held.elapsed());
        if let (Some(at), None) = (stopped, take.left_after) {
            if barrier.parts() == 0 {
                take.left_after = Some(at.elapsed());
            }
        }
    }
    take.emitted = k;
    take
}

/// A lane on a sim clock fed by [`simulate`]; `lockstep` makes it a
/// barrier participant.
async fn run_take(
    lockstep: bool,
    work: Duration,
    step_wall: Duration,
    scan_until: u64,
    run_until: u64,
) -> (Take, Arc<SlowPose>, FakeBarrier) {
    let dir = std::env::temp_dir().join(format!(
        "nav-lockstep-{}-{}",
        std::process::id(),
        u8::from(lockstep)
    ));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let socket = dir.join("participants.sock");
    let barrier = FakeBarrier::default();
    barrier.serve(socket.clone());
    let feed = SimFeed::new();
    let clock = Clock::Sim(feed.clone());
    // The simulator's run starts at its first tick.
    feed.ingest(
        &Tick {
            flags: FLAG_LOCKSTEP,
            seq: 0,
            sim_ns: 0,
            grant_ns: GRANT,
            speed: 0.0,
        },
        Instant::now(),
    );
    let pose = SlowPose::new(clock.clone(), work);
    let (mut deps, mut out) = crate::lane::tests::deps(Some(pose.clone()), false);
    let (scan_tx, scan_rx) = mpsc::channel(crate::lidar_source::SCAN_CHANNEL_DEPTH);
    deps.lidar_rx = Some(scan_rx);
    deps.clock = clock;
    deps.lockstep = lockstep.then_some(socket);
    let drain = tokio::spawn(async move { while out.recv().await.is_some() {} });
    let cancel = Shutdown::new();
    let lane = tokio::spawn(crate::lane::run_lane(deps, cancel.clone()));
    let take = simulate(&feed, &barrier, scan_tx, step_wall, scan_until, run_until).await;
    cancel.trigger();
    lane.await.unwrap();
    drain.abort();
    let _ = std::fs::remove_dir_all(&dir);
    (take, pose, barrier)
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn outside_the_barrier_a_max_speed_run_starves_the_odometry() {
    let (take, pose, _) = run_take(
        false,
        Duration::from_millis(5),
        Duration::ZERO,
        5_000 * MS,
        5_000 * MS,
    )
    .await;
    // What was queued gets matched; what was dropped never is.
    tokio::time::sleep(Duration::from_millis(100)).await;
    let matched = pose.produced.lock().unwrap().len() as u64;
    eprintln!(
        "outside the barrier: {matched} of {} scans matched, {} dropped",
        take.emitted,
        pose.dropped.load(Ordering::Relaxed)
    );
    assert!(
        pose.dropped.load(Ordering::Relaxed) > 0 && matched < take.emitted,
        "{matched} of {} scans matched",
        take.emitted
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn in_the_barrier_every_scan_is_matched_faster_than_real_time_and_the_lane_leaves_when_scans_stop(
) {
    let scan_until = 6_000 * MS;
    let (take, pose, barrier) = run_take(
        true,
        Duration::from_millis(2),
        // 20 ms of sim time a millisecond: a run as fast as the other
        // participants let it go.
        Duration::from_millis(1),
        scan_until,
        scan_until + 3_000 * MS,
    )
    .await;
    let produced = pose.produced.lock().unwrap().clone();
    // The lane joins from its second scan; from then on nothing is dropped.
    assert_eq!(pose.dropped.load(Ordering::Relaxed), 0, "no scan dropped");
    let stamps: Vec<u64> = produced.iter().map(|(s, _)| *s).collect();
    let expected: Vec<u64> = (0..take.emitted).map(|k| k * PERIOD).collect();
    assert_eq!(
        stamps, expected,
        "every scan matched, in order, none missing"
    );
    // No starvation: each pose is ready before the simulator passes the
    // point its next scan is due by (the hold, half a period past the next
    // scan's arrival, rounded up to a grant).
    for (stamp, at) in produced.iter().skip(2) {
        assert!(
            *at <= stamp + 2 * PERIOD,
            "pose of {} ms made at sim {} ms",
            stamp / MS,
            at / MS
        );
    }
    let speed = scan_until as f64 / take.wall.as_nanos() as f64;
    eprintln!(
        "in the barrier: {} of {} scans matched at {speed:.1}x real time, longest hold {:?}, left {:?} after the scans stopped",
        produced.len(),
        take.emitted,
        take.longest_hold,
        take.left_after
    );
    assert!(speed > 2.0, "ran at {speed:.1}x real time");
    assert!(
        take.longest_hold < Duration::from_millis(1_500),
        "{:?}",
        take.longest_hold
    );
    let left = take
        .left_after
        .expect("the lane left the barrier once scans stopped");
    assert!(left < Duration::from_secs(3), "left after {left:?}");
    assert_eq!(barrier.parts(), 0, "the stopped lane is not registered");
}
