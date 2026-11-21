use super::*;

const MS: i64 = 1_000_000;

/// Replay a recorded flight (`f t cx cy q gd`, `v t vn ve`, `y t yaw`; t in
/// sim milliseconds) through the monitor: the first time it disagrees and
/// goes lost, seconds.
fn replay(rec: &str) -> (Option<f64>, Option<f64>) {
    let mut m = FlowEkfMonitor::default();
    let (mut disagree, mut lost) = (None, None);
    for line in rec.lines() {
        let f: Vec<&str> = line.split_whitespace().collect();
        let n = |i: usize| f[i].parse::<f32>().unwrap();
        let t = f[1].parse::<i64>().unwrap() * MS;
        match f[0] {
            "v" => m.on_velocity(n(2), n(3), t),
            "y" => m.on_yaw(n(2), t),
            "f" => {
                m.on_flow([n(2), n(3)], f[4].parse().unwrap(), n(5), t);
                let s = t as f64 * 1e-9;
                match m.state() {
                    FlowEkf::Disagree => {
                        disagree.get_or_insert(s);
                    }
                    FlowEkf::Lost => {
                        disagree.get_or_insert(s);
                        lost.get_or_insert(s);
                    }
                    FlowEkf::Agree => {}
                }
            }
            _ => unreachable!("{line}"),
        }
    }
    (disagree, lost)
}

/// Mine drift and shaft over dusty floor (flow quality 18-68), with the
/// external-nav pose frozen along the tunnel: the EKF followed it while the
/// vehicle reached 5-6 m/s. Recorded EKF error at the alarm: 3.9 m (drift,
/// 65.8 s) and 4.3 m (shaft, 39.2 s); 45-50 m by the end without it.
#[test]
fn fc_flow_and_the_ekf_velocity_disagreeing_degrades_then_loses_the_pose() {
    let (d, l) = replay(include_str!("../tests/data/flow_ekf_drift.txt"));
    let (d, l) = (d.unwrap(), l.unwrap());
    assert!((65.0..66.5).contains(&d), "drift disagrees at {d}");
    assert!((l - d - 3.0).abs() < 0.2, "drift lost at {l}");
    let (d, l) = replay(include_str!("../tests/data/flow_ekf_shaft.txt"));
    let (d, l) = (d.unwrap(), l.unwrap());
    assert!((38.5..40.0).contains(&d), "shaft disagrees at {d}");
    assert!((l - d - 3.0).abs() < 0.2, "shaft lost at {l}");
}

/// A goto down a mine drift over a dusty floor (flow at quality 64 and
/// below in flight, some at 0), a planar LiDAR pose frozen along the drift from
/// about 30 s while the vehicle flew on at 1 to 2 m/s: the EKF followed
/// the frozen pose. The monitor parts the two within 3 s of the freeze and
/// loses the pose 3 s later.
#[test]
fn a_drift_route_on_a_frozen_pose_degrades_then_loses_it() {
    let (d, l) = replay(include_str!("../tests/data/flow_ekf_drift_route.txt"));
    let (d, l) = (d.unwrap(), l.unwrap());
    assert!((32.0..34.0).contains(&d), "disagrees at {d}");
    assert!((l - d - 3.0).abs() < 0.2, "lost at {l}");
}

/// A whole warehouse ext-nav flight and a whole wall-inspection flight,
/// the EKF tracking: never an alarm (their largest 3 s gap is 0.37 m).
#[test]
fn a_tracking_flight_raises_no_flow_disagreement() {
    for rec in [
        include_str!("../tests/data/flow_ekf_warehouse.txt"),
        include_str!("../tests/data/flow_ekf_wall.txt"),
    ] {
        assert_eq!(replay(rec), (None, None));
    }
}

fn steady(m: &mut FlowEkfMonitor, from_s: f64, to_s: f64, flow_fwd: f32, ekf_n: f32, q: u8) {
    let mut t = (from_s * 1e9) as i64;
    while t < (to_s * 1e9) as i64 {
        m.on_yaw(0.0, t);
        m.on_velocity(ekf_n, 0.0, t);
        // Forward (north at yaw 0) at `flow_fwd` over 2 m of ground.
        m.on_flow([0.0, flow_fwd / 2.0], q, 2.0, t);
        t += 100 * MS;
    }
}

#[test]
fn a_disagreement_clears_only_after_agreeing_for_a_while() {
    let mut m = FlowEkfMonitor::default();
    steady(&mut m, 0.0, 4.0, 2.0, 2.0, 40);
    assert_eq!(m.state(), FlowEkf::Agree);
    // The EKF stalls at 0.5 m/s while flow reads 2 m/s: 1.5 m/s apart, so
    // past 1.5 m one second into a full window.
    steady(&mut m, 4.0, 6.0, 2.0, 0.5, 40);
    assert_eq!(m.state(), FlowEkf::Disagree);
    steady(&mut m, 6.0, 9.5, 2.0, 0.5, 40);
    assert_eq!(m.state(), FlowEkf::Lost);
    // Agreeing again: the window empties of the stall, then it holds 3 s.
    steady(&mut m, 9.5, 14.0, 2.0, 2.0, 40);
    assert_eq!(m.state(), FlowEkf::Lost, "not yet clear");
    steady(&mut m, 14.0, 16.0, 2.0, 2.0, 40);
    assert_eq!(m.state(), FlowEkf::Agree);
}

#[test]
fn flow_of_quality_zero_or_without_ground_is_not_used() {
    let mut m = FlowEkfMonitor::default();
    steady(&mut m, 0.0, 8.0, 3.0, 0.0, 0);
    assert_eq!(m.state(), FlowEkf::Agree);
    let mut t = 0;
    while t < 8_000 * MS {
        m.on_yaw(0.0, t);
        m.on_velocity(0.0, 0.0, t);
        m.on_flow([0.0, 1.5], 200, 0.0, t);
        t += 100 * MS;
    }
    assert_eq!(m.state(), FlowEkf::Agree);
}

/// The one reason the tick reports: the monitor first, then the publisher's
/// estimator, then its FC IMU feed. A pose degraded by `RAW_IMU` under
/// 40 Hz says so (it said nothing before), and every key is one the tick
/// clears again.
#[test]
fn the_reason_falls_back_to_the_imu_feed() {
    let corridor = Some(PoseReason::CorridorUnmeasured);
    let starved = Some(ImuFeed::Starved);
    assert_eq!(
        health_reason(FlowEkf::Agree, None, starved),
        Some("imu_feed_starved")
    );
    assert_eq!(
        health_reason(FlowEkf::Agree, None, Some(ImuFeed::HeadingUnconfirmed)),
        Some("heading_unconfirmed")
    );
    assert_eq!(
        health_reason(FlowEkf::Agree, corridor, starved),
        Some("corridor_unmeasured")
    );
    assert_eq!(
        health_reason(FlowEkf::Disagree, corridor, starved),
        Some(FlowEkf::REASON)
    );
    // A wide sigma is informational: the IMU feed's reason outranks it.
    let wide = Some(PoseReason::PoseSigmaWide);
    assert_eq!(
        health_reason(FlowEkf::Agree, wide, starved),
        Some("imu_feed_starved")
    );
    assert_eq!(
        health_reason(FlowEkf::Agree, wide, None),
        Some("pose_sigma_wide")
    );
    assert_eq!(health_reason(FlowEkf::Agree, None, None), None);
    for feed in [
        ImuFeed::AttitudeStandIn,
        ImuFeed::Stalled,
        ImuFeed::Starved,
        ImuFeed::HeadingUnconfirmed,
        ImuFeed::NorthUnknown,
    ] {
        assert!(NAV_REASONS.contains(&feed.reason_key()), "{feed:?}");
    }
}

/// A seeded xorshift64* with Box-Muller normals, for flow noise.
struct Noise(u64);

impl Noise {
    fn new(seed: u64) -> Self {
        Self(
            seed.wrapping_mul(0x9E37_79B9_7F4A_7C15)
                .wrapping_add(1)
                .max(1),
        )
    }
    /// Uniform in (0, 1].
    fn unit(&mut self) -> f64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        ((x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) + 1) as f64 / (1u64 << 53) as f64
    }
    fn normal(&mut self) -> f64 {
        let (u1, u2) = (self.unit(), self.unit());
        (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
    }
}

/// One standard deviation of the flow rate's noise, rad/s, per axis and
/// sample: about 1.1 m/s at 1.9 m, a dusty floor at quality 20.
const DUSTY_SIGMA_RAD_S: f64 = 0.6;

/// A flight north at yaw 0, 1.9 m up, `secs` long: `LOCAL_POSITION_NED`
/// every 20 ms, `OPTICAL_FLOW` at 20 Hz with seeded white noise at
/// [`DUSTY_SIGMA_RAD_S`]. The ground velocity is `ground(t)`; the EKF's
/// follows it 0.3 s behind, less `freeze.1` m/s from `freeze.0` s on (a
/// frozen pose holding it back). Returns when the monitor first disagreed, s.
fn dusty_flight(
    seed: u64,
    secs: f64,
    ground: impl Fn(f64) -> f64,
    freeze: Option<(f64, f64)>,
) -> Option<f64> {
    const AGL: f64 = 1.9;
    let mut m = FlowEkfMonitor::default();
    let mut noise = Noise::new(seed);
    let mut est = 0.0;
    let mut sample = None;
    for tick in 0..(secs * 50.0) as i64 {
        let (now, t) = (tick * 20 * MS, tick as f64 * 0.02);
        let g = ground(t);
        est += (g - est) * 0.02 / 0.3;
        let held = freeze.filter(|(at, _)| t >= *at).map_or(0.0, |(_, v)| v);
        m.on_yaw(0.0, now);
        m.on_velocity((est - held) as f32, 0.0, now);
        if sample != Some(tick * 20 / 50) {
            sample = Some(tick * 20 / 50);
            let comp_y = g / AGL + DUSTY_SIGMA_RAD_S * noise.normal();
            let comp_x = DUSTY_SIGMA_RAD_S * noise.normal();
            m.on_flow([comp_x as f32, comp_y as f32], 20, AGL as f32, now);
            if m.state() != FlowEkf::Agree {
                return Some(t);
            }
        }
    }
    None
}

/// Stop, speed up at 1 m/s² to 1.6 m/s, cruise, slow down: a 7.6 s cycle.
fn stop_and_go(t: f64) -> f64 {
    match t % 7.6 {
        c if c < 2.0 => 0.0,
        c if c < 3.6 => c - 2.0,
        c if c < 6.0 => 1.6,
        c => 1.6 - (c - 6.0),
    }
}

/// Noisy flow over a dusty floor, its mean on the ground velocity, against
/// an EKF velocity that is right but for its lag: never a disagreement.
/// Each seed crossed the bare 1.5 m threshold (noise alone moves the 3 s
/// gap by about 0.4 m per axis at 20 Hz, the lag through the speed changes
/// by more).
#[test]
fn noisy_flow_around_a_correct_ekf_raises_no_disagreement() {
    for seed in 1..=3 {
        let at = dusty_flight(seed, 30.0, stop_and_go, None);
        assert_eq!(at, None, "seed {seed}: disagreed at {at:?} s");
    }
}

/// The same noisy flow at a steady 1.6 m/s, the EKF velocity frozen 1 m/s
/// short of it from 10 s on: the monitor still parts them within 3.5 s.
#[test]
fn a_frozen_pose_is_caught_through_noisy_flow() {
    for seed in 1..=3 {
        let at = dusty_flight(seed, 20.0, |_| 1.6, Some((10.0, 1.0)));
        let at = at.unwrap_or_else(|| panic!("seed {seed}: never caught"));
        eprintln!("seed {seed}: disagreed {:.2} s after the freeze", at - 10.0);
        assert!(
            (10.0..13.5).contains(&at),
            "seed {seed}: disagreed at {at} s"
        );
    }
}
