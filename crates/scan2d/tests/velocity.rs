//! Local SLAM with a measured ground velocity (a downward optical-flow
//! sensor over its range to the ground): along a direction the scans do not
//! constrain, the pose follows the measured motion.

mod common;

use std::f64::consts::PI;

use ados_scan2d::{ImuSample, LocalSlam, PoseOut, Scan2dConfig, ScanInput, VelocitySample};
use common::{rect, walk, walk_forced, Path, Rng, BEAMS, C1, PERIOD_NS};

fn cfg() -> Scan2dConfig {
    Scan2dConfig::for_sensor(0.05, 12.0, &[])
}

/// Feed `imu` and `vel` (each in time order) up to every scan's last return,
/// as the capture service does, then the scan; one pose per scan.
fn replay(scans: &[ScanInput], imu: &[ImuSample], vel: &[VelocitySample]) -> Vec<PoseOut> {
    replay_forced(scans, imu, &[], vel)
}

/// [`replay`], each IMU sample followed by its specific force from `force`
/// (by index; none past its end).
fn replay_forced(
    scans: &[ScanInput],
    imu: &[ImuSample],
    force: &[[f64; 3]],
    vel: &[VelocitySample],
) -> Vec<PoseOut> {
    replay_counted(scans, imu, force, vel)
        .into_iter()
        .map(|(p, _)| p)
        .collect()
}

/// [`replay_forced`], with the scans the matcher's map holds after each one
/// (`LocalSlam::inserted_scans`, from which the capture counts a new map).
fn replay_counted(
    scans: &[ScanInput],
    imu: &[ImuSample],
    force: &[[f64; 3]],
    vel: &[VelocitySample],
) -> Vec<(PoseOut, usize)> {
    let mut slam = LocalSlam::new(cfg());
    let (mut i, mut v) = (imu.iter().enumerate().peekable(), vel.iter().peekable());
    let feed_imu = |slam: &mut LocalSlam, (k, s): (usize, &ImuSample)| {
        slam.feed_imu(s);
        if let Some(f) = force.get(k) {
            slam.feed_specific_force(s.t_ns, *f);
        }
    };
    scans
        .iter()
        .map(|s| {
            let end = s.t_ns + s.t_offset_ns.iter().copied().max().unwrap_or(0) as u64;
            loop {
                let ti = i.peek().map(|x| x.1.t_ns).filter(|&t| t <= end);
                let tv = v.peek().map(|x| x.t_ns).filter(|&t| t <= end);
                match (ti, tv) {
                    (Some(a), Some(b)) if b < a => slam.feed_velocity(v.next().unwrap()),
                    (Some(_), _) => feed_imu(&mut slam, i.next().unwrap()),
                    (None, Some(_)) => slam.feed_velocity(v.next().unwrap()),
                    (None, None) => break,
                }
            }
            let p = slam.feed_scan(s);
            (p, slam.inserted_scans())
        })
        .collect()
}

/// A walker goes 14 m down a bare 2 m wide corridor from its closed end and
/// stops. The scans pin the walker across the corridor only; a velocity
/// measured at 50 Hz with 5 % noise carries the pose along it, where the
/// motion prior alone lags the walker.
#[test]
fn bare_corridor_follows_the_measured_velocity() {
    let world = vec![
        ([-2.0, -1.0], [200.0, -1.0]),
        ([-2.0, 1.0], [200.0, 1.0]),
        ([-2.0, -1.0], [-2.0, 1.0]),
    ];
    let path = Path::new(0.0, 0.0, 0.0)
        .wait(1.0)
        .speed_to(1.5, 0.75)
        .line(13.0)
        .speed_to(0.0, 0.75)
        .wait(2.0);
    let (scans, imu, truth) = walk(&world, &path, C1, 7);
    let mut rng = Rng::new(3);
    let vel: Vec<VelocitySample> = (0..=(path.duration_s() * 50.0) as u64)
        .map(|k| {
            let t = k as f64 * 0.02;
            let (a, b) = (path.at_time((t - 0.01).max(0.0)), path.at_time(t + 0.01));
            let d = (b.x - a.x).hypot(b.y - a.y) / (t + 0.01 - (t - 0.01).max(0.0));
            VelocitySample {
                t_ns: (t * 1e9).round() as u64,
                v: [d * (1.0 + 0.05 * rng.gauss()), 0.02 * rng.gauss()],
            }
        })
        .collect();
    let walked = truth.last().unwrap().x;
    let with = replay(&scans, &imu, &vel);
    let without = replay(&scans, &imu, &[]);
    let degenerate = with
        .iter()
        .filter(|p| p.score > 0.0 && p.degeneracy < cfg().degenerate_ratio)
        .count();
    let (end, end_without) = (with.last().unwrap(), without.last().unwrap());
    let across = with.iter().map(|p| p.y.abs()).fold(0.0, f64::max);
    eprintln!(
        "corridor: walked {walked:.2} m; with the velocity {:.2} m, without {:.2} m; \
         {degenerate} of {} scans degenerate, {across:.3} m across",
        end.x,
        end_without.x,
        with.len()
    );
    assert!(degenerate > with.len() / 2, "{degenerate} degenerate scans");
    assert!(
        (end.x - walked).abs() < 0.2,
        "ended at {:.3} m of {walked:.3}",
        end.x
    );
    assert!(across < 0.05, "{across} m across the corridor");
}

/// One recording of `tests/data/mine_drift_flow.txt`: IMU samples, measured
/// velocities, and revolutions with their true position and yaw.
struct Recording {
    imu: Vec<ImuSample>,
    vel: Vec<VelocitySample>,
    scans: Vec<(ScanInput, [f64; 2], f64)>,
}

/// Parse a `tests/data` flight (format in the file's header).
fn recorded(text: &str) -> Recording {
    let mut rec = Recording {
        imu: Vec::new(),
        vel: Vec::new(),
        scans: Vec::new(),
    };
    for l in text
        .lines()
        .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
    {
        let mut f = l.split_whitespace();
        let kind = f.next().expect("a line kind");
        let v: Vec<i64> = f.map(|x| x.parse().expect("an integer")).collect();
        let t_ns = v[0] as u64 * 1_000;
        match kind {
            "g" => rec.imu.push(ImuSample {
                t_ns,
                gyro: [v[1] as f64 * 1e-6, v[2] as f64 * 1e-6, v[3] as f64 * 1e-6],
                accel: [v[4] as f64 * 1e-3, v[5] as f64 * 1e-3, v[6] as f64 * 1e-3],
            }),
            "v" => rec.vel.push(VelocitySample {
                t_ns,
                v: [v[1] as f64 * 1e-3, v[2] as f64 * 1e-3],
            }),
            "s" => {
                let n = v[4] as usize;
                assert_eq!(v.len(), 5 + 2 * n, "return count");
                let (mut k, mut points, mut t_offset_ns) = (0u64, Vec::new(), Vec::new());
                for p in v[5..].chunks(2) {
                    k += p[0] as u64;
                    let az = k as f64 * 2.0 * PI / BEAMS as f64;
                    let r = p[1] as f64 * 1e-3;
                    points.push([(r * az.cos()) as f32, (r * az.sin()) as f32, 0.0]);
                    t_offset_ns.push((k * PERIOD_NS / BEAMS as u64) as u32);
                }
                let scan = ScanInput {
                    t_ns,
                    points,
                    t_offset_ns,
                };
                rec.scans.push((
                    scan,
                    [v[1] as f64 * 1e-3, v[2] as f64 * 1e-3],
                    v[3] as f64 * 1e-6,
                ));
            }
            other => panic!("unknown line kind `{other}`"),
        }
    }
    rec
}

/// Down a mine drift at up to 8 m/s: rough rock walls 5 m apart pin the
/// pose across the drift and barely along it. With the motion prior alone
/// the matched distance fell to 40 % of the true 27.9 m and the pose ended
/// 16.7 m off (the flight controller, fusing that pose, sped up until its
/// EKF failed). With the flight controller's flow velocity it is 5 % short
/// (the flow itself reads 3 % long here; the matches in the weakly
/// constrained scans pull back), and the scans the drift leaves degenerate
/// move with the measured motion.
#[test]
fn mine_drift_keeps_its_length_on_the_flow_velocity() {
    let rec = recorded(include_str!("data/mine_drift_flow.txt"));
    assert_eq!(rec.scans.len(), 107);
    let scans: Vec<ScanInput> = rec.scans.iter().map(|s| s.0.clone()).collect();
    let est = replay(&scans, &rec.imu, &rec.vel);
    let truth = local_truth(&rec);
    let (t_end, e_end) = (truth.last().unwrap(), est.last().unwrap());
    let length = t_end[0].hypot(t_end[1]);
    let axis = [t_end[0] / length, t_end[1] / length];
    let along = |x: f64, y: f64| x * axis[0] + y * axis[1];
    let matched = along(e_end.x, e_end.y);
    let end_err = (e_end.x - t_end[0]).hypot(e_end.y - t_end[1]);
    // Along-drift steps of the scans rated degenerate.
    let ratio = cfg().degenerate_ratio;
    let (mut deg_truth, mut deg_est, mut degenerate) = (0.0, 0.0, 0);
    for k in 1..est.len() {
        if est[k].score > 0.0 && est[k].degeneracy < ratio {
            degenerate += 1;
            deg_truth += along(truth[k][0] - truth[k - 1][0], truth[k][1] - truth[k - 1][1]);
            deg_est += along(est[k].x - est[k - 1].x, est[k].y - est[k - 1].y);
        }
    }
    eprintln!(
        "mine drift: {length:.2} m flown, {matched:.2} m matched along it ({:+.1} %), \
         ends {end_err:.2} m off; {degenerate} degenerate scans moved {deg_est:.2} m of {deg_truth:.2}",
        100.0 * (matched / length - 1.0)
    );
    assert!(degenerate >= 5, "{degenerate} degenerate scans");
    assert!(
        (matched / length - 1.0).abs() < 0.08,
        "matched {matched:.2} m of {length:.2}"
    );
    assert!(end_err < 2.5, "ends {end_err:.2} m off");
    assert!(
        (deg_est / deg_truth - 1.0).abs() < 0.05,
        "degenerate scans moved {deg_est:.2} m of {deg_truth:.2}"
    );
}

/// A recording's true positions in the local frame: the LiDAR frame at the
/// first scan, levelled.
fn local_truth(rec: &Recording) -> Vec<[f64; 2]> {
    let (p0, yaw0) = (rec.scans[0].1, rec.scans[0].2);
    let (s, c) = yaw0.sin_cos();
    rec.scans
        .iter()
        .map(|(_, p, _)| {
            let (dx, dy) = (p[0] - p0[0], p[1] - p0[1]);
            [c * dx + s * dy, -s * dx + c * dy]
        })
        .collect()
}

/// The same drift with nothing measuring the motion along it (the flow
/// sensor's samples all dropped): no pose the capture would call good is
/// more than 2 m off. The drift goes unmeasured and loses the pose; where
/// the walls measure it again the map starts over, and the capture places
/// the new map, and each pose while it warms up lost, at the last pose it
/// trusted moved by the flight controller's own displacement since (the
/// truth's stands in for it here). Health is the capture's
/// (`world-engine-capture` `health_of`): lost before the map holds 10 scans
/// and while unmeasured; good when matched, not degenerate, the heading
/// vouched for and the flow not against it. Before, the pose coasted to
/// rest and re-locked into the old map wherever it froze, good while
/// metres behind.
#[test]
fn mine_drift_without_flow_is_never_good_far_off() {
    let rec = recorded(include_str!("data/mine_drift_flow.txt"));
    let scans: Vec<ScanInput> = rec.scans.iter().map(|s| s.0.clone()).collect();
    let est = replay_counted(&scans, &rec.imu, &[], &[]);
    let truth = local_truth(&rec);
    let ratio = cfg().degenerate_ratio;
    let lost_s = ados_scan2d::UNMEASURED_FREE_LOST_S;
    let (mut offset, mut trusted, mut seating) = ([0.0; 2], None::<(usize, [f64; 2])>, false);
    let (mut good, mut lost, mut restarts, mut worst) = (0, 0, 0, 0.0f64);
    for (k, (p, inserted)) in est.iter().enumerate() {
        let is_lost = *inserted < 10
            || p.heading_unconfirmed
            || p.unmeasured_free_s >= lost_s
            || (p.unmeasured_free_s > 0.0 && p.sigma_xy_m > 1.0);
        restarts += usize::from(p.map_restarted);
        seating = (seating || p.map_restarted) && is_lost;
        if let (true, Some((j, at))) = (seating, trusted) {
            let moved = [truth[k][0] - truth[j][0], truth[k][1] - truth[j][1]];
            offset = [at[0] + moved[0] - p.x, at[1] + moved[1] - p.y];
        }
        let placed = [p.x + offset[0], p.y + offset[1]];
        if !is_lost && p.unmeasured_free_s == 0.0 {
            trusted = Some((k, placed));
        }
        let is_good =
            !is_lost && p.score > 0.0 && p.degeneracy >= ratio && p.heading_ok && !p.flow_disagrees;
        lost += usize::from(is_lost);
        if is_good {
            good += 1;
            worst = worst.max((placed[0] - truth[k][0]).hypot(placed[1] - truth[k][1]));
        }
    }
    eprintln!(
        "mine drift without flow: {good} good, {lost} lost of {}; {restarts} restarts; worst good {worst:.2} m off",
        est.len()
    );
    assert!(
        lost > 10,
        "the unmeasured drift loses the pose ({lost} lost)"
    );
    assert!(good > 10, "{good} good scans");
    assert!(worst < 2.0, "good while {worst:.2} m off");
}

/// The walker's measured velocity along `path` at 50 Hz in its own frame,
/// with 5 % noise: forward speed times `scale`, plus `sideways` m/s.
fn flow(path: &Path, scale: f64, sideways: f64, seed: u64) -> Vec<VelocitySample> {
    let mut rng = Rng::new(seed);
    (0..=(path.duration_s() * 50.0) as u64)
        .map(|k| {
            let t = k as f64 * 0.02;
            let (a, b) = (path.at_time((t - 0.01).max(0.0)), path.at_time(t + 0.01));
            let d = (b.x - a.x).hypot(b.y - a.y) / (t + 0.01 - (t - 0.01).max(0.0));
            VelocitySample {
                t_ns: (t * 1e9).round() as u64,
                v: [
                    d * scale * (1.0 + 0.05 * rng.gauss()),
                    sideways + 0.02 * rng.gauss(),
                ],
            }
        })
        .collect()
}

fn bare_corridor() -> (Vec<common::Seg>, Path) {
    let world = vec![
        ([-2.0, -1.0], [200.0, -1.0]),
        ([-2.0, 1.0], [200.0, 1.0]),
        ([-2.0, -1.0], [-2.0, 1.0]),
    ];
    let path = Path::new(0.0, 0.0, 0.0)
        .wait(1.0)
        .speed_to(2.0, 1.0)
        .line(20.0)
        .speed_to(0.0, 1.0)
        .wait(1.0);
    (world, path)
}

/// A bare 2 m wide corridor from its closed end at -2 m to a 10 m by 10 m
/// room at 30 m, and a walk 30 m down it into the room: the room's walls
/// measure the walk along the corridor again.
fn corridor_into_a_room() -> (Vec<common::Seg>, Path) {
    let world = vec![
        ([-2.0, -1.0], [30.0, -1.0]),
        ([-2.0, 1.0], [30.0, 1.0]),
        ([-2.0, -1.0], [-2.0, 1.0]),
        ([30.0, 1.0], [30.0, 5.0]),
        ([30.0, 5.0], [40.0, 5.0]),
        ([40.0, 5.0], [40.0, -5.0]),
        ([40.0, -5.0], [30.0, -5.0]),
        ([30.0, -5.0], [30.0, -1.0]),
    ];
    let path = Path::new(0.0, 0.0, 0.0)
        .wait(1.0)
        .speed_to(2.0, 1.0)
        .line(30.0)
        .speed_to(0.0, 1.0)
        .wait(1.0);
    (world, path)
}

/// Down a bare corridor with nothing measuring the motion along it, the
/// pose there is the motion prior coasting to rest while the walker goes
/// on at 2 m/s. Its sigma grows at the speed the corridor went free with,
/// it reports how long it has gone unmeasured, and past a second it stays
/// unmeasured (lost) for as long as the matches stay degenerate: no new map
/// is started inside the corridor, where it would freeze again. Once the
/// room measures the walk again, the map starts over at that scan, once,
/// rather than the pose re-locking into the old map wherever it froze.
/// Before, the sigma stayed at the anchor's 0.05 m the whole way and the
/// pose re-locked metres behind the walker.
#[test]
fn a_corridor_without_measured_velocity_stays_lost_until_measured_again() {
    let (world, path) = corridor_into_a_room();
    let (scans, imu, truth) = walk(&world, &path, C1, 11);
    let est = replay(&scans, &imu, &[]);
    let lost = ados_scan2d::UNMEASURED_FREE_LOST_S;
    let first = est
        .iter()
        .position(|p| p.unmeasured_free_s >= lost)
        .expect("the corridor goes unmeasured");
    let sigma = est[..=first]
        .iter()
        .map(|p| p.sigma_xy_m)
        .fold(0.0, f64::max);
    let again = first
        + est[first..]
            .iter()
            .position(|p| p.unmeasured_free_s == 0.0)
            .expect("the recess measures the corridor again");
    let p = &est[again];
    let err = (p.x - truth[again].x).hypot(p.y - truth[again].y);
    eprintln!(
        "corridor without flow: lost at {:.1} m, measured again at {:.1} m, {err:.2} m off there, sigma {sigma:.2} m",
        truth[first].x, truth[again].x
    );
    assert!(sigma >= 0.5, "sigma {sigma:.2} m while the pose froze");
    assert!(
        est[first..again]
            .iter()
            .all(|p| p.unmeasured_free_s >= lost),
        "lost for as long as the corridor stays unmeasured"
    );
    assert!(err > 3.0, "the old map froze {err:.2} m behind the walker");
    assert!(
        p.map_restarted,
        "the map starts over where the corridor is measured again"
    );
    assert_eq!(
        est.iter().filter(|p| p.map_restarted).count(),
        1,
        "once per unmeasured stretch, never inside it"
    );
}

/// The same corridor with a measured velocity that agrees with the walls
/// is followed and never goes unmeasured. One that reads 2 m/s across the
/// corridor while the walls hold the walker still across it is not
/// followed: the corridor goes unmeasured as without one.
#[test]
fn a_measured_velocity_against_the_walls_is_not_followed() {
    let (world, path) = bare_corridor();
    let (scans, imu, truth) = walk(&world, &path, C1, 11);
    let good = replay(&scans, &imu, &flow(&path, 1.0, 0.0, 5));
    let worst = good.iter().map(|p| p.unmeasured_free_s).fold(0.0, f64::max);
    let end = good.last().unwrap();
    let walked = truth.last().unwrap().x;
    assert_eq!(worst, 0.0, "agreeing flow measures the corridor");
    assert!(
        (end.x - walked).abs() < 0.3,
        "ended at {:.2} of {walked:.2}",
        end.x
    );
    let bad = replay(&scans, &imu, &flow(&path, 1.0, 2.0, 5));
    assert!(
        bad.iter()
            .any(|p| p.unmeasured_free_s >= ados_scan2d::UNMEASURED_FREE_LOST_S),
        "flow across the walls is not taken for the corridor's motion"
    );
}

/// In a room whose walls and a pillar pin the pose, a flow sensor reading
/// twice the true speed leaves the matched displacement more than 1.5 m
/// from its own over 3 s: the pose reports it, and keeps reporting it while
/// the two disagree. Once the drone holds still the doubled flow reads zero
/// with the scans; agreeing within 1 m for 3 s with every direction pinned
/// clears it, with no map restart. Before, it stayed flagged until landing.
/// The true flow raises nothing.
#[test]
fn a_pose_that_slid_against_the_flow_is_flagged_until_they_agree_again() {
    let mut world = rect(-1.0, -2.0, 9.0, 2.0);
    world.extend(rect(4.0, 0.6, 4.4, 1.0));
    let path = Path::new(0.0, 0.0, 0.0)
        .wait(1.0)
        .speed_to(1.2, 0.6)
        .line(5.0)
        .speed_to(0.0, 0.6)
        .wait(9.0);
    let (scans, imu, _) = walk(&world, &path, C1, 13);
    let true_flow = replay(&scans, &imu, &flow(&path, 1.0, 0.0, 7));
    assert!(
        true_flow.iter().all(|p| !p.flow_disagrees),
        "true flow agrees"
    );
    let doubled = replay(&scans, &imu, &flow(&path, 2.0, 0.0, 7));
    let first = doubled
        .iter()
        .position(|p| p.flow_disagrees)
        .expect("doubled flow disagrees");
    let cleared = first
        + doubled[first..]
            .iter()
            .position(|p| !p.flow_disagrees)
            .expect("agreeing again clears it");
    let held_s = (doubled[cleared].t_ns - doubled[first].t_ns) as f64 * 1e-9;
    assert!(held_s >= 3.0, "held {held_s:.1} s");
    assert!(
        doubled[cleared..].iter().all(|p| !p.flow_disagrees),
        "stays clear"
    );
    assert!(doubled.iter().all(|p| !p.map_restarted), "no restart");
}

/// Down a bare corridor at 1.5 m/s, a flow sensor losing the ground reads
/// near zero (its quality still over the floor). Along the corridor's axis
/// the walls cannot judge it, so it agreed and the pose froze, never
/// unmeasured. It now stalls: the stretch is unmeasured from the first
/// degenerate scan, named past 0.3 s and lost past 1 s. Agreeing flow is
/// followed as before.
#[test]
fn a_flow_stalling_along_the_corridor_leaves_it_unmeasured() {
    let (world, _) = bare_corridor();
    let path = Path::new(0.0, 0.0, 0.0)
        .wait(1.0)
        .speed_to(1.5, 1.0)
        .line(20.0)
        .speed_to(0.0, 1.0)
        .wait(1.0);
    let (scans, imu, truth) = walk(&world, &path, C1, 11);
    let ratio = cfg().degenerate_ratio;

    let good = replay(&scans, &imu, &flow(&path, 1.0, 0.0, 5));
    let worst = good.iter().map(|p| p.unmeasured_free_s).fold(0.0, f64::max);
    let (end, walked) = (good.last().unwrap(), truth.last().unwrap().x);
    assert_eq!(worst, 0.0, "agreeing flow measures the corridor");
    assert!(
        (end.x - walked).abs() < 0.3,
        "ended at {:.2} of {walked:.2}",
        end.x
    );

    let stalled = replay(&scans, &imu, &flow(&path, 0.0, 0.0, 5));
    let lost = stalled
        .iter()
        .position(|p| p.unmeasured_free_s >= ados_scan2d::UNMEASURED_FREE_LOST_S)
        .expect("the stalled stretch loses the pose");
    // The stretch that lost it starts at its first degenerate scan (which
    // reports 0 s): the stalled flow covered none of it.
    let start = (0..lost)
        .rev()
        .find(|&i| stalled[i].unmeasured_free_s == 0.0)
        .expect("a scan starts the stretch");
    assert!(stalled[start].degeneracy < ratio);
    assert!(
        stalled[start - 1].degeneracy >= ratio,
        "a degenerate scan went uncounted"
    );
    let after = |i: usize| (stalled[i].t_ns - stalled[start].t_ns) as f64 * 1e-9;
    let named = start
        + stalled[start..]
            .iter()
            .position(|p| p.unmeasured_free_s >= 0.3)
            .expect("named");
    let off = (stalled[lost].x - truth[lost].x).abs();
    eprintln!(
        "stalled flow: degenerate at {:.1} m, named after {:.2} s, lost after {:.2} s, {off:.2} m off",
        truth[start].x,
        after(named),
        after(lost)
    );
    assert!(after(named) <= 0.4, "named after {:.2} s", after(named));
    assert!(after(lost) <= 1.1, "lost after {:.2} s", after(lost));
}

/// Down the 2 m corridor at 5 m/s, a hard brake at 6 m/s² to rest with
/// sound flow: the flow drops to zero ahead of the matched chord, which
/// lags it by its window, yet no faster than braking explains, so it is not
/// taken for a stalled sensor and the corridor stays measured.
#[test]
fn a_hard_brake_with_sound_flow_stays_measured() {
    let (world, _) = bare_corridor();
    let path = Path::new(0.0, 0.0, 0.0)
        .wait(1.0)
        .speed_to(5.0, 5.0)
        .line(15.0)
        .speed_to(0.0, 25.0 / 12.0)
        .wait(2.0);
    let (scans, imu, truth, force) = walk_forced(&world, &path, C1, 11);
    let est = replay_forced(&scans, &imu, &force, &flow(&path, 1.0, 0.0, 5));
    let worst = est.iter().map(|p| p.unmeasured_free_s).fold(0.0, f64::max);
    let (end, walked) = (est.last().unwrap(), truth.last().unwrap().x);
    eprintln!(
        "hard brake: ended at {:.2} of {walked:.2}, worst unmeasured {worst:.2} s",
        end.x
    );
    assert_eq!(worst, 0.0, "braking flow keeps the corridor measured");
    assert!(
        (end.x - walked).abs() < 0.3,
        "ended at {:.2} of {walked:.2}",
        end.x
    );
}

/// Down the 2 m corridor at 1.5 m/s on sound flow, the flow sensor loses
/// the ground 9 s in and reads zero from then on (its quality still over
/// the floor). Followed, the zero would decay the chord inside its window
/// before a stop's time ran out, and the pose froze unflagged. The drop is
/// faster than braking explains: the corridor goes unmeasured at once,
/// named after 0.3 s and lost after 1 s.
#[test]
fn a_flow_stalling_midway_down_the_corridor_leaves_it_unmeasured() {
    let (world, _) = bare_corridor();
    let path = Path::new(0.0, 0.0, 0.0)
        .wait(1.0)
        .speed_to(1.5, 1.0)
        .line(20.0)
        .speed_to(0.0, 1.0)
        .wait(1.0);
    let (scans, imu, truth) = walk(&world, &path, C1, 11);
    let stall_ns = 9_000_000_000u64;
    let vel: Vec<VelocitySample> = flow(&path, 1.0, 0.0, 5)
        .into_iter()
        .map(|s| VelocitySample {
            v: if s.t_ns >= stall_ns { [0.0, 0.0] } else { s.v },
            ..s
        })
        .collect();
    let est = replay(&scans, &imu, &vel);
    let since = |i: usize| (est[i].t_ns as f64 - stall_ns as f64) * 1e-9;
    assert!(
        est.iter()
            .filter(|p| p.t_ns < stall_ns)
            .all(|p| p.unmeasured_free_s == 0.0),
        "measured while the flow is sound"
    );
    let named = est
        .iter()
        .position(|p| p.unmeasured_free_s >= 0.3)
        .expect("the stalled stretch is named");
    let lost = est
        .iter()
        .position(|p| p.unmeasured_free_s >= ados_scan2d::UNMEASURED_FREE_LOST_S)
        .expect("the stalled stretch loses the pose");
    let off = (est[lost].x - truth[lost].x).abs();
    eprintln!(
        "midway stall: named {:.2} s after, lost {:.2} s after, {off:.2} m off",
        since(named),
        since(lost)
    );
    // One scan to see it, 0.3 s to name it, a stop from 1.5 m/s at most.
    let bound = 0.1 + 0.3 + 1.5 / 8.0 + 0.1;
    assert!(since(named) <= bound, "named {:.2} s after", since(named));
    assert!(
        since(lost) <= bound + 0.7,
        "lost {:.2} s after",
        since(lost)
    );
}

/// The walker stops 14 m down the corridor and hovers 15 s on flow as noisy
/// as a dusty floor reads (0.85 m/s a sample on each axis). Single noisy
/// samples falling to zero from a noisy high are no stalled sensor: the
/// hover stays measured.
#[test]
fn a_noisy_hover_in_the_corridor_stays_measured() {
    let (world, _) = bare_corridor();
    let path = Path::new(0.0, 0.0, 0.0)
        .wait(1.0)
        .speed_to(1.5, 1.0)
        .line(14.0)
        .speed_to(0.0, 1.0)
        .wait(15.0);
    let (scans, imu, _, force) = walk_forced(&world, &path, C1, 11);
    let mut rng = Rng::new(17);
    let mut vel = flow(&path, 1.0, 0.0, 5);
    for s in vel.iter_mut() {
        s.v = [s.v[0] + 0.85 * rng.gauss(), s.v[1] + 0.85 * rng.gauss()];
    }
    let est = replay_forced(&scans, &imu, &force, &vel);
    let t_stop = 1.0 + 1.0 + 14.0 / 1.5 + 1.0;
    let hover: Vec<_> = est
        .iter()
        .filter(|p| p.t_ns as f64 * 1e-9 > t_stop + 1.0)
        .collect();
    let lost = hover
        .iter()
        .filter(|p| p.unmeasured_free_s >= ados_scan2d::UNMEASURED_FREE_LOST_S)
        .count();
    eprintln!("noisy hover: {lost} of {} hover scans lost", hover.len());
    assert!(hover.len() > 100);
    assert_eq!(lost, 0, "the noisy hover is never lost");
}

/// Down the 2 m corridor at 1.5 m/s on flow as noisy as a dusty floor reads
/// (0.85 m/s a sample on each axis), the sensor loses the ground 9 s in and
/// reads that noise around zero from then on: from the flow alone a noisy
/// hover. The accelerometer shows no braking, so the flow is taken for
/// stalled: the corridor goes unmeasured within 0.5 s and the pose is lost
/// within 1.5 s. Before, it was never unmeasured and ended 11 m off.
#[test]
fn a_noisy_flow_stalling_midway_is_caught_by_the_accelerometer() {
    let (world, _) = bare_corridor();
    let path = Path::new(0.0, 0.0, 0.0)
        .wait(1.0)
        .speed_to(1.5, 1.0)
        .line(20.0)
        .speed_to(0.0, 1.0)
        .wait(1.0);
    let (scans, imu, _, force) = walk_forced(&world, &path, C1, 11);
    let stall_ns = 9_000_000_000u64;
    let noisy = |stall: bool| {
        let mut rng = Rng::new(17);
        let mut vel = flow(&path, 1.0, 0.0, 5);
        for s in vel.iter_mut() {
            let fwd = if stall && s.t_ns > stall_ns {
                0.0
            } else {
                s.v[0]
            };
            s.v = [fwd + 0.85 * rng.gauss(), s.v[1] + 0.85 * rng.gauss()];
        }
        replay_forced(&scans, &imu, &force, &vel)
    };
    let control = noisy(false);
    let lost_control = control
        .iter()
        .filter(|p| p.unmeasured_free_s >= ados_scan2d::UNMEASURED_FREE_LOST_S)
        .count();
    let est = noisy(true);
    let since = |i: usize| (est[i].t_ns as f64 - stall_ns as f64) * 1e-9;
    let first = est
        .iter()
        .position(|p| p.t_ns > stall_ns && p.unmeasured_free_s > 0.0)
        .expect("the stalled flow leaves the corridor unmeasured");
    let lost = est
        .iter()
        .position(|p| {
            p.t_ns > stall_ns && p.unmeasured_free_s >= ados_scan2d::UNMEASURED_FREE_LOST_S
        })
        .expect("the stalled flow loses the pose");
    eprintln!(
        "noisy stall: unmeasured {:.2} s after, lost {:.2} s after; control {lost_control} of {} lost",
        since(first),
        since(lost),
        control.len()
    );
    assert_eq!(lost_control, 0, "sound noisy flow is never lost");
    assert!(
        since(first) <= 0.5,
        "unmeasured {:.2} s after",
        since(first)
    );
    assert!(since(lost) <= 1.5, "lost {:.2} s after", since(lost));
}
