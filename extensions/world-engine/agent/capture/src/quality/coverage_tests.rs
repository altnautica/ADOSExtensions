use super::*;
use crate::lidar_scan::LidarScan;

/// OpenCV camera looking along world +x (ENU east), y down = world −z.
const LOOK_EAST: [f64; 9] = [0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0];
const IDENTITY: [f64; 9] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
const S: u64 = 1_000_000_000;

#[test]
fn a_frustum_hits_voxels_ahead_and_none_behind() {
    let mut c = Coverage::new();
    c.observe_camera(&LOOK_EAST, [0.1, 0.1, 1.5], 0.6, 0.4, None);
    let n = c.voxels();
    assert!(n > 40, "{n}");
    let v = c.view();
    assert!(
        v.cells.iter().all(|[x, _, _]| *x >= 0),
        "nothing behind the camera"
    );
    assert!(v.cells.iter().any(|[x, _, _]| *x >= 15), "reaches ~4 m");
    // Re-observing the same view adds nothing.
    c.observe_camera(&LOOK_EAST, [0.1, 0.1, 1.5], 0.6, 0.4, None);
    assert_eq!(c.voxels(), n);
}

#[test]
fn trajectory_heading_room_share_and_loop_closure() {
    let mut c = Coverage::new();
    // Walk a 6 m x 4 m rectangle twice at one sample a second, looking
    // forward: the second lap passes the first more than 30 s later.
    let corners = [[0.0, 0.0], [6.0, 0.0], [6.0, 4.0], [0.0, 4.0], [0.0, 0.0]];
    let mut t = 0;
    for _ in 0..2 {
        for w in corners.windows(2) {
            for k in 0..=20 {
                let s = k as f64 / 20.0;
                let p = [
                    w[0][0] + s * (w[1][0] - w[0][0]),
                    w[0][1] + s * (w[1][1] - w[0][1]),
                    1.5,
                ];
                c.observe_body(&IDENTITY, p, t);
                c.observe_camera(&LOOK_EAST, p, 0.6, 0.4, None);
                t += S;
            }
        }
    }
    let s = c.summary();
    assert!(s.loop_closed);
    assert!(!s.close_loop_prompt);
    assert!(s.distance_from_start_m.unwrap() < 0.3);
    let pct = s.room_pct.unwrap();
    assert!(pct > 30.0 && pct <= 100.0, "{pct}");
    let v = c.view();
    assert_eq!(v.heading_deg, Some(90.0));
    assert_eq!(v.cell_m, 0.25);
    assert!(v.trajectory.len() > 20);
    c.mark_room();
    assert!(c.room_pct().is_some());
}

#[test]
fn revisit_prompt_repeats_every_40m() {
    let mut c = Coverage::new();
    // Straight out along x at 1 m/s, a pose every 0.5 m.
    let at = |c: &mut Coverage, x: f64, t_s: f64| {
        c.observe_body(&IDENTITY, [x, 0.0, 1.5], (t_s * 1e9) as u64);
        c.summary()
    };
    for k in 0..=79 {
        let x = k as f64 * 0.5;
        let s = at(&mut c, x, x);
        assert!(!s.close_loop_prompt, "no prompt before 40 m (at {x} m)");
    }
    let s = at(&mut c, 40.0, 40.0);
    assert!(s.close_loop_prompt);
    assert_eq!(s.close_loop_prompts, 1);
    let s = at(&mut c, 60.0, 60.0);
    assert!(s.close_loop_prompt, "held until a revisit");
    assert_eq!(s.close_loop_prompts, 1);
    for k in 121..=240 {
        let x = k as f64 * 0.5;
        at(&mut c, x, x);
    }
    let s = c.summary();
    assert_eq!(s.close_loop_prompts, 3, "raised again at 80 m and 120 m");
    assert!(!s.loop_closed);
    // Turn back: the poses just walked are too recent to count, the ones
    // passed more than 30 s ago close the loop and clear the prompt.
    let mut t = 120.0;
    let mut x = 120.0;
    let mut closed_at = None;
    while x > 90.0 {
        x -= 0.5;
        t += 0.5;
        let s = at(&mut c, x, t);
        if s.loop_closed && closed_at.is_none() {
            closed_at = Some(x);
            assert!(!s.close_loop_prompt);
        }
    }
    let closed_at = closed_at.expect("the walk back closes a loop");
    assert!((104.0..=106.0).contains(&closed_at), "{closed_at}");
    assert_eq!(c.summary().close_loop_prompts, 3);
}

/// A planar scan of `n` points `range` m out, over azimuths `[from, to)`
/// degrees.
fn ring(range: f32, from: f32, to: f32, n: usize) -> LidarScan {
    let points = (0..n)
        .map(|k| {
            let az = (from + (to - from) * k as f32 / n as f32).to_radians();
            [range * az.cos(), range * az.sin(), 0.0]
        })
        .collect();
    LidarScan {
        sensor_id: "lidar0".into(),
        t_ns: 5 * S,
        points,
        t_offset_ns: vec![0; n],
        intensity: vec![0; n],
    }
}

#[test]
fn coverage_rays_stop_at_lidar_range() {
    let pos = [0.1, 0.1, 1.5];
    let max_x = |c: &Coverage| c.view().cells.iter().map(|[x, _, _]| *x).max().unwrap();

    // A wall 1.5 m away all round stops every ray there.
    let walls = LidarDepth::from_scan(&ring(1.5, 0.0, 360.0, 500), Some(&IDENTITY));
    assert!((walls.median_m.unwrap() - 1.5).abs() < 1e-6);
    assert!(walls.fresh_at(5 * S + S / 2) && !walls.fresh_at(7 * S));
    let mut near = Coverage::new();
    near.observe_camera(&LOOK_EAST, pos, 0.6, 0.4, Some(&walls));
    assert!(max_x(&near) <= 6, "{}", max_x(&near));

    // Returns only behind the camera leave the rays ahead at 4 m.
    let behind = LidarDepth::from_scan(&ring(1.5, 150.0, 210.0, 100), Some(&IDENTITY));
    assert_eq!(behind.range_at(0.0), None);
    assert!(behind.range_at(185.0).is_some());
    let mut open = Coverage::new();
    open.observe_camera(&LOOK_EAST, pos, 0.6, 0.4, Some(&behind));
    assert!(max_x(&open) >= 15);

    // A far wall never lengthens a ray past 4 m.
    let far = LidarDepth::from_scan(&ring(10.0, 0.0, 360.0, 500), Some(&IDENTITY));
    let mut far_c = Coverage::new();
    far_c.observe_camera(&LOOK_EAST, pos, 0.6, 0.4, Some(&far));
    assert_eq!(max_x(&far_c), max_x(&open));

    // Without the scan's world attitude there are no sectors.
    let unposed = LidarDepth::from_scan(&ring(1.5, 0.0, 360.0, 500), None);
    assert_eq!(unposed.range_at(0.0), None);
    assert!(unposed.median_m.is_some());
}
