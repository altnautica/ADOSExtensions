//! At 25° pitch and 3 m above the floor, the RPLIDAR C1's forward beams meet the floor about 7 m ahead: inside
//! its 12 m range. Levelled, those points land on a line across the scan and pull the match; with the height
//! known they are dropped.

use ados_scan2d::scan::Motion;
use ados_scan2d::{Scan2d, Scan2dConfig, ScanInput};

const PITCH: f64 = 25.0 * std::f64::consts::PI / 180.0;

/// A level floor `h` below, a wall 20 m ahead (beyond range): the beams that tilt down hit the floor.
fn pitched_scan(h: f64) -> ScanInput {
    let mut s = ScanInput::default();
    for k in 0..500 {
        let phi = 2.0 * std::f64::consts::PI * k as f64 / 500.0;
        // Sensor-plane beam in the levelled frame: Ry(pitch) tilts +x down by `pitch`.
        let (x, y) = (phi.cos(), phi.sin());
        let down = x * PITCH.sin();
        if down <= 1e-6 {
            continue;
        }
        let r = h / down;
        if r > 12.0 {
            continue;
        }
        s.points.push([(r * x) as f32, (r * y) as f32, 0.0]);
        s.t_offset_ns.push(0);
    }
    s
}

/// A ceiling `above` the sensor seen by the beams that tilt up.
fn ceiling_scan(above: f64) -> ScanInput {
    let mut s = ScanInput::default();
    for k in 0..500 {
        let phi = 2.0 * std::f64::consts::PI * k as f64 / 500.0;
        let (x, y) = (phi.cos(), phi.sin());
        let up = -x * PITCH.sin();
        if up <= 1e-6 {
            continue;
        }
        let r = above / up;
        if r > 12.0 {
            continue;
        }
        s.points.push([(r * x) as f32, (r * y) as f32, 0.0]);
        s.t_offset_ns.push(0);
    }
    s
}

#[test]
fn floor_hits_are_dropped_when_the_height_is_known() {
    let cfg = Scan2dConfig::for_sensor(0.05, 12.0, &[]);
    let scan = pitched_scan(3.0);
    assert!(scan.points.len() > 100, "the pitched scan sees the floor");
    let blind = Scan2d::from_cloud(&scan, &cfg, &Motion { pitch: PITCH, ..Default::default() });
    assert_eq!(blind.points.len(), scan.points.len(), "no height: kept as today");
    let seen = Scan2d::from_cloud(
        &scan,
        &cfg,
        &Motion { pitch: PITCH, agl_m: Some(3.0), ..Default::default() },
    );
    assert!(seen.points.is_empty(), "{} floor points kept", seen.points.len());
}

/// On the ground, or with a height datum a little below the floor, the
/// margin is larger than the height: nothing level is dropped.
#[test]
fn a_level_scan_on_the_ground_keeps_every_point() {
    let cfg = Scan2dConfig::for_sensor(0.05, 12.0, &[]);
    let mut scan = ScanInput::default();
    for k in 0..360 {
        let phi = (k as f64).to_radians();
        scan.points.push([(4.0 * phi.cos()) as f32, (4.0 * phi.sin()) as f32, 0.0]);
    }
    for h in [-0.3, 0.0, 0.2] {
        let out = Scan2d::from_cloud(&scan, &cfg, &Motion { agl_m: Some(h), ..Default::default() });
        assert_eq!(out.points.len(), scan.points.len(), "height {h}");
    }
}

#[test]
fn ceiling_hits_are_dropped_when_the_ceiling_is_configured() {
    let mut cfg = Scan2dConfig::for_sensor(0.05, 12.0, &[]);
    // The sensor 1.5 m up under a 4 m ceiling.
    let scan = ceiling_scan(2.5);
    assert!(scan.points.len() > 100, "the pitched scan sees the ceiling");
    let m = Motion { pitch: PITCH, agl_m: Some(1.5), ..Default::default() };
    assert_eq!(Scan2d::from_cloud(&scan, &cfg, &m).points.len(), scan.points.len(), "no ceiling: kept");
    cfg.ceiling_m = Some(4.0);
    let out = Scan2d::from_cloud(&scan, &cfg, &m);
    assert!(out.points.is_empty(), "{} ceiling points kept", out.points.len());
}
