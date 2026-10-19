//! Two rooms sharing a 0.1 m thick wall with one 0.9 m door.
//!
//! Room A interior is x 0..5, y 0..4 (20 m²); room B interior is x 5.1..8.1,
//! y 0..4 (12 m²). Both faces of the shared wall are scanned (x = 5 and
//! x = 5.1), the outer walls only from inside. Room polygons are snapped onto
//! the nearest wall face, so the reported areas are interior (net) areas; the
//! 5 % tolerance absorbs the ±1 cm wall noise and a half-cell (2.5 cm) raster
//! error on any edge that does not snap.

use super::super::floorplan::{build, to_svg, FloorplanOptions, RoomTrack};
use super::super::geom::Rng;
use super::super::PointCloud;
use super::jitter;

const H: f64 = 2.6;
const DOOR: (f64, f64) = (1.5, 2.4);
const DOOR_TOP: f64 = 2.1;

/// Vertical wall face from `a` to `b`, sampled every 2 cm along and 5 cm up,
/// with ±1 cm noise across the face; `door` cuts an opening (t range along).
fn wall(
    pts: &mut Vec<[f64; 3]>,
    a: [f64; 2],
    b: [f64; 2],
    door: Option<(f64, f64)>,
    rng: &mut Rng,
) {
    let len = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
    let (u, n) = (
        [(b[0] - a[0]) / len, (b[1] - a[1]) / len],
        [-(b[1] - a[1]) / len, (b[0] - a[0]) / len],
    );
    let steps = (len / 0.02) as usize;
    for s in 0..=steps {
        let t = s as f64 * 0.02;
        for k in 0..=(H / 0.05) as usize {
            let z = k as f64 * 0.05;
            if let Some((t0, t1)) = door {
                if t > t0 && t < t1 && z < DOOR_TOP {
                    continue;
                }
            }
            let e = jitter(rng, 0.01);
            pts.push([
                a[0] + u[0] * t + n[0] * e,
                a[1] + u[1] * t + n[1] * e,
                z + jitter(rng, 0.005),
            ]);
        }
    }
}

fn plane(pts: &mut Vec<[f64; 3]>, x: (f64, f64), y: (f64, f64), z: f64, rng: &mut Rng) {
    let (nx, ny) = (((x.1 - x.0) / 0.05) as usize, ((y.1 - y.0) / 0.05) as usize);
    for i in 0..=nx {
        for j in 0..=ny {
            pts.push([
                x.0 + i as f64 * 0.05,
                y.0 + j as f64 * 0.05,
                z + jitter(rng, 0.005),
            ]);
        }
    }
}

fn scene() -> PointCloud {
    let mut rng = Rng::new(11);
    let mut pts = Vec::new();
    let r = &mut rng;
    wall(&mut pts, [0.0, 0.0], [8.1, 0.0], None, r);
    wall(&mut pts, [0.0, 4.0], [8.1, 4.0], None, r);
    wall(&mut pts, [0.0, 0.0], [0.0, 4.0], None, r);
    wall(&mut pts, [8.1, 0.0], [8.1, 4.0], None, r);
    wall(&mut pts, [5.0, 0.0], [5.0, 4.0], Some(DOOR), r);
    wall(&mut pts, [5.1, 0.0], [5.1, 4.0], Some(DOOR), r);
    plane(&mut pts, (0.0, 8.1), (0.0, 4.0), 0.0, r);
    plane(&mut pts, (0.0, 8.1), (0.0, 4.0), H, r);
    PointCloud {
        points: pts,
        colors: None,
    }
}

fn path(corners: &[[f64; 2]], z: f64) -> Vec<[f64; 3]> {
    let mut out = Vec::new();
    for w in corners.windows(2) {
        for k in 0..20 {
            let t = k as f64 / 20.0;
            out.push([
                w[0][0] + (w[1][0] - w[0][0]) * t,
                w[0][1] + (w[1][1] - w[0][1]) * t,
                z,
            ]);
        }
    }
    out
}

fn tracks() -> Vec<RoomTrack> {
    let mut a = path(
        &[[1.0, 1.0], [4.0, 1.0], [4.0, 3.0], [1.0, 3.0], [1.0, 1.0]],
        1.55,
    );
    // The operator leaves through the doorway while still marked in A.
    a.extend([[4.8, 1.95, 1.55], [5.05, 1.95, 1.55]]);
    vec![
        RoomTrack {
            name: "Living".into(),
            floor: 0,
            positions: a,
        },
        RoomTrack {
            name: "Study".into(),
            floor: 0,
            positions: path(&[[6.0, 1.0], [7.5, 1.0], [7.5, 3.0]], 1.45),
        },
        // A second visit to the same room merges with the first.
        RoomTrack {
            name: "Study".into(),
            floor: 0,
            positions: path(&[[7.5, 3.0], [6.0, 3.0], [6.0, 1.5]], 1.45),
        },
    ]
}

/// A detected wall on `x = v` (vertical) or `y = v` covering `range`.
fn has_wall(walls: &[[f64; 4]], vertical: bool, v: f64, range: (f64, f64)) -> bool {
    let (c, a) = if vertical { (0, 1) } else { (1, 0) };
    let covered: f64 = walls
        .iter()
        .filter(|w| (w[c] - v).abs() < 0.05 && (w[c + 2] - v).abs() < 0.05)
        .map(|w| {
            let (lo, hi) = (
                w[a].min(w[a + 2]).max(range.0),
                w[a].max(w[a + 2]).min(range.1),
            );
            (hi - lo).max(0.0)
        })
        .sum();
    covered >= 0.8 * (range.1 - range.0)
}

#[test]
fn two_rooms_walls_door_and_areas() {
    let plan = build(&scene(), &tracks(), &FloorplanOptions::default());
    assert_eq!(plan.contract, "ados.capture/1");
    assert_eq!(plan.floors.len(), 1);
    let f = &plan.floors[0];
    assert!(f.z_m.abs() < 0.03, "floor z {}", f.z_m);

    for (vertical, v, range) in [
        (true, 0.0, (0.0, 4.0)),
        (true, 8.1, (0.0, 4.0)),
        (true, 5.0, (0.0, 1.5)),
        (true, 5.0, (2.4, 4.0)),
        (true, 5.1, (0.0, 1.5)),
        (true, 5.1, (2.4, 4.0)),
        (false, 0.0, (0.0, 8.1)),
        (false, 4.0, (0.0, 8.1)),
    ] {
        assert!(
            has_wall(&f.walls, vertical, v, range),
            "wall {vertical} {v} {range:?} missing in {:?}",
            f.walls
        );
    }

    assert_eq!(f.doors.len(), 1, "doors {:?}", f.doors);
    let d = f.doors[0];
    let (y0, y1) = (d[1].min(d[3]), d[1].max(d[3]));
    assert!(((d[0] + d[2]) / 2.0 - 5.05).abs() < 0.1, "door x {d:?}");
    assert!(
        (y0 - DOOR.0).abs() < 0.1 && (y1 - DOOR.1).abs() < 0.1,
        "door y {d:?}"
    );

    assert_eq!(f.rooms.len(), 2, "rooms {:?}", f.rooms);
    for (name, truth) in [("Living", 20.0), ("Study", 12.0)] {
        let r = f
            .rooms
            .iter()
            .find(|r| r.name == name)
            .expect("room present");
        assert!(
            (r.area_m2 - truth).abs() / truth < 0.05,
            "{name} area {} vs {truth}: {:?}",
            r.area_m2,
            r.polygon
        );
        assert!((r.area_ft2 / r.area_m2 - 10.7639).abs() < 1e-3);
        assert!(super::super::geom::point_in_polygon(
            [r.hotspot[0], r.hotspot[1]],
            &r.polygon
        ));
    }

    let svg = to_svg(&plan, 0).expect("floor 0 drawn");
    assert!(svg.starts_with("<svg") && svg.contains("<line") && svg.contains("Living"));
    assert!(to_svg(&plan, 3).is_none());

    let json = serde_json::to_value(&plan).unwrap();
    assert_eq!(json["frame"], "enu_local");
    assert_eq!(json["floors"][0]["walls"][0].as_array().unwrap().len(), 4);
    assert_eq!(
        json["floors"][0]["rooms"][0]["hotspot"]
            .as_array()
            .unwrap()
            .len(),
        3
    );
}

#[test]
fn no_tracks_gives_one_floor_without_rooms() {
    let plan = build(&scene(), &[], &FloorplanOptions::default());
    assert_eq!(plan.floors.len(), 1);
    let f = &plan.floors[0];
    assert_eq!(f.floor, 0);
    assert!(f.z_m.abs() < 0.03, "floor z {}", f.z_m);
    assert!(f.rooms.is_empty());
    assert!(f.walls.len() >= 6);
}
