use super::super::aisles::{detect, to_geojson};
use super::super::geom::Rng;
use super::super::PointCloud;
use super::jitter;

/// Five 1 m deep, 20 m long, 5 m high rack rows at a 3 m pitch (2 m aisles),
/// rotated by 25° and standing on a floor at z = 10.
fn racks() -> PointCloud {
    let mut rng = Rng::new(5);
    let (s, c) = 25f64.to_radians().sin_cos();
    let floor = 10.0;
    let mut pts = Vec::new();
    let put = |u: f64, v: f64, h: f64, pts: &mut Vec<[f64; 3]>| {
        pts.push([c * u - s * v + 3.0, s * u + c * v - 7.0, floor + h]);
    };
    for r in 0..5 {
        let v0 = r as f64 * 3.0;
        for i in 0..=200 {
            let u = i as f64 * 0.1;
            for k in 0..=20 {
                let h = k as f64 * 0.25;
                put(u, v0 + jitter(&mut rng, 0.01), h, &mut pts);
                put(u, v0 + 1.0 + jitter(&mut rng, 0.01), h, &mut pts);
            }
            for shelf in [0.5, 1.5, 2.5, 3.5, 4.5] {
                for j in 1..20 {
                    put(u, v0 + j as f64 * 0.05, shelf, &mut pts);
                }
            }
        }
    }
    // Open floor, below the slab band.
    for i in 0..=100 {
        for j in 0..=60 {
            put(
                i as f64 * 0.2,
                j as f64 * 0.25 - 1.0,
                jitter(&mut rng, 0.01),
                &mut pts,
            );
        }
    }
    PointCloud {
        points: pts,
        colors: None,
    }
}

#[test]
fn five_rows_give_four_aisles() {
    let aisles = detect(&racks(), 10.0);
    assert_eq!(aisles.len(), 4, "{aisles:?}");
    for (k, a) in aisles.iter().enumerate() {
        assert_eq!(a.id, k as u32 + 1);
        assert!((a.width_m - 2.0).abs() < 0.1, "width {}", a.width_m);
        assert!((a.length_m - 20.0).abs() < 0.5, "length {}", a.length_m);
        let d = [
            a.centerline[1][0] - a.centerline[0][0],
            a.centerline[1][1] - a.centerline[0][1],
        ];
        let ang = d[1].atan2(d[0]).to_degrees().rem_euclid(180.0);
        assert!((ang - 25.0).abs() < 1.0, "angle {ang}");
    }
    let gj = to_geojson(&aisles);
    assert_eq!(gj["type"], "FeatureCollection");
    assert_eq!(gj["frame"], "enu_local");
    assert!(gj["crs"].is_null());
    let f = &gj["features"][0];
    assert_eq!(f["geometry"]["type"], "LineString");
    assert_eq!(f["geometry"]["coordinates"].as_array().unwrap().len(), 2);
    assert!(f["properties"]["width_m"].as_f64().is_some());
}

#[test]
fn empty_floor_has_no_aisles() {
    let cloud = PointCloud {
        points: (0..1000).map(|i| [i as f64 * 0.01, 0.0, 0.0]).collect(),
        colors: None,
    };
    assert!(detect(&cloud, 0.0).is_empty());
}
