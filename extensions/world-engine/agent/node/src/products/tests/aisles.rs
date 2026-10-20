use super::super::aisles::{detect, to_geojson, Aisle, AisleKind};
use super::super::geom::Rng;
use super::super::PointCloud;
use super::jitter;

/// Rack geometry in the rack frame: `u` along the rows, `v` across.
struct Racks {
    /// Row `r` occupies `v ∈ [3r, 3r + 1]` (1 m deep, 2 m aisles).
    rows: usize,
    /// Along-row blocks `[u0, u1]`.
    blocks: Vec<(f64, f64)>,
    /// Open floor `u` and `v` ranges.
    floor_u: (f64, f64),
    floor_v: (f64, f64),
}

/// 5 m high racks with shelves, standing on a floor at z = 10, the whole
/// layout rotated by 25° and shifted.
fn scan(r: &Racks) -> PointCloud {
    let mut rng = Rng::new(5);
    let (s, c) = 25f64.to_radians().sin_cos();
    let floor = 10.0;
    let mut pts = Vec::new();
    let put = |u: f64, v: f64, h: f64, pts: &mut Vec<[f64; 3]>| {
        pts.push([c * u - s * v + 3.0, s * u + c * v - 7.0, floor + h]);
    };
    for row in 0..r.rows {
        let v0 = row as f64 * 3.0;
        for &(u0, u1) in &r.blocks {
            for i in 0..=((u1 - u0) / 0.1).round() as usize {
                let u = u0 + i as f64 * 0.1;
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
    }
    let nu = ((r.floor_u.1 - r.floor_u.0) / 0.2).round() as usize;
    let nv = ((r.floor_v.1 - r.floor_v.0) / 0.25).round() as usize;
    for i in 0..=nu {
        for j in 0..=nv {
            put(
                r.floor_u.0 + i as f64 * 0.2,
                r.floor_v.0 + j as f64 * 0.25,
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

/// Direction of a centreline in degrees, folded into `[0, 180)`.
fn heading(a: &Aisle) -> f64 {
    let d = [
        a.centerline[1][0] - a.centerline[0][0],
        a.centerline[1][1] - a.centerline[0][1],
    ];
    d[1].atan2(d[0]).to_degrees().rem_euclid(180.0)
}

#[test]
fn five_rows_give_four_aisles() {
    let aisles = detect(
        &scan(&Racks {
            rows: 5,
            blocks: vec![(0.0, 20.0)],
            floor_u: (0.0, 20.0),
            floor_v: (0.0, 13.0),
        }),
        10.0,
    );
    assert_eq!(aisles.len(), 4, "{aisles:?}");
    for (k, a) in aisles.iter().enumerate() {
        assert_eq!(a.id, k as u32 + 1);
        assert_eq!(a.kind, AisleKind::Aisle);
        assert!((a.width_m - 2.0).abs() < 0.2, "width {}", a.width_m);
        assert!(
            a.length_m > 17.0 && a.length_m < 21.0,
            "length {}",
            a.length_m
        );
        assert!((heading(a) - 25.0).abs() < 1.0, "angle {}", heading(a));
    }
    let gj = to_geojson(&aisles);
    assert_eq!(gj["type"], "FeatureCollection");
    assert_eq!(gj["frame"], "enu_local");
    assert!(gj["crs"].is_null());
    let f = &gj["features"][0];
    assert_eq!(f["geometry"]["type"], "LineString");
    assert_eq!(f["geometry"]["coordinates"].as_array().unwrap().len(), 2);
    assert!(f["properties"]["width_m"].as_f64().is_some());
    assert_eq!(f["properties"]["kind"], "aisle");
}

#[test]
fn aisles_and_cross_aisles_found_in_a_rack_grid() {
    // Two blocks of five rows split by a 3 m cross-aisle, with 3 m of open
    // floor at both ends: four aisles crossed by three cross-aisles.
    let aisles = detect(
        &scan(&Racks {
            rows: 5,
            blocks: vec![(0.0, 10.0), (13.0, 23.0)],
            floor_u: (-3.0, 26.0),
            floor_v: (0.0, 13.0),
        }),
        10.0,
    );
    let (along, across): (Vec<&Aisle>, Vec<&Aisle>) =
        aisles.iter().partition(|a| a.kind == AisleKind::Aisle);
    assert_eq!(along.len(), 4, "{aisles:?}");
    assert_eq!(across.len(), 3, "{aisles:?}");
    for a in &along {
        assert!(
            (heading(a) - 25.0).abs() < 2.0,
            "aisle angle {}",
            heading(a)
        );
        assert!((a.width_m - 2.0).abs() < 0.25, "aisle width {}", a.width_m);
        assert!(a.length_m > 20.0, "aisle length {}", a.length_m);
    }
    for a in &across {
        assert!(
            (heading(a) - 115.0).abs() < 2.0,
            "cross angle {}",
            heading(a)
        );
        assert!((a.width_m - 3.0).abs() < 0.35, "cross width {}", a.width_m);
        assert!(a.length_m > 8.0, "cross length {}: {aisles:?}", a.length_m);
    }
    // Ids run over aisles first, then cross-aisles.
    let ids: Vec<u32> = aisles.iter().map(|a| a.id).collect();
    assert_eq!(ids, (1..=7).collect::<Vec<_>>());
    assert!(aisles[..4].iter().all(|a| a.kind == AisleKind::Aisle));
    assert_eq!(
        to_geojson(&aisles)["features"][6]["properties"]["kind"],
        "cross_aisle"
    );
}

#[test]
fn empty_floor_has_no_aisles() {
    let cloud = PointCloud {
        points: (0..1000).map(|i| [i as f64 * 0.01, 0.0, 0.0]).collect(),
        colors: None,
    };
    assert!(detect(&cloud, 0.0).is_empty());
}
