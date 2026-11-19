use super::*;

/// The host refuses an event payload over this many bytes.
const HOST_EVENT_MAX: usize = 64 * 1024;

fn scan(n: usize) -> Vec<u8> {
    let pts: Vec<[f32; 3]> = (0..n).map(|i| [i as f32, -(i as f32), 0.5]).collect();
    let inten: Vec<u8> = (0..n).map(|i| (i % 256) as u8).collect();
    let off: Vec<u32> = (0..n).map(|i| i as u32 * 1000).collect();
    pack_points(&pts, &inten, &off)
}

#[test]
fn a_3d_scan_crosses_in_parts_that_each_fit_one_host_event() {
    let packed = scan(20_000);
    let parts = chunk_scan("mid360", 42, 7, &packed);
    assert_eq!(parts.len(), 6);
    for (i, c) in parts.iter().enumerate() {
        assert_eq!((c.part, c.parts, c.seq, c.t_ns), (i as u16, 6, 7, 42));
        assert!(c.len() <= SCAN_CHUNK_POINTS);
        let body = c.to_msgpack().unwrap();
        // The bytes cross as one msgpack bin, not 17 integers a point.
        assert!(
            body.len() < c.points.len() + 128,
            "{} bytes for {}",
            body.len(),
            c.points.len()
        );
        assert!(body.len() < HOST_EVENT_MAX);
    }
    assert_eq!(parts.iter().map(ScanChunk::len).sum::<usize>(), 20_000);
}

#[test]
fn a_planar_scan_is_one_part_and_an_empty_scan_still_arrives() {
    assert_eq!(chunk_scan("rplidar", 1, 0, &scan(360)).len(), 1);
    let empty = chunk_scan("rplidar", 1, 1, &[]);
    assert_eq!(empty.len(), 1);
    assert!(empty[0].is_empty());
}

#[test]
fn parts_reassemble_into_the_scan_byte_for_byte_in_any_order() {
    let packed = scan(20_000);
    let mut asm = ScanAssembler::new(500_000_000);
    let mut parts = chunk_scan("mid360", 5, 3, &packed);
    parts.reverse();
    parts.swap(1, 4);
    let mut out = Vec::new();
    for c in parts {
        let c = ScanChunk::from_msgpack(&c.to_msgpack().unwrap()).unwrap();
        out.extend(asm.push(c, 10));
    }
    assert_eq!(out.len(), 1, "one scan, once");
    let s = &out[0];
    assert_eq!((s.sensor_id.as_str(), s.t_ns, s.seq), ("mid360", 5, 3));
    assert_eq!(s.points, packed);
    let (pts, inten, off) = unpack_points(&s.points);
    assert_eq!(pts[19_999], [19_999.0, -19_999.0, 0.5]);
    assert_eq!((inten[300], off[300]), (44, 300_000));
}

#[test]
fn a_scan_missing_a_part_past_the_window_is_counted_and_others_still_assemble() {
    let mut asm = ScanAssembler::new(500);
    let first = chunk_scan("mid360", 1, 1, &scan(8_000));
    assert!(asm.push(first[0].clone(), 0).is_none());
    // Part 1 never arrives; scan 2 interleaves with scan 1's last part.
    let second = chunk_scan("mid360", 2, 2, &scan(100));
    assert!(asm.push(first[2].clone(), 100).is_none());
    assert_eq!(asm.push(second[0].clone(), 200).map(|s| s.seq), Some(2));
    assert_eq!(asm.incomplete(), 0);
    // A repeated part neither completes nor corrupts a scan.
    let third = chunk_scan("mid360", 3, 3, &scan(8_000));
    assert!(asm.push(third[0].clone(), 300).is_none());
    assert!(asm.push(third[0].clone(), 300).is_none());
    asm.expire(601);
    assert_eq!(asm.incomplete(), 1, "scan 1 expired, scan 3 still open");
    assert!(asm.push(third[1].clone(), 650).is_none());
    assert_eq!(
        asm.push(third[2].clone(), 700).map(|s| s.points.len()),
        Some(8_000 * POINT_STRIDE)
    );
}

#[test]
fn a_pose_track_batch_round_trips() {
    let b = PoseTrackBatch {
        source: PoseSource::Lidar,
        estimates: vec![PoseTrackEntry {
            t_ns: 9,
            r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            t: [1.0, 2.0, 3.0],
            cov: vec![0.0; 36],
            health: VioHealth::Good,
            sigma_xy_m: Some(0.05),
            heading_ok: true,
        }],
    };
    assert_eq!(
        PoseTrackBatch::from_msgpack(&b.to_msgpack().unwrap()).unwrap(),
        b
    );
}
