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
fn parts_reassemble_into_the_scan_byte_for_byte() {
    let packed = scan(9_000);
    let mut asm = ScanAssembler::default();
    let mut out = None;
    for c in chunk_scan("mid360", 5, 3, &packed) {
        let c = ScanChunk::from_msgpack(&c.to_msgpack().unwrap()).unwrap();
        out = asm.push(c).or(out);
    }
    let (sensor, t_ns, seq, bytes) = out.expect("whole scan");
    assert_eq!((sensor.as_str(), t_ns, seq), ("mid360", 5, 3));
    assert_eq!(bytes, packed);
    let (pts, inten, off) = unpack_points(&bytes);
    assert_eq!(pts[8_999], [8_999.0, -8_999.0, 0.5]);
    assert_eq!((inten[300], off[300]), (44, 300_000));
}

#[test]
fn a_missing_part_drops_that_scan_and_the_next_one_still_assembles() {
    let mut asm = ScanAssembler::default();
    let first = chunk_scan("mid360", 1, 1, &scan(8_000));
    assert!(asm.push(first[0].clone()).is_none());
    // Part 1 lost; part 2 arrives.
    assert!(asm.push(first[2].clone()).is_none());
    let mut done = None;
    for c in chunk_scan("mid360", 2, 2, &scan(100)) {
        done = asm.push(c);
    }
    assert_eq!(done.map(|d| d.2), Some(2));
    assert_eq!(asm.dropped(), 1);
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
