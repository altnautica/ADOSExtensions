use super::{map64, SEC};
use crate::{Pose, VoxTier, VoxelState, VOX_OCCUPIED, VOX_UNKNOWN};
use ados_protocol::voxmap::{ReadStatus, VoxSnapshot, VoxmapReader, VoxmapWriter};

const O: [f32; 3] = [0.07, 0.07, 0.07];

/// Moving the window keeps every voxel that stays inside it at the same
/// world position, forgets the voxels that scroll out, and starts the voxels
/// that scroll in unknown.
#[test]
fn recentre_keeps_world_fixed_voxels() {
    let mut m = map64();
    let ahead = [2.07, 0.07, 0.07];
    let behind = [-4.5, 0.07, 0.07];
    m.insert_cloud(SEC, O, &[ahead, behind], 30.0);
    let (ca, cb) = (m.cell_of(ahead), m.cell_of(behind));
    let free_between = [5, 0, 0];
    assert_eq!(m.state(ca), VoxelState::Occupied);
    assert_eq!(m.state(cb), VoxelState::Occupied);
    assert_eq!(m.state(free_between), VoxelState::Free);

    // Slack 0: any drift moves the window, here 1.5 m (10 voxels) north.
    assert!(m.recentre([1.57, 0.07, 0.07]));
    assert_eq!(m.origin_cell(), [10 - 32, -32, -16]);
    assert_eq!(m.state(ca), VoxelState::Occupied, "same world voxel");
    assert_eq!(m.state(free_between), VoxelState::Free);
    assert_eq!(m.state(cb), VoxelState::Unknown, "scrolled out");
    let entered = [m.origin_cell()[0] + 63, 0, 0];
    assert_eq!(m.state(entered), VoxelState::Unknown);
    assert!(!m.recentre([1.57, 0.07, 0.07]), "already centred");

    // Back again: the forgotten voxel stays forgotten.
    assert!(m.recentre(O));
    assert_eq!(m.state(cb), VoxelState::Unknown);
    assert_eq!(m.state(ca), VoxelState::Occupied);
    assert_eq!(m.obstacles(), vec![ca]);

    // A jump beyond the window clears everything.
    assert!(m.recentre([100.0, 0.0, 0.0]));
    assert!(m.obstacles().is_empty());
    assert!(m.state.iter().all(|&s| s == VOX_UNKNOWN));
}

/// The slack keeps the window still while the vehicle stays near its centre
/// (on the fine window); the default window is the 0.2 m one.
#[test]
fn recentre_waits_for_the_slack() {
    let default = crate::VoxMap::new(crate::VoxMapConfig::default()).unwrap();
    assert_eq!((default.dims(), default.res_m()), ([120, 120, 50], 0.2));
    let mut m = crate::VoxMap::new(crate::VoxMapConfig {
        recentre_slack_m: 0.45,
        ..crate::VoxMapConfig::fine_window()
    })
    .unwrap();
    assert_eq!((m.dims(), m.res_m()), ([200, 200, 80], 0.15));
    assert!(!m.recentre([0.4, -0.4, 0.4]));
    assert!(m.recentre([0.7, 0.0, 0.0]));
    assert_eq!(m.origin_cell(), [4 - 100, -100, -40]);
}

/// What a reader copies off the seam is the map in window order: each
/// voxel's class and decimetre distance at `x + nx * (y + ny * z)` from the
/// published origin, with the ring offset unrolled.
#[test]
fn the_seam_carries_the_window_in_order() {
    let mut m = map64();
    m.insert_cloud(SEC, O, &[[2.07, 0.37, -0.53], [-1.0, 2.0, 0.8]], 30.0);
    m.update_esdf();
    m.recentre([0.75, -0.6, 0.3]);
    assert_ne!(m.om, [32, 32, 16]);

    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("ados-voxmap-v1");
    let mut w = VoxmapWriter::create(&path, m.dims(), None).unwrap();
    let body = Pose::at([0.75, -0.6, 0.3]);
    m.to_seam(&mut w, &body, VoxTier::Lidar3d).unwrap();

    let mut r = VoxmapReader::open(&path).unwrap();
    let mut snap = VoxSnapshot::default();
    assert_eq!(r.read_into(&mut snap).unwrap(), ReadStatus::Fresh);
    assert_eq!(snap.meta.origin_cell, m.origin_cell());
    assert_eq!(snap.meta.dims, [64, 64, 32]);
    assert_eq!(snap.meta.stamp_ns, SEC);
    assert_eq!(snap.meta.tier, VoxTier::Lidar3d);
    assert_eq!(
        snap.meta.map_from_body,
        [0.75, -0.6, 0.3, 1.0, 0.0, 0.0, 0.0]
    );
    let mut occupied = 0;
    for wv in m.slab(0, 0..64) {
        let c = [0, 1, 2].map(|a| wv[a] + m.origin_cell()[a]);
        let i = snap.index(wv.map(i64::from)).unwrap();
        let idx = m.idx_of_window(wv);
        assert_eq!(snap.occupancy[i], m.state[idx], "voxel {c:?}");
        assert_eq!(
            snap.esdf_dm[i],
            m.esdf.dm_of(m.esdf.cells[idx].d2),
            "voxel {c:?}"
        );
        occupied += usize::from(snap.occupancy[i] == VOX_OCCUPIED);
    }
    assert_eq!(occupied, 2);
    let p = m.centre_of(m.cell_of([2.07, 0.37, -0.53]));
    let i = snap.index_of_point(p).unwrap();
    assert_eq!(snap.occupancy[i], VOX_OCCUPIED);
    assert_eq!(snap.esdf_dm[i], 0);
}
