//! The Rerun-aligned world-model recording adapter.
//!
//! The world model is logged as Rerun entity paths so the wire contract and the
//! visualizer are one model. The Atlas keyframe envelope is already a Rerun
//! frame (a `Pinhole` for the intrinsics, a `Transform3D` for the pose, an
//! `Image`, and IMU `Scalars`), so this is the thin adapter that maps an
//! envelope + the world-model descriptors onto the entity tree:
//!
//! ```text
//! world/                       the reconstruction (static, grows)
//!   splat | points | mesh | occupancy
//! world/camera/<camera_id>/    one subtree per camera
//!   (pinhole) (transform) rgb
//! world/imu/                   SLAM-health time series
//!   accel/{x,y,z} gyro/{x,y,z}
//! ```
//!
//! This produces the structured log entries the Rerun viewer renders; the
//! viewer embed and the on-disk `.rrd` / MCAP write are the GCS-side
//! visualization layer. Keeping the adapter pure (no Rerun SDK) makes the
//! mapping testable and keeps the compute crate light.

use serde::{Deserialize, Serialize};

use world_engine_protocol::atlas::{
    ImageEncoding, KeyframeEnvelope, MeshDescriptor, OccupancyDescriptor, PointCloudDescriptor,
    SplatDescriptor,
};

/// A Rerun archetype, the subset the Atlas world model logs. The variant names
/// are the manifest discriminators a viewer maps, so they are the Rerun
/// archetype names verbatim (no `rename_all`, which would mangle `Transform3D`
/// into `transform3_d`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "archetype")]
pub enum RerunArchetype {
    /// Camera intrinsics (row-major 3x3 K) + the image plane size.
    Pinhole {
        image_from_camera: [f64; 9],
        width: u32,
        height: u32,
    },
    /// A 6-DoF pose: row-major 3x3 rotation + translation (world-from-camera).
    Transform3D {
        rotation: [f64; 9],
        translation: [f64; 3],
    },
    /// An encoded RGB image (`jpeg` | `hevc-keyframe`).
    Image {
        width: u32,
        height: u32,
        encoding: String,
    },
    /// One scalar sample on a timeline (an IMU axis).
    Scalar { value: f64 },
    /// A point cloud summary (count + axis-aligned bounds).
    Points3D { count: u64, bounds: [f64; 6] },
    /// A mesh summary.
    Mesh3D { vertex_count: u64, face_count: u64 },
    /// A gaussian-splat slab summary.
    SplatSlab { gaussian_count: u64, step: u64 },
    /// An occupancy voxel grid summary.
    VoxelGrid {
        dims: [u32; 3],
        resolution_m: f32,
        origin: [f64; 3],
    },
}

/// One entry to log: the entity path, the time it is logged at, and the data.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RerunLogEntry {
    pub entity_path: String,
    pub timeline_ms: i64,
    #[serde(flatten)]
    pub archetype: RerunArchetype,
}

fn encoding_str(enc: ImageEncoding) -> &'static str {
    match enc {
        ImageEncoding::Jpeg => "jpeg",
        ImageEncoding::HevcKeyframe => "hevc-keyframe",
    }
}

/// Map one keyframe to its Rerun log entries: the camera's pinhole + pose +
/// image under `world/camera/<id>`, and the IMU window as per-axis scalars under
/// `world/imu/{accel,gyro}`.
pub fn log_keyframe(kf: &KeyframeEnvelope) -> Vec<RerunLogEntry> {
    let cam = format!("world/camera/{}", kf.camera_id);
    let t = kf.ts_unix_ms;
    let mut entries = vec![
        RerunLogEntry {
            entity_path: cam.clone(),
            timeline_ms: t,
            archetype: RerunArchetype::Pinhole {
                image_from_camera: kf.camera.k_matrix(),
                width: kf.image.width,
                height: kf.image.height,
            },
        },
        RerunLogEntry {
            entity_path: cam.clone(),
            timeline_ms: t,
            archetype: RerunArchetype::Transform3D {
                rotation: kf.pose.r,
                translation: kf.pose.t,
            },
        },
        RerunLogEntry {
            entity_path: format!("{cam}/rgb"),
            timeline_ms: t,
            archetype: RerunArchetype::Image {
                width: kf.image.width,
                height: kf.image.height,
                encoding: encoding_str(kf.image.encoding).into(),
            },
        },
    ];

    // IMU samples share the frame's timebase, so each lands on the wall-clock
    // timeline at its offset from the frame's exposure.
    let imu_ms = |t_ns: u64| t + (t_ns as i64 - kf.time.t_exposure_mid_ns as i64) / 1_000_000;
    for sample in &kf.imu_window {
        let at = imu_ms(sample.t_ns);
        for (axis, value) in ["x", "y", "z"].iter().zip(sample.accel.iter().copied()) {
            entries.push(RerunLogEntry {
                entity_path: format!("world/imu/accel/{axis}"),
                timeline_ms: at,
                archetype: RerunArchetype::Scalar { value },
            });
        }
        for (axis, value) in ["x", "y", "z"].iter().zip(sample.gyro.iter().copied()) {
            entries.push(RerunLogEntry {
                entity_path: format!("world/imu/gyro/{axis}"),
                timeline_ms: at,
                archetype: RerunArchetype::Scalar { value },
            });
        }
    }

    entries
}

/// Log a splat descriptor under `world/splat`.
pub fn log_splat(desc: &SplatDescriptor, timeline_ms: i64) -> RerunLogEntry {
    RerunLogEntry {
        entity_path: "world/splat".into(),
        timeline_ms,
        archetype: RerunArchetype::SplatSlab {
            gaussian_count: desc.gaussian_count,
            step: desc.step,
        },
    }
}

/// Log a point-cloud descriptor under `world/points`.
pub fn log_pointcloud(desc: &PointCloudDescriptor, timeline_ms: i64) -> RerunLogEntry {
    RerunLogEntry {
        entity_path: "world/points".into(),
        timeline_ms,
        archetype: RerunArchetype::Points3D {
            count: desc.point_count,
            bounds: desc.bounds,
        },
    }
}

/// Log a mesh descriptor under `world/mesh`.
pub fn log_mesh(desc: &MeshDescriptor, timeline_ms: i64) -> RerunLogEntry {
    RerunLogEntry {
        entity_path: "world/mesh".into(),
        timeline_ms,
        archetype: RerunArchetype::Mesh3D {
            vertex_count: desc.vertex_count,
            face_count: desc.face_count,
        },
    }
}

/// Log an occupancy descriptor under `world/occupancy`.
pub fn log_occupancy(desc: &OccupancyDescriptor, timeline_ms: i64) -> RerunLogEntry {
    RerunLogEntry {
        entity_path: "world/occupancy".into(),
        timeline_ms,
        archetype: RerunArchetype::VoxelGrid {
            dims: desc.dims,
            resolution_m: desc.resolution_m,
            origin: desc.origin,
        },
    }
}

/// An accumulating Rerun recording: the ordered log entries that, fed to the
/// Rerun viewer, render the building world. Serializes to a JSON manifest the
/// GCS viewer (or an MCAP writer) consumes.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct RerunRecording {
    pub entries: Vec<RerunLogEntry>,
    /// Cameras whose static Pinhole intrinsics have already been logged, so each
    /// camera's pinhole is logged once (internal state, not serialized).
    #[serde(skip)]
    pub(crate) logged_cameras: std::collections::HashSet<String>,
    /// Real dense geometry (positions + colours) attached for the `.rrd` write —
    /// the gaussian-splat / point-cloud points the summary `entries` only count.
    /// Kept out of the JSON manifest (it is the heavy payload, not the summary),
    /// so `to_json()` stays the lightweight manifest the GCS viewer reads.
    #[serde(skip)]
    pub(crate) real_points: Vec<crate::rerun_world::RealPoints>,
    /// Real encoded keyframe images attached for the `.rrd` write — the captured
    /// frames the summary `Image` entries only describe. Also kept out of JSON.
    #[serde(skip)]
    pub(crate) real_images: Vec<crate::rerun_world::RealImage>,
}

impl RerunRecording {
    pub fn new() -> Self {
        Self::default()
    }

    /// Append a keyframe's entries. The camera's static Pinhole intrinsics are
    /// logged only on the first keyframe from that camera (per the world-model
    /// schema, "logged once"); the per-frame Transform / Image / IMU entries are
    /// always appended.
    pub fn push_keyframe(&mut self, kf: &KeyframeEnvelope) {
        let first_for_camera = self.logged_cameras.insert(kf.camera_id.clone());
        for entry in log_keyframe(kf) {
            if !first_for_camera && matches!(entry.archetype, RerunArchetype::Pinhole { .. }) {
                continue;
            }
            self.entries.push(entry);
        }
    }

    /// Append a splat-descriptor entry.
    pub fn push_splat(&mut self, desc: &SplatDescriptor, timeline_ms: i64) {
        self.entries.push(log_splat(desc, timeline_ms));
    }

    /// Append a point-cloud entry.
    pub fn push_pointcloud(&mut self, desc: &PointCloudDescriptor, timeline_ms: i64) {
        self.entries.push(log_pointcloud(desc, timeline_ms));
    }

    /// Append a mesh entry.
    pub fn push_mesh(&mut self, desc: &MeshDescriptor, timeline_ms: i64) {
        self.entries.push(log_mesh(desc, timeline_ms));
    }

    /// Append an occupancy entry.
    pub fn push_occupancy(&mut self, desc: &OccupancyDescriptor, timeline_ms: i64) {
        self.entries.push(log_occupancy(desc, timeline_ms));
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Serialize the recording to a JSON manifest.
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string(self)
    }
}

#[cfg(test)]
#[path = "rerun_log_tests.rs"]
mod tests;
