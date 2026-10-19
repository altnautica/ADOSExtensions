//! ADOS capture contract `ados.capture/1`.
//!
//! The MIT schema files under `extensions/world-engine/contract/` are the
//! authority; everything typed here is generated from them at build time:
//!
//! - [`schema`] (re-exported at the crate root): the JSON documents —
//!   [`Manifest`], [`Rig`], [`CameraModel`], [`GeoAnchor`], [`Placement`],
//!   [`ArtifactDescriptor`] and the `transforms.json` extension.
//! - [`v1`]: the protobuf messages of the MCAP profile (`ImuBatch`,
//!   `TriggerRecord`, `ClockSample`, `GnssRaw`, `PoseCov`).
//! - [`foxglove`]: the Foxglove message schemas the profile names.
//!
//! [`SessionWriter`] and [`SessionReader`] read and write the session
//! directory: `manifest.json`, `rig.json`, rolled `segments/NNNNNN.mcap` and
//! `checksums.b3`.

#[allow(clippy::all, missing_docs)]
pub mod schema {
    include!(concat!(env!("OUT_DIR"), "/schema_types.rs"));
}

/// `ados.capture.v1` protobuf messages.
#[allow(clippy::all, missing_docs)]
pub mod v1 {
    include!(concat!(env!("OUT_DIR"), "/ados.capture.v1.rs"));
}

/// Foxglove protobuf message schemas (MIT, vendored from the Foxglove SDK).
#[allow(clippy::all, missing_docs)]
pub mod foxglove {
    include!(concat!(env!("OUT_DIR"), "/foxglove.rs"));
}

mod checksums;
mod geometry;
mod reader;
mod session_id;
pub mod topics;
mod writer;

pub use checksums::{
    check_checksums, parse_checksums, verify_checksums, write_checksums, ChecksumReport,
    CHECKSUMS_FILE,
};
pub use reader::{SessionMessage, SessionReader};
pub use schema::*;
pub use session_id::session_id;
pub use writer::{SegmentLimits, SessionWriter};

/// The contract id every JSON document carries in `contract`.
pub const CONTRACT_ID: &str = "ados.capture/1";

/// Semantic version of the schema files this crate was generated from.
pub const CONTRACT_VERSION: &str = "1.0.0";

/// MCAP profile name written in every segment header.
pub const MCAP_PROFILE: &str = "ados.capture/1";

/// Encoded `google.protobuf.FileDescriptorSet` covering every contract and
/// Foxglove message, the schema data of every protobuf MCAP channel.
pub const FILE_DESCRIPTOR_SET: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/capture_fds.bin"));

/// The contract's JSON Schema files, `(file name, contents)`, for consumers
/// that validate documents beyond the structural Rust types.
pub const SCHEMA_FILES: &[(&str, &str)] = &[
    (
        "artifact.schema.json",
        include_str!("../../../contract/schema/artifact.schema.json"),
    ),
    (
        "camera_model.schema.json",
        include_str!("../../../contract/schema/camera_model.schema.json"),
    ),
    (
        "geo_anchor.schema.json",
        include_str!("../../../contract/schema/geo_anchor.schema.json"),
    ),
    (
        "manifest.schema.json",
        include_str!("../../../contract/schema/manifest.schema.json"),
    ),
    (
        "placement.schema.json",
        include_str!("../../../contract/schema/placement.schema.json"),
    ),
    (
        "rig.schema.json",
        include_str!("../../../contract/schema/rig.schema.json"),
    ),
    (
        "transforms_ext.schema.json",
        include_str!("../../../contract/schema/transforms_ext.schema.json"),
    ),
];

/// Base URI of the schema `$id`s; a file's `$id` is this plus its file name.
pub const SCHEMA_BASE_URI: &str = "https://altnautica.com/schemas/ados-capture/1/";

/// Session directory and MCAP errors.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("io: {0}")]
    Io(#[from] std::io::Error),
    #[error("json: {0}")]
    Json(#[from] serde_json::Error),
    #[error("mcap: {0}")]
    Mcap(#[from] mcap::McapError),
    #[error("protobuf decode: {0}")]
    Decode(#[from] prost::DecodeError),
    #[error("checksum mismatch for {0}")]
    Checksum(String),
    #[error("invalid session: {0}")]
    Invalid(String),
}

/// Crate result alias.
pub type Result<T, E = Error> = std::result::Result<T, E>;
