//! The capture and reconstruction wire contract.
//!
//! The topic names and wire structs between a capture device and a
//! reconstruction node: a drone or Visor captures pose-tagged keyframes, a
//! node reconstructs a splat / cloud / mesh / occupancy from them. Two topic
//! roots, neither on the plugin host's shared bus:
//!
//! - `capture.*`: the capture service's private bus to its link (capture
//!   state, the keyframe lane), forwarded to the node.
//! - `reconstruct.*`: what a drone streams to the node for a live map
//!   (`reconstruct.live.scan`) and what the node serves on its world stream.
//!
//! Heavy payloads ride the stream lane; these topics carry small descriptors
//! only. The envelope is transport-agnostic: the identical struct travels on any
//! bearer (direct LAN, the WFB relay, or the cloud relay), so no transport
//! strings are baked in. It is also tier-aware: a light descriptor (pose plus a
//! thumbnail or an occupancy slice) is small enough for an in-flight relay link,
//! while a full keyframe (full-resolution image bytes plus the IMU window) is a
//! LAN-bulk artifact.

use serde::{Deserialize, Serialize};
use thiserror::Error;

// --- Topics ---------------------------------------------------------------

/// Capture-session state (state, keyframe counts, pose health), on the
/// capture service's private bus to its link.
pub const CAPTURE_STATE_TOPIC: &str = "capture.state";

/// A selected pose-tagged keyframe ([`KeyframeEnvelope`]) the capture service
/// emits drone-to-compute. The extension's internal namespace (the keyframe is
/// the capture artifact, not shared plugin data); a compute node and the
/// reconstruction stream lane consume it.
pub const CAPTURE_KEYFRAME_TOPIC: &str = "capture.keyframe";

/// One raw planar scan with its sensor pose, as a binary [`PolarScan`]
/// record (not msgpack), sent to the reconstruction node while the drone
/// streams its map there, so the node builds the full-resolution live map.
pub const RECONSTRUCT_LIVE_SCAN_TOPIC: &str = "reconstruct.live.scan";

// --- The node's world stream (`/ws/live/:device_id`) ------------------------

/// A reconstructed point cloud's descriptor (count, bounds, url).
pub const RECONSTRUCT_POINTCLOUD_TOPIC: &str = "reconstruct.pointcloud";

/// A reconstructed occupancy grid's descriptor (origin, resolution, dims,
/// handle).
pub const RECONSTRUCT_OCCUPANCY_TOPIC: &str = "reconstruct.occupancy";

/// "Splat updated" (gaussian count, url / handle).
pub const RECONSTRUCT_SPLAT_TOPIC: &str = "reconstruct.splat";

/// A reconstructed mesh's descriptor (vertex / face count, handle / url).
pub const RECONSTRUCT_MESH_TOPIC: &str = "reconstruct.mesh";

/// The node's live occupancy tiles ([`LiveOccupancy`]) from a streaming
/// drone's scans.
pub const RECONSTRUCT_LIVE_OCCUPANCY_TOPIC: &str = "reconstruct.live.occupancy";

/// The node's live sweep-cloud delta ([`LivePointCloud`]) from a streaming
/// drone's scans.
pub const RECONSTRUCT_LIVE_POINTCLOUD_TOPIC: &str = "reconstruct.live.pointcloud";

/// Whether a bearer can carry a full keyframe, from the bearer name the
/// forwarder already reports (`direct-lan` / `wfb-relay` / `cloud`).
///
/// This is a pure property of the bearer, not a runtime measurement: the WFB
/// relay's datagram ceiling is 1300 bytes against a multi-MB JPEG keyframe, so
/// on a no-LAN field topology the ladder degrades to pose and status only —
/// i.e. NO reconstruction — while the operator surface would otherwise report
/// `bearer: "wfb-relay"` as though the lane were working. Deciding it here, at
/// the contract, keeps one answer for the forwarder, the capture service and
/// the GCS instead of three.
pub fn bearer_carries_keyframes(bearer: &str) -> bool {
    !matches!(bearer, "wfb-relay")
}

/// The operator-facing reason a bearer cannot carry keyframes, or `None` when
/// it can. Short, factual, and safe to render verbatim.
pub fn bearer_keyframe_degraded_reason(bearer: &str) -> Option<&'static str> {
    match bearer {
        "wfb-relay" => Some(
            "relay datagrams are too small for full keyframes; pose and status only until a LAN bearer is available",
        ),
        _ => None,
    }
}

// --- Forwarder status -----------------------------------------------------

/// Schema version stamped on an [`KeyframeForwardStatus`] by its writer.
pub const FORWARD_STATUS_VERSION: u16 = 1;

/// The drone-side forwarder's transport facts: the compute node it resolved,
/// the bearer currently carrying the stream, and when a keyframe last left the
/// drone. Only the forwarder knows these (the capture service never sees the
/// bearer ladder), so they are folded into the capture state the extension
/// publishes. The bearer is already in the GCS vocabulary (`direct-lan` /
/// `wfb-relay` / `cloud`); every field is optional so a forwarder with no
/// resolved node reports an honest "nothing yet".
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct KeyframeForwardStatus {
    /// Schema version, stamped [`FORWARD_STATUS_VERSION`] on write. An
    /// older writer emitted no field, so a reader defaults it to `0`.
    #[serde(default)]
    pub version: u16,
    /// The resolved compute node's device id (the mDNS `deviceId`), or `None`
    /// while no compute node is on the LAN.
    #[serde(default)]
    pub compute_node_id: Option<String>,
    /// The bearer currently carrying the stream, in the GCS vocabulary
    /// (`direct-lan` / `wfb-relay` / `cloud`), or `None` before anything has been
    /// forwarded this run.
    #[serde(default)]
    pub bearer: Option<String>,
    /// Epoch ms a keyframe was last forwarded toward the compute node, or `None`
    /// if none has been forwarded yet this run.
    #[serde(default)]
    pub last_kf_at_ms: Option<i64>,
    /// Local write time (epoch ms), for the reader's own freshness reasoning.
    pub generated_at_ms: i64,
}

mod descriptors;
mod keyframe;
pub mod nav;
pub mod polar;
pub mod stream;
#[cfg(test)]
mod tests;
#[cfg(test)]
mod tests_bus;
#[cfg(test)]
mod tests_nav;
#[cfg(test)]
mod tests_stream;

pub use descriptors::{
    Generation, MeshDescriptor, OccupancyDescriptor, OccupancyField, PointCloudDescriptor,
    SplatDescriptor,
};
pub use keyframe::{
    CameraRole, CaptureState, CaptureStatus, ImageEncoding, KeyframeBudget, KeyframeEnvelope,
    KeyframeFlags, KeyframeImage, KeyframeReject, KeyframeTier, Pose, PoseDescriptor, PoseSource,
    TimeAlignment, VioHealth, KEYFRAME_MAX_SIGMA_NS, POSE_COV_LEN,
};
pub use nav::{
    ComputePlacement, ImuFeedReason, ImuFeedState, ImuFeedStatus, LidarHealth, LidarStatus,
    LiveOccupancy, LivePointCloud, MapNorth, MapPayloadKind, MapStatus, NavFrame, NavPoseSource,
    OccupancyTile, ScanDescriptor, StreamPhase, StreamProgress, UploadPhase, UploadProgress,
    LIDAR_STATUS_VERSION,
};
pub use polar::{PolarError, PolarScan};
pub use stream::{
    chunk_scan, pack_points, unpack_points, AssembledScan, PoseTrackBatch, PoseTrackEntry,
    ScanAssembler, ScanChunk, POINT_STRIDE, SCAN_CHUNK_POINTS,
};

/// The on-wire version stamped on every [`NodeEvent`] envelope. Every node
/// message crosses as an envelope, and the inner topic structs are decoded from
/// its `payload` only after the envelope decodes, so versioning the envelope
/// gates the whole contract.
///
/// Version 2: keyframes carry the capture contract's camera model, geo anchor,
/// timebase-stamped exposure time and IMU samples (`t_ns`, `seq`), and every
/// artifact descriptor carries a placement.
pub const EVENT_ENVELOPE_VERSION: u16 = 2;

/// Errors raised decoding an [`NodeEvent`] envelope.
#[derive(Debug, Error)]
pub enum EnvelopeError {
    /// The msgpack body did not decode into an envelope (also fires when the
    /// required version field is absent).
    #[error("msgpack decode error: {0}")]
    Decode(#[from] rmp_serde::decode::Error),
    /// The envelope carried a version this build does not speak.
    #[error("unsupported event envelope version {got} (this build speaks {ours})")]
    Version { got: u16, ours: u16 },
}

/// One framed message between a capture device and a reconstruction node (and
/// on the capture service's private bus), tagged with the topic it belongs to
/// (one of the `capture.*` / `reconstruct.*` constants above) so a receiver
/// demultiplexes the streams off one connection. `payload` is the topic's own
/// struct already msgpack-encoded (e.g. a [`KeyframeEnvelope`] on
/// [`CAPTURE_KEYFRAME_TOPIC`]), so the wrapper stays agnostic to which struct
/// it carries.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NodeEvent {
    /// On-wire envelope version, the first field. A decoder that sees a version
    /// it does not speak rejects the frame in [`NodeEvent::decode`] rather than
    /// silently mis-parsing the payload; a frame missing this field also fails to
    /// decode (no serde default). Stamped by [`NodeEvent::new`]; held equal to
    /// [`EVENT_ENVELOPE_VERSION`].
    #[serde(rename = "v")]
    pub v: u16,
    pub topic: String,
    /// The capturing drone's device id, stamped by the drone-side forwarder as
    /// the event leaves the drone (the single choke point every bearer passes
    /// through). The compute node reads it to attribute a reconstruct job to the
    /// drone that captured it (the reconstruction job's `deviceId`). Additive +
    /// optional: an event on the local publish bus (before egress) omits it, and
    /// a receiver decoding an older frame defaults it to `None`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub device_id: Option<String>,
    pub payload: Vec<u8>,
}

impl NodeEvent {
    /// Build an envelope stamped with the current [`EVENT_ENVELOPE_VERSION`].
    /// `device_id` is `None` on the local publish bus and `Some(..)` once the
    /// drone-side forwarder stamps it on egress.
    pub fn new(topic: impl Into<String>, device_id: Option<String>, payload: Vec<u8>) -> Self {
        Self {
            v: EVENT_ENVELOPE_VERSION,
            topic: topic.into(),
            device_id,
            payload,
        }
    }

    /// Encode as a msgpack map with named keys (the version field rides as `v`).
    pub fn encode(&self) -> Result<Vec<u8>, rmp_serde::encode::Error> {
        rmp_serde::to_vec_named(self)
    }

    /// Decode a msgpack envelope, rejecting a frame whose version this build does
    /// not speak (and a frame missing the version field). Decoding the envelope
    /// gates the whole event contract: a subscriber only reads the inner topic
    /// struct out of `payload` once this succeeds.
    pub fn decode(bytes: &[u8]) -> Result<Self, EnvelopeError> {
        let event: NodeEvent = rmp_serde::from_slice(bytes)?;
        if event.v != EVENT_ENVELOPE_VERSION {
            return Err(EnvelopeError::Version {
                got: event.v,
                ours: EVENT_ENVELOPE_VERSION,
            });
        }
        Ok(event)
    }
}

macro_rules! impl_msgpack {
    ($($t:ty),+ $(,)?) => {
        $(impl $t {
            /// Encode as a msgpack map with named keys.
            pub fn to_msgpack(&self) -> Result<Vec<u8>, rmp_serde::encode::Error> {
                rmp_serde::to_vec_named(self)
            }
            /// Decode from a msgpack map.
            pub fn from_msgpack(bytes: &[u8]) -> Result<Self, rmp_serde::decode::Error> {
                rmp_serde::from_slice(bytes)
            }
        })+
    };
}

impl_msgpack!(
    KeyframeEnvelope,
    CaptureStatus,
    PoseDescriptor,
    PointCloudDescriptor,
    OccupancyDescriptor,
    SplatDescriptor,
    MeshDescriptor,
    TimeAlignment,
    ScanDescriptor,
    LiveOccupancy,
    LivePointCloud,
    MapStatus,
    LidarStatus,
);
