//! Atlas world-model wire contract.
//!
//! The topic names and wire structs for the world-model program: a drone
//! captures pose-tagged keyframes, a compute node reconstructs a 3D world model
//! (splat / cloud / mesh / occupancy), and the result is consumable as shared
//! data rather than only viewable. Two topic roots:
//!
//! - `atlas.*` is the extension's own internal namespace (capture state, the
//!   keyframe lane). It stays between the
//!   extension's processes and the compute node it streams to.
//! - `plugin.world-engine.*` is the shared-data namespace other plugins
//!   subscribe to through the extension's manifest `shared_topics` declaration.
//!   It is not an open namespace: a reconstruction is derived imagery of
//!   wherever the aircraft flew, and an occupancy field is a planning input, so
//!   each topic names the capability a subscriber must hold
//!   ([`atlas_topic_subscribe_capability`]).
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

/// Capture-session state (state, keyframe counts, VIO health).
pub const ATLAS_CAPTURE_STATE_TOPIC: &str = "atlas.capture.state";

/// A selected pose-tagged keyframe ([`KeyframeEnvelope`]) the capture service
/// emits drone-to-compute. The extension's internal namespace (the keyframe is
/// the capture artifact, not shared plugin data); a compute node and the
/// world-model stream lane consume it.
pub const ATLAS_KEYFRAME_TOPIC: &str = "atlas.keyframe";

/// Shared-data: current 6-DoF pose plus world anchor (~10 Hz).
pub const PLUGIN_ATLAS_POSE_TOPIC: &str = "plugin.world-engine.pose";

/// Shared-data: point-cloud descriptor (count, bounds, url).
pub const PLUGIN_ATLAS_POINTCLOUD_TOPIC: &str = "plugin.world-engine.pointcloud";

/// Shared-data: occupancy-grid descriptor (origin, resolution, dims, handle).
pub const PLUGIN_ATLAS_OCCUPANCY_TOPIC: &str = "plugin.world-engine.occupancy";

/// Shared-data: "splat updated" descriptor (gaussian count, url / handle).
pub const PLUGIN_ATLAS_SPLAT_TOPIC: &str = "plugin.world-engine.splat";

/// Shared-data: mesh descriptor (vertex / face count, handle / url).
pub const PLUGIN_ATLAS_MESH_TOPIC: &str = "plugin.world-engine.mesh";

/// Every shared-data world-model topic, in one place, so a consumer or a gate
/// iterates the set rather than hard-coding five string literals that drift.
pub const PLUGIN_ATLAS_TOPICS: &[&str] = &[
    PLUGIN_ATLAS_POSE_TOPIC,
    PLUGIN_ATLAS_POINTCLOUD_TOPIC,
    PLUGIN_ATLAS_OCCUPANCY_TOPIC,
    PLUGIN_ATLAS_SPLAT_TOPIC,
    PLUGIN_ATLAS_MESH_TOPIC,
];

// --- Shared-data access policy --------------------------------------------

/// The capability a plugin must hold to subscribe to `topic`, or `None` when
/// `topic` is not a world-model shared-data topic. The manifest's
/// `shared_topics[].subscribe_capability` entries state the same mapping; the
/// plugin host enforces it on top of `event.subscribe`, never instead of it.
///
/// **Exact match, never a prefix.** `plugin.world-engine.occupancy.evil`
/// resolves to `None`, so it inherits nothing from
/// `plugin.world-engine.occupancy`. A prefix match would let a topic minted
/// under a gated prefix be handed that prefix's capability, which turns the
/// gate into a naming convention.
pub fn atlas_topic_subscribe_capability(topic: &str) -> Option<&'static str> {
    match topic {
        PLUGIN_ATLAS_POSE_TOPIC => Some(ATLAS_POSE_READ_CAP),
        PLUGIN_ATLAS_POINTCLOUD_TOPIC
        | PLUGIN_ATLAS_OCCUPANCY_TOPIC
        | PLUGIN_ATLAS_SPLAT_TOPIC
        | PLUGIN_ATLAS_MESH_TOPIC => Some(ATLAS_WORLD_READ_CAP),
        _ => None,
    }
}

/// The capability gating the world-model artifact descriptors (cloud /
/// occupancy / splat / mesh). Declared by the extension itself
/// (`agent.declared_capabilities`): a descriptor names where a reconstruction
/// of the flown area can be fetched.
pub const ATLAS_WORLD_READ_CAP: &str = "plugin.world-engine.world.read";

/// The capability gating the live ~10 Hz world pose. It is the same class of
/// data as the vehicle telemetry a plugin reads under `telemetry.read`,
/// expressed in the world frame.
pub const ATLAS_POSE_READ_CAP: &str = "telemetry.read";

/// Whether a bearer can carry a full keyframe, from the bearer name the
/// forwarder already reports (`direct-lan` / `wfb-relay` / `cloud`).
///
/// This is a pure property of the bearer, not a runtime measurement: the WFB
/// relay's datagram ceiling is 1300 bytes against a multi-MB JPEG keyframe, so
/// on a no-LAN field topology the ladder degrades to pose and status only —
/// i.e. NO world model — while the operator surface would otherwise report
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

/// Schema version stamped on an [`AtlasForwardStatus`] by its writer.
pub const ATLAS_FORWARD_STATUS_VERSION: u16 = 1;

/// The drone-side forwarder's transport facts: the compute node it resolved,
/// the bearer currently carrying the stream, and when a keyframe last left the
/// drone. Only the forwarder knows these (the capture service never sees the
/// bearer ladder), so they are folded into the atlas state the extension
/// publishes. The bearer is already in the GCS vocabulary (`direct-lan` /
/// `wfb-relay` / `cloud`); every field is optional so a forwarder with no
/// resolved node reports an honest "nothing yet".
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AtlasForwardStatus {
    /// Schema version, stamped [`ATLAS_FORWARD_STATUS_VERSION`] on write. An
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
#[cfg(test)]
mod tests;
#[cfg(test)]
mod tests_bus;

pub use descriptors::{
    Generation, MeshDescriptor, OccupancyDescriptor, OccupancyField, PointCloudDescriptor,
    SplatDescriptor,
};
pub use keyframe::{
    CameraRole, CaptureState, CaptureStatus, ImageEncoding, KeyframeBudget, KeyframeEnvelope,
    KeyframeFlags, KeyframeImage, KeyframeReject, KeyframeTier, Pose, PoseDescriptor, PoseSource,
    TimeAlignment, VioHealth, KEYFRAME_MAX_SIGMA_NS, POSE_COV_LEN,
};

/// The on-wire version stamped on every [`AtlasEvent`] envelope. Every atlas
/// message crosses as an envelope, and the inner topic structs are decoded from
/// its `payload` only after the envelope decodes, so versioning the envelope
/// gates the whole contract.
///
/// Version 2: keyframes carry the capture contract's camera model, geo anchor,
/// timebase-stamped exposure time and IMU samples (`t_ns`, `seq`), and every
/// artifact descriptor carries a placement.
pub const ATLAS_ENVELOPE_VERSION: u16 = 2;

/// Errors raised decoding an [`AtlasEvent`] envelope.
#[derive(Debug, Error)]
pub enum AtlasError {
    /// The msgpack body did not decode into an envelope (also fires when the
    /// required version field is absent).
    #[error("msgpack decode error: {0}")]
    Decode(#[from] rmp_serde::decode::Error),
    /// The envelope carried a version this build does not speak.
    #[error("unsupported atlas envelope version {got} (this build speaks {ours})")]
    Version { got: u16, ours: u16 },
}

/// One framed message on the agent's local atlas bus. The capture service binds
/// a single broadcast socket and tags every message with the topic it belongs to
/// (one of the `atlas.*` / `plugin.world-engine.*` constants above) so a subscriber can
/// demultiplex pose, keyframe, and capture-state streams off one connection.
/// `payload` is the topic's own struct already msgpack-encoded (e.g. a
/// [`KeyframeEnvelope`] for [`PLUGIN_ATLAS_POSE_TOPIC`]'s sibling keyframe lane),
/// so the wrapper stays agnostic to which struct it carries.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AtlasEvent {
    /// On-wire envelope version, the first field. A decoder that sees a version
    /// it does not speak rejects the frame in [`AtlasEvent::decode`] rather than
    /// silently mis-parsing the payload; a frame missing this field also fails to
    /// decode (no serde default). Stamped by [`AtlasEvent::new`]; held equal to
    /// [`ATLAS_ENVELOPE_VERSION`].
    #[serde(rename = "v")]
    pub v: u16,
    pub topic: String,
    /// The capturing drone's device id, stamped by the drone-side forwarder as
    /// the event leaves the drone (the single choke point every bearer passes
    /// through). The compute node reads it to attribute a reconstruct job to the
    /// drone that captured it (the world-model job's `deviceId`). Additive +
    /// optional: an event on the local publish bus (before egress) omits it, and
    /// a receiver decoding an older frame defaults it to `None`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub device_id: Option<String>,
    pub payload: Vec<u8>,
}

impl AtlasEvent {
    /// Build an envelope stamped with the current [`ATLAS_ENVELOPE_VERSION`].
    /// `device_id` is `None` on the local publish bus and `Some(..)` once the
    /// drone-side forwarder stamps it on egress.
    pub fn new(topic: impl Into<String>, device_id: Option<String>, payload: Vec<u8>) -> Self {
        Self {
            v: ATLAS_ENVELOPE_VERSION,
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
    /// gates the whole atlas contract: a subscriber only reads the inner topic
    /// struct out of `payload` once this succeeds.
    pub fn decode(bytes: &[u8]) -> Result<Self, AtlasError> {
        let event: AtlasEvent = rmp_serde::from_slice(bytes)?;
        if event.v != ATLAS_ENVELOPE_VERSION {
            return Err(AtlasError::Version {
                got: event.v,
                ours: ATLAS_ENVELOPE_VERSION,
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
);
