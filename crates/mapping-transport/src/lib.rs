//! Mapping transport: the capture stream lane plus the device-side client of a
//! reconstruction node.
//!
//! The keyframe stream (drone -> compute) and the reconstruction descriptor stream
//! (compute -> GCS) get their own lane so the reconstruction never competes with
//! the bounded MAVLink queue or the video pipeline. The same framed
//! [`NodeEvent`] rides any bearer; the bearer is chosen by topology through a
//! priority failover ladder:
//!
//! 1. **Direct LAN/WiFi** ([`LanHttpBearer`]) — first-class. The drone, compute
//!    node, and GCS share a network; keyframes stream direct over LAN HTTP.
//! 2. **WFB relay** ([`WfbRelayBearer`]) — the auxiliary application stream on
//!    the radio link, bridged back onto the LAN by the ground station.
//! 3. **Cloud relay** — an opt-in lane the link builds over the plugin host's
//!    cloud publish method.
//!
//! [`LoopbackBearer`] is the in-process bearer for tests and the same-host case.
//! [`WorldBroadcaster`] is the compute-side fan-out a reconstruction consumer (the
//! GCS Live World, or the drone republishing onto its own plugin bus) subscribes
//! to over a per-device WebSocket. All carry the identical envelope, so swapping
//! a bearer never changes the reconstruction contract.
//!
//! The compute->GCS lane carries generation-versioned reconstruction DESCRIPTORS,
//! not splat deltas. See [`world_stream`] for why: SPZ and SOG are whole-scene
//! containers with global quantisation and Morton ordering, and neither the
//! formats nor the Khronos glTF extensions define an incremental append or delta
//! codec, so a delta lane would have required inventing a codec nothing else
//! reads.
//!
//! The drone-side compute client lives here too, so the light drone binary
//! reaches a node without linking the node's reconstruction stack:
//! [`ComputeClient`] (the job API), [`BulkClient`] (the resumable session
//! upload lane), [`mdns`] (the compute-node service type and the pick over a
//! host browse), and the perception-offload orchestrator with its
//! safety-gated return bridge. So do the pieces the capture and reconstruct
//! processes both build on: the plugin-host facade they are tested through
//! ([`host`]), its aux, cloud and detection adapters ([`sdk`]), the paired
//! node's resolution ([`compute_node`], [`lan_node`]), the node-credential
//! store and its route ([`credentials`]), and the error replies of their
//! routes and tools ([`reply`]).

mod bearer;
mod bulk;
mod bulk_plan;
mod client;
pub mod compute_node;
pub mod credentials;
mod error;
pub mod host;
mod job_status;
pub mod jobs;
mod ladder;
mod lan_http;
pub mod lan_node;
mod loopback;
/// The compute node's job-API service type and TXT record, and the pick of a
/// node from the host's `mdns.browse` answer.
pub mod mdns;
mod offload_bridge;
mod offload_client;
mod offload_orchestrator;
pub mod reply;
pub mod sdk;
/// A recording [`host::Host`] for tests.
#[cfg(any(test, feature = "testing"))]
pub mod testing;
pub mod tools;
mod unix_http;
mod wfb_relay;
mod world_stream;

pub use bearer::{BearerKind, EventBearer};
pub use bulk::BulkClient;
pub use bulk_plan::{
    BulkConfig, BulkError, Committed, PlanFile, SessionPlan, CHECKSUMS_FILE, MANIFEST_FILE,
    MAX_CHUNK_BYTES,
};
pub use client::{node_identity, ClientError, ComputeClient, PinnedHost};
pub use error::TransportError;
pub use job_status::{JobError, JobGpu, JobStage, JobStatus};
pub use ladder::BearerLadder;
pub use lan_http::{event_router, LanHttpBearer};
pub use loopback::LoopbackBearer;
pub use mdns::{
    compute_advert_txt, pick_compute_node, ResolvedComputeNode, COMPUTE_SERVICE, DEVICE_ID_TXT,
};
pub use offload_bridge::{CameraGeometry, DetectionPublisher, OffloadReturnBridge};
pub use offload_client::stream_offload_detections;
pub use offload_orchestrator::{
    run_offload_orchestrator, DetectionTee, NodeEndpoint, OrchestratorConfig,
};
pub use unix_http::{serve_unix, UnixPeer};
pub use wfb_relay::{AuxLane, WfbRelayBearer, WFB_MAX_DATAGRAM};
pub use world_stream::{world_ws_path, world_ws_router, WorldBroadcaster, WORLD_WS_ROUTE};

// Re-export the framed event the lane carries so callers get one import surface.
pub use ados_mapping_protocol::capture::NodeEvent;
