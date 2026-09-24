//! World Engine transport: the Atlas world-model stream lane plus the
//! drone-side client of a compute node.
//!
//! The keyframe stream (drone -> compute) and the world-model descriptor stream
//! (compute -> GCS) get their own lane so the world model never competes with
//! the bounded MAVLink queue or the video pipeline. The same framed
//! [`AtlasEvent`] rides any bearer; the bearer is chosen by topology through a
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
//! [`WorldBroadcaster`] is the compute-side fan-out a world-model consumer (the
//! GCS Live World, or the drone republishing onto its own plugin bus) subscribes
//! to over a per-device WebSocket. All carry the identical envelope, so swapping
//! a bearer never changes the world-model contract.
//!
//! The compute->GCS lane carries generation-versioned world-model DESCRIPTORS,
//! not splat deltas. See [`world_stream`] for why: SPZ and SOG are whole-scene
//! containers with global quantisation and Morton ordering, and neither the
//! formats nor the Khronos glTF extensions define an incremental append or delta
//! codec, so a delta lane would have required inventing a codec nothing else
//! reads.
//!
//! The drone-side compute client lives here too, so the light drone binary
//! reaches a node without linking the node's reconstruction stack:
//! [`ComputeClient`] (the job API), [`mdns`] (the compute-node service type
//! and the pick over a host browse), and the
//! perception-offload orchestrator with its safety-gated return bridge.

mod bearer;
mod client;
mod error;
mod ladder;
mod lan_http;
mod loopback;
/// The compute node's job-API service type and TXT record, and the pick of a
/// node from the host's `mdns.browse` answer.
pub mod mdns;
mod offload_bridge;
mod offload_client;
mod offload_orchestrator;
mod unix_http;
mod wfb_relay;
mod world_stream;

pub use bearer::{AtlasBearer, BearerKind};
pub use client::{ClientError, ComputeClient};
pub use error::TransportError;
pub use ladder::BearerLadder;
pub use lan_http::{atlas_event_router, LanHttpBearer};
pub use loopback::LoopbackBearer;
pub use mdns::{
    compute_advert_txt, pick_compute_node, ResolvedComputeNode, COMPUTE_SERVICE, DEVICE_ID_TXT,
};
pub use offload_bridge::{DetectionPublisher, OffloadReturnBridge};
pub use offload_client::stream_offload_detections;
pub use offload_orchestrator::{
    run_offload_orchestrator, DetectionTee, NodeEndpoint, OrchestratorConfig,
};
pub use unix_http::{serve_unix, UnixPeer};
pub use wfb_relay::{AuxLane, WfbRelayBearer, WFB_MAX_DATAGRAM};
pub use world_stream::{world_ws_path, world_ws_router, WorldBroadcaster, WORLD_WS_ROUTE};

// Re-export the framed event the lane carries so callers get one import surface.
pub use world_engine_protocol::atlas::AtlasEvent;
