//! The mapping extensions' wire contract (Capture, Reconstruct, Navigation).
//!
//! - [`capture`]: the capture and reconstruction contract (topics, the framed
//!   [`capture::NodeEvent`] envelope, keyframes, reconstruction descriptors
//!   and the navigation payloads).
//! - [`capture_state`]: the `capture` member of Capture's `status` telemetry.
//! - [`compute`]: the compute-node job, dataset and cluster types.
//! - [`edit`]: the point-cloud editor's job params, operations and provenance.
//! - [`node_credential`]: the scoped credential a compute node issues a drone
//!   or ground station, its lanes, and the installed-credential store.
//! - [`offload`]: the perception-offload frame reference and detection batch.
//! - [`paths`]: where one extension's processes find each other on one node.
//!
//! No runtime: the only I/O here is the credential store's small file read
//! and atomic write, so every mapping extension binary (and its tests) links this
//! crate without pulling an async runtime.

pub mod capture;
pub mod capture_ctl;
pub mod capture_state;
pub mod compute;
pub mod edit;
pub mod node_credential;
pub mod offload;
pub mod paths;

/// The telemetry channel the extension publishes its merged state on
/// (`telemetry.extend`), read by the GCS half as `ctx.telemetry.subscribe`.
pub const STATUS_CHANNEL: &str = "status";
