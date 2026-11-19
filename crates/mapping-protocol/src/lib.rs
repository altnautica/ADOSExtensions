//! World Engine wire contract.
//!
//! - [`atlas`]: the world-model capture and reconstruction contract (topics,
//!   the framed [`atlas::AtlasEvent`] envelope, keyframes and world-model
//!   descriptors).
//! - [`atlas_state`]: the `atlas` half of the `status` telemetry channel.
//! - [`compute`]: the compute-node job, dataset and cluster types.
//! - [`node_credential`]: the scoped credential a compute node issues a drone
//!   or ground station, its lanes, and the installed-credential store.
//! - [`offload`]: the perception-offload frame reference and detection batch.
//! - [`paths`]: where the World Engine processes find each other on one node.
//!
//! No runtime: the only I/O here is the credential store's small file read
//! and atomic write, so every World Engine binary (and its tests) links this
//! crate without pulling an async runtime.

pub mod atlas;
pub mod atlas_state;
pub mod capture_ctl;
pub mod compute;
pub mod node_credential;
pub mod offload;
pub mod paths;

/// The telemetry channel the extension publishes its merged state on
/// (`telemetry.extend`), read by the GCS half as `ctx.telemetry.subscribe`.
pub const STATUS_CHANNEL: &str = "status";
