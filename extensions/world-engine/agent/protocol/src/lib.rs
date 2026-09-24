//! World Engine wire contract.
//!
//! - [`atlas`]: the world-model capture and reconstruction contract (topics,
//!   the framed [`atlas::AtlasEvent`] envelope, keyframes and world-model
//!   descriptors).
//! - [`atlas_state`]: the `atlas` half of the `status` telemetry channel.
//! - [`compute`]: the compute-node job, dataset and cluster types.
//! - [`offload`]: the perception-offload frame reference and detection batch.
//! - [`paths`]: where the World Engine processes find each other on one node.
//!
//! Pure types: nothing here performs I/O, so every World Engine binary (and
//! its tests) links this crate without pulling a runtime.

pub mod atlas;
pub mod atlas_state;
pub mod compute;
pub mod offload;
pub mod paths;

/// The extension's plugin id, as installed by the plugin host.
pub const PLUGIN_ID: &str = "com.altnautica.world-engine";

/// The last segment of [`PLUGIN_ID`]: the namespace the extension's shared
/// topics and declared capabilities live under (`plugin.world-engine.*`).
pub const PLUGIN_LEAF: &str = "world-engine";

/// The telemetry channel the extension publishes its merged state on
/// (`telemetry.extend`), read by the GCS half as `ctx.telemetry.subscribe`.
pub const STATUS_CHANNEL: &str = "status";
