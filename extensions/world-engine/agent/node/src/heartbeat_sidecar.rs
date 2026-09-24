//! The compute half of the extension's `status` telemetry channel.
//!
//! The node publishes its cluster + queue state through the plugin host's
//! `telemetry.extend` on every heartbeat tick, as the `compute` member of the
//! `status` channel. The keys are the camelCase `cmd_droneStatus` field names
//! the GCS reads (`computeRole`, `computeClusterSlaves`, …) plus the snake_case
//! host `gpu` block. The same value backs the node's `GET /compute/status` on
//! its operator socket.

use serde::Serialize;
use world_engine_protocol::compute::ComputeHeartbeat;

use crate::{ComputeGpu, ComputeRole};

/// Schema version stamped on every [`ComputeHeartbeatSidecar`] so a reader can
/// detect a producer/reader drift.
pub const COMPUTE_HEARTBEAT_SIDECAR_VERSION: u16 = 1;

/// One slave node's capacity, in the camelCase shape the GCS expects under
/// `cmd_droneStatus.computeClusterSlaves`.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct SlaveEntry {
    pub node_id: String,
    pub accelerators: Vec<String>,
    pub workers_idle: u32,
    pub queue_depth: u32,
}

/// The flat, camelCase compute status — exactly the `cmd_droneStatus`
/// `compute*` field names, so a reader folds it with no remapping.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct ComputeHeartbeatSidecar {
    /// Schema version, stamped [`COMPUTE_HEARTBEAT_SIDECAR_VERSION`]. NOT a
    /// heartbeat field — a consumer folding the status onto a heartbeat never
    /// forwards it.
    pub version: u16,
    /// `ComputeRole` serializes lowercase ("master" / "slave").
    pub compute_role: ComputeRole,
    pub compute_cluster_master_id: String,
    pub compute_queue_depth: u32,
    pub compute_active_jobs: u32,
    /// Live streaming perception-offload sessions (a node streaming to N drones).
    /// Additive; a reader that does not know the key ignores it.
    pub compute_active_sessions: u32,
    pub compute_workers_idle: u32,
    pub compute_cluster_aggregate_workers_idle: u32,
    pub compute_cluster_slaves: Vec<SlaveEntry>,
    /// The host GPU block (identity + live utilisation). Serialized under `gpu`;
    /// its own snake_case keys (`unified_memory_mb`, `utilization_pct`) are the
    /// wire shape the GCS compute card reads. All-`null` on a non-macOS node.
    pub gpu: ComputeGpu,
    /// Epoch ms this status was sampled, so a reader can tell a fresh value
    /// from a stale one. NOT a heartbeat field.
    pub generated_at_ms: i64,
}

impl ComputeHeartbeatSidecar {
    /// Map the engine's heartbeat + a host-GPU sample to the wire shape.
    /// `now_ms` is the local epoch-ms sample time.
    pub fn from_heartbeat(hb: &ComputeHeartbeat, gpu: ComputeGpu, now_ms: i64) -> Self {
        Self {
            version: COMPUTE_HEARTBEAT_SIDECAR_VERSION,
            generated_at_ms: now_ms,
            compute_role: hb.role,
            compute_cluster_master_id: hb.cluster.master_id.clone(),
            compute_queue_depth: hb.queue_depth,
            compute_active_jobs: hb.active_jobs,
            compute_active_sessions: hb.active_sessions,
            compute_workers_idle: hb.workers_idle,
            compute_cluster_aggregate_workers_idle: hb.cluster.aggregate_workers_idle,
            compute_cluster_slaves: hb
                .cluster
                .slaves
                .iter()
                .map(|s| SlaveEntry {
                    node_id: s.node_id.clone(),
                    accelerators: s.accelerators.clone(),
                    workers_idle: s.workers_idle,
                    queue_depth: s.queue_depth,
                })
                .collect(),
            gpu,
        }
    }
}

/// The `status` telemetry payload the node publishes: its compute status, and
/// `atlas: null` (the drone half owns the atlas member).
pub fn status_payload(compute: &ComputeHeartbeatSidecar) -> serde_json::Value {
    serde_json::json!({ "atlas": null, "compute": compute })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ClusterDescriptor, SlaveDescriptor};

    fn heartbeat() -> ComputeHeartbeat {
        ComputeHeartbeat {
            role: ComputeRole::Master,
            cluster: ClusterDescriptor {
                master_id: "node-master".into(),
                slaves: vec![SlaveDescriptor {
                    node_id: "node-slave-1".into(),
                    accelerators: vec!["cuda:0".into()],
                    workers_idle: 2,
                    queue_depth: 1,
                }],
                aggregate_workers_idle: 6,
            },
            queue_depth: 3,
            active_jobs: 1,
            workers_idle: 4,
            active_sessions: 5,
        }
    }

    fn sample_gpu() -> ComputeGpu {
        ComputeGpu {
            name: Some("Apple M1 Pro".into()),
            cores: Some(16),
            unified_memory_mb: Some(32768),
            metal: Some("Metal 3".into()),
            utilization_pct: Some(12.5),
        }
    }

    #[test]
    fn the_sidecar_uses_the_camelcase_cmd_drone_status_field_names() {
        let v = serde_json::to_value(ComputeHeartbeatSidecar::from_heartbeat(
            &heartbeat(),
            sample_gpu(),
            1700,
        ))
        .unwrap();
        assert_eq!(v["computeRole"], "master");
        assert_eq!(v["computeClusterMasterId"], "node-master");
        assert_eq!(v["computeQueueDepth"], 3);
        assert_eq!(v["computeActiveJobs"], 1);
        assert_eq!(v["computeActiveSessions"], 5);
        assert_eq!(v["computeWorkersIdle"], 4);
        assert_eq!(v["computeClusterAggregateWorkersIdle"], 6);
        assert_eq!(v["generatedAtMs"], 1700);
        assert_eq!(v["version"], COMPUTE_HEARTBEAT_SIDECAR_VERSION);
        let slave = &v["computeClusterSlaves"][0];
        assert_eq!(slave["nodeId"], "node-slave-1");
        assert_eq!(slave["accelerators"][0], "cuda:0");
        assert_eq!(slave["workersIdle"], 2);
        assert_eq!(slave["queueDepth"], 1);
        // The gpu block rides under `gpu` with its own snake_case wire keys (it is
        // NOT camelCased by the sidecar's rename — nested types keep their own).
        assert_eq!(v["gpu"]["name"], "Apple M1 Pro");
        assert_eq!(v["gpu"]["cores"], 16);
        assert_eq!(v["gpu"]["unified_memory_mb"], 32768);
        assert_eq!(v["gpu"]["metal"], "Metal 3");
        assert_eq!(v["gpu"]["utilization_pct"], 12.5);
    }

    #[test]
    fn the_status_payload_carries_the_compute_status_and_a_null_atlas() {
        let sidecar = ComputeHeartbeatSidecar::from_heartbeat(&heartbeat(), sample_gpu(), 1700);
        let v = status_payload(&sidecar);
        assert!(v["atlas"].is_null());
        assert_eq!(v["compute"], serde_json::to_value(&sidecar).unwrap());
        assert_eq!(v["compute"]["computeRole"], "master");
        assert_eq!(v["compute"]["gpu"]["cores"], 16);
    }
}
