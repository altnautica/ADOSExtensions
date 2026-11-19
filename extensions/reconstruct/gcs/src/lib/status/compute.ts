/**
 * @module status/compute
 * @description Maps the `compute` slice of the extension's agent state (the
 * compute node's heartbeat report: the master/slave cluster view, this node's
 * queue depth and worker occupancy, in the camelCase `compute*` shape) onto
 * the compute store's cluster slice. Pure.
 */

import type { ComputeClusterStatus, ComputeSlave } from "../../stores/compute-store";

export interface ComputeFanOutCurrent {
  cluster: ComputeClusterStatus;
}

function asNumber(v: unknown): number | null {
  return typeof v === "number" && Number.isFinite(v) ? v : null;
}

function asString(v: unknown): string | null {
  return typeof v === "string" && v.length > 0 ? v : null;
}

function coerceSlaves(raw: unknown): ComputeSlave[] {
  if (!Array.isArray(raw)) return [];
  const out: ComputeSlave[] = [];
  for (const item of raw) {
    if (!item || typeof item !== "object" || Array.isArray(item)) continue;
    const row = item as Record<string, unknown>;
    const nodeId = asString(row.nodeId);
    if (!nodeId) continue;
    out.push({
      nodeId,
      accelerators: Array.isArray(row.accelerators)
        ? row.accelerators.filter((a): a is string => typeof a === "string")
        : [],
      workersIdle: asNumber(row.workersIdle),
      queueDepth: asNumber(row.queueDepth),
    });
  }
  return out;
}

/**
 * Build a node's merged cluster slice from its compute report. Returns `null`
 * when the report carries no compute fields, so a sparse or empty report never
 * clears the last-known values.
 */
export function mapComputeSlice(
  slice: Record<string, unknown>,
  current: ComputeFanOutCurrent,
  nowMs: number,
): { cluster: ComputeClusterStatus } | null {
  const role = asString(slice.computeRole);
  const masterId = asString(slice.computeClusterMasterId);
  const queueDepth = asNumber(slice.computeQueueDepth);
  const activeJobs = asNumber(slice.computeActiveJobs);
  const activeSessions = asNumber(slice.computeActiveSessions);
  const workersIdle = asNumber(slice.computeWorkersIdle);
  const aggregateWorkersIdle = asNumber(
    slice.computeClusterAggregateWorkersIdle,
  );
  const hasSlaves = Array.isArray(slice.computeClusterSlaves);

  // Nothing compute-shaped in this heartbeat — leave the slice untouched.
  if (
    role === null &&
    masterId === null &&
    queueDepth === null &&
    activeJobs === null &&
    activeSessions === null &&
    workersIdle === null &&
    aggregateWorkersIdle === null &&
    !hasSlaves
  ) {
    return null;
  }

  // Merge over the current slice so a sparse heartbeat (e.g. only queue depth
  // changed) preserves the other last-known values.
  const cluster: ComputeClusterStatus = {
    role: role ?? current.cluster.role,
    masterId: masterId ?? current.cluster.masterId,
    queueDepth: queueDepth ?? current.cluster.queueDepth,
    activeJobs: activeJobs ?? current.cluster.activeJobs,
    activeSessions: activeSessions ?? current.cluster.activeSessions,
    workersIdle: workersIdle ?? current.cluster.workersIdle,
    aggregateWorkersIdle:
      aggregateWorkersIdle ?? current.cluster.aggregateWorkersIdle,
    slaves: hasSlaves
      ? coerceSlaves(slice.computeClusterSlaves)
      : current.cluster.slaves,
    updatedAt: nowMs,
  };

  return { cluster };
}
