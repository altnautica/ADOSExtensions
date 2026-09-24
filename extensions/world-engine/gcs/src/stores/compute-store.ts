/**
 * @module compute-store
 * @description Per-node compute runtime telemetry, keyed by the compute
 * node's device id: the master/slave cluster view, this node's job-queue
 * depth and worker occupancy, and its GPU snapshot with a short utilisation
 * history for the sparkline.
 *
 * Fed from the `compute` slice of the extension's agent state on a
 * workstation or compute node (see `use-world-engine-state`). A node's slice
 * stays empty (every field null / no slaves) until its compute service
 * reports, so the cards render an "awaiting heartbeat" state.
 */

import { create } from "zustand";

/** One slave node registered with the master in the compute cluster. */
export interface ComputeSlave {
  nodeId: string;
  /** Accelerator ids the slave offers (e.g. "cuda:0", "mps"). */
  accelerators: string[];
  /** Null when the slave did not report it. */
  workersIdle: number | null;
  /** Null when the slave did not report it. */
  queueDepth: number | null;
}

/**
 * GPU / accelerator snapshot for the focused compute node, sampled from its
 * `GET /api/compute/status` sidecar each poll. Null until a poll lands; each
 * field is independently null when the node can't report it (a headless box
 * with no GPU, or a backend that exposes no utilization counter). The Mac/
 * Apple-Silicon path reports a `metal` family string (e.g. "Metal 4") and a
 * live `utilizationPct`.
 */
export interface ComputeGpuInfo {
  name: string | null;
  cores: number | null;
  unifiedMemoryMb: number | null;
  /** Metal feature-set family string (e.g. "Metal 4"), or null off Apple GPUs. */
  metal: string | null;
  /** Live GPU utilisation, 0..100, or null when the backend exposes none. */
  utilizationPct: number | null;
}

/** The compute node's runtime status: its own queue + workers, and the
 * cluster (master id + aggregate idle capacity + registered slaves) it
 * fronts. Every scalar is null until a compute heartbeat populates it. */
export interface ComputeClusterStatus {
  /** "master" | "slave", or null before the first heartbeat. */
  role: string | null;
  masterId: string | null;
  queueDepth: number | null;
  activeJobs: number | null;
  /** Live perception-OFFLOAD sessions this node is currently serving (a drone
   * streaming frames here for detection). Distinct from `activeJobs` (batch
   * reconstruction jobs). Null before the first heartbeat / on a node that
   * does not serve offload. */
  activeSessions: number | null;
  workersIdle: number | null;
  /** Sum of idle workers across the master and all slaves. */
  aggregateWorkersIdle: number | null;
  slaves: ComputeSlave[];
  /** Epoch ms of the heartbeat this slice was last populated from (the
   * source row's updatedAt), or null before the first heartbeat. */
  updatedAt: number | null;
}

export const EMPTY_COMPUTE_CLUSTER: ComputeClusterStatus = {
  role: null,
  masterId: null,
  queueDepth: null,
  activeJobs: null,
  activeSessions: null,
  workersIdle: null,
  aggregateWorkersIdle: null,
  slaves: [],
  updatedAt: null,
};

/** GPU utilisation samples kept for the sparkline (one per state update). */
export const GPU_HISTORY_MAX = 60;

/** One node's compute telemetry. */
export interface ComputeNodeState {
  cluster: ComputeClusterStatus;
  /** Live GPU snapshot, or null before the first report (or on a node that
   * reports no GPU). */
  gpu: ComputeGpuInfo | null;
  /** Wall-clock ms of the last GPU report, or null before the first one, so
   * the GPU card can age a reading when reports stop arriving. */
  gpuUpdatedAt: number | null;
  /** Recent finite GPU utilisation samples, oldest first. */
  gpuHistory: number[];
}

export const EMPTY_COMPUTE_NODE: ComputeNodeState = {
  cluster: EMPTY_COMPUTE_CLUSTER,
  gpu: null,
  gpuUpdatedAt: null,
  gpuHistory: [],
};

interface ComputeStoreState {
  nodes: Record<string, ComputeNodeState>;
  /** Replace a node's cluster slice (the caller passes a fully-merged slice). */
  setCluster: (deviceId: string, cluster: ComputeClusterStatus) => void;
  /** Record a GPU report (null = the node reports no GPU) at `at` ms. */
  setGpu: (deviceId: string, gpu: ComputeGpuInfo | null, at: number) => void;
}

export const useComputeStore = create<ComputeStoreState>((set) => ({
  nodes: {},
  setCluster: (deviceId, cluster) =>
    set((s) => ({
      nodes: { ...s.nodes, [deviceId]: { ...(s.nodes[deviceId] ?? EMPTY_COMPUTE_NODE), cluster } },
    })),
  setGpu: (deviceId, gpu, at) =>
    set((s) => {
      const prev = s.nodes[deviceId] ?? EMPTY_COMPUTE_NODE;
      const util = gpu?.utilizationPct;
      const gpuHistory =
        util != null && Number.isFinite(util)
          ? [...prev.gpuHistory, util].slice(-GPU_HISTORY_MAX)
          : prev.gpuHistory;
      return { nodes: { ...s.nodes, [deviceId]: { ...prev, gpu, gpuUpdatedAt: at, gpuHistory } } };
    }),
}));
