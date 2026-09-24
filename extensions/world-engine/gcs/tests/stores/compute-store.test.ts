/**
 * The compute store's GPU history: a bounded, oldest-first window of finite
 * utilisation samples per node, which a report with no utilisation reading
 * never pads.
 */

import { beforeEach, describe, expect, it } from "vitest";

import {
  EMPTY_COMPUTE_CLUSTER,
  EMPTY_COMPUTE_NODE,
  GPU_HISTORY_MAX,
  useComputeStore,
  type ComputeGpuInfo,
} from "../../src/stores/compute-store";

function gpu(utilizationPct: number | null): ComputeGpuInfo {
  return { name: "GPU", cores: null, unifiedMemoryMb: null, metal: null, utilizationPct };
}

const node = (deviceId: string) => useComputeStore.getState().nodes[deviceId] ?? EMPTY_COMPUTE_NODE;

beforeEach(() => {
  useComputeStore.setState({ nodes: {} });
});

describe("compute-store gpu history", () => {
  it("keeps the newest GPU_HISTORY_MAX samples, oldest first", () => {
    const { setGpu } = useComputeStore.getState();
    for (let i = 0; i < GPU_HISTORY_MAX + 5; i++) setGpu("ws", gpu(i), i);
    const history = node("ws").gpuHistory;
    expect(history).toHaveLength(GPU_HISTORY_MAX);
    expect(history[0]).toBe(5);
    expect(history[history.length - 1]).toBe(GPU_HISTORY_MAX + 4);
  });

  it("does not record a sample for a report with no finite utilisation", () => {
    const { setGpu } = useComputeStore.getState();
    setGpu("ws", gpu(40), 1);
    setGpu("ws", gpu(null), 2);
    setGpu("ws", gpu(Number.NaN), 3);
    setGpu("ws", null, 4);
    expect(node("ws").gpuHistory).toEqual([40]);
    // The latest report still replaces the snapshot and its age.
    expect(node("ws").gpu).toBeNull();
    expect(node("ws").gpuUpdatedAt).toBe(4);
  });

  it("keeps a zero-utilisation sample (an idle GPU is a reading)", () => {
    useComputeStore.getState().setGpu("ws", gpu(0), 1);
    expect(node("ws").gpuHistory).toEqual([0]);
  });

  it("keeps each node's history apart", () => {
    const { setGpu } = useComputeStore.getState();
    setGpu("a", gpu(10), 1);
    setGpu("b", gpu(90), 1);
    expect(node("a").gpuHistory).toEqual([10]);
    expect(node("b").gpuHistory).toEqual([90]);
  });

  it("a cluster update keeps the GPU state and a GPU update keeps the cluster", () => {
    const { setGpu, setCluster } = useComputeStore.getState();
    setGpu("ws", gpu(20), 1);
    setCluster("ws", { ...EMPTY_COMPUTE_CLUSTER, role: "master", updatedAt: 2 });
    setGpu("ws", gpu(30), 3);
    expect(node("ws").cluster.role).toBe("master");
    expect(node("ws").gpuHistory).toEqual([20, 30]);
  });
});
