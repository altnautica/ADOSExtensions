/**
 * `applyReconstructState`: one node report fans out into the compute cluster
 * slice and the GPU snapshot, keyed by the reporting node and merged over
 * what that node said before.
 */

import { beforeEach, describe, expect, it } from "vitest";

import { applyReconstructState } from "../../src/hooks/use-reconstruct-state";
import { EMPTY_COMPUTE_NODE, useComputeStore } from "../../src/stores/compute-store";

const compute = (deviceId: string) => useComputeStore.getState().nodes[deviceId] ?? EMPTY_COMPUTE_NODE;

beforeEach(() => {
  useComputeStore.setState({ nodes: {} });
});

describe("applyReconstructState", () => {
  it("fans the compute slice into the cluster and the gpu block into the snapshot", () => {
    applyReconstructState(
      "ws-1",
      {
        compute: {
          computeRole: "master",
          computeQueueDepth: 2,
          gpu: { name: "RTX", cores: 10, utilization_pct: 55 },
        },
      },
      900,
    );
    const node = compute("ws-1");
    expect(node.cluster).toMatchObject({ role: "master", queueDepth: 2, updatedAt: 900 });
    expect(node.gpu).toMatchObject({ name: "RTX", cores: 10, utilizationPct: 55 });
    expect(node.gpuUpdatedAt).toBe(900);
    expect(node.gpuHistory).toEqual([55]);
    expect(useComputeStore.getState().nodes["ws-2"]).toBeUndefined();
  });

  it("records a gpu-only report without inventing cluster facts", () => {
    applyReconstructState("ws-1", { compute: { gpu: { utilization_pct: 10 } } }, 3);
    const node = compute("ws-1");
    expect(node.cluster.updatedAt).toBeNull();
    expect(node.gpuHistory).toEqual([10]);
  });

  it("records a node that says it has no gpu as a null snapshot, keeping the cluster", () => {
    applyReconstructState("ws-1", { compute: { computeRole: "master" } }, 1);
    applyReconstructState("ws-1", { compute: { gpu: null } }, 2);
    const node = compute("ws-1");
    expect(node.gpu).toBeNull();
    expect(node.gpuUpdatedAt).toBe(2);
    expect(node.cluster.role).toBe("master");
  });

  it("leaves the gpu snapshot alone when a report omits the gpu key", () => {
    applyReconstructState("ws-1", { compute: { gpu: { name: "RTX" } } }, 1);
    applyReconstructState("ws-1", { compute: { computeQueueDepth: 4 } }, 2);
    const node = compute("ws-1");
    expect(node.gpu?.name).toBe("RTX");
    expect(node.gpuUpdatedAt).toBe(1);
  });

  it("ignores a report that is not an object, and a slice that is not one", () => {
    for (const payload of [null, "status", 3, [{ compute: { computeRole: "master" } }]]) {
      applyReconstructState("ws-1", payload, 1);
    }
    applyReconstructState("ws-1", { compute: "master" }, 1);
    expect(useComputeStore.getState().nodes).toEqual({});
  });
});
