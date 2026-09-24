/**
 * `applyWorldEngineState`: one merged state report from a node's agent half
 * fans out into the atlas store (the `atlas` slice), the compute cluster slice
 * and the GPU snapshot (the `compute` slice), keyed by the reporting node and
 * merged over what that node said before.
 */

import { beforeEach, describe, expect, it } from "vitest";

import { applyWorldEngineState } from "../../src/hooks/use-world-engine-state";
import { EMPTY_ATLAS_LIVE, useAtlasStore } from "../../src/stores/atlas-store";
import { EMPTY_COMPUTE_NODE, useComputeStore } from "../../src/stores/compute-store";

const live = (deviceId: string) => useAtlasStore.getState().live[deviceId] ?? EMPTY_ATLAS_LIVE;
const compute = (deviceId: string) =>
  useComputeStore.getState().nodes[deviceId] ?? EMPTY_COMPUTE_NODE;

beforeEach(() => {
  useAtlasStore.setState({ live: {} });
  useComputeStore.setState({ nodes: {} });
});

describe("applyWorldEngineState", () => {
  it("fans the atlas slice into the reporting drone's live state", () => {
    applyWorldEngineState(
      "drone-1",
      { atlas: { state: "capturing", keyframesIngested: 12, bearer: "direct-lan" } },
      500,
    );
    expect(live("drone-1")).toMatchObject({
      state: "capturing",
      keyframesIngested: 12,
      bearer: "direct-lan",
      updatedAt: 500,
    });
    // Another node's state is untouched.
    expect(useAtlasStore.getState().live["drone-2"]).toBeUndefined();
    expect(useComputeStore.getState().nodes["drone-1"]).toBeUndefined();
  });

  it("merges a sparse atlas report over the node's previous one", () => {
    applyWorldEngineState("drone-1", { atlas: { state: "capturing", sessionId: "s1" } }, 1);
    applyWorldEngineState("drone-1", { atlas: { keyframesIngested: 40 } }, 2);
    expect(live("drone-1")).toMatchObject({
      state: "capturing",
      sessionId: "s1",
      keyframesIngested: 40,
      updatedAt: 2,
    });
  });

  it("fans the compute slice into the cluster and the gpu block into the snapshot", () => {
    applyWorldEngineState(
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
  });

  it("records a gpu-only report without inventing cluster facts", () => {
    applyWorldEngineState("ws-1", { compute: { gpu: { utilization_pct: 10 } } }, 3);
    const node = compute("ws-1");
    expect(node.cluster.updatedAt).toBeNull();
    expect(node.gpuHistory).toEqual([10]);
  });

  it("records a node that says it has no gpu as a null snapshot, keeping the cluster", () => {
    applyWorldEngineState("ws-1", { compute: { computeRole: "master" } }, 1);
    applyWorldEngineState("ws-1", { compute: { gpu: null } }, 2);
    const node = compute("ws-1");
    expect(node.gpu).toBeNull();
    expect(node.gpuUpdatedAt).toBe(2);
    expect(node.cluster.role).toBe("master");
  });

  it("leaves the gpu snapshot alone when a report omits the gpu key", () => {
    applyWorldEngineState("ws-1", { compute: { gpu: { name: "RTX" } } }, 1);
    applyWorldEngineState("ws-1", { compute: { computeQueueDepth: 4 } }, 2);
    const node = compute("ws-1");
    expect(node.gpu?.name).toBe("RTX");
    expect(node.gpuUpdatedAt).toBe(1);
  });

  it("applies both slices of one report", () => {
    applyWorldEngineState(
      "node-1",
      { atlas: { state: "idle" }, compute: { computeActiveJobs: 1 } },
      7,
    );
    expect(live("node-1").state).toBe("idle");
    expect(compute("node-1").cluster.activeJobs).toBe(1);
  });

  it("ignores a report that is not an object, and slices that are not objects", () => {
    for (const payload of [null, "status", 3, [{ atlas: { state: "capturing" } }]]) {
      applyWorldEngineState("drone-1", payload, 1);
    }
    applyWorldEngineState("drone-1", { atlas: ["capturing"], compute: "master" }, 1);
    expect(useAtlasStore.getState().live).toEqual({});
    expect(useComputeStore.getState().nodes).toEqual({});
  });

  it("does not touch the store for an empty atlas slice", () => {
    applyWorldEngineState("drone-1", { atlas: { state: "capturing" } }, 1);
    const before = useAtlasStore.getState().live;
    applyWorldEngineState("drone-1", { atlas: {} }, 2);
    expect(useAtlasStore.getState().live).toBe(before);
  });
});
