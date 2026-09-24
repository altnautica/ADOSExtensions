/**
 * `mapComputeSlice`: maps the `compute` slice of the extension's agent state
 * (the node's heartbeat report) into the compute store's cluster slice. Covers
 * the no-compute-fields no-op, full + sparse mapping (merge over current), slave
 * coercion, and malformed-entry dropping.
 */

import { describe, it, expect } from "vitest";
import { mapComputeSlice } from "../../../src/lib/status/compute";
import { EMPTY_COMPUTE_CLUSTER } from "../../../src/stores/compute-store";

const current = { cluster: { ...EMPTY_COMPUTE_CLUSTER } };

describe("mapComputeSlice — no compute fields", () => {
  it("returns null when the slice carries nothing compute-shaped", () => {
    expect(mapComputeSlice({}, current, 1)).toBeNull();
    expect(mapComputeSlice({ profile: "workstation" }, current, 1)).toBeNull();
    // Wrong-typed fields are not compute facts either.
    expect(
      mapComputeSlice({ computeRole: "", computeQueueDepth: "3" }, current, 1),
    ).toBeNull();
  });

  it("treats an empty slave list as a statement (the cluster lost its slaves)", () => {
    const prior = {
      cluster: {
        ...EMPTY_COMPUTE_CLUSTER,
        slaves: [{ nodeId: "node-b", accelerators: [], workersIdle: 1, queueDepth: 0 }],
      },
    };
    expect(mapComputeSlice({ computeClusterSlaves: [] }, prior, 2)?.cluster.slaves).toEqual([]);
  });
});

describe("mapComputeSlice — full mapping", () => {
  it("maps role, queue, active, idle, aggregate, and master id", () => {
    const patch = mapComputeSlice(
      {
        computeRole: "master",
        computeClusterMasterId: "node-a",
        computeQueueDepth: 3,
        computeActiveJobs: 1,
        computeActiveSessions: 4,
        computeWorkersIdle: 2,
        computeClusterAggregateWorkersIdle: 6,
      },
      current,
      1234,
    );
    expect(patch?.cluster).toMatchObject({
      role: "master",
      masterId: "node-a",
      queueDepth: 3,
      activeJobs: 1,
      activeSessions: 4,
      workersIdle: 2,
      aggregateWorkersIdle: 6,
      slaves: [],
      updatedAt: 1234,
    });
  });

  it("coerces the slaves array and drops malformed entries", () => {
    const patch = mapComputeSlice(
      {
        computeRole: "master",
        computeClusterSlaves: [
          { nodeId: "node-b", accelerators: ["cuda:0", 7], workersIdle: 4, queueDepth: 0 },
          { accelerators: ["mps"], workersIdle: 1, queueDepth: 0 }, // no nodeId -> dropped
          "not-an-object", // dropped
          ["node-x"], // an array is not a row -> dropped
          { nodeId: "node-c" }, // missing counters -> null (not reported), accelerators []
        ],
      },
      current,
      5,
    );
    expect(patch?.cluster.slaves).toEqual([
      { nodeId: "node-b", accelerators: ["cuda:0"], workersIdle: 4, queueDepth: 0 },
      { nodeId: "node-c", accelerators: [], workersIdle: null, queueDepth: null },
    ]);
  });
});

describe("mapComputeSlice — sparse report merges over current", () => {
  it("preserves prior values for fields absent in this report", () => {
    const prior = {
      cluster: {
        role: "master",
        masterId: "node-a",
        queueDepth: 5,
        activeJobs: 2,
        activeSessions: 3,
        workersIdle: 0,
        aggregateWorkersIdle: 0,
        slaves: [
          { nodeId: "node-b", accelerators: ["cuda:0"], workersIdle: 4, queueDepth: 0 },
        ],
        updatedAt: 100,
      },
    };
    // Only queue depth changes; everything else (incl. the slave list + the
    // serving-sessions count) is kept.
    const patch = mapComputeSlice({ computeQueueDepth: 7 }, prior, 200);
    expect(patch?.cluster).toMatchObject({
      role: "master",
      masterId: "node-a",
      queueDepth: 7,
      activeJobs: 2,
      activeSessions: 3,
      slaves: prior.cluster.slaves,
      updatedAt: 200,
    });
  });

  it("ignores non-finite / wrong-typed numerics (treated as absent)", () => {
    const patch = mapComputeSlice(
      {
        computeRole: "slave",
        computeQueueDepth: "nan",
        computeWorkersIdle: Number.NaN,
      },
      current,
      9,
    );
    // role applied; the bad numerics fall back to current (null).
    expect(patch?.cluster.role).toBe("slave");
    expect(patch?.cluster.queueDepth).toBeNull();
    expect(patch?.cluster.workersIdle).toBeNull();
  });
});
