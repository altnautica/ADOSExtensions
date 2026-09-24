import { describe, expect, it } from "vitest";
import type { PluginRecord } from "@altnautica/plugin-sdk/inline";

import { coerceCloudJob } from "../../src/hooks/use-cloud-jobs";

function record(data: unknown, over: Partial<PluginRecord> = {}): PluginRecord {
  return {
    collection: "jobs",
    key: "job-1",
    deviceId: "drone-1",
    data,
    updatedAt: 5000,
    writtenBy: "agent",
    ...over,
  };
}

describe("coerceCloudJob", () => {
  it("maps a recorded job, keyed by the record key", () => {
    const metadata = { gaussianCount: 900_000, backend: "brush", viewerHint: "splat" };
    expect(
      coerceCloudJob(
        record({
          deviceId: "drone-1",
          computeNodeId: "ws-1",
          kind: "splat",
          status: "done",
          sessionId: "atlas-drone-1-1000",
          outputUrl: "https://cdn.example/worlds/job-1.ply",
          metadata,
          createdAt: 1234,
        }),
      ),
    ).toEqual({
      id: "job-1",
      deviceId: "drone-1",
      computeNodeId: "ws-1",
      kind: "splat",
      status: "done",
      sessionId: "atlas-drone-1-1000",
      outputUrl: "https://cdn.example/worlds/job-1.ply",
      metadata,
      createdAt: 1234,
    });
  });

  it("falls back to the record's device and write time", () => {
    const job = coerceCloudJob(record({ kind: "cloud", status: "running" }, { deviceId: "drone-9" }));
    expect(job?.deviceId).toBe("drone-9");
    expect(job?.createdAt).toBe(5000);
  });

  it("prefers the device the job names over the record's", () => {
    expect(coerceCloudJob(record({ deviceId: "drone-2" }))?.deviceId).toBe("drone-2");
  });

  it("reads absent or empty optional fields as unknown, never as empty strings", () => {
    const job = coerceCloudJob(
      record({ computeNodeId: "", sessionId: 7, outputUrl: null, kind: 3 }),
    );
    expect(job).toMatchObject({
      computeNodeId: null,
      sessionId: null,
      outputUrl: null,
      kind: "",
      status: "",
      metadata: null,
    });
  });

  it("rejects a record that is not a job", () => {
    expect(coerceCloudJob(record(null))).toBeNull();
    expect(coerceCloudJob(record("job"))).toBeNull();
    expect(coerceCloudJob(record([{ kind: "splat" }]))).toBeNull();
    // A job must belong to some drone.
    expect(coerceCloudJob(record({ kind: "splat" }, { deviceId: null }))).toBeNull();
  });
});
