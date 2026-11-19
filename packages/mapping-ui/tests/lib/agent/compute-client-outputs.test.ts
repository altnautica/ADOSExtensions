/**
 * Coercion of a compute node's job outputs: the backend is lifted from
 * `meta.backend`, the file/format/size/placement sidecar fields are carried,
 * and each artifact carries a source that reads its bytes back through the
 * node that produced it (the job's file route when the node names the file).
 */

import { describe, it, expect } from "vitest";
import { ComputeAgentClient, type ComputeOutput } from "../../../src/lib/agent/compute-client";
import { callAt, json, stubAgent } from "../../helpers/agent";

const NODE = "ws-node-1";

function byId(outs: ComputeOutput[] | null): Record<string, ComputeOutput> {
  if (!outs) throw new Error("expected outputs");
  return Object.fromEntries(outs.map((o) => [o.id, o]));
}

function get(map: Record<string, ComputeOutput>, id: string): ComputeOutput {
  const o = map[id];
  if (!o) throw new Error(`no output ${id}`);
  return o;
}

describe("ComputeAgentClient.getOutputs — field coercion", () => {
  it("lifts backend, file fields and the placement sidecar, drops junk", async () => {
    const { agent, fetch } = stubAgent(() =>
      json(200, [
        {
          id: "o-splat",
          job_id: "j1",
          kind: "splat",
          uri: "https://cdn.example/real.spz",
          file: "scene.spz",
          format: "spz",
          size_bytes: 1234,
          place_json: "scene.spz.place.json",
          meta: { gaussian_count: 900000, backend: "gsplat" },
          created_ms: 6,
        },
        { id: "o-unknown", job_id: "j1", kind: "mesh", uri: "https://cdn.example/x.glb", created_ms: 8 },
        // malformed (missing uri) → dropped
        { id: "bad", job_id: "j1", kind: "splat" },
      ]),
    );

    const outs = await new ComputeAgentClient(NODE, agent).getOutputs("j1");
    expect(callAt(fetch).path).toBe("api/compute/jobs/j1/outputs");
    expect(outs).toHaveLength(2);
    const m = byId(outs);

    const splat = get(m, "o-splat");
    expect(splat.backend).toBe("gsplat");
    expect(splat.meta?.gaussian_count).toBe(900000);
    expect(splat.file).toBe("scene.spz");
    expect(splat.format).toBe("spz");
    expect(splat.sizeBytes).toBe(1234);
    expect(splat.placeJson).toBe("scene.spz.place.json");

    const unknown = get(m, "o-unknown");
    expect(unknown.backend).toBeNull();
    expect(unknown.meta).toBeNull();
    expect(unknown.file).toBeNull();
    expect(unknown.placeJson).toBeNull();
  });

  it("ignores a non-string / empty meta.backend and reports unknown", async () => {
    const { agent } = stubAgent(() =>
      json(200, [
        { id: "o1", job_id: "j1", kind: "splat", uri: "https://cdn.example/a.splat", meta: { backend: 42 } },
        { id: "o2", job_id: "j1", kind: "splat", uri: "https://cdn.example/b.splat", meta: { backend: "" } },
      ]),
    );
    const outs = await new ComputeAgentClient(NODE, agent).getOutputs("j1");
    expect(outs?.map((o) => o.backend)).toEqual([null, null]);
  });

  it("returns null when the node is unreachable and [] on a non-array body", async () => {
    const down = stubAgent(() => Promise.reject(new Error("network")));
    expect(await new ComputeAgentClient(NODE, down.agent).getOutputs("j1")).toBeNull();

    const odd = stubAgent(() => json(200, { not: "an array" }));
    expect(await new ComputeAgentClient(NODE, odd.agent).getOutputs("j1")).toEqual([]);
  });

  it("returns null on a refusal so it is not read as an empty job", async () => {
    const { agent } = stubAgent(() => json(404, { error: "no such job" }));
    expect(await new ComputeAgentClient(NODE, agent).getOutputs("j1")).toBeNull();
  });

  it("percent-encodes the job id into the path", async () => {
    const { agent, fetch } = stubAgent(() => json(200, []));
    await new ComputeAgentClient(NODE, agent).getOutputs("a/b c");
    expect(callAt(fetch).path).toBe("api/compute/jobs/a%2Fb%20c/outputs");
  });
});

describe("ComputeAgentClient.getOutputs — artifact source", () => {
  it("reads an engine artifact through this node, ignoring the stamped host", async () => {
    const { agent, fetch } = stubAgent((path) =>
      path.startsWith("api/")
        ? json(200, [
            {
              id: "o1",
              job_id: "j1",
              kind: "splat",
              uri: "http://drifting-host.local:8092/artifacts/recon-1/output.ply",
              meta: { backend: "brush" },
            },
          ])
        : Promise.resolve(new Response("bytes")),
    );
    const outs = await new ComputeAgentClient(NODE, agent).getOutputs("j1");
    const source = outs?.[0]?.source;
    expect(source?.key).toBe(`${NODE}:artifacts/recon-1/output.ply`);
    expect(source?.name).toBe("output.ply");

    const res = await source?.fetch(new AbortController().signal);
    expect(await res?.text()).toBe("bytes");
    expect(callAt(fetch, 1).path).toBe("artifacts/recon-1/output.ply");
  });

  it("reads a named output file through the job's file route, siblings too", async () => {
    const { agent, fetch } = stubAgent((path) =>
      path.endsWith("/outputs")
        ? json(200, [
            { id: "o1", job_id: "j 1", kind: "splat_lod", uri: "x", file: "lod/scene-lod-0.rad" },
            { id: "o2", job_id: "j 1", kind: "mesh", uri: "mock://gone" },
          ])
        : Promise.resolve(new Response("bytes")),
    );
    const outs = await new ComputeAgentClient(NODE, agent).getOutputs("j 1");
    const source = outs?.[0]?.source;
    expect(source?.name).toBe("scene-lod-0.rad");
    await source?.fetch(new AbortController().signal, { headers: { Range: "bytes=0-9" } });
    expect(callAt(fetch, 1).path).toBe("api/compute/jobs/j%201/files/lod/scene-lod-0.rad");
    expect(new Headers(callAt(fetch, 1).init.headers).get("Range")).toBe("bytes=0-9");
    await source?.sibling?.("scene-lod-1.rad").fetch(new AbortController().signal);
    expect(callAt(fetch, 2).path).toBe("api/compute/jobs/j%201/files/lod/scene-lod-1.rad");
    // A uri that names neither a file nor an artifact path has nothing to read.
    expect(outs?.[1]?.source).toBeNull();
  });

  it("keys two nodes' identical artifact paths apart", async () => {
    const reply = () =>
      json(200, [{ id: "o1", job_id: "j1", kind: "splat", uri: "/artifacts/r/output.ply" }]);
    const a = await new ComputeAgentClient("node-a", stubAgent(reply).agent).getOutputs("j1");
    const b = await new ComputeAgentClient("node-b", stubAgent(reply).agent).getOutputs("j1");
    expect(a?.[0]?.source?.key).not.toBe(b?.[0]?.source?.key);
  });
});
