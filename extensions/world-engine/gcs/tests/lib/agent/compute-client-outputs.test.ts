/**
 * Coercion of a compute node's job outputs: the reconstruction honesty field
 * (no fabricated reading) is lifted from `meta.backend`, with a `mock://`
 * uri-scheme fallback so a pre-field agent that emits a placeholder is still
 * flagged; and each real artifact carries a source that reads its bytes back
 * through the node that produced it.
 */

import { describe, it, expect } from "vitest";
import {
  ComputeAgentClient,
  isPlaceholderArtifact,
  type ComputeOutput,
} from "../../../src/lib/agent/compute-client";
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

describe("ComputeAgentClient.getOutputs — backend/meta coercion", () => {
  it("lifts backend from meta.backend, falls back to the mock:// uri, drops junk", async () => {
    const { agent, fetch } = stubAgent(() =>
      json(200, [
        // real backend, real artifact
        {
          id: "o-real",
          job_id: "j1",
          kind: "splat",
          uri: "https://cdn.example/real.splat",
          meta: { gaussian_count: 900000, backend: "brush" },
          created_ms: 6,
        },
        // explicit mock backend in meta
        {
          id: "o-mock",
          job_id: "j1",
          kind: "splat",
          uri: "mock://splat/ds-7",
          meta: { gaussian_count: 1000, backend: "mock" },
          created_ms: 5,
        },
        // no meta, but a mock:// uri → backend defaults to "mock"
        { id: "o-legacy-mock", job_id: "j1", kind: "cloud", uri: "mock://splat/ds-8", created_ms: 7 },
        // no meta, real uri → backend null (unknown)
        { id: "o-unknown", job_id: "j1", kind: "mesh", uri: "https://cdn.example/x.ply", created_ms: 8 },
        // malformed (missing uri) → dropped
        { id: "bad", job_id: "j1", kind: "splat" },
      ]),
    );

    const outs = await new ComputeAgentClient(NODE, agent).getOutputs("j1");
    expect(callAt(fetch).path).toBe("api/compute/jobs/j1/outputs");
    expect(outs).toHaveLength(4);
    const m = byId(outs);

    expect(get(m, "o-real").backend).toBe("brush");
    expect(get(m, "o-real").meta?.gaussian_count).toBe(900000);
    expect(isPlaceholderArtifact(get(m, "o-real"))).toBe(false);

    expect(get(m, "o-mock").backend).toBe("mock");
    expect(isPlaceholderArtifact(get(m, "o-mock"))).toBe(true);

    // Pre-field agent: no meta.backend, but the mock:// scheme still flags it.
    expect(get(m, "o-legacy-mock").backend).toBe("mock");
    expect(get(m, "o-legacy-mock").meta).toBeNull();
    expect(isPlaceholderArtifact(get(m, "o-legacy-mock"))).toBe(true);

    expect(get(m, "o-unknown").backend).toBeNull();
    expect(get(m, "o-unknown").meta).toBeNull();
    expect(isPlaceholderArtifact(get(m, "o-unknown"))).toBe(false);
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

  it("gives a placeholder no source, even when its uri looks like an artifact", async () => {
    const { agent } = stubAgent(() =>
      json(200, [
        { id: "o1", job_id: "j1", kind: "splat", uri: "mock://splat/ds-1" },
        {
          id: "o2",
          job_id: "j1",
          kind: "splat",
          uri: "http://h.local:8092/artifacts/mock/output.ply",
          meta: { backend: "mock" },
        },
      ]),
    );
    const outs = await new ComputeAgentClient(NODE, agent).getOutputs("j1");
    expect(outs?.map((o) => o.source)).toEqual([null, null]);
  });

  it("keys two nodes' identical artifact paths apart", async () => {
    const reply = () =>
      json(200, [{ id: "o1", job_id: "j1", kind: "splat", uri: "/artifacts/r/output.ply" }]);
    const a = await new ComputeAgentClient("node-a", stubAgent(reply).agent).getOutputs("j1");
    const b = await new ComputeAgentClient("node-b", stubAgent(reply).agent).getOutputs("j1");
    expect(a?.[0]?.source?.key).not.toBe(b?.[0]?.source?.key);
  });
});
