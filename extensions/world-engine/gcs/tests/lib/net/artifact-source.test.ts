import { describe, it, expect, vi, afterEach } from "vitest";
import { artifactRelPath, artifactSource } from "../../../src/lib/net/artifact-source";
import { callAt, stubAgent } from "../../helpers/agent";

describe("artifactRelPath", () => {
  it("extracts the artifacts/ path from a full engine URL, ignoring the host", () => {
    expect(artifactRelPath("http://some-host.local:8092/artifacts/recon-1/output.ply")).toBe(
      "artifacts/recon-1/output.ply",
    );
  });

  it("handles a bare path", () => {
    expect(artifactRelPath("/artifacts/ds-9/output.rrd")).toBe("artifacts/ds-9/output.rrd");
    expect(artifactRelPath("artifacts/ds-9/output.rrd")).toBe("artifacts/ds-9/output.rrd");
  });

  it("drops a query string from a URL", () => {
    expect(artifactRelPath("http://h:8092/artifacts/r/output.ply?v=2")).toBe(
      "artifacts/r/output.ply",
    );
  });

  it("returns null when there is no artifacts/ segment (e.g. a mock:// uri)", () => {
    expect(artifactRelPath("mock://splat/1")).toBeNull();
    expect(artifactRelPath("http://h:8092/other/x")).toBeNull();
  });
});

describe("artifactSource", () => {
  afterEach(() => {
    vi.unstubAllGlobals();
  });

  it("reads an artifact path through the node agent, never the stamped host", async () => {
    const { agent, fetch } = stubAgent(() => Promise.resolve(new Response("ply")));
    const src = artifactSource("http://drifting.local:8092/artifacts/recon-1/output.ply", {
      deviceId: "node-1",
      agent,
    });
    expect(src?.key).toBe("node-1:artifacts/recon-1/output.ply");
    expect(src?.name).toBe("output.ply");
    const signal = new AbortController().signal;
    await src?.fetch(signal);
    const { path, init } = callAt(fetch);
    expect(path).toBe("artifacts/recon-1/output.ply");
    // The caller's signal reaches the transfer so a closed viewer cancels it.
    expect(init.signal).toBe(signal);
  });

  it("fetches a plain http(s) URL directly when it names no artifact path", async () => {
    const direct = vi.fn(() => Promise.resolve(new Response("x")));
    vi.stubGlobal("fetch", direct);
    const { agent, fetch } = stubAgent(() => Promise.resolve(new Response("")));
    const src = artifactSource("https://cdn.example/worlds/scene.splat?sig=1", {
      deviceId: "node-1",
      agent,
    });
    expect(src?.key).toBe("https://cdn.example/worlds/scene.splat?sig=1");
    expect(src?.name).toBe("scene.splat");
    await src?.fetch(new AbortController().signal);
    expect(direct).toHaveBeenCalledTimes(1);
    expect(fetch).not.toHaveBeenCalled();
  });

  it("fetches an artifact URL directly when no node is known", () => {
    const src = artifactSource("http://h.local:8092/artifacts/r/output.ply", null);
    expect(src?.key).toBe("http://h.local:8092/artifacts/r/output.ply");
  });

  it("has no source for a uri that is neither an artifact path nor http(s)", () => {
    expect(artifactSource("mock://splat/1", null)).toBeNull();
    expect(artifactSource("file:///tmp/x.ply", null)).toBeNull();
    expect(artifactSource("artifacts/r/output.ply", null)).toBeNull();
  });
});
