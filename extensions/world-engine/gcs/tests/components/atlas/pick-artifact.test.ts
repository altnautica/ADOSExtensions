import { describe, it, expect } from "vitest";
import { pickArtifactForViewer } from "../../../src/components/atlas/viewer-types";

const outs = [
  { kind: "splat", uri: "splat.ply" },
  { kind: "cloud", uri: "cloud.ply" },
  { kind: "rerun", uri: "world.rrd" },
];

describe("pickArtifactForViewer", () => {
  it("gives the Splat viewer the splat .ply, never the point cloud", () => {
    expect(pickArtifactForViewer(outs, "splat")?.uri).toBe("splat.ply");
  });

  it("gives Cloud / LOD the point-cloud .ply", () => {
    expect(pickArtifactForViewer(outs, "cloud")?.uri).toBe("cloud.ply");
    expect(pickArtifactForViewer(outs, "lod")?.uri).toBe("cloud.ply");
  });

  it("gives World (Rerun) the .rrd", () => {
    expect(pickArtifactForViewer(outs, "rerun")?.uri).toBe("world.rrd");
  });

  it("matches ply / pointcloud kinds for the cloud viewer", () => {
    expect(
      pickArtifactForViewer([{ kind: "pointcloud", uri: "p" }], "cloud")?.uri,
    ).toBe("p");
    expect(pickArtifactForViewer([{ kind: "ply", uri: "q" }], "cloud")?.uri).toBe(
      "q",
    );
  });

  it("returns undefined (caller falls back) when no matching kind exists", () => {
    const splatOnly = [{ kind: "splat", uri: "s" }];
    expect(pickArtifactForViewer(splatOnly, "cloud")).toBeUndefined();
    expect(pickArtifactForViewer(splatOnly, "rerun")).toBeUndefined();
  });
});
