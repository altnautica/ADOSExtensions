import { describe, expect, it } from "vitest";
import { splatArtifactExt } from "../../../../src/components/atlas/viewers/splat-format";

describe("splatArtifactExt", () => {
  it("detects every splat format from the file name", () => {
    expect(splatArtifactExt("output.ply")).toBe("ply");
    expect(splatArtifactExt("output.splat")).toBe("splat");
    expect(splatArtifactExt("output.ksplat")).toBe("ksplat");
    expect(splatArtifactExt("output.spz")).toBe("spz");
  });

  it("reads the last extension of a path, not an earlier dot", () => {
    expect(splatArtifactExt("artifacts/job.v2/output.splat")).toBe("splat");
    expect(splatArtifactExt("scene.splat.ply")).toBe("ply");
  });

  it("ignores a query string or fragment after the name", () => {
    expect(splatArtifactExt("artifacts/ds/output.spz?token=abc.ply")).toBe("spz");
    expect(splatArtifactExt("output.ksplat#view.ply")).toBe("ksplat");
  });

  it("is case-insensitive", () => {
    expect(splatArtifactExt("OUTPUT.PLY")).toBe("ply");
    expect(splatArtifactExt("O.SPLAT")).toBe("splat");
  });

  it("defaults to ply for an unknown or missing extension", () => {
    expect(splatArtifactExt("artifacts/ds/output")).toBe("ply");
    expect(splatArtifactExt("output.bin")).toBe("ply");
    expect(splatArtifactExt("")).toBe("ply");
  });
});
