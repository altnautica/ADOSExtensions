// @vitest-environment node
/**
 * The viewer's splat loader on real SPZ bytes: version 2 (the tour fixture)
 * and versions 3 and 4 as the pinned splat-transform (3.10.1) writes them from
 * it (`splat-transform scene.spz [--spz-version 3] scene-v<N>.spz`). Version 4
 * (plaintext `NGSP` header, ZSTD streams) is what the node's products stage
 * writes.
 */
import { readFileSync } from "node:fs";
import { join } from "node:path";

import { afterAll, beforeAll, describe, expect, it } from "vitest";

import { splatMesh, splatViewerBounds } from "../../../src/components/world/spark-stage";
import { installInProcessWorker } from "../../helpers/spark-worker";

const FIXTURES = join(__dirname, "../../fixtures");
const FILES = { 2: "tour/scene.spz", 3: "spz/scene-v3.spz", 4: "spz/scene-v4.spz" } as const;
const SPLATS = 14367;

let restore: () => void;
beforeAll(() => {
  restore = installInProcessWorker();
});
afterAll(() => restore());

async function spark() {
  const [THREE, SPARK] = await Promise.all([import("three"), import("@sparkjsdev/spark")]);
  return { THREE, SPARK };
}

async function load(version: keyof typeof FILES) {
  const { THREE, SPARK } = await spark();
  const bytes = new Uint8Array(readFileSync(join(FIXTURES, FILES[version])));
  const mesh = splatMesh(SPARK, { bytes, fileName: "scene.spz" });
  await mesh.initialized;
  return { mesh, ...splatViewerBounds(THREE, mesh) };
}

describe("spark splat loader", () => {
  it("the v4 fixture is the pinned writer's version 4 layout", () => {
    const bytes = readFileSync(join(FIXTURES, FILES[4]));
    expect(bytes.subarray(0, 4).toString("latin1")).toBe("NGSP");
    expect(bytes.readUInt32LE(4)).toBe(4);
  });

  it.each([2, 3, 4] as const)("decodes SPZ version %i and frames it", async (version) => {
    const { mesh, box, solid } = await load(version);
    expect(mesh.splats?.getNumSplats()).toBe(SPLATS);
    expect(solid).toBe(true);
    expect(box.isEmpty()).toBe(false);
    const ref = (await load(2)).box;
    for (const k of ["min", "max"] as const) {
      for (const a of ["x", "y", "z"] as const) expect(box[k][a]).toBeCloseTo(ref[k][a], 2);
    }
    mesh.dispose();
  }, 30_000);

  it("reports a fully transparent splat as having nothing solid", async () => {
    const { THREE, SPARK } = await spark();
    const mesh = new SPARK.SplatMesh({
      constructSplats: (splats) => {
        const scales = new THREE.Vector3(0.05, 0.05, 0.05);
        for (let i = 0; i < 100; i++) {
          splats.pushSplat(new THREE.Vector3(i, 0, 0), scales, new THREE.Quaternion(), 0.01, new THREE.Color(1, 1, 1));
        }
      },
    });
    await mesh.initialized;
    const { box, solid } = splatViewerBounds(THREE, mesh);
    expect(solid).toBe(false);
    expect(box.isEmpty()).toBe(false);
    mesh.dispose();
  });
});
