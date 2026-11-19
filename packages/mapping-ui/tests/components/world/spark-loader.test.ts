// @vitest-environment node
/**
 * The viewer's splat loader on real files: SPZ version 2 (the tour fixture)
 * and versions 3 and 4 as the pinned splat-transform (3.10.1) writes them from
 * it (`splat-transform scene.spz [--spz-version 3] scene-v<N>.spz`; version 4,
 * plaintext `NGSP` header and ZSTD streams, is what the node's products stage
 * writes), and the `.rad` LoD file the pinned spark-build-lod (2.4.0) builds
 * from it (`build-lod --rad scene.spz`), streamed over HTTP ranges as the
 * viewer streams it.
 */
import { readFileSync } from "node:fs";
import { createServer, type Server } from "node:http";
import type { AddressInfo } from "node:net";
import { join } from "node:path";

import * as SPARK from "@sparkjsdev/spark";
import type { PagedSplats, SplatPager } from "@sparkjsdev/spark";
import * as THREE from "three";
import { afterAll, beforeAll, describe, expect, it } from "vitest";

import { splatMesh, splatViewerBounds } from "../../../src/components/world/spark-stage";
import { splatFramingBounds } from "../../../src/lib/world/splat-bounds";
import { installInProcessWorker } from "../../helpers/spark-worker";

const FIXTURES = join(__dirname, "../../fixtures");
const FILES = { 2: "tour/scene.spz", 3: "spz/scene-v3.spz", 4: "spz/scene-v4.spz" } as const;
const SPLATS = 14367;
/** The LoD tree build-lod makes of the fixture: its splats plus the merged levels. */
const LOD_SPLATS = 26354;
/** A chunk decoded into packed splats (Spark does not export the type). */
type PackedResult = Extract<Awaited<ReturnType<PagedSplats["fetchDecodeChunk"]>>, { packedArray: Uint32Array }>;

let restore: () => void;
let server: Server;
let base: string;
beforeAll(async () => {
  restore = installInProcessWorker();
  const rad = readFileSync(join(FIXTURES, "rad/scene-lod.rad"));
  // A static host answering `Range` requests, as the node's file routes do.
  server = createServer((req, res) => {
    const m = /^bytes=(\d+)-(\d*)$/.exec(req.headers.range ?? "");
    if (!m) {
      res.writeHead(200, { "content-length": rad.length }).end(rad);
      return;
    }
    const start = Number(m[1]);
    const end = Math.min(m[2] ? Number(m[2]) : rad.length - 1, rad.length - 1);
    res
      .writeHead(206, { "content-range": `bytes ${start}-${end}/${rad.length}`, "content-length": end - start + 1 })
      .end(rad.subarray(start, end + 1));
  });
  await new Promise<void>((ok) => server.listen(0, "127.0.0.1", ok));
  base = `http://127.0.0.1:${(server.address() as AddressInfo).port}`;
});
afterAll(() => {
  restore();
  server.close();
});

async function load(version: keyof typeof FILES) {
  const bytes = new Uint8Array(readFileSync(join(FIXTURES, FILES[version])));
  const mesh = splatMesh(SPARK, { bytes, fileName: "scene.spz" });
  await mesh.initialized;
  const framing = splatViewerBounds(THREE, mesh);
  if (framing === "streaming") throw new Error("an SPZ read whole does not stream");
  return { mesh, ...framing };
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

  it("streams the build-lod .rad: not judged empty before paging, header and root chunk decode", async () => {
    const mesh = splatMesh(SPARK, { url: `${base}/scene.rad`, fileName: "scene.rad" });
    await mesh.initialized;
    // Nothing is paged in until the renderer asks: framing waits, no empty verdict.
    expect(mesh.paged).toBeDefined();
    expect(splatViewerBounds(THREE, mesh)).toBe("streaming");

    const paged = mesh.paged!;
    const { meta } = await paged.getRadMeta();
    expect(meta.count).toBe(LOD_SPLATS);
    expect(meta.chunks.length).toBeGreaterThan(0);

    // The renderer's pager only says which encoding to decode into; packed is
    // the default. (Cast: a full SplatPager needs a WebGL renderer.)
    paged.pager = { extSplats: false } as unknown as SplatPager;
    const chunk = (await paged.fetchDecodeChunk(0)) as PackedResult;
    expect(chunk.numSplats).toBe(LOD_SPLATS);
    // The paged splats, read as PagedSplats.forEachSplat reads them, frame
    // where the SPZ does.
    const f = splatFramingBounds((cb) => {
      for (let i = 0; i < chunk.numSplats; i++) {
        const s = SPARK.unpackSplat(chunk.packedArray, i, chunk.splatEncoding);
        cb(i, s.center, s.scales, s.quaternion, s.opacity);
      }
    }, chunk.numSplats);
    expect(f?.solid).toBe(true);
    const ref = (await load(2)).box;
    const box = new THREE.Box3(new THREE.Vector3(...f!.min), new THREE.Vector3(...f!.max));
    for (const k of ["min", "max"] as const) {
      for (const a of ["x", "y", "z"] as const) expect(Math.abs(box[k][a] - ref[k][a])).toBeLessThan(0.25);
    }
    mesh.dispose();
  }, 30_000);

  it("reports a fully transparent splat as having nothing solid", async () => {
    const mesh = new SPARK.SplatMesh({
      constructSplats: (splats) => {
        const scales = new THREE.Vector3(0.05, 0.05, 0.05);
        for (let i = 0; i < 100; i++) {
          splats.pushSplat(new THREE.Vector3(i, 0, 0), scales, new THREE.Quaternion(), 0.01, new THREE.Color(1, 1, 1));
        }
      },
    });
    await mesh.initialized;
    const framing = splatViewerBounds(THREE, mesh);
    expect(framing).not.toBe("streaming");
    if (framing === "streaming") return;
    expect(framing.solid).toBe(false);
    expect(framing.box.isEmpty()).toBe(false);
    mesh.dispose();
  });
});
