/**
 * @module atlas/viewers/cloud-laz
 * @description Decodes a reconstruction's LAZ 1.4 point cloud with loaders.gl
 * (the laz-rs decoder, which reads LAS 1.4) into viewer-space positions and
 * `0..1` colours. The cloud is in local ENU metres, so it is mapped through
 * the same ENU → viewer axes as every other artifact.
 *
 * The decoder's wasm is bundled as bytes and instantiated here: its own loader
 * fetches the file next to the module URL, which an inline plugin module does
 * not have.
 */

import { enuToViewer } from "../../../lib/world/placement";

export interface DecodedCloud {
  positions: Float32Array;
  colors: Float32Array | null;
}

/** ENU positions (x east, y north, z up) → viewer positions, in place. */
export function enuCloudToViewer(positions: Float32Array): Float32Array {
  for (let i = 0; i + 2 < positions.length; i += 3) {
    const [x, y, z] = enuToViewer([positions[i]!, positions[i + 1]!, positions[i + 2]!]);
    positions[i] = x;
    positions[i + 1] = y;
    positions[i + 2] = z;
  }
  return positions;
}

/** RGBA bytes → RGB floats `0..1`. */
export function rgbaToFloatRgb(rgba: Uint8Array): Float32Array {
  const out = new Float32Array((rgba.length / 4) * 3);
  for (let i = 0, j = 0; i + 3 < rgba.length; i += 4, j += 3) {
    out[j] = rgba[i]! / 255;
    out[j + 1] = rgba[i + 1]! / 255;
    out[j + 2] = rgba[i + 2]! / 255;
  }
  return out;
}

export async function decodeLaz(buffer: ArrayBuffer): Promise<DecodedCloud> {
  // On demand: the decoder and its wasm load only when a LAZ cloud is opened.
  const [{ LAZRsLoader }, lazRs, wasm] = await Promise.all([
    import("@loaders.gl/las"),
    import("laz-rs-wasm"),
    import("laz-rs-wasm/bytes"),
  ]);
  lazRs.initSync({ module: wasm.default });
  const mesh = await LAZRsLoader.parse(buffer, { las: {} });
  const pos = mesh.attributes.POSITION?.value;
  if (!(pos instanceof Float32Array) || pos.length === 0) throw new Error("LAZ cloud has no points");
  const rgba = mesh.attributes.COLOR_0?.value;
  return {
    positions: enuCloudToViewer(pos),
    colors: rgba instanceof Uint8Array ? rgbaToFloatRgb(rgba) : null,
  };
}
