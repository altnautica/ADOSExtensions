/**
 * @module atlas/viewers/rerun-start
 * @description Starts a Rerun web viewer whose wasm is the extension's own
 * asset. The viewer fetches `re_viewer_bg.wasm` relative to `base_url` with no
 * way to hand it bytes, so the base is a sentinel origin whose one URL is
 * answered from `host.readAsset` for as long as `start` runs (the wasm module
 * is compiled once and cached by the package after that).
 */

import type { WebViewer, WebViewerOptions } from "@rerun-io/web-viewer";

import { SENTINEL_ORIGIN, serveAtSentinel } from "../../../lib/net/sentinel-fetch";

/** Where the build stages the viewer's wasm, relative to the `gcs/` dir. */
export const RERUN_WASM_ASSET = "assets/re_viewer_bg.wasm";

export async function startRerunViewer(
  viewer: WebViewer,
  parent: HTMLElement,
  readAsset: (path: string) => Promise<Blob>,
): Promise<void> {
  const route = serveAtSentinel(
    "rerun/re_viewer_bg.wasm",
    async () => {
      const wasm = await readAsset(RERUN_WASM_ASSET);
      return new Response(wasm, {
        headers: { "Content-Type": "application/wasm", "Content-Length": String(wasm.size) },
      });
    },
    { exactPath: true },
  );
  // `base_url` is read by the package but absent from its option type.
  const options: WebViewerOptions & { base_url: string } = {
    width: "100%",
    height: "100%",
    base_url: `${SENTINEL_ORIGIN}/rerun/`,
  };
  try {
    await viewer.start(null, parent, options);
  } finally {
    route.release();
  }
}
