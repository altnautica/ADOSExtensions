// The standalone tour viewer: a static page (no backend) that opens the
// `tour.json` next to it. Built into `gcs/tour-viewer/` (shipped in the
// packed extension; these sources under `gcs/src/` are not) with relative
// asset URLs, so the node copies it into any tour bundle and any static host
// serves it.

import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

import tailwindcss from "@tailwindcss/vite";
import { defineConfig } from "vite";

const root = dirname(fileURLToPath(import.meta.url));

export default defineConfig({
  root,
  base: "./",
  plugins: [tailwindcss()],
  esbuild: { jsx: "automatic" },
  define: { "process.env.NODE_ENV": '"production"' },
  build: {
    outDir: join(root, "../../tour-viewer"),
    emptyOutDir: true,
    target: "es2022",
    // three and Spark (with its inlined wasm) are one large chunk by design.
    chunkSizeWarningLimit: 6000,
  },
});
