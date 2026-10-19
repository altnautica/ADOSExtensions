// Builds the World Engine inline GCS module and the standalone tour viewer.
//
//   world-engine.mjs            ESM bundle; React comes from the host
//   plugin.css                  Tailwind utilities under the `we:` prefix
//   assets/re_viewer_bg.wasm    the Rerun viewer's wasm, served as a plugin asset
//   dist/tour-viewer/           static tour viewer (Vite; no backend), copied
//                               into every tour bundle the node builds
//
// Module workers are bundled separately and embedded as source strings
// (`import src from "./x.ts?worker-source"`), so the page starts them from a
// blob URL: the host's CSP admits `blob:` workers and nothing else is needed.

import { spawnSync } from "node:child_process";
import { copyFileSync, mkdirSync, realpathSync, statSync } from "node:fs";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";

import { build } from "esbuild";
import { build as viteBuild } from "vite";
import { inlineSharedExternals } from "@altnautica/plugin-sdk/esbuild";

const here = dirname(fileURLToPath(import.meta.url));
const TARGET = "es2022";
const WORKER_SUFFIX = "?worker-source";

/**
 * `laz-rs-wasm` / `laz-rs-wasm/bytes` → the laz-rs decoder inside
 * `@loaders.gl/las` and its wasm (bundled as bytes). The package does not
 * export that path, and its own wasm loader fetches next to the module URL,
 * which an inline module does not have; the viewer instantiates the bytes
 * into the same module instance the loader uses (real paths, so esbuild
 * dedupes them).
 */
function lazRsWasm() {
  const dist = realpathSync(dirname(fileURLToPath(import.meta.resolve("@loaders.gl/las"))));
  const lib = join(dist, "libs/laz-rs-wasm");
  return {
    name: "laz-rs-wasm",
    setup(b) {
      b.onResolve({ filter: /^laz-rs-wasm$/ }, () => ({ path: join(lib, "laz_rs_wasm.js") }));
      b.onResolve({ filter: /^laz-rs-wasm\/bytes$/ }, () => ({ path: join(lib, "laz_rs_wasm_bg.wasm") }));
    },
  };
}

/** Resolve `./file.ts?worker-source` to the bundled worker's source text. */
function workerSource() {
  return {
    name: "worker-source",
    setup(b) {
      b.onResolve({ filter: /\?worker-source$/ }, (args) => ({
        path: resolve(args.resolveDir, args.path.slice(0, -WORKER_SUFFIX.length)),
        namespace: "worker-source",
      }));
      b.onLoad({ filter: /.*/, namespace: "worker-source" }, async (args) => {
        const out = await build({
          entryPoints: [args.path],
          bundle: true,
          format: "esm",
          target: TARGET,
          minify: true,
          write: false,
          metafile: true,
          logLevel: "silent",
        });
        const file = out.outputFiles[0];
        if (!file) throw new Error(`worker ${args.path} produced no output`);
        return {
          contents: `export default ${JSON.stringify(file.text)};`,
          loader: "js",
          watchFiles: Object.keys(out.metafile?.inputs ?? {}),
        };
      });
    },
  };
}

await build({
  entryPoints: [join(here, "src/index.tsx")],
  outfile: join(here, "world-engine.mjs"),
  bundle: true,
  format: "esm",
  target: TARGET,
  jsx: "automatic",
  minify: true,
  legalComments: "none",
  define: { "process.env.NODE_ENV": '"production"' },
  loader: { ".json": "json", ".wasm": "binary" },
  plugins: [inlineSharedExternals(), workerSource(), lazRsWasm()],
  logLevel: "warning",
});

// Run the CLI's JS entry through this Node binary: `node_modules/.bin/tailwindcss`
// is a POSIX shell shim that Windows cannot spawn directly.
const tailwind = spawnSync(
  process.execPath,
  [
    join(here, "node_modules/@tailwindcss/cli/dist/index.mjs"),
    "-i",
    join(here, "src/styles.css"),
    "-o",
    join(here, "plugin.css"),
    "--minify",
  ],
  { cwd: here, stdio: "inherit" },
);
if (tailwind.status !== 0) {
  throw new Error(
    `tailwindcss exited with ${tailwind.status}${tailwind.error ? `: ${tailwind.error.message}` : ""}`,
  );
}

// The package exports no `./package.json`; resolve its ESM entry instead.
const rerunDir = dirname(fileURLToPath(import.meta.resolve("@rerun-io/web-viewer")));
mkdirSync(join(here, "assets"), { recursive: true });
copyFileSync(join(rerunDir, "re_viewer_bg.wasm"), join(here, "assets/re_viewer_bg.wasm"));

await viteBuild({ configFile: join(here, "tour-viewer/vite.config.mjs"), logLevel: "warn" });

for (const file of ["world-engine.mjs", "plugin.css", "assets/re_viewer_bg.wasm", "dist/tour-viewer/index.html"]) {
  const kib = statSync(join(here, file)).size / 1024;
  console.log(`${file.padEnd(28)} ${kib.toFixed(1)} KiB`);
}
