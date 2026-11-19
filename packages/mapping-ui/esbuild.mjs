// The esbuild plugins and stylesheet step every module that bundles the
// viewers needs.
//
// - `lazRsWasm()`: `laz-rs-wasm` / `laz-rs-wasm/bytes` → the laz-rs decoder
//   inside `@loaders.gl/las` and its wasm (bundled as bytes). The package does
//   not export that path, and its own wasm loader fetches next to the module
//   URL, which an inline module does not have; the viewer instantiates the
//   bytes into the same module instance the loader uses (real paths, so
//   esbuild dedupes them).
// - `workerSource()`: `./file.ts?worker-source` → the bundled worker's source
//   text, so the page starts it from a blob URL (the host's CSP admits `blob:`
//   workers and nothing else is needed).
// - `viewerCss()`: the viewers' utilities (`map:` prefix), compiled once and
//   appended to each module's own stylesheet.
// - `rerunWasm()`: the Rerun viewer's wasm, served as a plugin asset.

import { spawnSync } from "node:child_process";
import { readFileSync, realpathSync } from "node:fs";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";

import { build } from "esbuild";

const here = dirname(fileURLToPath(import.meta.url));
const WORKER_SUFFIX = "?worker-source";

export function lazRsWasm() {
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

export function workerSource(target) {
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
          target,
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

/** Compile the viewers' stylesheet and return the CSS text. */
export function viewerCss() {
  // The CLI's JS entry through this Node binary: `node_modules/.bin/tailwindcss`
  // is a POSIX shell shim that Windows cannot spawn directly.
  const cli = join(here, "node_modules/@tailwindcss/cli/dist/index.mjs");
  const out = spawnSync(
    process.execPath,
    [cli, "-i", join(here, "src/styles.css"), "--minify"],
    { cwd: here, encoding: "utf8", stdio: ["ignore", "pipe", "inherit"] },
  );
  if (out.status !== 0) {
    throw new Error(`tailwindcss exited with ${out.status}${out.error ? `: ${out.error.message}` : ""}`);
  }
  return out.stdout;
}

/** The Rerun web viewer's wasm bytes. */
export function rerunWasm() {
  // The package exports no `./package.json`; resolve its ESM entry instead.
  const dir = dirname(fileURLToPath(import.meta.resolve("@rerun-io/web-viewer")));
  return readFileSync(join(dir, "re_viewer_bg.wasm"));
}
