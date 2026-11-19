// Builds the Reconstruct inline GCS module and the standalone tour viewer.
//
//   reconstruct.mjs             ESM bundle; React comes from the host
//   plugin.css                  Tailwind utilities under the `rec:` prefix,
//                               then the shared viewers' (`map:`)
//   assets/re_viewer_bg.wasm    the Rerun viewer's wasm, served as a plugin asset
//   tour-viewer/                static tour viewer (Vite; no backend; sources in
//                               src/tour-viewer/), shipped in the packed extension
//                               and copied into every tour bundle the node builds

import { spawnSync } from "node:child_process";
import { mkdirSync, readFileSync, readdirSync, statSync, writeFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

import { build } from "esbuild";
import { build as viteBuild } from "vite";
import { inlineSharedExternals } from "@altnautica/plugin-sdk/esbuild";
import { lazRsWasm, rerunWasm, viewerCss, workerSource } from "@altnautica/mapping-ui/esbuild.mjs";

const here = dirname(fileURLToPath(import.meta.url));
const TARGET = "es2022";

await build({
  entryPoints: [join(here, "src/index.tsx")],
  outfile: join(here, "reconstruct.mjs"),
  bundle: true,
  format: "esm",
  target: TARGET,
  jsx: "automatic",
  minify: true,
  legalComments: "none",
  define: { "process.env.NODE_ENV": '"production"' },
  loader: { ".json": "json", ".wasm": "binary" },
  plugins: [inlineSharedExternals(), workerSource(TARGET), lazRsWasm()],
  logLevel: "warning",
});

// The CLI's JS entry through this Node binary: `node_modules/.bin/tailwindcss`
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
const shared = viewerCss();
const css = join(here, "plugin.css");
writeFileSync(css, readFileSync(css, "utf8") + shared);

mkdirSync(join(here, "assets"), { recursive: true });
writeFileSync(join(here, "assets/re_viewer_bg.wasm"), rerunWasm());

await viteBuild({ configFile: join(here, "src/tour-viewer/vite.config.mjs"), logLevel: "warn" });
// The tour viewer draws the shared viewers too: their utilities ride along.
const tourAssets = join(here, "tour-viewer/assets");
for (const file of readdirSync(tourAssets).filter((f) => f.endsWith(".css"))) {
  const path = join(tourAssets, file);
  writeFileSync(path, readFileSync(path, "utf8") + shared);
}

for (const file of ["reconstruct.mjs", "plugin.css", "assets/re_viewer_bg.wasm", "tour-viewer/index.html"]) {
  const kib = statSync(join(here, file)).size / 1024;
  console.log(`${file.padEnd(28)} ${kib.toFixed(1)} KiB`);
}
