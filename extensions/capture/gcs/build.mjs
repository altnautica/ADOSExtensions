// Builds the Capture inline GCS module.
//
//   capture.mjs                 ESM bundle; React comes from the host
//   plugin.css                  Tailwind utilities under the `cap:` prefix,
//                               then the shared viewers' (`map:`)
//   assets/re_viewer_bg.wasm    the Rerun viewer's wasm, served as a plugin asset

import { spawnSync } from "node:child_process";
import { mkdirSync, readFileSync, statSync, writeFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

import { build } from "esbuild";
import { inlineSharedExternals } from "@altnautica/plugin-sdk/esbuild";
import { lazRsWasm, rerunWasm, viewerCss, workerSource } from "@altnautica/mapping-ui/esbuild.mjs";

const here = dirname(fileURLToPath(import.meta.url));
const TARGET = "es2022";

await build({
  entryPoints: [join(here, "src/index.tsx")],
  outfile: join(here, "capture.mjs"),
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
const css = join(here, "plugin.css");
writeFileSync(css, readFileSync(css, "utf8") + viewerCss());

mkdirSync(join(here, "assets"), { recursive: true });
writeFileSync(join(here, "assets/re_viewer_bg.wasm"), rerunWasm());

for (const file of ["capture.mjs", "plugin.css", "assets/re_viewer_bg.wasm"]) {
  const kib = statSync(join(here, file)).size / 1024;
  console.log(`${file.padEnd(28)} ${kib.toFixed(1)} KiB`);
}
