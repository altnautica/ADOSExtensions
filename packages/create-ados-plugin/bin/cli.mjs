#!/usr/bin/env node
import { existsSync, mkdirSync, readdirSync, readFileSync, statSync, writeFileSync } from "node:fs";
import { dirname, join, relative, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import readline from "node:readline";

const SELF = fileURLToPath(import.meta.url);
const ROOT = resolve(dirname(SELF), "..");
const TEMPLATES = resolve(ROOT, "templates");

const TEMPLATE_HALVES = ["gcs-only", "agent-only", "hybrid"];

main().catch((err) => {
  process.stderr.write(`create-ados-plugin: ${err.message}\n`);
  process.exit(1);
});

async function main() {
  const args = process.argv.slice(2);
  const opts = parseArgs(args);

  const target = await prompt(opts.target, "Plugin folder name", "my-ados-plugin");
  const id = await prompt(
    opts.id,
    "Reverse-DNS plugin id (e.g. com.example.hello)",
    `com.example.${slug(target)}`,
  );
  const half = await pickHalf(opts.half);
  const author = await prompt(opts.author, "Author", "Anonymous");
  const signer = await prompt(
    opts.signer,
    "Signer id for `pnpm pack` (see README: ados plugin sign)",
    `${slug(author) || "example"}-2026-A`,
  );

  const dest = resolve(process.cwd(), target);
  if (existsSync(dest)) {
    throw new Error(`refusing to overwrite existing folder: ${dest}`);
  }

  const tpl = resolve(TEMPLATES, half);
  copyTemplate(tpl, dest, { id, half, author, signer });
  // Inside the ADOSExtensions pnpm workspace the SDK is a sibling package, so
  // the published range would resolve to the registry copy and a first-party
  // extension would silently build against a stale SDK. Outside it — every
  // third-party developer — the workspace protocol is unresolvable and the
  // very first command in the generated README fails, which is why the
  // template ships the published range and this rewrites it rather than the
  // other way round.
  if (findWorkspaceRoot(dest)) {
    rewriteSdkDepToWorkspace(dest);
    process.stdout.write(
      "\nDetected the ADOSExtensions pnpm workspace: pinned @altnautica/plugin-sdk to workspace:^\n",
    );
  }

  process.stdout.write(`\nCreated ${relative(process.cwd(), dest)}\n`);
  process.stdout.write(`\nNext steps:\n`);
  process.stdout.write(`  cd ${target}\n`);
  if (half !== "agent-only") {
    process.stdout.write(`  pnpm install\n`);
    process.stdout.write(`  pnpm test\n`);
    process.stdout.write(`  pnpm build\n`);
  }
  process.stdout.write(`  See README.md for the full release flow.\n`);
}

/** Walk up from `dir` looking for a pnpm workspace root. */
function findWorkspaceRoot(dir) {
  let cur = resolve(dir);
  for (;;) {
    if (existsSync(join(cur, "pnpm-workspace.yaml"))) return cur;
    const parent = dirname(cur);
    if (parent === cur) return null;
    cur = parent;
  }
}

/** Point the scaffolded GCS half at the workspace SDK. No-op for a template
 * half that ships no GCS package.json (agent-only). */
function rewriteSdkDepToWorkspace(dest) {
  const pkg = join(dest, "gcs", "package.json");
  if (!existsSync(pkg)) return;
  const body = readFileSync(pkg, "utf-8").replace(
    /("@altnautica\/plugin-sdk":\s*)"[^"]+"/,
    '$1"workspace:^"',
  );
  writeFileSync(pkg, body);
}

function parseArgs(argv) {
  const out = { target: null, id: null, half: null, author: null, signer: null };
  for (let i = 0; i < argv.length; i++) {
    const a = argv[i];
    if (a === "--target") out.target = argv[++i];
    else if (a === "--id") out.id = argv[++i];
    else if (a === "--half") out.half = argv[++i];
    else if (a === "--author") out.author = argv[++i];
    else if (a === "--signer") out.signer = argv[++i];
    else if (!a.startsWith("--") && out.target === null) out.target = a;
  }
  return out;
}

async function pickHalf(preset) {
  if (preset && TEMPLATE_HALVES.includes(preset)) return preset;
  const ans = await prompt(
    null,
    `Plugin half (${TEMPLATE_HALVES.join(" / ")})`,
    "gcs-only",
  );
  if (!TEMPLATE_HALVES.includes(ans)) {
    throw new Error(`unknown half: ${ans}`);
  }
  return ans;
}

function prompt(preset, question, fallback) {
  if (preset !== null && preset !== undefined) return Promise.resolve(preset);
  if (!process.stdin.isTTY) return Promise.resolve(fallback);
  return new Promise((resolve) => {
    const rl = readline.createInterface({
      input: process.stdin,
      output: process.stdout,
    });
    rl.question(`${question} [${fallback}] `, (answer) => {
      rl.close();
      resolve(answer.trim() || fallback);
    });
  });
}

function copyTemplate(srcDir, destDir, vars) {
  mkdirSync(destDir, { recursive: true });
  for (const entry of readdirSync(srcDir)) {
    const srcPath = join(srcDir, entry);
    const destEntry = entry.replace(/^_/, ".");
    const destPath = join(destDir, destEntry);
    if (statSync(srcPath).isDirectory()) {
      copyTemplate(srcPath, destPath, vars);
      continue;
    }
    let body = readFileSync(srcPath, "utf-8");
    body = body
      .replace(/__PLUGIN_ID__/g, vars.id)
      .replace(/__PLUGIN_AUTHOR__/g, vars.author)
      .replace(/__PLUGIN_HALF__/g, vars.half)
      .replace(/__PLUGIN_SIGNER__/g, vars.signer);
    writeFileSync(destPath, body);
  }
}

function slug(value) {
  return value.toLowerCase().replace(/[^a-z0-9-]+/g, "-").replace(/^-+|-+$/g, "");
}
