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

  const dest = resolve(process.cwd(), target);
  if (existsSync(dest)) {
    throw new Error(`refusing to overwrite existing folder: ${dest}`);
  }

  const tpl = resolve(TEMPLATES, half);
  copyTemplate(tpl, dest, { id, half, author });

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

function parseArgs(argv) {
  const out = { target: null, id: null, half: null, author: null };
  for (let i = 0; i < argv.length; i++) {
    const a = argv[i];
    if (a === "--target") out.target = argv[++i];
    else if (a === "--id") out.id = argv[++i];
    else if (a === "--half") out.half = argv[++i];
    else if (a === "--author") out.author = argv[++i];
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
      .replace(/__PLUGIN_HALF__/g, vars.half);
    writeFileSync(destPath, body);
  }
}

function slug(value) {
  return value.toLowerCase().replace(/[^a-z0-9-]+/g, "-").replace(/^-+|-+$/g, "");
}
