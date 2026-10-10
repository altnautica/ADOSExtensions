#!/usr/bin/env node
/**
 * Emit docs/reference/extensions.json: one row per extensions/<dir>/manifest.yaml.
 *
 *   node scripts/emit-reference.mjs           write the file
 *   node scripts/emit-reference.mjs --check   exit 1 when the committed file differs
 *
 * `published` is true when a release workflow carries the extension's
 * `<dir>-v*` tag trigger; that trigger is what makes an archive publishable.
 */

import { readdirSync, readFileSync, writeFileSync, mkdirSync, existsSync } from "node:fs";
import { dirname, join, relative } from "node:path";
import { fileURLToPath } from "node:url";
import { parse } from "yaml";

const ROOT = join(dirname(fileURLToPath(import.meta.url)), "..");
const EXT_DIR = join(ROOT, "extensions");
const OUT = join(ROOT, "docs", "reference", "extensions.json");
const RELEASE_WORKFLOWS = ["release.yml", "vision-nav-release.yml", "world-engine-release.yml"];

function releaseTriggers() {
  return RELEASE_WORKFLOWS.map((name) => join(ROOT, ".github", "workflows", name))
    .filter((p) => existsSync(p))
    .map((p) => readFileSync(p, "utf8"))
    .join("\n");
}

function permissionIds(half) {
  return Array.isArray(half?.permissions)
    ? half.permissions.map((p) => (typeof p === "string" ? p : p?.id)).filter(Boolean)
    : [];
}

export function buildReference() {
  const triggers = releaseTriggers();
  const extensions = [];
  for (const entry of readdirSync(EXT_DIR, { withFileTypes: true })) {
    if (!entry.isDirectory()) continue;
    const manifestPath = join(EXT_DIR, entry.name, "manifest.yaml");
    if (!existsSync(manifestPath)) continue;
    const m = parse(readFileSync(manifestPath, "utf8"));
    const tagPattern = new RegExp(`^\\s*-\\s*["']?${entry.name.replace(/[.*+?^${}()|[\]\\]/g, "\\$&")}-v\\*["']?\\s*$`, "m");
    extensions.push({
      id: m.id,
      name: m.name ?? null,
      version: m.version ?? null,
      description: m.description ?? null,
      category: m.category ?? null,
      supported_boards: m.compatibility?.supported_boards ?? null,
      permissions: [...new Set([...permissionIds(m.agent), ...permissionIds(m.gcs)])].sort(),
      halves: ["agent", "gcs"].filter((h) => m[h] != null),
      documentation_url: m.documentation_url ?? null,
      published: tagPattern.test(triggers),
    });
  }
  extensions.sort((a, b) => (a.id < b.id ? -1 : a.id > b.id ? 1 : 0));
  return `${JSON.stringify({ schema: 1, extensions }, null, 2)}\n`;
}

const text = buildReference();
if (process.argv.includes("--check")) {
  const current = existsSync(OUT) ? readFileSync(OUT, "utf8") : null;
  if (current !== text) {
    console.error("docs/reference/extensions.json is out of date; run node scripts/emit-reference.mjs");
    process.exit(1);
  }
  console.log("docs/reference/extensions.json is current");
} else {
  mkdirSync(dirname(OUT), { recursive: true });
  writeFileSync(OUT, text);
  console.log(`wrote ${relative(ROOT, OUT)}`);
}
