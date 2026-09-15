#!/usr/bin/env node
/**
 * Check one extension manifest against the code it describes.
 *
 * Every `gcs/package.json` in this repo declares
 *   "lint:manifest": "node ../../../scripts/lint-manifest.mjs ../manifest.yaml"
 * so this is the script a contributor runs after touching a manifest, and the
 * one that keeps a manifest from drifting away from the bundle it ships with.
 *
 * Usage:
 *   node scripts/lint-manifest.mjs <path-to-manifest.yaml>
 *
 * Exits 0 when the manifest agrees with the code, 1 with one FAIL line per
 * disagreement. The manifest is the public contract an operator reads at
 * install time, so each of these is a real defect:
 *
 *   1. a `telemetry.subscribe.<topic>` permission with no matching
 *      `ctx.telemetry.subscribe("<topic>")` call, or a subscribe call with no
 *      matching permission. The SDK derives the capability string from the
 *      topic (packages/plugin-sdk/src/client.ts), so a partial topic in the
 *      manifest is denied at runtime, not narrowed.
 *   2. a host-surface permission (`mission.*`, `recording.write`,
 *      `command.send`, `ui.slot.notification-channel`) with no call site. A
 *      decorative permission is worse than none: the install dialog shows the
 *      operator a capability the bundle never uses.
 *   3. a `contributes.tabs[].slot` / `contributes.panels[].slot` with no
 *      implementation. A bundle that serves more than one slot has to
 *      discriminate between them, or one of the two renders the wrong surface.
 *   4. a version that disagrees across manifest.yaml, the extension
 *      package.json, gcs/package.json and the `definePlugin({ version })`
 *      literal the host actually registers.
 *
 * No YAML dependency: the manifest subset that matters here is flat enough to
 * scan line-wise, and adding a parser dependency to a lint script that runs
 * from six package directories is not worth it.
 */

import { readFileSync, readdirSync, existsSync, statSync } from "node:fs";
import { dirname, join, resolve } from "node:path";

const failures = [];
const fail = (msg) => failures.push(msg);

const manifestPath = resolve(process.argv[2] ?? "manifest.yaml");
if (!existsSync(manifestPath)) {
  console.error(`lint-manifest: no manifest at ${manifestPath}`);
  process.exit(2);
}
const extDir = dirname(manifestPath);
const manifest = readFileSync(manifestPath, "utf8");
const lines = manifest.split("\n");

/** Collect every source file under a directory, recursively. */
function sources(dir, exts) {
  if (!existsSync(dir)) return [];
  const out = [];
  for (const entry of readdirSync(dir)) {
    if (entry === "node_modules" || entry === "dist" || entry === "__pycache__") {
      continue;
    }
    const full = join(dir, entry);
    if (statSync(full).isDirectory()) {
      out.push(...sources(full, exts));
    } else if (exts.some((e) => entry.endsWith(e))) {
      out.push(full);
    }
  }
  return out;
}

function readAll(files) {
  return files.map((f) => readFileSync(f, "utf8")).join("\n");
}

const gcsSrc = readAll(sources(join(extDir, "gcs", "src"), [".ts", ".tsx"]));
const agentSrc = readAll(
  sources(join(extDir, "agent"), [".py", ".rs"]).filter(
    (f) => !f.includes(`${join("agent", "tests")}`),
  ),
);

/** The `id:` values inside a named top-level block's `permissions:` list. */
function permissionsOf(block) {
  const start = lines.findIndex((l) => l === `${block}:`);
  if (start < 0) return [];
  const out = [];
  let inPerms = false;
  for (let i = start + 1; i < lines.length; i += 1) {
    const l = lines[i];
    if (/^[A-Za-z]/.test(l)) break;
    if (/^\s{2}permissions:\s*$/.test(l)) {
      inPerms = true;
      continue;
    }
    if (inPerms) {
      const m = l.match(/^\s{4}-\s*id:\s*(\S+)/);
      if (m) {
        out.push(m[1]);
        continue;
      }
      if (l.trim() !== "" && !l.trimStart().startsWith("#")) inPerms = false;
    }
  }
  return out;
}

/** Every `slot:` value declared under contributes. */
function declaredSlots() {
  return lines
    .map((l) => l.match(/^\s*slot:\s*(\S+)/))
    .filter(Boolean)
    .map((m) => m[1]);
}

function manifestScalar(key) {
  const m = manifest.match(new RegExp(`^${key}:\\s*"?([^"\\n]*)"?`, "m"));
  return m ? m[1].trim() : null;
}

// -- 1 + 2: permissions against call sites ---------------------------------

const gcsPerms = permissionsOf("gcs");
const agentPerms = permissionsOf("agent");

const declaredTopics = gcsPerms
  .filter((p) => p.startsWith("telemetry.subscribe."))
  .map((p) => p.slice("telemetry.subscribe.".length));

const usedTopics = [
  ...gcsSrc.matchAll(/telemetry\s*\.\s*subscribe(?:<[^>]*>)?\(\s*["'`]([^"'`]+)["'`]/g),
].map((m) => m[1]);

for (const topic of new Set(usedTopics)) {
  if (!declaredTopics.includes(topic)) {
    fail(
      `FAIL gcs.permissions is missing 'telemetry.subscribe.${topic}': the bundle ` +
        `subscribes to "${topic}" and the SDK sends the full dotted topic as the ` +
        `capability, so the host denies this subscription`,
    );
  }
}
for (const topic of new Set(declaredTopics)) {
  if (!usedTopics.includes(topic)) {
    fail(
      `FAIL gcs.permissions declares 'telemetry.subscribe.${topic}' but no ` +
        `ctx.telemetry.subscribe("${topic}") call exists in gcs/src`,
    );
  }
}

/** permission id -> the call-site substring that justifies it. */
const GCS_SURFACE_CALLS = {
  "mission.read": "ctx.mission.",
  "mission.write": "ctx.mission.",
  "recording.write": "ctx.recording.",
  "command.send": "ctx.command.",
  "ui.slot.notification-channel": "ctx.notifications.",
};

for (const [perm, call] of Object.entries(GCS_SURFACE_CALLS)) {
  if (gcsPerms.includes(perm) && !gcsSrc.includes(call)) {
    fail(
      `FAIL gcs.permissions declares '${perm}' but no ${call} call site exists ` +
        `in gcs/src — a permission the bundle never uses is shown to the ` +
        `operator in the install dialog for nothing`,
    );
  }
}

// Matched against the agent source with whitespace collapsed, because the Rust
// half chains `.events` and `.publish(` across lines.
const AGENT_SURFACE_CALLS = {
  "telemetry.extend": [/\w+\s*\.\s*extend\s*\(/],
  "event.publish": [/events\s*\.\s*publish\s*\(/],
  "video.source.set": [/set_source\s*\(/],
  "vision.detection.publish": [/publish_box|vision\.detection/],
  "mcp.expose": [/tools\s*\.\s*register\s*\(/],
};

const agentFlat = agentSrc.replace(/\s+/g, " ");
for (const [perm, patterns] of Object.entries(AGENT_SURFACE_CALLS)) {
  if (agentPerms.includes(perm) && !patterns.some((re) => re.test(agentFlat))) {
    fail(
      `FAIL agent.permissions declares '${perm}' but the agent half contains no ` +
        `matching call site`,
    );
  }
}

// -- 3: declared slots against implementations -----------------------------

const slots = declaredSlots();
if (slots.length > 1 && gcsSrc && !gcsSrc.includes("video.overlay.props")) {
  fail(
    `FAIL the manifest contributes ${slots.length} slots (${slots.join(", ")}) ` +
      `but the bundle never discriminates between them (no ` +
      `ctx.events.subscribe("video.overlay.props") branch) — one of those slots ` +
      `renders the wrong surface`,
  );
}
if (slots.includes("map.overlay") && !/map[-.]overlay/.test(gcsSrc)) {
  fail(
    "FAIL the manifest contributes a map.overlay panel with no implementation " +
      "in gcs/src",
  );
}

// -- 4: version agreement --------------------------------------------------

const manifestVersion = manifestScalar("version");
const versionSites = [["manifest.yaml", manifestVersion]];

for (const rel of ["package.json", join("gcs", "package.json")]) {
  const p = join(extDir, rel);
  if (!existsSync(p)) continue;
  versionSites.push([rel, JSON.parse(readFileSync(p, "utf8")).version ?? null]);
}

const definePluginVersion = gcsSrc.match(/version:\s*["'`]([0-9][^"'`]*)["'`]/);
if (definePluginVersion) {
  versionSites.push(["definePlugin({ version })", definePluginVersion[1]]);
}

for (const [where, value] of versionSites) {
  if (value !== null && value !== manifestVersion) {
    fail(
      `FAIL version drift: ${where} says ${value}, manifest.yaml says ` +
        `${manifestVersion} — the host registry reports the definePlugin value ` +
        `while the Plugins tab shows the manifest one`,
    );
  }
}

// -- report ----------------------------------------------------------------

if (failures.length > 0) {
  for (const f of failures) console.error(f);
  console.error(`\n${failures.length} manifest problem(s) in ${manifestPath}`);
  process.exit(1);
}
console.log(`lint-manifest: OK ${manifestPath}`);
