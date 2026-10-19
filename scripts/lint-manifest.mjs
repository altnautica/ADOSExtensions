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
 *   node scripts/lint-manifest.mjs --self-test
 *
 * Exits 0 when the manifest agrees with the code, 1 with one FAIL line per
 * disagreement. The manifest is the public contract an operator reads at
 * install time, so each of these is a real defect:
 *
 *   1. a `telemetry.subscribe.<topic>` permission with no matching
 *      `ctx.telemetry.subscribe("<topic>")` call (a string literal or a
 *      `const NAME = "<topic>"` in gcs/src), or a subscribe call with no
 *      matching permission. The SDK derives the capability string from the
 *      topic, so a partial topic in the manifest is denied at runtime.
 *   2. a host-surface permission with no call site, on either half. A
 *      decorative permission is worse than none: the install dialog shows the
 *      operator a capability the bundle never uses.
 *   3. a `contributes` slot with no implementation.
 *   4. a version that disagrees across manifest.yaml, the extension and gcs
 *      package.json, the `definePlugin({ version })` literal, and the agent
 *      half's pyproject.toml / Cargo.toml (top level and one directory down).
 *   5. a permission id that is neither in the capability catalog
 *      (packages/plugin-sdk/src/capabilities.generated.ts) nor scoped to this
 *      plugin as `plugin.<leaf>.<...>`, where <leaf> is the last segment of
 *      the manifest id.
 *   6. a `documentation_url` on the docs site whose page does not exist in a
 *      sibling Documentation checkout (anchors are not checked). Skipped with
 *      a note when no checkout is present; ADOS_DOCS_ROOT overrides the path.
 *   7. an agent config read (`config_kv.get("k")`, `config_kv.static("k")`,
 *      Rust `config.get("k")`) of a key that is in neither config-schema.json,
 *      a manifest `config_key:` nor an agent write of that key.
 *
 * `--self-test` runs every case under scripts/lint-fixtures/: each case is
 * the `base/` extension overlaid with the case's own files, and its
 * `expect.txt` is either `OK` or one expected FAIL substring per line.
 *
 * No YAML dependency: the manifest subset that matters here is flat enough to
 * scan line-wise, and adding a parser dependency to a lint script that runs
 * from five package directories is not worth it.
 */

import {
  cpSync,
  existsSync,
  mkdtempSync,
  readFileSync,
  readdirSync,
  rmSync,
  statSync,
} from "node:fs";
import { tmpdir } from "node:os";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const REPO_ROOT = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const CATALOG_PATH = join(REPO_ROOT, "packages", "plugin-sdk", "src", "capabilities.generated.ts");
const DOCS_HOST = "https://docs.altnautica.com/";

/** Capability ids from the generated SDK catalog (the `CAPABILITY_IDS` array). */
export function loadCatalog(path = CATALOG_PATH) {
  const src = readFileSync(path, "utf8");
  const block = src.match(/CAPABILITY_IDS\s*=\s*\[([\s\S]*?)\]/);
  if (!block) throw new Error(`no CAPABILITY_IDS array in ${path}`);
  return new Set([...block[1].matchAll(/"([^"]+)"/g)].map((m) => m[1]));
}

/** Collect every source file under a directory, recursively. */
function sources(dir, exts) {
  if (!existsSync(dir)) return [];
  const out = [];
  for (const entry of readdirSync(dir)) {
    if (["node_modules", "dist", "__pycache__", "target", ".pytest_cache"].includes(entry)) {
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

/** The `version` under `[package]` / `[project]` of a TOML file, or null. */
function tomlVersion(path) {
  const text = readFileSync(path, "utf8");
  const section = text.match(/^\[(?:package|project)\]\s*$([\s\S]*?)(?=^\[|(?![\s\S]))/m);
  const m = section?.[1].match(/^version\s*=\s*"([^"]+)"/m);
  return m ? m[1] : null;
}

/** Agent-half pyproject.toml / Cargo.toml files, top level and one directory down. */
function agentVersionFiles(agentDir) {
  if (!existsSync(agentDir)) return [];
  const dirs = [agentDir];
  for (const entry of readdirSync(agentDir)) {
    const full = join(agentDir, entry);
    if (entry !== "tests" && statSync(full).isDirectory()) dirs.push(full);
  }
  return dirs.flatMap((d) =>
    ["pyproject.toml", "Cargo.toml"].map((f) => join(d, f)).filter((p) => existsSync(p)),
  );
}

/** permission id -> the gcs/src substring that justifies it. */
const GCS_SURFACE_CALLS = {
  "mission.read": "ctx.mission.",
  "mission.write": "ctx.mission.",
  "recording.write": "ctx.recording.",
  "command.send": "ctx.command.",
  "ui.slot.notification-channel": "ctx.notifications.",
};

// Matched against the agent source with whitespace collapsed, because the Rust
// half chains `.events` and `.publish(` across lines. Every `hardware.*` id in
// the catalog has an entry: a hardware grant with no driver code is the most
// alarming kind of decorative permission an install dialog can show.
const AGENT_SURFACE_CALLS = {
  "telemetry.extend": [/\w+\s*\.\s*extend\s*\(/],
  "event.publish": [/events\s*\.\s*publish\s*\(/],
  "video.source.set": [/set_source\s*\(/],
  "vision.detection.publish": [/publish_box|vision\.detection/],
  "mcp.expose": [/tools\s*\.\s*(register|serve)\s*\(/],
  "mavlink.write": [/mavlink\s*\.\s*send\w*\s*\(/],
  "vision.frame.read": [/subscribe_frames/],
  "hardware.uart": [/serial\.Serial\b|serial_asyncio|serialport::|tokio_serial/],
  "hardware.usb.uvc": [/uvc/i],
  "hardware.usb": [/usb/i],
  "hardware.gpu": [/onnx|ort::|coreml|cuda|tensorrt|wgpu|\bgpu\b/i],
  "hardware.i2c": [/i2c|smbus/i],
  "hardware.spi": [/spidev|\bspi\b/i],
  "hardware.gpio": [/gpio/i],
  "hardware.gpio_out": [/gpio/i],
  "hardware.camera.csi": [/libcamera|\bcsi\b/i],
};

/** Lint one manifest. Returns `{ failures, notes }`; never exits. */
export function lintManifest(manifestPath, { catalog, docsRoot }) {
  const failures = [];
  const notes = [];
  const fail = (msg) => failures.push(msg);

  const extDir = dirname(manifestPath);
  const manifest = readFileSync(manifestPath, "utf8").replace(/\r\n/g, "\n");
  const lines = manifest.split("\n");

  const gcsSrc = readAll(sources(join(extDir, "gcs", "src"), [".ts", ".tsx"]));
  const agentFiles = sources(join(extDir, "agent"), [".py", ".rs"]).filter(
    (f) => !f.includes(join("agent", "tests")),
  );
  const pySrc = readAll(agentFiles.filter((f) => f.endsWith(".py")));
  const rsSrc = readAll(agentFiles.filter((f) => f.endsWith(".rs")));
  const agentFlat = `${pySrc}\n${rsSrc}`.replace(/\s+/g, " ");

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

  function manifestScalar(key) {
    const m = manifest.match(new RegExp(`^${key}:\\s*"?([^"\\n]*)"?`, "m"));
    return m ? m[1].trim() : null;
  }

  // -- 1 + 2: permissions against call sites -------------------------------

  const gcsPerms = permissionsOf("gcs");
  const agentPerms = permissionsOf("agent");

  const declaredTopics = gcsPerms
    .filter((p) => p.startsWith("telemetry.subscribe."))
    .map((p) => p.slice("telemetry.subscribe.".length));

  const constants = new Map(
    [...gcsSrc.matchAll(/const\s+([A-Z][A-Z0-9_]*)\s*(?::\s*\w+\s*)?=\s*["'`]([^"'`]+)["'`]/g)].map(
      (m) => [m[1], m[2]],
    ),
  );
  const usedTopics = [
    ...gcsSrc.matchAll(
      /telemetry\s*\.\s*subscribe(?:<[^>]*>)?\(\s*(?:["'`]([^"'`]+)["'`]|([A-Z][A-Z0-9_]*)\b)/g,
    ),
  ]
    .map((m) => m[1] ?? constants.get(m[2]))
    .filter((t) => t !== undefined);

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

  for (const [perm, call] of Object.entries(GCS_SURFACE_CALLS)) {
    if (gcsPerms.includes(perm) && !gcsSrc.includes(call)) {
      fail(
        `FAIL gcs.permissions declares '${perm}' but no ${call} call site exists ` +
          `in gcs/src — a permission the bundle never uses is shown to the ` +
          `operator in the install dialog for nothing`,
      );
    }
  }

  for (const [perm, patterns] of Object.entries(AGENT_SURFACE_CALLS)) {
    if (agentPerms.includes(perm) && !patterns.some((re) => re.test(agentFlat))) {
      fail(
        `FAIL agent.permissions declares '${perm}' but the agent half contains no ` +
          `matching call site`,
      );
    }
  }

  // -- 3: declared slots against implementations ---------------------------

  const slots = lines
    .map((l) => l.match(/^\s*slot:\s*(\S+)/))
    .filter(Boolean)
    .map((m) => m[1]);
  if (slots.length > 1 && gcsSrc && !gcsSrc.includes("video.overlay.props")) {
    fail(
      `FAIL the manifest contributes ${slots.length} slots (${slots.join(", ")}) ` +
        `but the bundle never discriminates between them (no ` +
        `ctx.events.subscribe("video.overlay.props") branch) — one of those slots ` +
        `renders the wrong surface`,
    );
  }
  if (slots.includes("map.overlay") && !/map[-.]overlay/.test(gcsSrc)) {
    fail("FAIL the manifest contributes a map.overlay panel with no implementation in gcs/src");
  }

  // -- 4: version agreement ------------------------------------------------

  const manifestVersion = manifestScalar("version");
  const versionSites = [];
  for (const rel of ["package.json", join("gcs", "package.json")]) {
    const p = join(extDir, rel);
    if (!existsSync(p)) continue;
    versionSites.push([rel, JSON.parse(readFileSync(p, "utf8")).version ?? null]);
  }
  const definePluginVersion = gcsSrc.match(/version:\s*["'`]([0-9][^"'`]*)["'`]/);
  if (definePluginVersion) {
    versionSites.push(["definePlugin({ version })", definePluginVersion[1]]);
  }
  for (const p of agentVersionFiles(join(extDir, "agent"))) {
    versionSites.push([p.slice(extDir.length + 1).replace(/\\/g, "/"), tomlVersion(p)]);
  }
  for (const [where, value] of versionSites) {
    if (value !== null && value !== manifestVersion) {
      fail(
        `FAIL version drift: ${where} says ${value}, manifest.yaml says ` +
          `${manifestVersion} — every version site ships in the same archive`,
      );
    }
  }

  // -- 5: permission ids against the capability catalog --------------------

  const leaf = (manifestScalar("id") ?? "").split(".").pop();
  const scoped = `plugin.${leaf}.`;
  for (const [half, perms] of [["gcs", gcsPerms], ["agent", agentPerms]]) {
    for (const id of perms) {
      const known =
        catalog.has(id) ||
        (leaf && id.startsWith(scoped)) ||
        (id.startsWith("telemetry.subscribe.") && catalog.has("telemetry.subscribe"));
      if (!known) {
        fail(
          `FAIL ${half}.permissions declares '${id}', which is not in the capability ` +
            `catalog and not scoped to this plugin (${scoped}*)`,
        );
      }
    }
  }

  // -- 6: documentation_url against the docs tree ---------------------------

  const docUrl = manifestScalar("documentation_url");
  if (docUrl?.startsWith(DOCS_HOST)) {
    const page = docUrl.slice(DOCS_HOST.length).split(/[#?]/)[0].replace(/\/+$/, "");
    if (!docsRoot || !existsSync(docsRoot)) {
      notes.push(`note: docs path check skipped, no Documentation checkout at ${docsRoot}`);
    } else {
      const candidates = [`${page}.mdx`, `${page}/index.mdx`].map((p) => join(docsRoot, p));
      let redirected = false;
      const docsJson = join(docsRoot, "docs.json");
      if (existsSync(docsJson)) {
        const redirects = JSON.parse(readFileSync(docsJson, "utf8")).redirects ?? [];
        redirected = redirects.some((r) => r.source?.replace(/^\/+|\/+$/g, "") === page);
      }
      if (!redirected && !candidates.some((p) => existsSync(p))) {
        fail(
          `FAIL documentation_url points at '/${page}', but ${page}.mdx does not exist ` +
            `in the Documentation tree — the install dialog link returns 404`,
        );
      }
    }
  }

  // -- 7: agent config reads against the schema ----------------------------

  const schemaPath = join(extDir, "config-schema.json");
  if (existsSync(schemaPath)) {
    const allowed = new Set(
      Object.keys(JSON.parse(readFileSync(schemaPath, "utf8")).properties ?? {}),
    );
    for (const m of manifest.matchAll(/config_key:\s*"?([\w.-]+)"?/g)) allowed.add(m[1]);
    const writes = /(?:config_kv\s*\.\s*set|config\s*\.\s*set|config_set)\(\s*["']([^"']+)["']/g;
    for (const m of `${pySrc}\n${rsSrc}`.matchAll(writes)) allowed.add(m[1]);
    const reads = [
      ...pySrc.matchAll(/config_kv\s*\.\s*(?:get|static)\(\s*["']([^"']+)["']/g),
      ...rsSrc.matchAll(/config\s*\.\s*get\(\s*"([^"]+)"/g),
    ].map((m) => m[1]);
    for (const key of new Set(reads)) {
      if (!allowed.has(key)) {
        fail(
          `FAIL the agent reads config key '${key}', which is not in config-schema.json, ` +
            `not a manifest config_key and never written by the agent — the operator ` +
            `has no way to set it`,
        );
      }
    }
  }

  return { failures, notes };
}

function defaultDocsRoot() {
  return process.env.ADOS_DOCS_ROOT ?? resolve(REPO_ROOT, "..", "Documentation");
}

/** Run every scripts/lint-fixtures case; returns the number of failed cases. */
function selfTest() {
  const root = join(REPO_ROOT, "scripts", "lint-fixtures");
  const base = join(root, "base");
  const catalog = loadCatalog();
  const cases = readdirSync(root).filter((d) => d !== "base" && statSync(join(root, d)).isDirectory());
  let bad = 0;
  for (const name of cases.sort()) {
    const work = mkdtempSync(join(tmpdir(), `lint-manifest-${name}-`));
    try {
      cpSync(base, work, { recursive: true });
      cpSync(join(root, name), work, { recursive: true });
      const expect = readFileSync(join(work, "expect.txt"), "utf8")
        .split(/\r?\n/)
        .map((l) => l.trim())
        .filter(Boolean);
      const { failures } = lintManifest(join(work, "manifest.yaml"), {
        catalog,
        docsRoot: join(work, "docs"),
      });
      const ok =
        expect[0] === "OK"
          ? failures.length === 0
          : failures.length > 0 && expect.every((e) => failures.some((f) => f.includes(e)));
      console.log(`${ok ? "pass" : "FAIL"} ${name}`);
      if (!ok) {
        bad += 1;
        console.error(`  expected: ${expect.join(" | ")}`);
        for (const f of failures) console.error(`  got: ${f}`);
      }
    } finally {
      rmSync(work, { recursive: true, force: true });
    }
  }
  console.log(`lint-manifest self-test: ${cases.length - bad}/${cases.length} cases pass`);
  return bad;
}

function main() {
  if (process.argv[2] === "--self-test") {
    process.exit(selfTest() === 0 ? 0 : 1);
  }
  const manifestPath = resolve(process.argv[2] ?? "manifest.yaml");
  if (!existsSync(manifestPath)) {
    console.error(`lint-manifest: no manifest at ${manifestPath}`);
    process.exit(2);
  }
  const { failures, notes } = lintManifest(manifestPath, {
    catalog: loadCatalog(),
    docsRoot: defaultDocsRoot(),
  });
  for (const n of notes) console.log(n);
  if (failures.length > 0) {
    for (const f of failures) console.error(f);
    console.error(`\n${failures.length} manifest problem(s) in ${manifestPath}`);
    process.exit(1);
  }
  console.log(`lint-manifest: OK ${manifestPath}`);
}

if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  main();
}
