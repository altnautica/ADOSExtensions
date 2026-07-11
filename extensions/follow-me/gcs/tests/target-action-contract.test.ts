import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";

import { describe, expect, it } from "vitest";

/**
 * Verify the manifest's target-action contribution matches the host's
 * target-action registry contract (a stable id, a designate step, a
 * per-drone config write, a bound key on the selected target), and that the
 * private video overlay it replaced is gone. Reads the real manifest.yaml
 * with a focused line scanner (no YAML dependency), mirroring
 * skill-contract.test.ts.
 */

const here = dirname(fileURLToPath(import.meta.url));
const manifestPath = resolve(here, "../../manifest.yaml");
const manifest = readFileSync(manifestPath, "utf8");

interface TargetAction {
  id?: string;
  label?: string;
  appliesToClass?: string;
  designate?: boolean;
  configKey?: string;
  configValue?: boolean;
  defaultKey?: string;
}

/** Extract the first entry under gcs.contributes.target_actions. */
function extractFirstTargetAction(text: string): TargetAction {
  const lines = text.split("\n");
  const out: TargetAction = {};
  let inBlock = false;
  let inEntry = false;
  const indentOf = (l: string): number => l.length - l.trimStart().length;

  for (const line of lines) {
    const trimmed = line.trim();
    if (trimmed === "target_actions:") {
      inBlock = true;
      continue;
    }
    if (!inBlock) continue;

    // A new sibling contributes key (same indent, ends in a colon) ends the
    // block. Comment lines and deeper entry keys do not.
    if (
      inEntry &&
      indentOf(line) <= 4 &&
      trimmed.endsWith(":") &&
      !trimmed.startsWith("#")
    ) {
      break;
    }

    if (trimmed.startsWith("- id:")) {
      if (inEntry) break; // Only the first entry is inspected.
      inEntry = true;
      out.id = trimmed.replace("- id:", "").trim();
      continue;
    }
    if (!inEntry) continue;

    const parts = trimmed.split(":");
    const key = (parts[0] ?? "").trim();
    const value = parts.slice(1).join(":").trim().replace(/^"|"$/g, "");
    if (key === "label") out.label = value;
    if (key === "applies_to_class") out.appliesToClass = value;
    if (key === "designate") out.designate = value === "true";
    if (key === "config_key") out.configKey = value;
    if (key === "config_value") out.configValue = value === "true";
    if (key === "default_key") out.defaultKey = value;
  }
  return out;
}

describe("manifest target-action contract", () => {
  const action = extractFirstTargetAction(manifest);

  it("declares the follow target action", () => {
    expect(action.id).toBe("follow");
    expect(action.label).toBe("Follow this target");
  });

  it("designates then writes the active config key on the selected target", () => {
    expect(action.designate).toBe(true);
    expect(action.configKey).toBe("active");
    expect(action.configValue).toBe(true);
  });

  it("applies to person detections and binds the f key", () => {
    expect(action.appliesToClass).toBe("person");
    expect(action.defaultKey).toBe("f");
  });
});

describe("the private video overlay is removed (host owns it now)", () => {
  it("no longer declares a video.overlay panel", () => {
    expect(manifest).not.toContain("slot: video.overlay");
  });

  it("drops the now-unused video-overlay and command.send permissions", () => {
    expect(manifest).not.toContain("ui.slot.video-overlay");
    expect(manifest).not.toContain("command.send");
  });
});
