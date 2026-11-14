import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";

import { describe, expect, it } from "vitest";

/**
 * Verify the manifest's skill contribution satisfies the host's v1 honor
 * rule: a flight Skill is registered only when activation.via is config,
 * state.via is event, and both config_key and topic are set
 * (manifest-parse.ts parseSkillContributions). This reads the real
 * manifest.yaml and extracts the skill block with a focused parser so the
 * test breaks if the contract drifts.
 */

const here = dirname(fileURLToPath(import.meta.url));
const manifestPath = resolve(here, "../../manifest.yaml");

interface SkillBlock {
  id?: string;
  toggle?: boolean;
  confirm?: boolean;
  armRequirement?: string;
  bindingKey?: string;
  stateTopic?: string;
  activationVia?: string;
  activationConfigKey?: string;
  stateVia?: string;
}

/** Extract the first skill entry under gcs.contributes.skills from the
 * manifest text. A focused line scanner (no YAML dependency) that reads the
 * exact nested keys the host honor rule checks. */
function extractFirstSkill(text: string): SkillBlock {
  const lines = text.split("\n");
  const out: SkillBlock = {};
  let inSkills = false;
  let inEntry = false;
  let section: "activation" | "state" | "binding" | null = null;

  const indentOf = (l: string): number => l.length - l.trimStart().length;

  for (const line of lines) {
    const trimmed = line.trim();
    if (trimmed === "skills:") {
      inSkills = true;
      continue;
    }
    if (!inSkills) continue;

    // A new top-level contributes key (e.g. panels:) ends the skills block.
    if (inEntry && indentOf(line) <= 4 && trimmed.endsWith(":") && trimmed !== "skills:") {
      break;
    }

    if (trimmed.startsWith("- id:")) {
      if (inEntry) break; // Only the first skill entry is inspected.
      inEntry = true;
      out.id = trimmed.replace("- id:", "").trim();
      section = null;
      continue;
    }
    if (!inEntry) continue;

    if (trimmed === "activation:") {
      section = "activation";
      continue;
    }
    if (trimmed === "state:") {
      section = "state";
      continue;
    }
    if (trimmed === "default_binding:") {
      section = "binding";
      continue;
    }

    const parts = trimmed.split(":");
    const key = (parts[0] ?? "").trim();
    const value = parts.slice(1).join(":").trim().replace(/^"|"$/g, "");

    if (section === "activation") {
      if (key === "via") out.activationVia = value;
      if (key === "config_key") out.activationConfigKey = value;
    } else if (section === "state") {
      if (key === "via") out.stateVia = value;
      if (key === "topic") out.stateTopic = value;
    } else if (section === "binding") {
      if (key === "key") out.bindingKey = value;
    } else {
      if (key === "toggle") out.toggle = value === "true";
      if (key === "confirm") out.confirm = value === "true";
      if (key === "arm_requirement") out.armRequirement = value;
    }
  }
  return out;
}

/** The `schema.default` of one manifest config param, read by line scan. */
function manifestParamDefault(text: string, key: string): string | undefined {
  const lines = text.split("\n");
  const start = lines.findIndex((l) => l.trim() === `- key: ${key}`);
  if (start < 0) return undefined;
  for (const line of lines.slice(start + 1)) {
    const trimmed = line.trim();
    if (trimmed.startsWith("- key:")) return undefined;
    if (trimmed.startsWith("default:")) return trimmed.slice("default:".length).trim();
  }
  return undefined;
}

describe("manifest skill contract", () => {
  const skill = extractFirstSkill(readFileSync(manifestPath, "utf8"));

  it("declares the follow-me skill id", () => {
    expect(skill.id).toBe("follow-me");
  });

  it("is honored by the host v1 rule (config activation + event state)", () => {
    // The exact predicate from parseSkillContributions.
    const honored =
      skill.activationVia === "config" &&
      skill.stateVia === "event" &&
      !!skill.activationConfigKey &&
      !!skill.stateTopic;
    expect(honored).toBe(true);
  });

  it("activates via the active config key and reads the follow.state topic", () => {
    expect(skill.activationConfigKey).toBe("active");
    expect(skill.stateTopic).toBe("follow.state");
  });

  it("is a toggle armed-only skill that confirms before arming", () => {
    expect(skill.toggle).toBe(true);
    // Arming streams guided setpoints, so it must not arm on a bare keypress.
    expect(skill.confirm).toBe(true);
    expect(skill.armRequirement).toBe("armed");
  });

  it("binds a distinct chord from the click-to-follow target action", () => {
    // The Skill Bar toggle uses shift+f so it never collides with the
    // "Follow this target" action bound to plain f.
    expect(skill.bindingKey).toBe("shift+f");
  });
});

describe("manifest config defaults", () => {
  it("follows at 6 m, where a walker is still tall enough in frame to detect", () => {
    expect(manifestParamDefault(readFileSync(manifestPath, "utf8"), "follow_distance_m")).toBe("6");
  });

  it("leaves the camera geometry unset, so the camera roster or the reference camera applies", () => {
    const text = readFileSync(manifestPath, "utf8");
    for (const key of ["camera_hfov_deg", "mount_pitch_deg"]) {
      expect(manifestParamDefault(text, key), key).toBeUndefined();
      const block = text.slice(text.indexOf(`- key: ${key}\n`));
      const widget = /widget: (\S+)/.exec(block)?.[1];
      expect(widget, key).toBe("number");
    }
  });
});

describe("manifest 0.5 release", () => {
  const text = readFileSync(manifestPath, "utf8");
  const keys = [...text.matchAll(/^\s+- key: (\S+)$/gm)].map((m) => m[1]);

  it("declares the 0.5 recovery settings and no reacquire window", () => {
    expect(keys).toEqual(
      expect.arrayContaining([
        "uncertain_mode",
        "predictive_search",
        "search_budget_s",
        "search_radius_m",
        "search_speed_mps",
      ]),
    );
    expect(keys).not.toContain("reacquire_window_s");
    expect(/^version: "([^"]+)"/m.exec(text)?.[1]).toBe("0.5.0");
  });

  it("needs the agent whose surfaces show the recovery phases and the waiting prompt", () => {
    expect(/^\s+ados_version: "([^"]+)"/m.exec(text)?.[1]).toBe(">=0.115.0");
  });

  it("may send commands, for the hold prompt's actions", () => {
    const gcs = text.slice(text.indexOf("\ngcs:"));
    const perms = gcs.slice(0, gcs.indexOf("contributes:"));
    expect(perms).toContain("- id: command.send");
  });
});
