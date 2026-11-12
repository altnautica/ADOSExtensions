/**
 * The host loads only the bundle into the sandboxed frame: styles must travel
 * inside it, the overlay slot must hide the console without them, and the
 * version the bundle reports must match the manifest.
 */

import { readFileSync } from "node:fs";
import { resolve } from "node:path";

import { build } from "esbuild";
import { describe, expect, it } from "vitest";

import { createPluginHarness } from "@altnautica/plugin-sdk/harness";

import { mountSiyi } from "../src/mount";
import { injectStyles } from "../src/styles";
import { PLUGIN_VERSION } from "../src/version";

const GCS = resolve(__dirname, "..");
const css = readFileSync(resolve(GCS, "plugin.css"), "utf8");

function ruleFor(selector: string): string {
  const blocks = [...css.matchAll(/([^{}]+)\{([^}]*)\}/g)];
  const hit = blocks.find(([, sel]) => sel!.split(",").some((s) => s.trim() === selector));
  return hit ? hit[2]! : "";
}

describe("styles", () => {
  it("inlines plugin.css into the bundle", async () => {
    const out = await build({
      entryPoints: [resolve(GCS, "src/plugin.ts")],
      bundle: true,
      format: "esm",
      write: false,
      loader: { ".css": "text" },
      logLevel: "silent",
    });
    const text = out.outputFiles[0]!.text;
    expect(text).toContain(".siyi-panel");
    expect(text).toContain("min-height: 48px");
  });

  it("injects the styles once as a style element", () => {
    document.head.innerHTML = "";
    injectStyles(css);
    injectStyles(css);
    const styles = document.head.querySelectorAll("style[data-siyi-pod]");
    expect(styles).toHaveLength(1);
    expect(styles[0]!.textContent).toContain(".siyi-panel");
  });

  it("gives every control a 48 px hit target", () => {
    for (const sel of [".siyi-btn", ".siyi-sel", ".siyi-range"]) {
      expect(ruleFor(sel)).toMatch(/min-height:\s*48px/);
    }
  });
});

describe("overlay mode", () => {
  it("hides the console inline when the video overlay props arrive", async () => {
    const harness = createPluginHarness({
      grantedCapabilities: ["command.send", "telemetry.subscribe.siyi"],
      mount: () => undefined,
    });
    await harness.start();
    document.head.innerHTML = "";
    const root = document.createElement("div");
    document.body.appendChild(root);
    const handles = await mountSiyi(harness.ctx, root);
    const panel = root.querySelector(".siyi-panel") as HTMLElement;
    expect(panel.style.display).toBe("");
    harness.pushVideoOverlayProps({
      droneId: "d1",
      cameraId: "main",
      streamWidth: 1920,
      streamHeight: 1080,
      renderedRect: { left: 0, top: 0, width: 640, height: 360 },
      frameTimestampMs: 1,
      frameAgeMs: null,
      attitude: null,
      attitudeAtFrameTime: false,
      detections: null,
    });
    await Promise.resolve();
    expect(panel.style.display).toBe("none");
    expect(
      (root.querySelector('[data-testid="siyi-command-status"]') as HTMLElement).style.display,
    ).toBe("none");
    for (const h of handles) h.destroy();
    root.remove();
    await harness.teardown();
  });
});

describe("version", () => {
  it("reports the manifest version", () => {
    const manifest = readFileSync(resolve(GCS, "../manifest.yaml"), "utf8");
    const m = manifest.match(/^version:\s*"([^"]+)"/m);
    expect(PLUGIN_VERSION).toBe(m![1]);
  });
});
