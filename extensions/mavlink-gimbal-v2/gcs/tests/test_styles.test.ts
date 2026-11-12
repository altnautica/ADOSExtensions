/**
 * The host loads only the bundle into the sandboxed frame, so the panel
 * styles must travel inside it and be injected at mount.
 */

import { readFileSync } from "node:fs";
import { resolve } from "node:path";

import { build } from "esbuild";
import { describe, expect, it } from "vitest";

import { injectStyles } from "../src/styles";

const GCS = resolve(__dirname, "..");
const css = readFileSync(resolve(GCS, "plugin.css"), "utf8");

function ruleFor(selector: string): string {
  const bare = css.replace(/\/\*[\s\S]*?\*\//g, "");
  const blocks = [...bare.matchAll(/([^{}]+)\{([^}]*)\}/g)];
  const hit = blocks.find(([, sel]) => sel!.split(",").some((s) => s.trim() === selector));
  return hit ? hit[2]! : "";
}

describe("panel styles", () => {
  it("inlines plugin.css into the bundle the package build script makes", async () => {
    // Build with exactly the loader options the package's build script
    // passes, so dropping the flag there fails this test.
    const script: string = JSON.parse(readFileSync(resolve(GCS, "package.json"), "utf8"))
      .scripts.build;
    const loader = Object.fromEntries(
      [...script.matchAll(/--loader:(\.[\w]+)=(\w+)/g)].map(([, ext, kind]) => [ext!, kind!]),
    ) as Record<string, "text">;
    expect(loader[".css"]).toBe("text");
    const out = await build({
      entryPoints: [resolve(GCS, "src/plugin.ts")],
      bundle: true,
      format: "esm",
      write: false,
      loader,
      logLevel: "silent",
    });
    const text = out.outputFiles[0]!.text;
    expect(text).toContain(".agm-root");
    expect(text).toContain("injectStyles");
  });

  it("injects the styles once as a style element", () => {
    document.head.innerHTML = "";
    injectStyles(css);
    injectStyles(css);
    const styles = document.head.querySelectorAll("style");
    expect(styles).toHaveLength(1);
    expect(styles[0]!.textContent).toContain(".agm-root");
  });

  it("lets the panel scroll inside the frame", () => {
    expect(ruleFor("html")).toMatch(/overflow-y:\s*auto/);
    expect(ruleFor(".agm-root")).toMatch(/overflow-y:\s*auto/);
  });

  it("gives every control a 48 px touch target", () => {
    for (const sel of [".agm-root button", ".agm-root select", ".agm-root input"]) {
      expect(ruleFor(sel), sel).toMatch(/min-height:\s*48px/);
    }
    expect(ruleFor('.agm-root input[type="range"]')).toMatch(/height:\s*48px/);
  });
});
