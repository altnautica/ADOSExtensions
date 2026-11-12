/**
 * Every operator string the world-model viewers show comes from the
 * catalogue: the loading defaults and the phase labels and failure subjects
 * the viewers pass in.
 */

import { readFileSync, readdirSync } from "node:fs";
import { join } from "node:path";
import { afterEach, describe, expect, it } from "vitest";
import { cleanup, render, screen } from "@testing-library/react";

import messages from "../../../../../locales/en.json";
import { ViewerLoading } from "../../../../src/components/atlas/viewers/ViewerLoading";

afterEach(cleanup);

const VIEWERS_DIR = join(__dirname, "../../../../src/components/atlas/viewers");

describe("viewer strings", () => {
  it("ViewerLoading's defaults come from the catalogue", () => {
    render(<ViewerLoading percent={10} />);
    expect(screen.getByRole("status").getAttribute("aria-label")).toBe(messages.atlas.viewerStatus.loadingViewer);
    expect(screen.getByText(messages.atlas.viewerStatus.loading)).toBeTruthy();
  });

  it("no viewer passes a literal label or failure subject", () => {
    const literal = /\b(label|what)="[^"]*"|\?\?\s*"[A-Z][^"]*"/;
    for (const file of readdirSync(VIEWERS_DIR).filter((f) => f.endsWith(".tsx"))) {
      const source = readFileSync(join(VIEWERS_DIR, file), "utf8");
      expect(source.match(literal)?.[0] ?? null, file).toBeNull();
    }
  });

  it("no viewer shows words from a template literal", () => {
    // A template literal's static text (its `${...}` parts removed) holding a
    // word is operator text that bypasses the catalogue. Comments are dropped
    // first so prose mentioning `.ply` or a symbol name does not count.
    for (const file of readdirSync(VIEWERS_DIR).filter((f) => f.endsWith(".tsx"))) {
      const code = readFileSync(join(VIEWERS_DIR, file), "utf8")
        .replace(/\/\*[\s\S]*?\*\//g, "")
        .replace(/^\s*\/\/.*$/gm, "");
      const worded = [...code.matchAll(/`((?:[^`\\]|\\.)*)`/g)]
        .map((m) => m[1] ?? "")
        .filter((body) => /[A-Za-z]{3,}/.test(body.replace(/\$\{[^}]*\}/g, "")));
      expect(worded, file).toEqual([]);
    }
  });

  it("the world viewport names failed viewers from the catalogue", () => {
    const source = readFileSync(join(VIEWERS_DIR, "../WorldModelViewport.tsx"), "utf8");
    expect(source).not.toMatch(/"point cloud"|"Rerun"/);
  });
});
