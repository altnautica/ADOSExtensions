/**
 * The reconstruction-honesty badge (no fabricated reading): a `mock`
 * reconstruction wears an unmissable warning chip, a real backend wears a calm
 * chip naming it, and an unknown/absent backend shows nothing. Also covers
 * `backendOf` (the cloud metadata reader) and the `isMockBackend` predicate.
 */

import { describe, it, expect, afterEach } from "vitest";
import { render, screen, cleanup } from "@testing-library/react";
import {
  ReconstructionBadge,
  isMockBackend,
} from "../../../src/components/atlas/ReconstructionBadge";
import { backendOf } from "../../../src/components/atlas/viewer-types";
import messages from "../../../../locales/en.json";

afterEach(cleanup);

function renderBadge(backend: string | null | undefined) {
  return render(<ReconstructionBadge backend={backend} />);
}

describe("isMockBackend", () => {
  it("matches mock case/whitespace-insensitively, nothing else", () => {
    expect(isMockBackend("mock")).toBe(true);
    expect(isMockBackend("  MOCK  ")).toBe(true);
    expect(isMockBackend("Mock")).toBe(true);
    expect(isMockBackend("brush")).toBe(false);
    expect(isMockBackend("mocked")).toBe(false);
    expect(isMockBackend("")).toBe(false);
    expect(isMockBackend(null)).toBe(false);
    expect(isMockBackend(undefined)).toBe(false);
  });
});

describe("backendOf (cloud metadata reader)", () => {
  it("reads a non-empty string metadata.backend, else null", () => {
    expect(backendOf({ backend: "nerfstudio" })).toBe("nerfstudio");
    expect(backendOf({ backend: "mock", viewerHint: "splat" })).toBe("mock");
    expect(backendOf({ backend: "" })).toBeNull();
    expect(backendOf({ backend: 7 })).toBeNull();
    expect(backendOf({})).toBeNull();
    expect(backendOf(null)).toBeNull();
    expect(backendOf("not-an-object")).toBeNull();
  });
});

describe("ReconstructionBadge", () => {
  it("shows an unmissable placeholder chip in a warning tone for a mock backend", () => {
    renderBadge("mock");
    const chip = screen.getByText(messages.atlas.placeholderArtifactBadge);
    // Warning tone so a mock is never mistaken for a real reconstruction.
    expect(chip.className).toContain("we:text-status-warning");
    expect(chip.getAttribute("title")).toBe(messages.atlas.placeholderArtifactHint);
  });

  it("names a real backend in a calm (non-warning) chip", () => {
    renderBadge(" brush ");
    const chip = screen.getByText("Reconstructed with brush");
    expect(chip.className).not.toContain("status-warning");
    expect(chip.getAttribute("title")).toBe("Reconstructed by the brush backend");
    // The placeholder label must NOT appear for a real reconstruction.
    expect(screen.queryByText(messages.atlas.placeholderArtifactBadge)).toBeNull();
  });

  it("renders nothing for an unknown/absent backend", () => {
    expect(renderBadge(null).container.firstChild).toBeNull();
    expect(renderBadge(undefined).container.firstChild).toBeNull();
    expect(renderBadge("   ").container.firstChild).toBeNull();
  });
});
