/**
 * The reconstruction badge names the backend that built a world model and
 * shows nothing for an unknown/absent backend. Also covers `backendOf` (the
 * cloud metadata reader).
 */

import { describe, it, expect, afterEach } from "vitest";
import { cleanup, render, screen } from "@testing-library/react";

import { ReconstructionBadge } from "../../../src/components/atlas/ReconstructionBadge";
import { backendOf } from "../../../src/components/atlas/viewer-types";

afterEach(cleanup);

function renderBadge(backend: string | null | undefined) {
  return render(<ReconstructionBadge backend={backend} />);
}

describe("backendOf (cloud metadata reader)", () => {
  it("reads a non-empty string metadata.backend, else null", () => {
    expect(backendOf({ backend: "gsplat" })).toBe("gsplat");
    expect(backendOf({ backend: "brush", viewerHint: "splat" })).toBe("brush");
    expect(backendOf({ backend: "" })).toBeNull();
    expect(backendOf({ backend: 7 })).toBeNull();
    expect(backendOf({})).toBeNull();
    expect(backendOf(null)).toBeNull();
    expect(backendOf("not-an-object")).toBeNull();
  });
});

describe("ReconstructionBadge", () => {
  it("names the backend", () => {
    renderBadge(" brush ");
    const chip = screen.getByText("Reconstructed with brush");
    expect(chip.getAttribute("title")).toBe("Reconstructed by the brush backend");
  });

  it("renders nothing for an unknown/absent backend", () => {
    expect(renderBadge(null).container.firstChild).toBeNull();
    expect(renderBadge(undefined).container.firstChild).toBeNull();
    expect(renderBadge("   ").container.firstChild).toBeNull();
  });
});
