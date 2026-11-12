/**
 * The viewer switcher's labels come from the catalogue, and every button shows
 * a focus ring and a touch-sized target under a coarse pointer.
 */

import { afterEach, describe, expect, it } from "vitest";
import { cleanup, render, screen } from "@testing-library/react";

import messages from "../../../../locales/en.json";
import { ViewerSwitcher } from "../../../src/components/atlas/ViewerSwitcher";

afterEach(cleanup);

describe("ViewerSwitcher", () => {
  it("labels each viewer from the catalogue with focus and touch styles", () => {
    render(<ViewerSwitcher viewer="rerun" onSelect={() => {}} ariaLabel="Viewer" />);
    for (const id of ["rerun", "splat", "cloud", "lod"] as const) {
      const button = screen.getByRole("button", { name: messages.atlas.viewer[id] });
      expect(button.className).toContain("we:focus-visible:outline-2");
      expect(button.className).toContain("we:pointer-coarse:min-h-12");
    }
  });
});
