import { describe, expect, it } from "vitest";
import { render, screen } from "@testing-library/react";

import { FramePreview, PoseCoverageMap } from "../src";

describe("FramePreview", () => {
  it("renders empty-state copy when no frames", () => {
    render(<FramePreview frames={[]} />);
    expect(screen.getByText(/No frames captured yet/)).not.toBeNull();
  });

  it("renders one thumb per frame with the right testid", () => {
    render(
      <FramePreview
        frames={[
          { id: "a", src: "data:image/png;base64,A" },
          { id: "b", src: "data:image/png;base64,B" },
        ]}
      />,
    );
    expect(
      screen.getByTestId("ext-ui-frame-preview-thumb-a"),
    ).not.toBeNull();
    expect(
      screen.getByTestId("ext-ui-frame-preview-thumb-b"),
    ).not.toBeNull();
  });

  it("shows the remove button only when onRemove is provided", () => {
    const { rerender } = render(
      <FramePreview
        frames={[{ id: "a", src: "data:image/png;base64,A" }]}
      />,
    );
    expect(
      screen.queryByTestId("ext-ui-frame-preview-remove-a"),
    ).toBeNull();

    rerender(
      <FramePreview
        frames={[{ id: "a", src: "data:image/png;base64,A" }]}
        onRemove={() => {}}
      />,
    );
    expect(
      screen.getByTestId("ext-ui-frame-preview-remove-a"),
    ).not.toBeNull();
  });
});

describe("PoseCoverageMap", () => {
  it("renders gridSize * gridSize cells", () => {
    const { container } = render(
      <PoseCoverageMap samples={[]} gridSize={4} />,
    );
    const cells = container.querySelectorAll(
      "[data-testid^='ext-ui-pose-coverage-cell-']",
    );
    expect(cells.length).toBe(16);
  });

  it("buckets samples into the right cell", () => {
    // Center of the map (tilt=45, rotation=180) lands in the middle
    // bucket of a 5x5 grid: tIdx=2, rIdx=2 -> idx = 12.
    const { container } = render(
      <PoseCoverageMap
        samples={[{ tiltDeg: 45, rotationDeg: 180 }]}
        gridSize={5}
      />,
    );
    const cell = container.querySelector(
      "[data-testid='ext-ui-pose-coverage-cell-12']",
    );
    expect(cell?.getAttribute("title")).toBe("1 pose");
  });
});
