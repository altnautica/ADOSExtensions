/**
 * The viewers' operator strings come from the package catalogue.
 */

import { afterEach, describe, expect, it } from "vitest";
import { cleanup, render, screen } from "@testing-library/react";

import messages from "../../locales/en.json";
import { ViewerError } from "../../src/components/model/viewers/ViewerError";
import { ViewerLoading } from "../../src/components/model/viewers/ViewerLoading";

afterEach(cleanup);

describe("viewer strings", () => {
  it("ViewerLoading's defaults come from the catalogue", () => {
    render(<ViewerLoading percent={10} />);
    expect(screen.getByRole("status").getAttribute("aria-label")).toBe(messages.viewerStatus.loadingViewer);
    expect(screen.getByText(messages.viewerStatus.loading)).toBeTruthy();
  });

  it("ViewerError names what failed", () => {
    render(<ViewerError what={messages.viewerStatus.what.pointCloud} />);
    expect(screen.getByText("Could not load the point cloud.")).toBeTruthy();
  });
});
