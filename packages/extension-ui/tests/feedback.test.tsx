import { describe, expect, it } from "vitest";
import { render, screen } from "@testing-library/react";

import { DiagnosticTable, Toast } from "../src";

describe("DiagnosticTable", () => {
  it("renders one row per entry", () => {
    render(
      <DiagnosticTable
        rows={[
          { label: "fx", value: "1385.4" },
          { label: "fy", value: "1384.1" },
        ]}
      />,
    );
    expect(screen.getByText("fx")).not.toBeNull();
    expect(screen.getByText("1384.1")).not.toBeNull();
  });

  it("shows the compare column when any row has compare", () => {
    render(
      <DiagnosticTable
        rows={[
          { label: "fx", value: "1385.4", compare: "1390.0" },
          { label: "fy", value: "1384.1" },
        ]}
        compareLabel="Previous"
      />,
    );
    expect(screen.getByText("Previous")).not.toBeNull();
    expect(screen.getByText("1390.0")).not.toBeNull();
  });
});

describe("Toast", () => {
  it("renders nothing when message is null", () => {
    const { container } = render(<Toast message={null} />);
    expect(container.firstChild).toBeNull();
  });

  it("renders the message when present", () => {
    render(<Toast message="Calibration applied" kind="success" />);
    expect(
      screen.getByTestId("ext-ui-toast-success"),
    ).not.toBeNull();
  });
});
