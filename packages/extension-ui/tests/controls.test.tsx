import { describe, expect, it, vi } from "vitest";
import { render, screen, fireEvent } from "@testing-library/react";

import {
  FilePicker,
  PrimaryButton,
  ProgressBar,
  ResultBanner,
  SecondaryButton,
} from "../src";

describe("PrimaryButton", () => {
  it("fires onClick when enabled", () => {
    const cb = vi.fn();
    render(<PrimaryButton onClick={cb}>Go</PrimaryButton>);
    fireEvent.click(screen.getByTestId("ext-ui-primary-button"));
    expect(cb).toHaveBeenCalledOnce();
  });

  it("suppresses onClick when disabled", () => {
    const cb = vi.fn();
    render(
      <PrimaryButton onClick={cb} disabled>
        Go
      </PrimaryButton>,
    );
    fireEvent.click(screen.getByTestId("ext-ui-primary-button"));
    expect(cb).not.toHaveBeenCalled();
  });

  it("appends testIdSuffix", () => {
    render(<PrimaryButton testIdSuffix="submit">Go</PrimaryButton>);
    expect(
      screen.getByTestId("ext-ui-primary-button-submit"),
    ).not.toBeNull();
  });
});

describe("SecondaryButton", () => {
  it("renders an outline-style button", () => {
    render(<SecondaryButton>Back</SecondaryButton>);
    expect(
      screen.getByTestId("ext-ui-secondary-button"),
    ).not.toBeNull();
  });
});

describe("FilePicker", () => {
  it("fires onPick when a file is selected", () => {
    const cb = vi.fn();
    render(<FilePicker label="Pick" onPick={cb} />);
    const input = screen.getByTestId(
      "ext-ui-file-picker-input",
    ) as HTMLInputElement;
    const file = new File(["hello"], "test.yaml", {
      type: "application/x-yaml",
    });
    Object.defineProperty(input, "files", {
      value: [file],
      configurable: true,
    });
    fireEvent.change(input);
    expect(cb).toHaveBeenCalledOnce();
    expect(cb.mock.calls[0]?.[0]?.name).toBe("test.yaml");
  });
});

describe("ProgressBar", () => {
  it("renders aria-valuenow when determinate", () => {
    render(<ProgressBar percent={42} />);
    const role = screen.getByRole("progressbar");
    expect(role.getAttribute("aria-valuenow")).toBe("42");
  });

  it("does not crash in indeterminate mode", () => {
    render(<ProgressBar percent={null} label="Loading..." />);
    expect(screen.getByTestId("ext-ui-progress-bar")).not.toBeNull();
  });
});

describe("ResultBanner", () => {
  it("renders a success banner with the right testid", () => {
    render(<ResultBanner kind="success" title="Calibration applied" />);
    expect(
      screen.getByTestId("ext-ui-result-banner-success"),
    ).not.toBeNull();
  });

  it("includes the body when provided", () => {
    render(
      <ResultBanner
        kind="error"
        title="Upload failed"
        body="invalid YAML schema"
      />,
    );
    expect(screen.getByText(/invalid YAML schema/)).not.toBeNull();
  });
});
