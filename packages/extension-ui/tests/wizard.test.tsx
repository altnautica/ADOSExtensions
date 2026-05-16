import { describe, expect, it } from "vitest";
import { render, screen, fireEvent, act } from "@testing-library/react";

import {
  StepIndicator,
  Wizard,
  WizardStep,
  useWizardState,
} from "../src";

describe("StepIndicator", () => {
  it("renders one chip per step", () => {
    render(<StepIndicator current={1} total={4} />);
    expect(
      screen.getByTestId("ext-ui-step-indicator-0"),
    ).not.toBeNull();
    expect(
      screen.getByTestId("ext-ui-step-indicator-3"),
    ).not.toBeNull();
  });

  it("marks the current step with aria-current=step", () => {
    render(<StepIndicator current={2} total={4} />);
    const chip = screen.getByTestId("ext-ui-step-indicator-2");
    expect(chip.getAttribute("aria-current")).toBe("step");
  });

  it("disables chips when onJump is not provided", () => {
    render(<StepIndicator current={1} total={3} />);
    const chip = screen.getByTestId(
      "ext-ui-step-indicator-0",
    ) as HTMLButtonElement;
    expect(chip.disabled).toBe(true);
  });
});

describe("Wizard", () => {
  it("renders dismiss button only when onDismiss is provided", () => {
    const { rerender } = render(
      <Wizard currentStep={0} totalSteps={3}>
        <WizardStep title="Step 1" />
      </Wizard>,
    );
    expect(
      screen.queryByTestId("ext-ui-wizard-dismiss"),
    ).toBeNull();

    rerender(
      <Wizard currentStep={0} totalSteps={3} onDismiss={() => {}}>
        <WizardStep title="Step 1" />
      </Wizard>,
    );
    expect(screen.getByTestId("ext-ui-wizard-dismiss")).not.toBeNull();
  });

  it("wraps content in an overlay when layout=modal", () => {
    render(
      <Wizard currentStep={0} totalSteps={2} layout="modal">
        <WizardStep title="Step 1" />
      </Wizard>,
    );
    expect(screen.getByTestId("ext-ui-wizard-overlay")).not.toBeNull();
  });
});

describe("useWizardState", () => {
  function Harness({
    autoOpen,
    replay,
  }: {
    autoOpen?: boolean;
    replay?: boolean;
  }) {
    const state = useWizardState({
      totalSteps: 3,
      autoOpenOnFirstRun: autoOpen,
      replay,
    });
    return (
      <div>
        <span data-testid="open">{state.open ? "yes" : "no"}</span>
        <span data-testid="step">{state.step}</span>
        <button onClick={state.next} data-testid="next">
          next
        </button>
        <button onClick={state.back} data-testid="back">
          back
        </button>
        <button onClick={() => state.dismiss(false)} data-testid="dismiss">
          dismiss
        </button>
        <button onClick={state.open_} data-testid="open-btn">
          open
        </button>
      </div>
    );
  }

  it("starts closed and at step 0 by default", () => {
    render(<Harness />);
    expect(screen.getByTestId("open").textContent).toBe("no");
    expect(screen.getByTestId("step").textContent).toBe("0");
  });

  it("auto-opens when autoOpen is true and no persistence exists", () => {
    render(<Harness autoOpen />);
    expect(screen.getByTestId("open").textContent).toBe("yes");
  });

  it("replay forces open regardless of state", () => {
    render(<Harness replay />);
    expect(screen.getByTestId("open").textContent).toBe("yes");
  });

  it("next advances + clamps at totalSteps - 1", () => {
    render(<Harness autoOpen />);
    act(() => {
      fireEvent.click(screen.getByTestId("next"));
    });
    expect(screen.getByTestId("step").textContent).toBe("1");
    act(() => {
      fireEvent.click(screen.getByTestId("next"));
      fireEvent.click(screen.getByTestId("next"));
      fireEvent.click(screen.getByTestId("next"));
    });
    expect(screen.getByTestId("step").textContent).toBe("2");
  });

  it("back retreats + clamps at 0", () => {
    render(<Harness autoOpen />);
    act(() => {
      fireEvent.click(screen.getByTestId("next"));
      fireEvent.click(screen.getByTestId("back"));
      fireEvent.click(screen.getByTestId("back"));
    });
    expect(screen.getByTestId("step").textContent).toBe("0");
  });

  it("dismiss closes the wizard and resets the step", () => {
    render(<Harness autoOpen />);
    act(() => {
      fireEvent.click(screen.getByTestId("next"));
      fireEvent.click(screen.getByTestId("dismiss"));
    });
    expect(screen.getByTestId("open").textContent).toBe("no");
    expect(screen.getByTestId("step").textContent).toBe("0");
  });
});
