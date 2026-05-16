import type { CSSProperties, ReactNode } from "react";

import { TOKENS } from "../theme/tokens";

import { StepIndicator } from "./StepIndicator";

interface Props {
  /** Step content to render. The consumer typically renders the
   * matching WizardStep for ``currentStep``. */
  children: ReactNode;
  /** Zero-based current step index. */
  currentStep: number;
  /** Total step count. */
  totalSteps: number;
  /** Optional label beside the indicator, e.g. "Step 3 of 7". */
  indicatorLabel?: string;
  /** Click handler for the indicator chips. Omit to make read-only. */
  onJump?: (step: number) => void;
  /** Optional dismiss handler. When provided, renders a close button. */
  onDismiss?: () => void;
  /** Optional dismiss button label (defaults to "Close"). */
  dismissLabel?: string;
  /** Modal-style overlay vs inline. Default: inline. */
  layout?: "inline" | "modal";
}

/**
 * Multi-step wizard shell. Layout + step indicator only; the
 * consumer renders the actual step content as ``children`` based on
 * ``currentStep``.
 *
 * Two layouts: ``inline`` renders the wizard as a card in the
 * current page flow; ``modal`` renders as a centered overlay with a
 * dim backdrop. Inline is the default and matches the tour
 * orchestrator's pattern; modal fits a heavy capture wizard.
 */
export function Wizard({
  children,
  currentStep,
  totalSteps,
  indicatorLabel,
  onJump,
  onDismiss,
  dismissLabel = "Close",
  layout = "inline",
}: Props): JSX.Element {
  const content = (
    <div style={wrapper} data-testid="ext-ui-wizard">
      <header style={header}>
        <StepIndicator
          current={currentStep}
          total={totalSteps}
          label={indicatorLabel}
          onJump={onJump}
        />
        {onDismiss !== undefined ? (
          <button
            type="button"
            style={dismissBtn}
            onClick={onDismiss}
            data-testid="ext-ui-wizard-dismiss"
          >
            {dismissLabel}
          </button>
        ) : null}
      </header>
      <div style={body}>{children}</div>
    </div>
  );

  if (layout === "modal") {
    return (
      <div style={overlay} data-testid="ext-ui-wizard-overlay">
        {content}
      </div>
    );
  }
  return content;
}

const wrapper: CSSProperties = {
  display: "flex",
  flexDirection: "column",
  gap: "0.75rem",
  width: "100%",
};
const header: CSSProperties = {
  display: "flex",
  alignItems: "center",
  justifyContent: "space-between",
  gap: "0.75rem",
};
const dismissBtn: CSSProperties = {
  background: "transparent",
  color: TOKENS.textMuted,
  border: `1px solid ${TOKENS.border}`,
  borderRadius: "0.25rem",
  padding: "0.25rem 0.625rem",
  cursor: "pointer",
  fontSize: "0.7rem",
};
const body: CSSProperties = {
  display: "flex",
  flexDirection: "column",
  gap: "0.625rem",
};
const overlay: CSSProperties = {
  position: "fixed",
  top: 0,
  left: 0,
  right: 0,
  bottom: 0,
  background: "rgba(0, 0, 0, 0.5)",
  display: "flex",
  alignItems: "center",
  justifyContent: "center",
  padding: "1.5rem",
  zIndex: 50,
};
