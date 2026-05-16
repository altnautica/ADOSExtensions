import type { CSSProperties } from "react";

import { TOKENS } from "../theme/tokens";

interface Props {
  current: number;
  total: number;
  /** Optional label shown beside the chips, e.g. "Step 3 of 7". */
  label?: string;
  /** Click handler when a step chip is tappable. Omit to make chips read-only. */
  onJump?: (step: number) => void;
}

/**
 * Compact progress chips for a wizard. Chip per step, filled up to
 * the current step. Optional tap-to-jump for non-destructive
 * navigation (the consumer decides whether jumping is safe given the
 * wizard's state).
 */
export function StepIndicator({
  current,
  total,
  label,
  onJump,
}: Props): JSX.Element {
  return (
    <div style={container} data-testid="ext-ui-step-indicator">
      {label !== undefined ? <span style={labelStyle}>{label}</span> : null}
      <div style={row} role="progressbar" aria-valuemin={1} aria-valuemax={total} aria-valuenow={current + 1}>
        {Array.from({ length: total }, (_, i) => {
          const filled = i <= current;
          const isCurrent = i === current;
          const clickable = onJump !== undefined;
          return (
            <button
              key={i}
              type="button"
              aria-label={`Step ${i + 1}`}
              aria-current={isCurrent ? "step" : undefined}
              data-testid={`ext-ui-step-indicator-${i}`}
              disabled={!clickable}
              onClick={() => {
                if (onJump) onJump(i);
              }}
              style={chip(filled, isCurrent, clickable)}
            />
          );
        })}
      </div>
    </div>
  );
}

const container: CSSProperties = {
  display: "flex",
  alignItems: "center",
  gap: "0.5rem",
};
const labelStyle: CSSProperties = {
  fontSize: "0.7rem",
  color: TOKENS.textMuted,
  fontWeight: 600,
};
const row: CSSProperties = {
  display: "flex",
  gap: "0.25rem",
};
const chip = (
  filled: boolean,
  current: boolean,
  clickable: boolean,
): CSSProperties => ({
  width: current ? "1rem" : "0.5rem",
  height: "0.375rem",
  borderRadius: "999px",
  background: filled ? TOKENS.accent : TOKENS.surface2,
  border: "none",
  cursor: clickable ? "pointer" : "default",
  transition: "width 120ms ease",
});
