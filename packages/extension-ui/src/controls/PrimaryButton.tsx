import type { ButtonHTMLAttributes, CSSProperties } from "react";

import { TOKENS } from "../theme/tokens";

type Props = ButtonHTMLAttributes<HTMLButtonElement> & {
  /** Optional testid suffix. Renders as ``ext-ui-primary-button-<suffix>``. */
  testIdSuffix?: string;
};

/**
 * Branded call-to-action button. Reads the accent token; disabled
 * state dims the button and stops the click. Tab-stop accessible by
 * default via the underlying <button>.
 */
export function PrimaryButton({
  testIdSuffix,
  style,
  disabled,
  children,
  ...rest
}: Props): JSX.Element {
  const tid = testIdSuffix
    ? `ext-ui-primary-button-${testIdSuffix}`
    : "ext-ui-primary-button";
  return (
    <button
      type="button"
      data-testid={tid}
      disabled={disabled}
      style={{ ...base(disabled === true), ...(style ?? {}) }}
      {...rest}
    >
      {children}
    </button>
  );
}

const base = (disabled: boolean): CSSProperties => ({
  padding: "0.5rem 1rem",
  background: disabled ? TOKENS.surface2 : TOKENS.accent,
  color: disabled ? TOKENS.textMuted : "white",
  border: "none",
  borderRadius: "0.375rem",
  fontWeight: 600,
  cursor: disabled ? "not-allowed" : "pointer",
  fontSize: "0.8125rem",
  opacity: disabled ? 0.7 : 1,
});
