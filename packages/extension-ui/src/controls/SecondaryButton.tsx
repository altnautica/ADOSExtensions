import type { ButtonHTMLAttributes, CSSProperties } from "react";

import { TOKENS } from "../theme/tokens";

type Props = ButtonHTMLAttributes<HTMLButtonElement> & {
  testIdSuffix?: string;
};

/**
 * Outlined secondary button. Same shape as :class:`PrimaryButton`
 * but no accent fill. Used for Back, Cancel, Skip, etc.
 */
export function SecondaryButton({
  testIdSuffix,
  style,
  disabled,
  children,
  ...rest
}: Props): JSX.Element {
  const tid = testIdSuffix
    ? `ext-ui-secondary-button-${testIdSuffix}`
    : "ext-ui-secondary-button";
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
  background: "transparent",
  color: disabled ? TOKENS.textMuted : TOKENS.text,
  border: `1px solid ${TOKENS.border}`,
  borderRadius: "0.375rem",
  fontWeight: 600,
  cursor: disabled ? "not-allowed" : "pointer",
  fontSize: "0.8125rem",
  opacity: disabled ? 0.7 : 1,
});
