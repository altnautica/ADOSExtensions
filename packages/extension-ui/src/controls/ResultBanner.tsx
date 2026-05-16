import type { CSSProperties, ReactNode } from "react";

import { TOKENS } from "../theme/tokens";

interface Props {
  kind: "success" | "warning" | "error" | "info";
  title: string;
  body?: ReactNode;
  /** Optional testid suffix. */
  testIdSuffix?: string;
}

/**
 * Status banner with an icon + title + optional body. Used to
 * summarise the outcome of a wizard step or a long-running
 * operation (calibration apply / retry, upload success / failure).
 */
export function ResultBanner({
  kind,
  title,
  body,
  testIdSuffix,
}: Props): JSX.Element {
  const tid = testIdSuffix
    ? `ext-ui-result-banner-${testIdSuffix}`
    : `ext-ui-result-banner-${kind}`;
  const tone = TONES[kind];
  return (
    <section style={banner(tone.color)} role="alert" data-testid={tid}>
      <div style={iconBox(tone.color)}>{tone.icon}</div>
      <div style={text}>
        <div style={titleStyle}>{title}</div>
        {body !== undefined ? <div style={bodyStyle}>{body}</div> : null}
      </div>
    </section>
  );
}

const TONES: Record<
  "success" | "warning" | "error" | "info",
  { color: string; icon: string }
> = {
  success: { color: TOKENS.ok, icon: "✓" },
  warning: { color: TOKENS.warn, icon: "!" },
  error: { color: TOKENS.error, icon: "✕" },
  info: { color: TOKENS.accent, icon: "i" },
};

const banner = (color: string): CSSProperties => ({
  display: "flex",
  alignItems: "flex-start",
  gap: "0.625rem",
  padding: "0.625rem 0.875rem",
  borderRadius: "0.5rem",
  border: `1px solid ${color}`,
  background: `color-mix(in srgb, ${color} 12%, transparent)`,
  color: TOKENS.text,
});
const iconBox = (color: string): CSSProperties => ({
  width: "1.5rem",
  height: "1.5rem",
  borderRadius: "50%",
  background: color,
  color: "white",
  display: "flex",
  alignItems: "center",
  justifyContent: "center",
  fontWeight: 700,
  flexShrink: 0,
});
const text: CSSProperties = {
  display: "flex",
  flexDirection: "column",
  gap: "0.2rem",
};
const titleStyle: CSSProperties = {
  fontSize: "0.8125rem",
  fontWeight: 600,
};
const bodyStyle: CSSProperties = {
  fontSize: "0.75rem",
  color: TOKENS.textMuted,
  lineHeight: 1.45,
};
