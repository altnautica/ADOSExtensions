import type { CSSProperties, ReactNode } from "react";

import { TOKENS } from "../theme/tokens";

interface Props {
  title: string;
  description?: string;
  children?: ReactNode;
  /** Footer slot for the consumer's Next / Back / Skip / Finish buttons. */
  actions?: ReactNode;
}

/**
 * One step shell inside a Wizard. Renders a card with a title +
 * optional description + the consumer-provided body + an actions
 * footer. Layout-only; the consumer composes its own controls.
 */
export function WizardStep({
  title,
  description,
  children,
  actions,
}: Props): JSX.Element {
  return (
    <section style={card} data-testid="ext-ui-wizard-step">
      <header style={header}>
        <h3 style={titleStyle}>{title}</h3>
        {description !== undefined ? (
          <p style={descStyle}>{description}</p>
        ) : null}
      </header>
      {children !== undefined ? <div style={body}>{children}</div> : null}
      {actions !== undefined ? <div style={footer}>{actions}</div> : null}
    </section>
  );
}

const card: CSSProperties = {
  background: TOKENS.surface,
  border: `1px solid ${TOKENS.border}`,
  borderRadius: "0.5rem",
  padding: "1rem 1.125rem",
  color: TOKENS.text,
  display: "flex",
  flexDirection: "column",
  gap: "0.875rem",
};
const header: CSSProperties = {
  display: "flex",
  flexDirection: "column",
  gap: "0.25rem",
};
const titleStyle: CSSProperties = {
  fontSize: "0.9375rem",
  fontWeight: 600,
  margin: 0,
};
const descStyle: CSSProperties = {
  fontSize: "0.8125rem",
  color: TOKENS.textMuted,
  margin: 0,
  lineHeight: 1.45,
};
const body: CSSProperties = {
  display: "flex",
  flexDirection: "column",
  gap: "0.625rem",
};
const footer: CSSProperties = {
  display: "flex",
  justifyContent: "flex-end",
  gap: "0.5rem",
  marginTop: "0.25rem",
};
