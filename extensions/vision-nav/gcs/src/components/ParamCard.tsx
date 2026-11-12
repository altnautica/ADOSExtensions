import type { CSSProperties, ReactNode } from "react";

import { tr, type TFn } from "../i18n";

export interface ParamGroup {
  /** Locale key under `<prefix>.groups`. */
  id: string;
  /** Parameter ids; each one's description is `<prefix>.rows.<id>`. */
  rows: string[];
}

interface Props {
  t: TFn;
  /** Locale prefix holding `title`, `note`, `groups.*` and `rows.*`. */
  prefix: string;
  groups: ParamGroup[];
  testId: string;
  children?: ReactNode;
}

/** A read-only summary of the flight-controller parameters one firmware
 * needs for vision navigation, grouped, with each one's purpose. */
export function ParamCard({ t, prefix, groups, testId, children }: Props): JSX.Element {
  return (
    <section style={card} data-testid={testId}>
      <h3 style={heading}>{tr(t, `${prefix}.title`)}</h3>
      <p style={note}>{tr(t, `${prefix}.note`)}</p>
      {groups.map((group) => (
        <div key={group.id}>
          <p style={subheading}>{tr(t, `${prefix}.groups.${group.id}`)}</p>
          <dl style={table}>
            {group.rows.map((id) => (
              <div key={id} style={{ display: "contents" }}>
                <dt style={paramName}>{id}</dt>
                <dd style={paramDesc}>{tr(t, `${prefix}.rows.${id}`)}</dd>
              </div>
            ))}
          </dl>
        </div>
      ))}
      {children}
    </section>
  );
}

export const card: CSSProperties = {
  background: "var(--vn-surface, rgba(255,255,255,0.02))",
  border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
  borderRadius: "0.5rem",
  padding: "0.875rem 1rem",
  color: "var(--vn-text, #ededed)",
  display: "flex",
  flexDirection: "column",
  gap: "0.75rem",
};
const heading: CSSProperties = {
  fontSize: "0.8125rem",
  fontWeight: 600,
  margin: 0,
  color: "var(--vn-text-2, #b4b4b4)",
};
export const subheading: CSSProperties = {
  fontSize: "0.75rem",
  fontWeight: 600,
  textTransform: "uppercase",
  letterSpacing: "0.05em",
  color: "var(--vn-text-3, #94a3b8)",
  margin: "0 0 0.375rem 0",
};
const note: CSSProperties = {
  fontSize: "0.75rem",
  color: "var(--vn-text-3, #94a3b8)",
  margin: 0,
};
const table: CSSProperties = {
  display: "grid",
  gridTemplateColumns: "max-content 1fr",
  columnGap: "0.875rem",
  rowGap: "0.25rem",
  fontSize: "0.8125rem",
};
const paramName: CSSProperties = {
  fontFamily: "var(--vn-mono, ui-monospace, SFMono-Regular, monospace)",
  color: "var(--vn-text, #ededed)",
};
const paramDesc: CSSProperties = {
  color: "var(--vn-text-3, #94a3b8)",
  margin: 0,
};
