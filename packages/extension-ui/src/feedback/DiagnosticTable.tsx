import type { CSSProperties } from "react";

import { TOKENS } from "../theme/tokens";

export interface DiagnosticRow {
  label: string;
  value: string;
  /** Optional second value rendered alongside the first for diffs
   * (e.g. before/after calibration). */
  compare?: string;
  /** Optional tone applied to the value cell. */
  tone?: "ok" | "warn" | "error" | "muted";
}

interface Props {
  rows: DiagnosticRow[];
  /** Column header for the value column (default: "Value"). */
  valueLabel?: string;
  /** Column header for the compare column when any row uses compare. */
  compareLabel?: string;
}

/**
 * Compact key-value table for diagnostic readouts. Optional second
 * column for compare-style displays (current vs previous, observed
 * vs expected). Tone driver paints the value cell with the matching
 * status colour.
 */
export function DiagnosticTable({
  rows,
  valueLabel = "Value",
  compareLabel,
}: Props): JSX.Element {
  const hasCompare = rows.some((r) => r.compare !== undefined);
  return (
    <table style={table} data-testid="ext-ui-diagnostic-table">
      <thead>
        <tr style={headerRow}>
          <th style={cellHeader}>Field</th>
          <th style={cellHeader}>{valueLabel}</th>
          {hasCompare ? (
            <th style={cellHeader}>{compareLabel ?? "Previous"}</th>
          ) : null}
        </tr>
      </thead>
      <tbody>
        {rows.map((row, idx) => (
          <tr key={idx} style={bodyRow}>
            <td style={cellLabel}>{row.label}</td>
            <td style={{ ...cellValue, color: toneColor(row.tone) }}>
              {row.value}
            </td>
            {hasCompare ? (
              <td style={cellCompare}>{row.compare ?? "—"}</td>
            ) : null}
          </tr>
        ))}
      </tbody>
    </table>
  );
}

function toneColor(tone?: DiagnosticRow["tone"]): string {
  if (tone === "ok") return TOKENS.ok;
  if (tone === "warn") return TOKENS.warn;
  if (tone === "error") return TOKENS.error;
  if (tone === "muted") return TOKENS.textMuted;
  return TOKENS.text;
}

const table: CSSProperties = {
  width: "100%",
  borderCollapse: "collapse",
  fontSize: "0.8125rem",
  color: TOKENS.text,
};
const headerRow: CSSProperties = {
  borderBottom: `1px solid ${TOKENS.border}`,
};
const bodyRow: CSSProperties = {
  borderBottom: `1px solid ${TOKENS.border}`,
};
const cellHeader: CSSProperties = {
  textAlign: "left",
  padding: "0.375rem 0.5rem",
  fontSize: "0.7rem",
  fontWeight: 600,
  color: TOKENS.textMuted,
};
const cellLabel: CSSProperties = {
  padding: "0.375rem 0.5rem",
  color: TOKENS.textMuted,
};
const cellValue: CSSProperties = {
  padding: "0.375rem 0.5rem",
  fontWeight: 600,
  fontVariantNumeric: "tabular-nums",
};
const cellCompare: CSSProperties = {
  padding: "0.375rem 0.5rem",
  color: TOKENS.textMuted,
  fontVariantNumeric: "tabular-nums",
};
