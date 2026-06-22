/**
 * Shared inline styling helpers for the Follow-Me iframe surfaces.
 *
 * The sandboxed iframe has no access to the host's Tailwind classes, so
 * colors and small style fragments are defined here as plain values that
 * fall back to dark-theme defaults and pick up host theme vars when the
 * host streams them in.
 *
 * @license GPL-3.0-or-later
 */

import type { CSSProperties } from "react";

import type { LockState } from "./types";

/** Border/text color for a lock state: green locked, amber uncertain, red
 * lost. Untracked detections fall back to a neutral tertiary tone. */
export function lockColor(lockState: LockState | null): string {
  switch (lockState) {
    case "locked":
      return "var(--fm-success, #22c55e)";
    case "uncertain":
      return "var(--fm-warning, #f59e0b)";
    case "lost":
      return "var(--fm-error, #ef4444)";
    default:
      return "var(--fm-tertiary, #94a3b8)";
  }
}

export const card: CSSProperties = {
  background: "var(--fm-surface, rgba(255,255,255,0.03))",
  border: "1px solid var(--fm-border, rgba(255,255,255,0.08))",
  borderRadius: "0.5rem",
  padding: "0.75rem 0.875rem",
};

export const sectionTitle: CSSProperties = {
  margin: "0 0 0.5rem 0",
  fontSize: "0.75rem",
  fontWeight: 600,
  textTransform: "uppercase",
  letterSpacing: "0.04em",
  color: "var(--fm-text-muted, #94a3b8)",
};

export const labelRow: CSSProperties = {
  display: "flex",
  justifyContent: "space-between",
  alignItems: "center",
  gap: "0.75rem",
  fontSize: "0.8125rem",
  padding: "0.25rem 0",
};

export const inputStyle: CSSProperties = {
  background: "var(--fm-input-bg, #0b1220)",
  border: "1px solid var(--fm-border, rgba(255,255,255,0.12))",
  borderRadius: "0.375rem",
  color: "var(--fm-text, #e5e7eb)",
  fontSize: "0.8125rem",
  padding: "0.3rem 0.5rem",
  width: "6.5rem",
  boxSizing: "border-box",
};
