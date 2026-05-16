/**
 * Shared CSS variable contract for the extension-ui primitives.
 *
 * Every primitive reads from this set. Consumers can override any
 * variable at any wrapper level; primitives inherit through normal
 * CSS cascade.
 *
 * The defaults are tuned for the dark theme Mission Control runs in
 * by default. A future light-theme variant only needs to redefine
 * these variables at a different selector.
 */

export const TOKENS = {
  accent: "var(--ext-ui-accent, #2563eb)",
  surface: "var(--ext-ui-surface, rgba(255,255,255,0.02))",
  surface2: "var(--ext-ui-surface-2, rgba(255,255,255,0.06))",
  border: "var(--ext-ui-border, rgba(255,255,255,0.08))",
  text: "var(--ext-ui-text, #e5e7eb)",
  textMuted: "var(--ext-ui-text-muted, #94a3b8)",
  ok: "var(--ext-ui-ok, #34d399)",
  warn: "var(--ext-ui-warn, #f59e0b)",
  error: "var(--ext-ui-error, #ef4444)",
} as const;

export type ThemeToken = keyof typeof TOKENS;
