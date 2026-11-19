/** Join the truthy class names. */
export function cn(...parts: Array<string | false | null | undefined>): string {
  return parts.filter(Boolean).join(" ");
}

/** The glyph shown for a value the node did not report. */
export const NO_DATA_GLYPH = "—";
