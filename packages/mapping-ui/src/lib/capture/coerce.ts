/**
 * @module capture/coerce
 * @description Defensive readers for the capture and session replies. A field
 * the agent did not send reads as null (unknown), never as a fabricated zero.
 */

export function obj(v: unknown): Record<string, unknown> {
  return v && typeof v === "object" && !Array.isArray(v) ? (v as Record<string, unknown>) : {};
}

export function objOrNull(v: unknown): Record<string, unknown> | null {
  return v && typeof v === "object" && !Array.isArray(v) ? (v as Record<string, unknown>) : null;
}

export function arr(v: unknown): unknown[] {
  return Array.isArray(v) ? v : [];
}

export function num(v: unknown): number | null {
  return typeof v === "number" && Number.isFinite(v) ? v : null;
}

export function str(v: unknown): string | null {
  return typeof v === "string" && v.length > 0 ? v : null;
}

export function bool(v: unknown): boolean | null {
  return typeof v === "boolean" ? v : null;
}

/** A numeric `[x, y]` / `[x, y, z]` tuple of exactly `n` finite numbers. */
export function vec(v: unknown, n: 2): [number, number] | null;
export function vec(v: unknown, n: 3): [number, number, number] | null;
export function vec(v: unknown, n: number): number[] | null {
  if (!Array.isArray(v) || v.length !== n) return null;
  return v.every((x) => typeof x === "number" && Number.isFinite(x)) ? (v as number[]) : null;
}
