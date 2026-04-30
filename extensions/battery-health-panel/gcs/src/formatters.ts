/**
 * Display formatters. Fixed digits, fixed unit suffix, no locale-aware
 * decimal separator (the rest of Mission Control reads numeric copy
 * in en-US format regardless of UI locale).
 */

export function formatVolts(v: number, digits = 2): string {
  if (!Number.isFinite(v)) return "—";
  return `${v.toFixed(digits)} V`;
}

export function formatMilliVolts(mv: number): string {
  if (!Number.isFinite(mv)) return "—";
  return `${Math.round(mv)} mV`;
}

export function formatAmps(a: number, digits = 1): string {
  if (!Number.isFinite(a)) return "—";
  return `${a.toFixed(digits)} A`;
}

export function formatCelsius(c: number | null, digits = 1): string {
  if (c === null || !Number.isFinite(c)) return "—";
  return `${c.toFixed(digits)} °C`;
}

export function formatPercent(p: number, digits = 0): string {
  if (!Number.isFinite(p) || p < 0) return "—";
  return `${p.toFixed(digits)}%`;
}

export function formatEta(seconds: number | null): string {
  if (seconds === null || !Number.isFinite(seconds) || seconds < 0) return "—";
  if (seconds < 60) return `${Math.round(seconds)}s`;
  const min = Math.floor(seconds / 60);
  const sec = Math.round(seconds % 60);
  if (min < 60) return `${min}m ${sec.toString().padStart(2, "0")}s`;
  const hr = Math.floor(min / 60);
  const minRest = min % 60;
  return `${hr}h ${minRest.toString().padStart(2, "0")}m`;
}
