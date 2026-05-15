import type { CSSProperties } from "react";

import type { VisionNavTelemetry } from "../types";

interface Props {
  state: VisionNavTelemetry["companionState"];
}

interface Tone {
  dot: string;
  label: string;
}

const TONES: Record<NonNullable<VisionNavTelemetry["companionState"]>, Tone> = {
  active: { dot: "var(--vn-ok, #34d399)", label: "Active" },
  critical: { dot: "var(--vn-error, #ef4444)", label: "Critical" },
  terminating: { dot: "var(--vn-error, #ef4444)", label: "Terminating" },
  absent: { dot: "var(--vn-muted, #6b7280)", label: "Offline" },
};

export function CompanionStatusPill({ state }: Props): JSX.Element {
  const tone = state ? TONES[state] : TONES.absent;
  const pillStyle: CSSProperties = {
    display: "inline-flex",
    alignItems: "center",
    gap: "0.5rem",
    padding: "0.25rem 0.75rem",
    borderRadius: "9999px",
    background: "var(--vn-surface-2, rgba(255,255,255,0.04))",
    color: "var(--vn-text, #e5e7eb)",
    border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
    fontSize: "0.8125rem",
    fontWeight: 500,
  };
  const dotStyle: CSSProperties = {
    width: "0.5rem",
    height: "0.5rem",
    borderRadius: "9999px",
    background: tone.dot,
  };
  return (
    <span
      style={pillStyle}
      data-testid="vn-companion-pill"
      data-state={state ?? "absent"}
    >
      <span style={dotStyle} aria-hidden="true" />
      <span>Companion: {tone.label}</span>
    </span>
  );
}
