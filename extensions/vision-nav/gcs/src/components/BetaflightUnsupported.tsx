import type { CSSProperties } from "react";

const BULLET_LINES: string[] = [
  "Betaflight is a racing and freestyle flight stack. It does not ship a position estimator.",
  "Optical flow needs a consumer inside the FC's EKF to fuse flow samples into. Betaflight has neither an EKF nor a position controller.",
  "This is a deliberate firmware-upstream choice. The plugin cannot work around it from the companion side.",
  "Operators on Betaflight hardware who need GPS-denied flight should cross-flash iNav or ArduPilot Copter on the same FC. Most STM32F405, F722, and H743 boards run all three firmwares.",
];

export function BetaflightUnsupported(): JSX.Element {
  const card: CSSProperties = {
    background: "var(--vn-surface, rgba(255,255,255,0.02))",
    border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
    borderRadius: "0.5rem",
    padding: "0.875rem 1rem",
    color: "var(--vn-text-2, #cbd5e1)",
    display: "flex",
    flexDirection: "column",
    gap: "0.5rem",
  };
  const heading: CSSProperties = {
    fontSize: "0.8125rem",
    fontWeight: 600,
    margin: 0,
    color: "var(--vn-text, #e5e7eb)",
  };
  const list: CSSProperties = {
    fontSize: "0.8125rem",
    color: "var(--vn-text-3, #94a3b8)",
    margin: 0,
    paddingLeft: "1.25rem",
  };
  return (
    <section style={card} data-testid="vn-betaflight-unsupported">
      <h3 style={heading}>Betaflight is not supported</h3>
      <ul style={list}>
        {BULLET_LINES.map((line) => (
          <li key={line}>{line}</li>
        ))}
      </ul>
    </section>
  );
}
