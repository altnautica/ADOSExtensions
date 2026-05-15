import type { CSSProperties } from "react";

import type { VisionNavTelemetry } from "../types";

interface Props {
  telemetry: VisionNavTelemetry;
}

interface Check {
  ok: boolean;
  label: string;
  hint?: string;
}

export function PreArmStatus({ telemetry }: Props): JSX.Element {
  const checks: Check[] = [
    {
      ok: telemetry.companionState === "active",
      label: "Companion process active",
      hint:
        telemetry.companionState === "active"
          ? undefined
          : "Vision companion is not running. Check agent logs.",
    },
    {
      ok: (telemetry.flowQuality ?? 0) > 50,
      label: "Optical flow streaming",
      hint:
        (telemetry.flowQuality ?? 0) > 50
          ? undefined
          : "Flow quality is below the pre-arm gate (50/255).",
    },
    {
      ok: false,
      label: "EKF source set is vision",
      hint: "Configure manually in parameter editor.",
    },
  ];

  const card: CSSProperties = {
    background: "var(--vn-surface, rgba(255,255,255,0.02))",
    border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
    borderRadius: "0.5rem",
    padding: "0.875rem 1rem",
    color: "var(--vn-text, #e5e7eb)",
  };
  const list: CSSProperties = {
    listStyle: "none",
    margin: 0,
    padding: 0,
    display: "flex",
    flexDirection: "column",
    gap: "0.375rem",
  };
  const item: CSSProperties = {
    display: "flex",
    flexDirection: "column",
    gap: "0.125rem",
    fontSize: "0.8125rem",
  };
  const row: CSSProperties = {
    display: "flex",
    gap: "0.5rem",
    alignItems: "center",
  };
  const hint: CSSProperties = {
    color: "var(--vn-text-3, #94a3b8)",
    fontSize: "0.75rem",
    marginLeft: "1.25rem",
  };
  const heading: CSSProperties = {
    fontSize: "0.8125rem",
    fontWeight: 600,
    margin: "0 0 0.5rem 0",
    color: "var(--vn-text-2, #cbd5e1)",
  };

  return (
    <section style={card} data-testid="vn-pre-arm">
      <h3 style={heading}>Pre-arm</h3>
      <ul style={list}>
        {checks.map((check, idx) => {
          const indicator: CSSProperties = {
            display: "inline-flex",
            width: "0.875rem",
            height: "0.875rem",
            borderRadius: "9999px",
            background: check.ok
              ? "var(--vn-ok, #34d399)"
              : "var(--vn-error, #ef4444)",
            color: "#0b1220",
            alignItems: "center",
            justifyContent: "center",
            fontSize: "0.625rem",
            fontWeight: 700,
            flexShrink: 0,
          };
          return (
            <li key={idx} style={item} data-state={check.ok ? "ok" : "fail"}>
              <span style={row}>
                <span style={indicator} aria-hidden="true">
                  {check.ok ? "✓" : "✕"}
                </span>
                <span>{check.label}</span>
              </span>
              {check.hint ? <span style={hint}>{check.hint}</span> : null}
            </li>
          );
        })}
      </ul>
    </section>
  );
}
