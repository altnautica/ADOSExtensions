import { useState, type CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import type { EkfSourceOption, EkfSourceSet, FirmwareType } from "../types";

interface Props {
  firmware: FirmwareType;
  ctx: PluginContext;
}

const OPTIONS: EkfSourceOption[] = [
  { set: 1, label: "GPS", description: "Default. GPS + baro + compass." },
  { set: 2, label: "VIO", description: "Visual-inertial odometry primary." },
  { set: 3, label: "OF", description: "Optical flow primary." },
];

export function EkfSourceSwitcher({ firmware, ctx }: Props): JSX.Element {
  const [pending, setPending] = useState<EkfSourceSet | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [lastSwitched, setLastSwitched] = useState<EkfSourceSet | null>(null);
  const disabled = firmware !== "ardupilot";

  async function confirm(): Promise<void> {
    if (pending === null) return;
    setError(null);
    try {
      await ctx.command.send("MAV_CMD_SET_EKF_SOURCE_SET", { param1: pending });
      setLastSwitched(pending);
      setPending(null);
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    }
  }

  const card: CSSProperties = {
    background: "var(--vn-surface, rgba(255,255,255,0.02))",
    border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
    borderRadius: "0.5rem",
    padding: "0.875rem 1rem",
    color: "var(--vn-text, #e5e7eb)",
    display: "flex",
    flexDirection: "column",
    gap: "0.625rem",
  };
  const heading: CSSProperties = {
    fontSize: "0.8125rem",
    fontWeight: 600,
    margin: 0,
    color: "var(--vn-text-2, #cbd5e1)",
  };
  const row: CSSProperties = {
    display: "flex",
    gap: "0.5rem",
    flexWrap: "wrap",
  };
  const button = (active: boolean): CSSProperties => ({
    padding: "0.5rem 0.875rem",
    background: active
      ? "var(--vn-accent, #2563eb)"
      : "var(--vn-surface-2, rgba(255,255,255,0.06))",
    color: "var(--vn-text, #e5e7eb)",
    border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
    borderRadius: "0.375rem",
    fontSize: "0.8125rem",
    cursor: disabled ? "not-allowed" : "pointer",
    opacity: disabled ? 0.5 : 1,
    fontWeight: 500,
  });
  const tooltip: CSSProperties = {
    fontSize: "0.75rem",
    color: "var(--vn-text-3, #94a3b8)",
  };
  const dialog: CSSProperties = {
    background: "var(--vn-surface-2, rgba(255,255,255,0.06))",
    border: "1px solid var(--vn-warn, #f59e0b)",
    borderRadius: "0.375rem",
    padding: "0.75rem",
    display: "flex",
    flexDirection: "column",
    gap: "0.5rem",
    fontSize: "0.8125rem",
  };
  const errStyle: CSSProperties = {
    color: "var(--vn-error, #ef4444)",
    fontSize: "0.75rem",
  };

  return (
    <section style={card} data-testid="vn-ekf-switcher">
      <h3 style={heading}>EKF source set</h3>
      <div style={row}>
        {OPTIONS.map((opt) => (
          <button
            key={opt.set}
            type="button"
            style={button(lastSwitched === opt.set)}
            disabled={disabled}
            onClick={() => setPending(opt.set)}
            data-testid={`vn-ekf-button-${opt.label.toLowerCase()}`}
            aria-disabled={disabled}
            title={opt.description}
          >
            {opt.label}
          </button>
        ))}
      </div>
      {disabled ? (
        <p style={tooltip} data-testid="vn-ekf-px4-note">
          PX4 does not support runtime source-set switching. Update parameters
          and restart the EKF.
        </p>
      ) : null}
      {pending !== null ? (
        <div role="dialog" aria-label="Confirm EKF source switch" style={dialog}>
          <p style={{ margin: 0 }}>
            Switching the EKF source set in flight can cause a one-time
            innovation spike. Confirm only if the vision navigation is healthy.
          </p>
          <div style={{ display: "flex", gap: "0.5rem" }}>
            <button
              type="button"
              style={{ ...button(true), background: "var(--vn-accent, #2563eb)" }}
              onClick={confirm}
              data-testid="vn-ekf-confirm"
            >
              Confirm switch
            </button>
            <button
              type="button"
              style={button(false)}
              onClick={() => setPending(null)}
              data-testid="vn-ekf-cancel"
            >
              Cancel
            </button>
          </div>
          {error ? <span style={errStyle}>{error}</span> : null}
        </div>
      ) : null}
    </section>
  );
}
