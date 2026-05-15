import { useState, type CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import type {
  EkfSourceOption,
  EkfSourceSet,
  FirmwareType,
  VisionNavTelemetry,
} from "../types";

interface Props {
  firmware: FirmwareType;
  ctx: PluginContext;
  /**
   * Companion process state from the agent heartbeat. The VIO and OF
   * options are only safe to engage when the companion is "active";
   * any other state (or undefined) keeps them disabled so a runtime
   * source switch does not feed the EKF stale or absent data.
   */
  companionState?: VisionNavTelemetry["companionState"];
  /**
   * Optical-flow quality from the agent heartbeat (0..255). Mirrors the
   * pre-arm gate at 50: below that, the EKF would receive frames it
   * would have to reject, triggering an innovation spike on switch.
   */
  flowQuality?: number;
}

const OPTIONS: EkfSourceOption[] = [
  { set: 1, label: "GPS", description: "Default. GPS + baro + compass." },
  { set: 2, label: "VIO", description: "Visual-inertial odometry primary." },
  { set: 3, label: "OF", description: "Optical flow primary." },
];

const FLOW_QUALITY_GATE = 50;

export function EkfSourceSwitcher({
  firmware,
  ctx,
  companionState,
  flowQuality,
}: Props): JSX.Element {
  const [pending, setPending] = useState<EkfSourceSet | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [lastSwitched, setLastSwitched] = useState<EkfSourceSet | null>(null);
  const isArdu = firmware === "ardupilot";
  const px4Note = firmware === "px4";
  const visionHealthy =
    companionState === "active" && (flowQuality ?? 0) >= FLOW_QUALITY_GATE;
  // GPS is the revert path; always available on ArduPilot regardless of
  // vision health.
  const gpsAlwaysReady = isArdu;
  const vioReady = isArdu && visionHealthy;
  const ofReady = isArdu && visionHealthy;

  function readyFor(opt: EkfSourceOption): boolean {
    if (opt.set === 1) return gpsAlwaysReady;
    if (opt.set === 2) return vioReady;
    if (opt.set === 3) return ofReady;
    return false;
  }

  const visionUnhealthyTooltip = `Vision not healthy. Companion state: ${
    companionState ?? "unknown"
  }, flow quality: ${flowQuality ?? 0}/255.`;

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
  const button = (
    active: boolean,
    isDisabled: boolean = !isArdu,
  ): CSSProperties => ({
    padding: "0.5rem 0.875rem",
    background: active
      ? "var(--vn-accent, #2563eb)"
      : "var(--vn-surface-2, rgba(255,255,255,0.06))",
    color: "var(--vn-text, #e5e7eb)",
    border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
    borderRadius: "0.375rem",
    fontSize: "0.8125rem",
    cursor: isDisabled ? "not-allowed" : "pointer",
    opacity: isDisabled ? 0.5 : 1,
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
        {OPTIONS.map((opt) => {
          const ready = readyFor(opt);
          const optDisabled = !ready;
          // GPS is always available on ArduPilot; only VIO and OF carry
          // the vision-health gate. PX4 stays fully disabled and the
          // px4Note below explains why.
          const gatedByHealth = isArdu && opt.set !== 1 && !visionHealthy;
          const title = gatedByHealth
            ? visionUnhealthyTooltip
            : opt.description;
          return (
            <button
              key={opt.set}
              type="button"
              style={button(lastSwitched === opt.set, optDisabled)}
              disabled={optDisabled}
              onClick={() => setPending(opt.set)}
              data-testid={`vn-ekf-button-${opt.label.toLowerCase()}`}
              aria-disabled={optDisabled}
              title={title}
            >
              {opt.label}
            </button>
          );
        })}
      </div>
      {px4Note ? (
        <p style={tooltip} data-testid="vn-ekf-px4-note">
          PX4 does not support runtime source-set switching. Update parameters
          and restart the EKF.
        </p>
      ) : null}
      {isArdu && !visionHealthy ? (
        <p style={tooltip} data-testid="vn-ekf-vision-unhealthy-note">
          {visionUnhealthyTooltip}
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
