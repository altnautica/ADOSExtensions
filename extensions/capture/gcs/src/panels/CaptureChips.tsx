/**
 * The cockpit's Capture panel: the session's state, duration and frames, the
 * quality chips, and whether what is recorded so far is enough to
 * reconstruct (with the reason when it is not).
 */

import { ScanLine } from "lucide-react";

import { CalmState } from "@altnautica/mapping-ui/components/ui/CalmState";

import { HealthChips } from "../components/capture/HealthChips";
import { translator } from "../host/i18n";
import { useCapture } from "../hooks/use-capture";
import type { CaptureState, HealthSignal } from "../lib/capture/capture-state";

const t = translator("capture");

/** Whether the session so far can reconstruct: frames recorded and no
 * quality signal at `bad`. The first bad signal is the reason it cannot. */
export function landingVerdict(state: CaptureState): { enough: boolean; reason: HealthSignal | null } {
  const bad = state.health.find((s) => s.level === "bad") ?? null;
  const frames = state.frames.total ?? 0;
  return { enough: frames > 0 && bad === null, reason: bad };
}

function duration(s: number | null): string {
  if (s === null) return "–";
  const m = Math.floor(s / 60);
  return `${m}:${String(Math.floor(s % 60)).padStart(2, "0")}`;
}

export function CaptureChips() {
  const { state, loading, absent } = useCapture();
  if (loading) return null;
  if (absent || !state) return <CalmState icon={ScanLine} message={t("absent")} />;
  const verdict = landingVerdict(state);
  return (
    <div className="cap:flex cap:flex-col cap:gap-2 cap:p-2 cap:text-[11px]">
      <div className="cap:flex cap:items-center cap:justify-between cap:gap-2 cap:tabular-nums">
        <span className="cap:font-medium cap:text-text-primary">{t(`chips.state.${state.state}`)}</span>
        <span className="cap:text-text-secondary">{duration(state.durationS)}</span>
        <span className="cap:text-text-secondary">{t("chips.frames", { n: state.frames.total ?? 0 })}</span>
      </div>
      <HealthChips signals={state.health} />
      {state.state !== "idle" && (
        <p role="status" className={verdict.enough ? "cap:text-status-success" : "cap:text-status-warning"}>
          {verdict.enough
            ? t("chips.enough")
            : verdict.reason
              ? t("chips.notEnoughBecause", { signal: t(`health.${verdict.reason.id}`) })
              : t("chips.notEnoughYet")}
        </p>
      )}
    </div>
  );
}
