/**
 * @module capture/health-chips
 * @description Maps the capture health signals onto the six chips the capture
 * page shows: status tone, label, the reading and, for a red chip, the operator
 * prompt. A chip the agent did not report reads as unknown (grey), never green.
 */

import type { StatusLevel } from "../../components/ui/StatusDot";
import { HEALTH_IDS, type HealthId, type HealthLevel, type HealthSignal } from "./capture-state";

export interface HealthChip {
  id: HealthId;
  /** null when the agent did not report this signal. */
  level: HealthLevel | null;
  tone: StatusLevel;
  /** i18n key under `capture.health` for the chip label. */
  labelKey: string;
  /** The reading as text, or null when there is none. */
  reading: string | null;
  /** i18n key under `capture.prompt` for a red chip's instruction, else null. */
  promptKey: string | null;
}

const TONE: Record<HealthLevel, StatusLevel> = { ok: "good", warn: "warning", bad: "critical" };

const PROMPT: Record<HealthId, string> = {
  blur: "slowDown",
  texture: "lowTexture",
  flicker: "lockExposure",
  corridor: "turnToCorner",
  storage: "storageFull",
  clock: "clockUnlocked",
};

/** The reading each chip shows, in the unit the agent reports it in. */
function reading(id: HealthId, value: number | null): string | null {
  if (value === null) return null;
  switch (id) {
    case "blur":
      return value.toFixed(0);
    case "texture":
      return `${Math.round(value)} pts`;
    case "flicker":
      return `${value.toFixed(1)} %`;
    case "corridor":
      return value.toFixed(2);
    case "storage":
      return `${value.toFixed(0)} %`;
    case "clock":
      return `σ ${value.toFixed(2)} ms`;
  }
}

/** All six chips in display order, filled from whatever the agent reported. */
export function healthChips(signals: readonly HealthSignal[]): HealthChip[] {
  return HEALTH_IDS.map((id) => {
    const s = signals.find((x) => x.id === id);
    const level = s?.level ?? null;
    return {
      id,
      level,
      tone: level ? TONE[level] : "offline",
      labelKey: id,
      reading: reading(id, s?.value ?? null),
      promptKey: level === "bad" ? PROMPT[id] : null,
    };
  });
}

/** The worst level among the reported chips (null when none reported). */
export function worstHealth(signals: readonly HealthSignal[]): HealthLevel | null {
  if (signals.some((s) => s.level === "bad")) return "bad";
  if (signals.some((s) => s.level === "warn")) return "warn";
  return signals.length > 0 ? "ok" : null;
}
