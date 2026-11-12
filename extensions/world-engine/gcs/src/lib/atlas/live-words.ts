/**
 * @module atlas/live-words
 * @description Catalogue keys (under the `atlas` namespace) for the words the
 * capture service reports — capture state, tracking health, bearer, pose
 * source — each with an "Other" fallback so a raw agent id never reaches the
 * operator, plus the Live World header chip. Pure.
 */

const HEALTH: Record<string, true> = { good: true, degraded: true, lost: true };
const BEARERS: Record<string, string> = {
  "direct-lan": "capture.bearerDirectLan",
  "wfb-relay": "capture.bearerWfbRelay",
  cloud: "capture.bearerCloud",
};
const POSE_TIERS: Record<string, true> = { fc_ekf: true, vio: true, lidar: true, replay: true };
/** Bearers whose keyframe-degraded reason the catalogue words itself. */
const DEGRADED: Record<string, true> = { "wfb-relay": true };

export function stateWordKey(state: string): string {
  return Object.hasOwn(STATE_TONE, state) ? `capture.state.${state}` : "capture.state.other";
}

export function healthWordKey(health: string): string {
  return Object.hasOwn(HEALTH, health) ? `capture.health.${health}` : "capture.health.other";
}

export function bearerWordKey(bearer: string): string {
  return Object.hasOwn(BEARERS, bearer) ? BEARERS[bearer]! : "capture.bearerOther";
}

export function poseTierKey(tier: string): string {
  return Object.hasOwn(POSE_TIERS, tier) ? `capture.poseTier.${tier}` : "capture.poseTier.other";
}

/** The catalogue key for why `bearer` carries no keyframes, or null when only
 * the agent's own sentence can say. */
export function degradedReasonKey(bearer: string | null): string | null {
  return bearer !== null && Object.hasOwn(DEGRADED, bearer) ? `capture.degraded.${bearer}` : null;
}

const STATE_TONE: Record<string, string> = {
  capturing: "we:text-status-success",
  active: "we:text-status-success",
  finalizing: "we:text-accent-primary",
  ready: "we:text-accent-primary",
  paused: "we:text-status-warning",
  bagged: "we:text-text-tertiary",
  ended: "we:text-text-tertiary",
  idle: "we:text-text-tertiary",
  error: "we:text-status-error",
};

export interface CaptureChipInput {
  /** The live slice's state, or null. */
  liveState: string | null;
  /** The live slice was ever populated. */
  hasLive: boolean;
  /** The live slice is past its staleness budget (or never arrived). */
  isStale: boolean;
  /** The current readiness snapshot's state, or null when there is none. */
  readinessState: string | null;
  /** The node answered once and has since stopped. */
  offline: boolean;
  /** The capture runs but produces no world model (no keyframes on the link,
   * or no position anchor yet): a capturing state is not shown as healthy. */
  producingNothing?: boolean;
}

/** The header chip: a catalogue key and a tone. Never "idle" for a drone that
 * has not answered. */
export function captureChip(input: CaptureChipInput): { key: string; tone: string } {
  if (input.hasLive && input.isStale) return { key: "stale", tone: "we:text-status-warning" };
  const state = input.hasLive && input.liveState !== null ? input.liveState : input.readinessState;
  if (state === null) {
    return input.offline
      ? { key: "capture.chip.notAnswering", tone: "we:text-status-error" }
      : { key: "capture.chip.unknown", tone: "we:text-text-tertiary" };
  }
  const tone =
    input.producingNothing && (state === "capturing" || state === "active")
      ? "we:text-status-warning"
      : (STATE_TONE[state] ?? "we:text-text-secondary");
  return { key: stateWordKey(state), tone };
}
