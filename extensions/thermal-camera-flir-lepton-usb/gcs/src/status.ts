/**
 * Readout state for the thermal overlay.
 *
 * The overlay has two inputs: the `thermal` state channel, which reports
 * whether the agent holds an open radiometric session and why not when it
 * does not, and the `camera.thermal.frame` channel, which carries frame
 * read-backs only while a session is open.
 *
 * Those inputs are deliberately kept apart from the rendered text, because
 * the readout must never show a plausible-looking default. "Awaiting thermal
 * frames" is only true of a connected camera that has not delivered a frame
 * yet; with no camera it is a false statement, and before the agent has said
 * anything at all the honest answer is that the state is unknown. A reading
 * that has stopped refreshing is not shown as live: once it is
 * {@link STALE_AFTER_MS} old the readout says the frames stopped and how long
 * ago, and drops the value.
 */

/** Translate a locale key, as `ctx.i18n.t` does. */
export type Translate = (
  key: string,
  params?: Record<string, string | number>,
) => string;

/** A spot reading older than this is stale and no longer shown as live. */
export const STALE_AFTER_MS = 2000;

/** The `thermal` state channel payload published by the agent half. */
export interface ThermalState {
  /** True only while the agent holds an open radiometric session. */
  connected: boolean;
  /** Machine-readable reason there is no session. Absent when connected. */
  reason?: string;
  palette?: string | null;
  gain?: boolean;
}

export type ReadoutKind =
  | "unknown"
  | "unavailable"
  | "awaiting"
  | "stale"
  | "spot";

export interface Readout {
  kind: ReadoutKind;
  /** Operator-facing text for the overlay readout. */
  text: string;
}

/** Locale key for each reason the agent half publishes. */
const REASON_KEY: Readonly<Record<string, string>> = {
  "no-backend": "overlay.reason.noBackend",
  "open-failed": "overlay.reason.openFailed",
  "stream-failed": "overlay.reason.streamFailed",
  "not-started": "overlay.reason.notStarted",
};

/**
 * Expand a machine-readable reason into operator-facing text. A reason this
 * bundle does not know is reported as unrecognised rather than shown as a
 * raw id.
 */
export function reasonText(reason: string | undefined, t: Translate): string {
  if (!reason) return t("overlay.reason.notReported");
  const key = REASON_KEY[reason];
  return key ? t(key) : t("overlay.reason.unrecognised");
}

/**
 * Derive the overlay readout from the inputs.
 *
 * `state` is `null` until the agent's first state publication; `spotC` is
 * `null` until a frame read-back carries a temperature; `ageMs` is how long
 * ago that reading arrived.
 */
export function readout(args: {
  state: ThermalState | null;
  spotC: number | null;
  ageMs: number | null;
  t: Translate;
}): Readout {
  const { state, spotC, ageMs, t } = args;

  if (state === null) {
    return { kind: "unknown", text: t("overlay.stateUnknown") };
  }

  if (!state.connected) {
    if (state.reason === "no-device") {
      return { kind: "unavailable", text: t("panel.noDevice") };
    }
    return {
      kind: "unavailable",
      text: t("overlay.unavailable", {
        reason: reasonText(state.reason, t),
      }),
    };
  }

  if (spotC === null) {
    return { kind: "awaiting", text: t("panel.awaitingFrames") };
  }

  if (ageMs === null || ageMs >= STALE_AFTER_MS) {
    const seconds = Math.floor(Math.max(0, ageMs ?? 0) / 1000);
    return { kind: "stale", text: t("overlay.framesStopped", { age: seconds }) };
  }

  return {
    kind: "spot",
    text: t("overlay.spotTemperature", { value: spotC.toFixed(1), unit: "°C" }),
  };
}

/** Locale key for each palette the agent applies. */
const PALETTE_KEY: Readonly<Record<string, string>> = {
  ironbow: "config.palette.ironbow",
  rainbow: "config.palette.rainbow",
  grayscale: "config.palette.grayscale",
};

/**
 * The agent-applied palette as read-only operator text, or `null` when the
 * read-back carries none (or one this bundle cannot name). Shown only while
 * a session is open: a disconnected camera applies no palette.
 */
export function paletteText(
  state: ThermalState | null,
  t: Translate,
): string | null {
  if (!state?.connected || !state.palette) return null;
  const key = PALETTE_KEY[state.palette];
  if (!key) return null;
  return t("overlay.palette", { name: t(key) });
}

/**
 * Narrow an untrusted host event into a {@link ThermalState}, or `null` when
 * the payload carries no usable connectivity flag. A payload the overlay
 * cannot read is not treated as a connected camera.
 */
export function parseThermalState(raw: unknown): ThermalState | null {
  if (typeof raw !== "object" || raw === null) return null;
  const obj = raw as Record<string, unknown>;
  if (typeof obj.connected !== "boolean") return null;
  const state: ThermalState = { connected: obj.connected };
  if (typeof obj.reason === "string") state.reason = obj.reason;
  if (typeof obj.palette === "string") state.palette = obj.palette;
  if (typeof obj.gain === "boolean") state.gain = obj.gain;
  return state;
}
