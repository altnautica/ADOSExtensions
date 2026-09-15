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
 * anything at all the honest answer is that the state is unknown.
 */

/** The `thermal` state channel payload published by the agent half. */
export interface ThermalState {
  /** True only while the agent holds an open radiometric session. */
  connected: boolean;
  /** Machine-readable reason there is no session. Absent when connected. */
  reason?: string;
  palette?: string | null;
  gain?: boolean;
}

export type ReadoutKind = "unknown" | "unavailable" | "awaiting" | "spot";

export interface Readout {
  kind: ReadoutKind;
  /** Operator-facing text for the overlay readout. */
  text: string;
}

/** Operator-facing text for each reason the agent half publishes. */
const REASON_TEXT: Readonly<Record<string, string>> = {
  "no-backend": "no UVC backend is installed on the agent",
  "no-device": "no thermal camera on the USB bus",
  "open-failed": "the thermal camera could not be opened",
  "stream-failed": "the thermal camera delivered no frames",
  "not-started": "the agent has not started the camera",
};

/**
 * Expand a machine-readable reason into operator-facing text. An unrecognised
 * reason is shown verbatim rather than dropped, so a newer agent reporting a
 * reason this bundle predates still names it.
 */
export function reasonText(reason: string | undefined): string {
  if (!reason) return "reason not reported";
  return REASON_TEXT[reason] ?? reason;
}

/**
 * Derive the overlay readout from the two inputs.
 *
 * `state` is `null` until the agent's first state publication; `spotC` is
 * `null` until a frame read-back carries a temperature.
 */
export function readout(args: {
  state: ThermalState | null;
  spotC: number | null;
}): Readout {
  const { state, spotC } = args;

  if (state === null) {
    return {
      kind: "unknown",
      text: "Thermal camera state unknown — no report from the agent",
    };
  }

  if (!state.connected) {
    if (state.reason === "no-device") {
      return { kind: "unavailable", text: "No thermal sensor detected" };
    }
    return {
      kind: "unavailable",
      text: `Thermal camera unavailable — ${reasonText(state.reason)}`,
    };
  }

  if (spotC === null) {
    return { kind: "awaiting", text: "Awaiting thermal frames..." };
  }

  return { kind: "spot", text: `Spot ${spotC.toFixed(1)} °C` };
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
