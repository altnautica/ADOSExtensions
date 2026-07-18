/**
 * Capability gating for the console.
 *
 * The agent publishes the negotiated capability profile inside the pod state;
 * the console renders only the controls the connected model supports, so one
 * GCS half drives an A2 mini (a fixed EO camera) and a ZT30 (a four-sensor pod)
 * with no per-model UI branches.
 */

import type { PodCapabilities, PodState } from "./types";

export type PodFeature = "gimbal" | "zoom" | "thermal" | "laser" | "ai_track";

export function capabilitiesOf(state: PodState | null): Partial<PodCapabilities> {
  return state?.capabilities ?? {};
}

export function supports(
  state: PodState | null,
  feature: PodFeature,
): boolean {
  const caps = capabilitiesOf(state);
  return caps[feature] === true;
}

/** The max zoom to bound the zoom control (defaults to 1 = no zoom). */
export function maxZoom(state: PodState | null): number {
  const caps = capabilitiesOf(state);
  return typeof caps.max_zoom === "number" ? caps.max_zoom : 1;
}

/** True when the console should show a control for a feature. */
export function showControl(state: PodState | null, feature: PodFeature): boolean {
  return supports(state, feature);
}
