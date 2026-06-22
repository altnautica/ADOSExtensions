/**
 * Designation dispatch from the GCS overlay.
 *
 * The operator's click is forwarded to the agent half so the agent owns
 * the resulting track lock: the overlay sends the chosen camera + box on
 * the `follow-me/designate` command, which the host bridges onto the
 * agent's event bus, where the Follow-Me agent plugin designates the box
 * with the vision engine and follows the returned track id. Routing the
 * lock through the agent keeps a single owner of the locked track, so the
 * GCS never holds a lock the agent does not know about.
 *
 * @license GPL-3.0-or-later
 */

import type { PluginContext } from "@altnautica/plugin-sdk";

import { DESIGNATE_TOPIC, type OverlayDetectionItem } from "./types";

export interface DesignatePayload {
  camera_id: string;
  bbox: { x: number; y: number; width: number; height: number };
  class_label: string;
  confidence: number;
  track_id: number | null;
}

/**
 * Build the designate payload from a clicked detection. Carries the box
 * in source-frame pixels (the agent re-projects from the same frame), the
 * class label + confidence for the engine's designate call, and the
 * track id the host-side detection already carried (the agent re-resolves
 * it through the engine designate regardless).
 */
export function buildDesignatePayload(
  cameraId: string,
  det: OverlayDetectionItem,
): DesignatePayload {
  return {
    camera_id: cameraId,
    bbox: {
      x: det.bbox.x,
      y: det.bbox.y,
      width: det.bbox.width,
      height: det.bbox.height,
    },
    class_label: det.classLabel,
    confidence: det.confidence,
    track_id: det.trackId,
  };
}

/**
 * Send a designate to the agent. Resolves true when the host accepted the
 * command, false when it threw (host denied or no agent reachable); the
 * overlay surfaces the failure rather than silently dropping the click.
 */
export async function sendDesignate(
  ctx: PluginContext,
  payload: DesignatePayload,
): Promise<boolean> {
  try {
    await ctx.command.send(DESIGNATE_TOPIC, payload);
    return true;
  } catch {
    return false;
  }
}
