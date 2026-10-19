/**
 * @module lib/capture/camera-empty
 * @description Why the capture page shows no camera tile. The link answers an
 * idle capture state with no cameras when its capture service is stopped, so
 * an empty camera list alone must not read as "no camera configured".
 */

import type { AtlasReadiness } from "../agent/atlas-control-client";

/** The locale key under `capture.` explaining an empty camera grid. */
export type CameraEmptyReason = "serviceStopped" | "noCameras" | "noCameraFrames" | "camerasUnknown";

export function cameraEmptyReason(readiness: AtlasReadiness | null): CameraEmptyReason {
  if (!readiness || readiness.serviceRunning === null) return "camerasUnknown";
  if (!readiness.serviceRunning) return "serviceStopped";
  return readiness.camerasConfigured === 0 ? "noCameras" : "noCameraFrames";
}
