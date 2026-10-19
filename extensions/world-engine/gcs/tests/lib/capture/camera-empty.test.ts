/**
 * An empty camera grid says why: a stopped capture service is not "no camera
 * configured".
 */

import { describe, expect, it } from "vitest";

import type { AtlasReadiness } from "../../../src/lib/agent/atlas-control-client";
import { cameraEmptyReason } from "../../../src/lib/capture/camera-empty";

function readiness(serviceRunning: boolean | null, camerasConfigured: number): AtlasReadiness {
  return { serviceRunning, camerasConfigured } as AtlasReadiness;
}

describe("cameraEmptyReason", () => {
  it("names a stopped service, not a missing camera", () => {
    expect(cameraEmptyReason(readiness(false, 2))).toBe("serviceStopped");
    expect(cameraEmptyReason(readiness(false, 0))).toBe("serviceStopped");
  });

  it("says no camera is configured only when the running service has none", () => {
    expect(cameraEmptyReason(readiness(true, 0))).toBe("noCameras");
    expect(cameraEmptyReason(readiness(true, 2))).toBe("noCameraFrames");
  });

  it("claims nothing while the service status is unknown", () => {
    expect(cameraEmptyReason(null)).toBe("camerasUnknown");
    expect(cameraEmptyReason(readiness(null, 2))).toBe("camerasUnknown");
  });
});
