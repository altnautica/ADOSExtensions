import { useEffect, useState } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import type { VisionNavTelemetry } from "./types";

const EMPTY: VisionNavTelemetry = {
  opticalFlowSupported: false,
};

/**
 * Subscribe to the navigation telemetry topic the agent half publishes
 * and surface the latest snapshot as React state. The plugin iframe
 * has no global state library; each instance owns its own subscription
 * lifecycle.
 */
export function useVisionNavTelemetry(
  ctx: PluginContext,
): VisionNavTelemetry {
  const [snapshot, setSnapshot] = useState<VisionNavTelemetry>(EMPTY);

  useEffect(() => {
    let cancelled = false;
    let off: (() => void) | null = null;

    void ctx.telemetry.subscribe<VisionNavTelemetry>("navigation", (next) => {
        if (cancelled || !next) return;
        // Defensive normalization: the agent may publish a partial
        // payload during warm-up. Keep the support flag sticky once
        // observed so the UI does not flicker between firmware tiers.
        setSnapshot((prev) => ({
          opticalFlowSupported:
            next.opticalFlowSupported ?? prev.opticalFlowSupported,
          flowQuality: next.flowQuality,
          flowRateHz: next.flowRateHz,
          flowDistanceM: next.flowDistanceM,
          companionState: next.companionState,
          rangefinderTopology: next.rangefinderTopology,
          recommendedCameraId: next.recommendedCameraId,
          // Estimator framework + IMU + calibration fields. All
          // optional; an older agent that does not emit them leaves
          // the new cards in their "awaiting wiring" empty state.
          mode: next.mode,
          availableEstimators: next.availableEstimators,
          estimatorState: next.estimatorState,
          flowScaleSource: next.flowScaleSource,
          imuSource: next.imuSource,
          imuRateHz: next.imuRateHz,
          cameraIntrinsicsLoaded: next.cameraIntrinsicsLoaded,
          cameraImuSyncOffsetMs: next.cameraImuSyncOffsetMs,
          preArmReport: next.preArmReport,
        }));
      })
      .then((unsubscribe) => {
        if (cancelled) {
          unsubscribe();
          return;
        }
        off = unsubscribe;
      })
      .catch(() => {
        // Host denied the subscription. Component renders the empty
        // baseline and pre-arm rows surface the gap to the operator.
      });

    return () => {
      cancelled = true;
      if (off) off();
    };
  }, [ctx]);

  return snapshot;
}

export function resetSnapshot(): VisionNavTelemetry {
  return { ...EMPTY };
}
