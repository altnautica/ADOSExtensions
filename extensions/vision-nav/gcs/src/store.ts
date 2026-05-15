import { useEffect, useState } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import type { VisionNavTelemetry } from "./types";

const EMPTY: VisionNavTelemetry = {
  opticalFlowSupported: false,
  vioSupported: false,
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

    void ctx.telemetry
      .subscribe<VisionNavTelemetry>("navigation", (next) => {
        if (cancelled || !next) return;
        // Defensive normalization: the agent may publish a partial
        // payload during warm-up. Keep the support flags sticky once
        // observed so the UI does not flicker between firmware tiers.
        setSnapshot((prev) => ({
          opticalFlowSupported:
            next.opticalFlowSupported ?? prev.opticalFlowSupported,
          vioSupported: next.vioSupported ?? prev.vioSupported,
          flowQuality: next.flowQuality,
          flowRateHz: next.flowRateHz,
          flowDistanceM: next.flowDistanceM,
          vioState: next.vioState,
          vioResetCounter: next.vioResetCounter,
          vioQuality: next.vioQuality,
          companionState: next.companionState,
          rangefinderTopology: next.rangefinderTopology,
          recommendedCameraId: next.recommendedCameraId,
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
