import { useEffect, useState } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import {
  EMPTY_NAV_STATE,
  NAV_STATE_TOPIC,
  normalizeNavState,
  type NavState,
  type RawNavState,
  type VisionNavTelemetry,
} from "./types";

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
          // Diagnostics the agent publishes on the same block; the PX4
          // parameter card reads fcParams.
          emitErrors: next.emitErrors,
          rangefinderError: next.rangefinderError,
          degradedReasons: next.degradedReasons,
          fcParams: next.fcParams,
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

/**
 * Subscribe to the agent's `nav.state` event (active EKF source set,
 * external-nav health, ODOMETRY and obstacle rates, nearest obstacle,
 * GPS fix) and surface the latest as React state.
 */
export function useNavState(ctx: PluginContext): NavState {
  const [state, setState] = useState<NavState>(EMPTY_NAV_STATE);

  useEffect(() => {
    const off = ctx.events.subscribe<RawNavState>(NAV_STATE_TOPIC, (raw) => {
      setState(normalizeNavState(raw));
    });
    return off;
  }, [ctx]);

  return state;
}

export function resetSnapshot(): VisionNavTelemetry {
  return { ...EMPTY };
}
