import { useEffect, useState } from "react";

import { HostError, type PluginContext } from "@altnautica/plugin-sdk";

import { agentTsOf, sampleTimeOf } from "./freshness";
import {
  EMPTY_NAV_STATE,
  ENGAGE_TOPIC,
  NAV_STATE_TOPIC,
  normalizeEngage,
  normalizeNavState,
  type NavState,
  type RawEngage,
  type RawNavState,
  type VisionNavTelemetry,
} from "./types";

const EMPTY: VisionNavTelemetry = {
  opticalFlowSupported: false,
};

/** The latest navigation block and when it was sampled. */
export interface TelemetryFeed {
  snapshot: VisionNavTelemetry;
  /** When the snapshot was sampled (see `sampleTimeOf`); `null` before
   * the first one. */
  receivedAt: number | null;
  /** The host refused the `navigation` telemetry subscription. */
  denied: boolean;
  /** The subscription failed for another reason (timeout, host fault). */
  subscribeFailed: boolean;
}

/**
 * Subscribe to the navigation telemetry topic the agent half publishes
 * and surface the latest snapshot, with its receive time, as React
 * state. The plugin iframe has no global state library; each instance
 * owns its own subscription lifecycle.
 */
export function useVisionNavTelemetry(ctx: PluginContext): TelemetryFeed {
  const [feed, setFeed] = useState<TelemetryFeed>({
    snapshot: EMPTY,
    receivedAt: null,
    denied: false,
    subscribeFailed: false,
  });

  useEffect(() => {
    let cancelled = false;
    let off: (() => void) | null = null;
    let lastTs: number | null = null;

    void ctx.telemetry
      .subscribe<VisionNavTelemetry>("navigation", (next) => {
        if (cancelled || !next) return;
        const ts = agentTsOf(next);
        if (ts !== null && ts === lastTs) return;
        lastTs = ts;
        const sampledAt = sampleTimeOf(next, Date.now());
        // Defensive normalization: the agent may publish a partial
        // payload during warm-up. Keep the support flag sticky once
        // observed so the UI does not flicker between firmware tiers.
        setFeed((prev) => ({
          denied: false,
          subscribeFailed: false,
          receivedAt: sampledAt,
          snapshot: {
            opticalFlowSupported:
              next.opticalFlowSupported ?? prev.snapshot.opticalFlowSupported,
            flowQuality: next.flowQuality,
            flowRateHz: next.flowRateHz,
            flowDistanceM: next.flowDistanceM,
            companionState: next.companionState,
            rangefinderTopology: next.rangefinderTopology,
            recommendedCameraId: next.recommendedCameraId,
            mode: next.mode,
            availableEstimators: next.availableEstimators,
            estimatorState: next.estimatorState,
            flowScaleSource: next.flowScaleSource,
            imuSource: next.imuSource,
            imuRateHz: next.imuRateHz,
            cameraIntrinsicsLoaded: next.cameraIntrinsicsLoaded,
            cameraImuSyncOffsetMs: next.cameraImuSyncOffsetMs,
            preArmReport: next.preArmReport,
            emitErrors: next.emitErrors,
            rangefinderError: next.rangefinderError,
            degradedReasons: next.degradedReasons,
            fcParams: next.fcParams,
            poseAgeMs: next.poseAgeMs,
          },
        }));
      })
      .then((unsubscribe) => {
        if (cancelled) {
          unsubscribe();
          return;
        }
        off = unsubscribe;
      })
      .catch((err: unknown) => {
        // Say why the feed is empty instead of showing it as if the
        // agent were silent: a denial only for `capability_denied`.
        if (cancelled) return;
        const denied = err instanceof HostError && err.code === "capability_denied";
        setFeed((prev) => ({ ...prev, denied, subscribeFailed: !denied }));
      });

    return () => {
      cancelled = true;
      if (off) off();
    };
  }, [ctx]);

  return feed;
}

/** The latest value of an agent event topic and when it was sampled. */
export interface EventFeed<T> {
  value: T;
  receivedAt: number | null;
}

function useEventFeed<R, T>(
  ctx: PluginContext,
  topic: string,
  empty: T,
  normalize: (raw: R) => T,
): EventFeed<T> {
  const [feed, setFeed] = useState<EventFeed<T>>({ value: empty, receivedAt: null });
  useEffect(() => {
    let lastTs: number | null = null;
    return ctx.events.subscribe<R>(topic, (raw) => {
      const ts = agentTsOf(raw);
      if (ts !== null && ts === lastTs) return;
      lastTs = ts;
      setFeed({ value: normalize(raw), receivedAt: sampleTimeOf(raw, Date.now()) });
    });
  }, [ctx, topic, normalize]);
  return feed;
}

/**
 * Subscribe to the agent's `nav.state` event (active EKF source set,
 * external-nav health, ODOMETRY and obstacle rates, nearest obstacle,
 * GPS fix).
 */
export function useNavState(ctx: PluginContext): EventFeed<NavState> {
  return useEventFeed<RawNavState, NavState>(ctx, NAV_STATE_TOPIC, EMPTY_NAV_STATE, normalizeNavState);
}

/**
 * Subscribe to the agent's `navigation.engage` event: whether vision
 * navigation is engaged (`true`), idle (`false`), or unknown (`null`).
 */
export function useEngage(ctx: PluginContext): EventFeed<boolean | null> {
  return useEventFeed<RawEngage, boolean | null>(ctx, ENGAGE_TOPIC, null, normalizeEngage);
}
