/**
 * @module use-atlas-control
 * @description Capture-control hook for a drone's Atlas world-model service.
 * Polls `atlas/readiness` on the drone into the per-drone
 * `atlas-readiness-store`, and returns the enable/disable + capture lifecycle
 * callbacks.
 *
 * The poll runs whenever a drone is given: every 1.5 s while the capture
 * service is enabled on it, every 10 s while it is off, so the page still
 * reflects a capture another GCS started. Each snapshot expires three poll
 * intervals after it was observed, so a node that stops answering reads as
 * offline instead of keeping its last state.
 */

import { useCallback, useEffect, useMemo, useRef, useState } from "react";

import { useNodeAgent } from "../host/context";
import {
  AtlasControlClient,
  isActiveCaptureState,
  type AtlasConfigPatch,
  type AtlasReadiness,
  type CaptureResult,
  type CaptureStatus,
} from "../lib/agent/atlas-control-client";
import { useNow } from "../lib/clock";
import { currentReadiness, useAtlasReadinessStore } from "../stores/atlas-readiness-store";

/** Readiness poll cadence while the capture service is enabled, in ms. */
export const ATLAS_READINESS_POLL_INTERVAL_MS = 1500;

/** Readiness poll cadence while the capture service is off, in ms. */
export const ATLAS_READINESS_IDLE_POLL_INTERVAL_MS = 10_000;

/** A snapshot stays current for this many poll intervals after it was observed. */
const READINESS_LIFETIME_POLLS = 3;

export interface AtlasControl {
  /** The drone this control targets, or null when none. */
  deviceId: string | null;
  /** Current readiness (reactive): null until the first poll resolves, and
   * again once the last snapshot expired because the node stopped answering. */
  readiness: AtlasReadiness | null;
  /** True when a snapshot was observed and has since expired: the node
   * stopped answering. False before the first answer. */
  offline: boolean;
  /** The agent's reason for the last config write it refused (a failed
   * service restart included), or null after a write that landed. */
  configError: string | null;
  /** True while a lifecycle/config action is in flight (drives button spinners). */
  busy: boolean;
  /** True when a drone is targeted, so its capture service can be driven. */
  live: boolean;
  /** Enable the Atlas capture service on this drone. Resolves false when the
   *  node refused or could not be reached — the caller must not flip a local
   *  flag on a write that did not land. */
  enable: () => Promise<boolean>;
  /** Disable the Atlas capture service. False when the write did not land;
   *  the node is still capturing. */
  disable: () => Promise<boolean>;
  /** Set the capture profile. */
  setCaptureProfile: (profile: string) => Promise<boolean>;
  /** Set the default reconstruction detail level, in Brush steps. Read at
   * reconstruct-submit time. */
  setReconstructSteps: (steps: number) => Promise<boolean>;
  start: () => Promise<CaptureResult>;
  stop: () => Promise<CaptureResult>;
  pause: () => Promise<CaptureResult>;
  resume: () => Promise<CaptureResult>;
}

/**
 * Drive a drone's Atlas capture service. Polls readiness while mounted and
 * returns the capture-control callbacks. Inert when `deviceId` is null.
 */
export function useAtlasControl(deviceId: string | null): AtlasControl {
  const agent = useNodeAgent(deviceId);
  const client = useMemo(() => (agent ? new AtlasControlClient(agent) : null), [agent]);
  const live = client !== null;

  // Re-render on a 1 Hz clock so an expiring snapshot is noticed even when no
  // new poll result arrives.
  const now = useNow();
  const snapshot = useAtlasReadinessStore((s) => (deviceId ? s.snapshots[deviceId] : undefined));
  const readiness = currentReadiness(snapshot, now);
  const offline = snapshot !== undefined && readiness === null;
  const [configError, setConfigError] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const busyRef = useRef(false);

  const pollIntervalMs = readiness?.enabled
    ? ATLAS_READINESS_POLL_INTERVAL_MS
    : ATLAS_READINESS_IDLE_POLL_INTERVAL_MS;

  const refreshReadiness = useCallback(async () => {
    if (!client || !deviceId) return;
    const r = await client.getReadiness();
    if (r) {
      useAtlasReadinessStore
        .getState()
        .setReadiness(deviceId, r, Date.now() + READINESS_LIFETIME_POLLS * pollIntervalMs);
    }
  }, [client, deviceId, pollIntervalMs]);

  // Readiness poll. Re-arms on drone switch and cadence changes. A failed poll
  // writes nothing, so the last snapshot expires on its own.
  useEffect(() => {
    if (!client) return;
    void refreshReadiness();
    const handle = setInterval(() => void refreshReadiness(), pollIntervalMs);
    return () => clearInterval(handle);
  }, [client, pollIntervalMs, refreshReadiness]);

  /**
   * Run one action at a time and report whether it LANDED, so a failed enable
   * never leaves the toggle checked and a failed disable never hides a drone
   * that is still capturing.
   */
  const runAction = useCallback(async (fn: () => Promise<boolean>): Promise<boolean> => {
    if (busyRef.current) return false;
    busyRef.current = true;
    setBusy(true);
    try {
      return await fn();
    } finally {
      busyRef.current = false;
      setBusy(false);
    }
  }, []);

  const mergeCaptureStatus = useCallback(
    (status: CaptureStatus) => {
      if (!deviceId) return;
      const observedAt = Date.now();
      const current = useAtlasReadinessStore.getState().getReadiness(deviceId, observedAt);
      if (!current) return;
      useAtlasReadinessStore.getState().setReadiness(
        deviceId,
        {
          ...current,
          state: status.state,
          capturing: isActiveCaptureState(status.state),
          sessionId: status.sessionId || current.sessionId,
          keyframes: status.keyframes,
          cameraCount: status.cameraCount || current.cameraCount,
          ingestRateHz: status.ingestRateHz,
        },
        observedAt + READINESS_LIFETIME_POLLS * pollIntervalMs,
      );
    },
    [deviceId, pollIntervalMs],
  );

  /** One config write against the agent; records the agent's refusal. */
  const writeConfig = useCallback(
    (patch: AtlasConfigPatch) =>
      runAction(async () => {
        if (!client) return false;
        const res = await client.setConfig(patch);
        setConfigError(res.ok ? null : res.message);
        await refreshReadiness();
        return res.ok;
      }),
    [client, runAction, refreshReadiness],
  );

  const enable = useCallback(() => writeConfig({ enabled: true }), [writeConfig]);
  const disable = useCallback(() => writeConfig({ enabled: false }), [writeConfig]);
  const setCaptureProfile = useCallback(
    (profile: string) => writeConfig({ captureProfile: profile }),
    [writeConfig],
  );
  const setReconstructSteps = useCallback(
    (steps: number) => writeConfig({ reconstructSteps: steps }),
    [writeConfig],
  );

  const captureAction = useCallback(
    async (sub: "start" | "stop" | "pause" | "resume"): Promise<CaptureResult> => {
      let result: CaptureResult = { ok: false, serviceDown: false, message: "inactive" };
      await runAction(async () => {
        if (!client) return false;
        result =
          sub === "start"
            ? await client.captureStart()
            : sub === "stop"
              ? await client.captureStop()
              : sub === "pause"
                ? await client.capturePause()
                : await client.captureResume();
        if (result.ok) mergeCaptureStatus(result.status);
        await refreshReadiness();
        return result.ok;
      });
      return result;
    },
    [client, runAction, mergeCaptureStatus, refreshReadiness],
  );

  const start = useCallback(() => captureAction("start"), [captureAction]);
  const stop = useCallback(() => captureAction("stop"), [captureAction]);
  const pause = useCallback(() => captureAction("pause"), [captureAction]);
  const resume = useCallback(() => captureAction("resume"), [captureAction]);

  return useMemo(
    () => ({
      deviceId,
      readiness,
      offline,
      configError,
      busy,
      live,
      enable,
      disable,
      setCaptureProfile,
      setReconstructSteps,
      start,
      stop,
      pause,
      resume,
    }),
    [
      deviceId,
      readiness,
      offline,
      configError,
      busy,
      live,
      enable,
      disable,
      setCaptureProfile,
      setReconstructSteps,
      start,
      stop,
      pause,
      resume,
    ],
  );
}
