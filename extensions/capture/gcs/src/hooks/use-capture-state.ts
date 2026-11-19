/**
 * @module use-capture-state
 * @description Feeds the capture link's `status` telemetry on the mounted
 * node into the capture store. The link publishes `{"capture": <slice or
 * null>}`; each slice is merged over the node's last-known values, so a
 * sparse report never clears what an earlier one said.
 */

import { useEffect } from "react";

import { useHost } from "@altnautica/mapping-ui/host/context";
import { mapCaptureSlice } from "../lib/status/capture";
import { EMPTY_CAPTURE_LIVE, useCaptureStore } from "../stores/capture-store";

/** The telemetry channel the link publishes its state on. */
export const STATE_CHANNEL = "status";

function objectOrNull(v: unknown): Record<string, unknown> | null {
  return typeof v === "object" && v !== null && !Array.isArray(v)
    ? (v as Record<string, unknown>)
    : null;
}

/** Apply one state report from `deviceId` to the store. */
export function applyCaptureState(deviceId: string, payload: unknown, nowMs: number): void {
  const slice = objectOrNull(objectOrNull(payload)?.capture);
  if (!slice) return;
  const live = useCaptureStore.getState().live[deviceId] ?? EMPTY_CAPTURE_LIVE;
  const patch = mapCaptureSlice(slice, { live }, nowMs);
  if (patch) useCaptureStore.getState().setLive(deviceId, patch.live);
}

/** Subscribe the mounted node's state for as long as the caller is mounted. */
export function useCaptureState(): void {
  const host = useHost();
  const deviceId = host.node.deviceId;
  useEffect(() => {
    if (!deviceId) return;
    let cancelled = false;
    let unsubscribe: (() => void) | null = null;
    host.ctx.telemetry
      .subscribe<unknown>(STATE_CHANNEL, (payload) => applyCaptureState(deviceId, payload, Date.now()))
      .then(
        (off) => {
          if (cancelled) off();
          else unsubscribe = off;
        },
        (err: unknown) => {
          console.warn("Capture state subscription refused", err);
        },
      );
    return () => {
      cancelled = true;
      unsubscribe?.();
    };
  }, [host, deviceId]);
}
