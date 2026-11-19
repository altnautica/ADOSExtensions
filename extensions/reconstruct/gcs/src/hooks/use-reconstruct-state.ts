/**
 * @module use-reconstruct-state
 * @description Feeds the node's `status` telemetry on the mounted node into
 * the compute store. The node publishes `{"compute": <heartbeat>}` (cluster
 * facts and the `gpu` block); each report is merged over the node's
 * last-known values, so a sparse report never clears what an earlier one
 * said.
 */

import { useEffect } from "react";

import { useHost } from "@altnautica/mapping-ui/host/context";
import { parseComputeGpu } from "@altnautica/mapping-ui/lib/agent/compute-client";
import { mapComputeSlice } from "../lib/status/compute";
import { EMPTY_COMPUTE_NODE, useComputeStore } from "../stores/compute-store";

/** The telemetry channel the node publishes its state on. */
export const STATE_CHANNEL = "status";

function objectOrNull(v: unknown): Record<string, unknown> | null {
  return typeof v === "object" && v !== null && !Array.isArray(v)
    ? (v as Record<string, unknown>)
    : null;
}

/** Apply one state report from `deviceId` to the store. */
export function applyReconstructState(deviceId: string, payload: unknown, nowMs: number): void {
  const compute = objectOrNull(objectOrNull(payload)?.compute);
  if (!compute) return;
  const store = useComputeStore.getState();
  const cluster = (store.nodes[deviceId] ?? EMPTY_COMPUTE_NODE).cluster;
  const patch = mapComputeSlice(compute, { cluster }, nowMs);
  if (patch) store.setCluster(deviceId, patch.cluster);
  if ("gpu" in compute) store.setGpu(deviceId, parseComputeGpu(compute.gpu), nowMs);
}

/** Subscribe the mounted node's state for as long as the caller is mounted. */
export function useReconstructState(): void {
  const host = useHost();
  const deviceId = host.node.deviceId;
  useEffect(() => {
    if (!deviceId) return;
    let cancelled = false;
    let unsubscribe: (() => void) | null = null;
    host.ctx.telemetry
      .subscribe<unknown>(STATE_CHANNEL, (payload) => applyReconstructState(deviceId, payload, Date.now()))
      .then(
        (off) => {
          if (cancelled) off();
          else unsubscribe = off;
        },
        (err: unknown) => {
          console.warn("Reconstruct state subscription refused", err);
        },
      );
    return () => {
      cancelled = true;
      unsubscribe?.();
    };
  }, [host, deviceId]);
}
