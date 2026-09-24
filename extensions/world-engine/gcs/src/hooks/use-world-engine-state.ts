/**
 * @module use-world-engine-state
 * @description Feeds the extension's own agent state on the mounted node into
 * the stores. The agent half publishes one merged object on its `status`
 * telemetry channel: `atlas` (the drone capture service's report) and
 * `compute` (a workstation or compute node's heartbeat report, including the
 * `gpu` block). Each slice is merged over the node's last-known values, so a
 * sparse report never clears what an earlier one said.
 */

import { useEffect } from "react";

import { useHost } from "../host/context";
import { parseComputeGpu } from "../lib/agent/compute-client";
import { mapAtlasSlice } from "../lib/status/atlas";
import { mapComputeSlice } from "../lib/status/compute";
import { EMPTY_ATLAS_LIVE, useAtlasStore } from "../stores/atlas-store";
import { EMPTY_COMPUTE_NODE, useComputeStore } from "../stores/compute-store";

/** The telemetry channel the agent half publishes its state on. */
export const STATE_CHANNEL = "status";

function objectOrNull(v: unknown): Record<string, unknown> | null {
  return typeof v === "object" && v !== null && !Array.isArray(v)
    ? (v as Record<string, unknown>)
    : null;
}

/** Apply one state report from `deviceId` to the stores. */
export function applyWorldEngineState(deviceId: string, payload: unknown, nowMs: number): void {
  const state = objectOrNull(payload);
  if (!state) return;

  const atlas = objectOrNull(state.atlas);
  if (atlas) {
    const live = useAtlasStore.getState().live[deviceId] ?? EMPTY_ATLAS_LIVE;
    const patch = mapAtlasSlice(atlas, { live }, nowMs);
    if (patch) useAtlasStore.getState().setLive(deviceId, patch.live);
  }

  const compute = objectOrNull(state.compute);
  if (compute) {
    const store = useComputeStore.getState();
    const cluster = (store.nodes[deviceId] ?? EMPTY_COMPUTE_NODE).cluster;
    const patch = mapComputeSlice(compute, { cluster }, nowMs);
    if (patch) store.setCluster(deviceId, patch.cluster);
    if ("gpu" in compute) store.setGpu(deviceId, parseComputeGpu(compute.gpu), nowMs);
  }
}

/** Subscribe the mounted node's state for as long as the caller is mounted. */
export function useWorldEngineState(): void {
  const host = useHost();
  const deviceId = host.node.deviceId;
  useEffect(() => {
    if (!deviceId) return;
    let cancelled = false;
    let unsubscribe: (() => void) | null = null;
    host.ctx.telemetry
      .subscribe<unknown>(STATE_CHANNEL, (payload) =>
        applyWorldEngineState(deviceId, payload, Date.now()),
      )
      .then(
        (off) => {
          if (cancelled) off();
          else unsubscribe = off;
        },
        (err: unknown) => {
          console.warn("World Engine state subscription refused", err);
        },
      );
    return () => {
      cancelled = true;
      unsubscribe?.();
    };
  }, [host, deviceId]);
}
