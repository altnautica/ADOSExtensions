/**
 * @module atlas-readiness-store
 * @description Per-drone capture capture-readiness cache, keyed by the bare device
 * id. `use-atlas-control` polls `GET /api/atlas/readiness` for the focused drone
 * (local-first) and writes the snapshot here; the node-detail surface reads
 * {@link CaptureReadinessState.isCapturing} to decide whether the "Live World"
 * tab is shown — one tab when the drone is not capturing, two while it is.
 *
 * Every polled snapshot carries an expiry of three poll intervals past the
 * moment it was observed. A node that powers off mid-capture stops answering,
 * its snapshot expires, and every reader sees "no readiness" instead of the
 * last "capturing" forever. A demo snapshot is simulated, not observed, and
 * carries no expiry.
 *
 * Not persisted: readiness is live agent state, re-fetched on tab mount.
 *
 */

import { create } from "zustand";

import {
  isActiveCaptureState,
  type CaptureReadiness,
} from "../lib/agent/capture-control-client";

/** One cached snapshot and the instant it stops being current. */
export interface CaptureReadinessSnapshot {
  readiness: CaptureReadiness;
  /** Epoch ms after which the snapshot is no longer current; null = simulated (demo). */
  expiresAt: number | null;
}

/** The snapshot's readiness while it is current at `now`, otherwise null. */
export function currentReadiness(
  snapshot: CaptureReadinessSnapshot | undefined,
  now: number,
): CaptureReadiness | null {
  if (!snapshot) return null;
  return snapshot.expiresAt === null || now < snapshot.expiresAt ? snapshot.readiness : null;
}

interface CaptureReadinessState {
  /** Last snapshot per bare device id, current or expired. */
  snapshots: Record<string, CaptureReadinessSnapshot>;
  /** Readiness polls in a row that got no answer, per bare device id. An
   * answer resets it; a drone that never answered is told apart from one not
   * yet asked by this count, since it has no snapshot to expire. */
  failedPolls: Record<string, number>;
  /** Store (replace) the snapshot for a device; an answer ends a failure run. */
  setReadiness: (deviceId: string, readiness: CaptureReadiness, expiresAt: number | null) => void;
  /** Count one readiness poll that got no answer. */
  recordFailedPoll: (deviceId: string) => void;
  /** Drop the readiness for a device (e.g. on unpair). */
  clear: (deviceId: string) => void;
  /** The device's readiness if its snapshot is current at `now`, or null. */
  getReadiness: (deviceId: string, now: number) => CaptureReadiness | null;
  /** Whether the device is actively capturing at `now`. Derived from BOTH the
   * standalone `capturing` bool AND the lifecycle `state` (capturing / paused /
   * finalizing) so a paused session — where an agent may report
   * `capturing:false` while `state:"paused"` — still keeps the Live World tab
   * visible. An expired snapshot is not capturing. */
  isCapturing: (deviceId: string, now: number) => boolean;
}

export const useReadinessStore = create<CaptureReadinessState>((set, get) => ({
  snapshots: {},
  failedPolls: {},
  setReadiness: (deviceId, readiness, expiresAt) =>
    set((state) => {
      const failedPolls = { ...state.failedPolls };
      delete failedPolls[deviceId];
      return { snapshots: { ...state.snapshots, [deviceId]: { readiness, expiresAt } }, failedPolls };
    }),
  recordFailedPoll: (deviceId) =>
    set((state) => ({
      failedPolls: { ...state.failedPolls, [deviceId]: (state.failedPolls[deviceId] ?? 0) + 1 },
    })),
  clear: (deviceId) =>
    set((state) => {
      if (!(deviceId in state.snapshots) && !(deviceId in state.failedPolls)) return state;
      const snapshots = { ...state.snapshots };
      delete snapshots[deviceId];
      const failedPolls = { ...state.failedPolls };
      delete failedPolls[deviceId];
      return { snapshots, failedPolls };
    }),
  getReadiness: (deviceId, now) => currentReadiness(get().snapshots[deviceId], now),
  isCapturing: (deviceId, now) => {
    const r = currentReadiness(get().snapshots[deviceId], now);
    return r ? r.capturing === true || isActiveCaptureState(r.state) : false;
  },
}));
