/**
 * A tiny store for the pod read-back the agent publishes on the "siyi"
 * telemetry channel. The panel and overlay subscribe; ingest normalises the raw
 * payload and notifies listeners on change.
 */

import type { PodState } from "./types";

export class PodStateStore {
  private state: PodState | null = null;
  private readonly listeners = new Set<(s: PodState) => void>();

  get(): PodState | null {
    return this.state;
  }

  subscribe(fn: (s: PodState) => void): () => void {
    this.listeners.add(fn);
    if (this.state) fn(this.state);
    return () => this.listeners.delete(fn);
  }

  ingest(raw: unknown): void {
    if (!isRecord(raw)) return;
    const next = normalise(raw);
    this.state = next;
    for (const fn of this.listeners) fn(next);
  }
}

function isRecord(v: unknown): v is Record<string, unknown> {
  return typeof v === "object" && v !== null;
}

function num(v: unknown): number | null {
  return typeof v === "number" && Number.isFinite(v) ? v : null;
}

function normalise(raw: Record<string, unknown>): PodState {
  return {
    model: typeof raw.model === "string" ? raw.model : "Unknown SIYI pod",
    known: raw.known === true,
    connected: raw.connected === true,
    firmware: typeof raw.firmware === "string" ? raw.firmware : null,
    capabilities: isRecord(raw.capabilities) ? (raw.capabilities as never) : {},
    yaw_deg: num(raw.yaw_deg),
    pitch_deg: num(raw.pitch_deg),
    roll_deg: num(raw.roll_deg),
    zoom: num(raw.zoom),
    gimbal_mode: typeof raw.gimbal_mode === "string" ? raw.gimbal_mode : "follow",
    palette: num(raw.palette),
    recording: raw.recording === true,
    laser_range_m: num(raw.laser_range_m),
    spot_temp_c: num(raw.spot_temp_c),
    track_active: raw.track_active === true,
    track_id: num(raw.track_id),
    link_ok: raw.link_ok === true,
    frames_received: num(raw.frames_received) ?? 0,
  };
}
