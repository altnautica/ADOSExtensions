/**
 * A tiny store for the pod read-back the agent publishes, once on the "siyi"
 * telemetry channel and once as the siyi.pod.state event. The panel and
 * overlay subscribe; ingest normalises the raw payload, stamps its sample
 * time, and notifies listeners only when the normalised state changed.
 *
 * The host delivers the agent's state by polling about every 750 ms and can skip
 * a tick, so 1500 ms gaps are normal. Each payload carries the agent's write
 * time (`ts_ms`), aged by the shared {@link SampleClock}: an old entry
 * delivered on the first poll after mount reads stale at once, while a live
 * feed stays live whatever the drone clock skew. A sample older than
 * {@link STALE_AFTER_MS} means the agent stopped publishing.
 *
 * The two channels are stamped separately on the drone (the event is
 * throttled and stamped when it is written), so one report arrives with two
 * different `ts_ms`. Each channel therefore has its own clock: the second
 * channel's copy of an old report is that channel's first sample and is
 * judged by its stamp, rather than continuing the other channel's run.
 */

import { agentTimestampOf, SampleClock, STALE_AFTER_MS } from "@altnautica/plugin-sdk";

import type { PodState } from "./types";

export { STALE_AFTER_MS };

/** A laser range is shown for this long after it was measured. */
export const RANGE_SHOW_MS = 10_000;

/** The feed a payload came in on: the telemetry channel or the event. */
export type PodChannel = "telemetry" | "event";

export class PodStateStore {
  private state: PodState | null = null;
  private stateKey = "";
  private sampledAt: number | null = null;
  private readonly clocks: Record<PodChannel, SampleClock> = {
    telemetry: new SampleClock(),
    event: new SampleClock(),
  };
  private readonly listeners = new Set<(s: PodState) => void>();

  get(): PodState | null {
    return this.state;
  }

  /**
   * When the shown sample was taken (epoch ms, browser clock), as aged by
   * its channel's sample clock. Null before the first payload.
   */
  receivedAt(): number | null {
    return this.sampledAt;
  }

  subscribe(fn: (s: PodState) => void): () => void {
    this.listeners.add(fn);
    if (this.state) fn(this.state);
    return () => this.listeners.delete(fn);
  }

  ingest(raw: unknown, channel: PodChannel = "telemetry", now: number = Date.now()): void {
    if (!isRecord(raw)) return;
    // A repeated or older ts_ms on this channel is ignored by its clock.
    const at = this.clocks[channel].ingest(agentTimestampOf(raw), now);
    if (at === null) return;
    // A sample judged older than the one shown (an old entry on the other
    // channel) does not replace it. In a live run both channels age from
    // arrival, so the throttled event copy may still replace the state with
    // content up to one throttle period older.
    if (this.sampledAt !== null && at < this.sampledAt) return;
    this.sampledAt = at;
    const next = normalise(raw);
    const key = JSON.stringify(next);
    if (this.state && key === this.stateKey) return;
    this.state = next;
    this.stateKey = key;
    for (const fn of this.listeners) fn(next);
  }
}

/** True while the last payload is younger than {@link STALE_AFTER_MS}. */
export function isLive(store: PodStateStore, now: number = Date.now()): boolean {
  const at = store.receivedAt();
  return store.get() !== null && at !== null && now - at < STALE_AFTER_MS;
}

/** Whole seconds since the last payload, or null before the first. */
export function dataAgeS(store: PodStateStore, now: number = Date.now()): number | null {
  const at = store.receivedAt();
  return at === null ? null : Math.max(0, Math.floor((now - at) / 1000));
}

/**
 * The laser range with its age in whole seconds, or null when there is none,
 * its measurement time is unknown, or it is older than {@link RANGE_SHOW_MS}.
 * The age compares the agent's measurement time with this clock.
 */
export function rangeReading(
  state: PodState | null,
  now: number = Date.now(),
): { range_m: number; age_s: number } | null {
  if (!state || state.laser_range_m === null || state.laser_range_at === null) return null;
  const ageMs = Math.max(0, now - state.laser_range_at);
  if (ageMs >= RANGE_SHOW_MS) return null;
  return { range_m: state.laser_range_m, age_s: Math.floor(ageMs / 1000) };
}

function isRecord(v: unknown): v is Record<string, unknown> {
  return typeof v === "object" && v !== null;
}

function num(v: unknown): number | null {
  return typeof v === "number" && Number.isFinite(v) ? v : null;
}

function strMap(v: unknown): Record<string, string> {
  if (!isRecord(v)) return {};
  const out: Record<string, string> = {};
  for (const [k, val] of Object.entries(v)) {
    if (typeof val === "string") out[k] = val;
  }
  return out;
}

function normalise(raw: Record<string, unknown>): PodState {
  return {
    model: typeof raw.model === "string" ? raw.model : "Unknown SIYI pod",
    known: raw.known === true,
    connected: raw.connected === true,
    firmware: typeof raw.firmware === "string" ? raw.firmware : null,
    capabilities: isRecord(raw.capabilities) ? (raw.capabilities as never) : {},
    assignment: strMap(raw.assignment),
    yaw_deg: num(raw.yaw_deg),
    pitch_deg: num(raw.pitch_deg),
    roll_deg: num(raw.roll_deg),
    zoom: num(raw.zoom),
    gimbal_mode: typeof raw.gimbal_mode === "string" ? raw.gimbal_mode : "follow",
    palette: num(raw.palette),
    recording: raw.recording === true,
    laser_range_m: num(raw.laser_range_m),
    laser_range_at: num(raw.laser_range_at),
    laser_enabled: raw.laser_enabled === true,
    spot_temp_c: num(raw.spot_temp_c),
    link_ok: raw.link_ok === true,
    frames_received: num(raw.frames_received) ?? 0,
  };
}
