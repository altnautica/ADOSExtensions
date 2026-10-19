/**
 * @module capture/sessions
 * @description The device's recorded capture sessions (`GET atlas/sessions`),
 * their upload progress, and the reconstruction presets an operator picks.
 */

import { arr, bool, num, obj, objOrNull, str } from "./coerce";

export type UploadState = "idle" | "queued" | "uploading" | "committed" | "failed";

export interface SessionUpload {
  state: UploadState;
  bytesSent: number;
  bytesTotal: number | null;
  node: string | null;
  error: string | null;
}

export interface CaptureSession {
  sessionId: string;
  startedAtUtc: string | null;
  durationS: number | null;
  sizeBytes: number | null;
  frames: number | null;
  rooms: string[];
  floors: number[];
  timebase: string | null;
  anchorSource: string | null;
  uploaded: boolean;
  upload: SessionUpload;
}

const UPLOAD_STATES: readonly UploadState[] = ["idle", "queued", "uploading", "committed", "failed"];

function coerceUpload(raw: unknown, fallbackTotal: number | null): SessionUpload {
  const e = obj(raw);
  const state = UPLOAD_STATES.find((s) => s === e.state) ?? "idle";
  return {
    state,
    bytesSent: num(e.bytes_sent) ?? 0,
    bytesTotal: num(e.bytes_total) ?? fallbackTotal,
    node: str(e.node),
    error: str(e.error),
  };
}

/** Coerce one session row, or null when it has no id. */
export function coerceSession(raw: unknown): CaptureSession | null {
  const e = objOrNull(raw);
  const sessionId = str(e?.session_id);
  if (!e || !sessionId) return null;
  const sizeBytes = num(e.size_bytes);
  return {
    sessionId,
    startedAtUtc: str(e.started_at_utc),
    durationS: num(e.duration_s),
    sizeBytes,
    frames: num(e.frames),
    rooms: arr(e.rooms).flatMap((r) => (typeof r === "string" ? [r] : [])),
    floors: arr(e.floors).flatMap((f) => (typeof f === "number" ? [f] : [])),
    timebase: str(e.timebase),
    anchorSource: str(e.anchor_source),
    uploaded: bool(e.uploaded) === true,
    upload: coerceUpload(e.upload, sizeBytes),
  };
}

/** Coerce a `GET atlas/sessions` body: `{sessions:[...]}`. */
export function coerceSessions(raw: unknown): CaptureSession[] {
  return arr(obj(raw).sessions).flatMap((s) => {
    const session = coerceSession(s);
    return session ? [session] : [];
  });
}

/** Upload progress as a fraction `0..1`; committed is complete, an unknown
 * total is null (the bar shows indeterminate, not a made-up percentage). */
export function uploadFraction(upload: SessionUpload): number | null {
  if (upload.state === "committed") return 1;
  if (upload.bytesTotal === null || upload.bytesTotal <= 0) return null;
  return Math.min(1, Math.max(0, upload.bytesSent / upload.bytesTotal));
}

/** Upload throughput in bytes/s between two observations of one session. */
export function uploadRate(
  prev: { bytesSent: number; atMs: number } | null,
  next: { bytesSent: number; atMs: number },
): number | null {
  if (!prev || next.atMs <= prev.atMs || next.bytesSent < prev.bytesSent) return null;
  return ((next.bytesSent - prev.bytesSent) * 1000) / (next.atMs - prev.atMs);
}

/** Seconds left at `rate` bytes/s, or null when it cannot be estimated. */
export function uploadEtaS(upload: SessionUpload, rate: number | null): number | null {
  if (rate === null || rate <= 0 || upload.bytesTotal === null) return null;
  return Math.max(0, (upload.bytesTotal - upload.bytesSent) / rate);
}

/** The reconstruction presets, in picker order. */
export const RECONSTRUCT_PRESETS = ["preview", "hq", "tour", "warehouse"] as const;
export type ReconstructPreset = (typeof RECONSTRUCT_PRESETS)[number];

/** A human byte size ("38.4 GiB"). */
export function formatBytes(bytes: number | null): string {
  if (bytes === null) return "–";
  const units = ["B", "KiB", "MiB", "GiB", "TiB"];
  let v = bytes;
  let i = 0;
  while (v >= 1024 && i < units.length - 1) {
    v /= 1024;
    i += 1;
  }
  return `${v.toFixed(i === 0 ? 0 : 1)} ${units[i]}`;
}

/** A clock duration ("14:22" / "1:02:05"). */
export function formatDuration(seconds: number | null): string {
  if (seconds === null) return "–";
  const s = Math.max(0, Math.round(seconds));
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  const sec = String(s % 60).padStart(2, "0");
  return h > 0 ? `${h}:${String(m).padStart(2, "0")}:${sec}` : `${m}:${sec}`;
}
