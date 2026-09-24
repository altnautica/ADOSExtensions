/**
 * @module atlas-control-client
 * @description Client for a drone's Atlas capture-control surface, served by
 * the extension on the drone under `atlas/`:
 *
 *  - `GET  atlas/readiness`        — capture readiness snapshot
 *  - `PUT  atlas/config`           — patch { enabled?, capture_profile?, reconstruct_steps? }
 *  - `POST atlas/capture/{start,stop,pause,resume}` — capture lifecycle
 *
 * Every reply is coerced defensively: a transport failure or a non-JSON body
 * returns `null` (reads) or a typed non-ok result (writes and capture actions)
 * so a poll loop or a button handler degrades instead of throwing. A `503` on a
 * capture action means the capture service is down; it is surfaced distinctly
 * so the UI can say so rather than show a generic failure.
 */

import type { InlineAgentApi } from "@altnautica/plugin-sdk/inline";

import { DEFAULT_RECONSTRUCTION_STEPS } from "../atlas/reconstruction-quality";
import {
  AGENT_SERVICE_RESTART_CLIENT_TIMEOUT_MS,
  requestJson,
  type HttpMethod,
  type JsonReply,
} from "./plugin-http";

/** Pose-estimation source the capture rig runs. Left open so a richer agent can
 * advertise another source without breaking the type. */
export type AtlasPoseSource =
  | "local_vio"
  | "offloaded_slam"
  | "hybrid"
  | (string & {});

/** Capture lifecycle state (lowercase on the wire). Open for forward states. */
export type AtlasCaptureState =
  | "idle"
  | "capturing"
  | "paused"
  | "finalizing"
  | "bagged"
  | (string & {});

/**
 * Whether a capture state counts as an active (Live-World-visible) session:
 * actively ingesting, paused, or finalizing. Derived from `state` so a surface
 * never depends on how an agent populates the standalone `capturing` bool during
 * a paused session — an agent that reports `capturing:false` while `state:"paused"`
 * must still read as an active session (consistent reading). An unknown state
 * (`null`) is not an active session.
 */
export function isActiveCaptureState(state: string | null): boolean {
  return state === "capturing" || state === "paused" || state === "finalizing";
}

/**
 * The `GET /api/atlas/readiness` snapshot, coerced to camelCase. The live
 * fields are `null` when the agent could not read them (the capture service
 * did not answer, or is not running): unknown, never "not running" or 0.
 */
export interface AtlasReadiness {
  enabled: boolean;
  profile: string;
  captureProfile: string;
  /** The default reconstruction detail level (Brush training steps) commissioned
   * from this drone's tab. Persisted on the drone's atlas config alongside
   * `capture_profile`; defaults to the recommended level when the agent has none. */
  reconstructSteps: number;
  camerasConfigured: number;
  poseSource: AtlasPoseSource;
  serviceRunning: boolean | null;
  capturing: boolean | null;
  state: AtlasCaptureState | null;
  sessionId: string | null;
  cameraCount: number | null;
  keyframes: number | null;
  ingestRateHz: number | null;
}

/** The `POST /api/atlas/capture/*` reply, coerced to camelCase. */
export interface CaptureStatus {
  sessionId: string;
  state: AtlasCaptureState;
  keyframes: number;
  vioHealth: string;
  cameraCount: number;
  ingestRateHz: number;
}

/** Result of a capture lifecycle action. Discriminated so a 503 (capture
 * service down) is distinguishable from a transport failure or a success. */
export type CaptureResult =
  | { ok: true; status: CaptureStatus }
  | { ok: false; serviceDown: boolean; message: string };

/** A `PUT /api/atlas/config` patch (camelCase; mapped to snake_case on the wire). */
export interface AtlasConfigPatch {
  enabled?: boolean;
  captureProfile?: string;
  /** Default reconstruction detail level, in Brush training steps. */
  reconstructSteps?: number;
}

/** The `PUT /api/atlas/config` outcome. The agent answers `502` with a
 * top-level `status:"error"` when the config landed but the service restart
 * failed, so the change is not live; that is a failure here too. */
export type AtlasConfigResult =
  | { ok: true; enabled: boolean; restart: Record<string, unknown> }
  | { ok: false; message: string };

function bool(v: unknown): boolean {
  return v === true;
}

/** A reported boolean, or null when the agent sent none (unknown). */
function boolOrNull(v: unknown): boolean | null {
  return typeof v === "boolean" ? v : null;
}

function num(v: unknown): number {
  return typeof v === "number" && Number.isFinite(v) ? v : 0;
}

/** A reported finite number, or null when the agent sent none (unknown). */
function numOrNull(v: unknown): number | null {
  return typeof v === "number" && Number.isFinite(v) ? v : null;
}

function str(v: unknown): string {
  return typeof v === "string" ? v : "";
}

function strOrNull(v: unknown): string | null {
  return typeof v === "string" && v.length > 0 ? v : null;
}

function obj(v: unknown): Record<string, unknown> {
  return v && typeof v === "object" && !Array.isArray(v)
    ? (v as Record<string, unknown>)
    : {};
}

/** Coerce a raw readiness body, or null when it is not an object. */
export function coerceReadiness(raw: unknown): AtlasReadiness | null {
  if (!raw || typeof raw !== "object" || Array.isArray(raw)) return null;
  const e = raw as Record<string, unknown>;
  return {
    enabled: bool(e.enabled),
    profile: str(e.profile),
    captureProfile: str(e.capture_profile),
    // Default to the recommended level when the agent reports none (older agent
    // or unset config), so the picker always has a valid selection.
    reconstructSteps:
      num(e.reconstruct_steps) > 0
        ? num(e.reconstruct_steps)
        : DEFAULT_RECONSTRUCTION_STEPS,
    camerasConfigured: num(e.cameras_configured),
    poseSource: str(e.pose_source) || "local_vio",
    serviceRunning: boolOrNull(e.service_running),
    capturing: boolOrNull(e.capturing),
    state: strOrNull(e.state),
    sessionId: strOrNull(e.session_id),
    cameraCount: numOrNull(e.camera_count),
    keyframes: numOrNull(e.keyframes),
    ingestRateHz: numOrNull(e.ingest_rate_hz),
  };
}

/** Coerce a raw capture-status body, or null when it is not an object. */
export function coerceCaptureStatus(raw: unknown): CaptureStatus | null {
  if (!raw || typeof raw !== "object" || Array.isArray(raw)) return null;
  const e = raw as Record<string, unknown>;
  return {
    sessionId: str(e.session_id),
    state: (str(e.state) || "idle") as AtlasCaptureState,
    keyframes: num(e.keyframes),
    vioHealth: str(e.vio_health),
    cameraCount: num(e.camera_count),
    ingestRateHz: num(e.ingest_rate_hz),
  };
}

export class AtlasControlClient {
  constructor(private readonly agent: InlineAgentApi) {}

  /** One request to `atlas/<path>` (e.g. `readiness`, `config`,
   * `capture/start`); null on a transport failure. */
  private request(
    path: string,
    method: HttpMethod,
    body?: unknown,
    timeoutMs?: number,
  ): Promise<JsonReply | null> {
    return requestJson(this.agent, `atlas/${path}`, method, body, timeoutMs);
  }

  /**
   * The drone's Atlas readiness, or `null` on non-2xx / transport / parse
   * failure — so a poll never throws and a drone with no Atlas surface (404) is
   * simply treated as "no readiness".
   */
  async getReadiness(): Promise<AtlasReadiness | null> {
    const res = await this.request("readiness", "GET");
    if (!res || res.status < 200 || res.status >= 300) return null;
    return coerceReadiness(res.json);
  }

  /**
   * Patch the Atlas config (enable/disable, capture profile, reconstruction
   * steps). The camelCase patch maps to the wire's snake_case. The agent
   * restarts the capture service to apply it, so the deadline covers that
   * restart. Anything but a 2xx `status:"ok"` reply is a failure carrying the
   * agent's reason — a failed restart's message first.
   */
  async setConfig(patch: AtlasConfigPatch): Promise<AtlasConfigResult> {
    const wire: Record<string, unknown> = {};
    if (patch.enabled !== undefined) wire.enabled = patch.enabled;
    if (patch.captureProfile !== undefined)
      wire.capture_profile = patch.captureProfile;
    if (patch.reconstructSteps !== undefined)
      wire.reconstruct_steps = patch.reconstructSteps;
    const res = await this.request("config", "PUT", wire, AGENT_SERVICE_RESTART_CLIENT_TIMEOUT_MS);
    if (!res) return { ok: false, message: "The drone did not answer the config write." };
    const e = obj(res.json);
    const restart = obj(e.restart);
    if (res.status >= 200 && res.status < 300 && str(e.status) === "ok") {
      return { ok: true, enabled: bool(e.enabled), restart };
    }
    const message =
      str(restart.message) || str(e.message) || str(e.error) || `HTTP ${res.status}`;
    return { ok: false, message };
  }

  /** Drive one capture lifecycle action. A `503` reports the capture service is
   * down (distinct from a transport failure) so the UI stays honest. */
  private async capture(
    sub: "start" | "stop" | "pause" | "resume",
  ): Promise<CaptureResult> {
    const res = await this.request(`capture/${sub}`, "POST");
    if (!res) {
      return { ok: false, serviceDown: false, message: "transport_error" };
    }
    if (res.status === 503) {
      return { ok: false, serviceDown: true, message: "service_unavailable" };
    }
    if (res.status < 200 || res.status >= 300) {
      const e = obj(res.json);
      return {
        ok: false,
        serviceDown: false,
        message: str(e.message) || `http_${res.status}`,
      };
    }
    const status = coerceCaptureStatus(res.json);
    if (!status) {
      return { ok: false, serviceDown: false, message: "bad_response" };
    }
    return { ok: true, status };
  }

  captureStart(): Promise<CaptureResult> {
    return this.capture("start");
  }

  captureStop(): Promise<CaptureResult> {
    return this.capture("stop");
  }

  capturePause(): Promise<CaptureResult> {
    return this.capture("pause");
  }

  captureResume(): Promise<CaptureResult> {
    return this.capture("resume");
  }
}
