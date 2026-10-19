/**
 * @module capture-client
 * @description Client for a capture device's (Visor or drone) capture surface,
 * served by the extension on the device under `atlas/`:
 *
 *  - `GET  atlas/capture/state`, `GET atlas/capture/coverage`
 *  - `POST atlas/capture/{start,stop,mark}`, `PUT atlas/capture/exposure`
 *  - `POST atlas/capture/whep?camera=<id>` (SDP offer → answer), `DELETE <location>`
 *  - `GET  atlas/sessions`, `POST atlas/sessions/{id}/{upload,reconstruct}`,
 *    `DELETE atlas/sessions/{id}`
 *  - `GET  atlas/calib`, `GET atlas/calib/{version}`, `PUT atlas/rig`
 *
 * Reads return null on any failure; actions return a discriminated result
 * carrying the agent's error code (`E_STORAGE_FULL`, `E_NOT_UPLOADED`, ...).
 */

import type { InlineAgentApi } from "@altnautica/plugin-sdk/inline";

import type { Rig } from "../contract/generated";
import {
  coerceCaptureState,
  coerceCoverage,
  coerceExposure,
  type CaptureCoverage,
  type CaptureState,
  type ExposureMode,
  type ExposureState,
} from "../capture/capture-state";
import { coerceCalibStore, type CalibStore } from "../capture/calibration";
import { obj, num, str } from "../capture/coerce";
import { coerceSessions, type CaptureSession, type ReconstructPreset } from "../capture/sessions";
import { requestJson, type HttpMethod } from "./plugin-http";

/** A capture action's outcome: the value, or the agent's error code. */
export type ActionResult<T> = { ok: true; value: T } | { ok: false; code: string; message: string };

export interface CaptureStartRequest {
  site?: string;
  floor?: number;
  room?: string;
  operatorAnchor?: { latDeg: number; lonDeg: number; hEllipsoidM: number; headingDeg: number | null } | null;
}

export interface ExposurePatch {
  locked: boolean;
  exposureUs?: number;
  gain?: number;
  mode?: ExposureMode;
}

export class CaptureClient {
  constructor(private readonly agent: InlineAgentApi) {}

  private async read<T>(path: string, coerce: (raw: unknown) => T): Promise<T | null> {
    const res = await requestJson(this.agent, path, "GET");
    return res && res.status >= 200 && res.status < 300 ? coerce(res.json) : null;
  }

  private async act<T>(
    path: string,
    method: HttpMethod,
    body: unknown,
    coerce: (raw: Record<string, unknown>) => T,
  ): Promise<ActionResult<T>> {
    const res = await requestJson(this.agent, path, method, body);
    if (!res) return { ok: false, code: "transport_error", message: "The device did not answer." };
    const e = obj(res.json);
    if (res.status >= 200 && res.status < 300) return { ok: true, value: coerce(e) };
    const code = str(e.error) ?? (res.status === 503 ? "service_unavailable" : `http_${res.status}`);
    return { ok: false, code, message: str(e.detail) ?? str(e.message) ?? code };
  }

  getState(): Promise<CaptureState | null> {
    return this.read("atlas/capture/state", coerceCaptureState);
  }

  getCoverage(): Promise<CaptureCoverage | null> {
    return this.read("atlas/capture/coverage", coerceCoverage);
  }

  start(req: CaptureStartRequest = {}): Promise<ActionResult<{ sessionId: string | null }>> {
    const a = req.operatorAnchor;
    const body = {
      site: req.site,
      floor: req.floor,
      room: req.room,
      operator_anchor: a
        ? { lat_deg: a.latDeg, lon_deg: a.lonDeg, h_ellipsoid_m: a.hEllipsoidM, heading_deg: a.headingDeg }
        : null,
    };
    return this.act("atlas/capture/start", "POST", body, (e) => ({ sessionId: str(e.session_id) }));
  }

  stop(): Promise<ActionResult<null>> {
    return this.act("atlas/capture/stop", "POST", undefined, () => null);
  }

  mark(room: string, floor: number): Promise<ActionResult<{ segment: number | null }>> {
    return this.act("atlas/capture/mark", "POST", { room, floor }, (e) => ({ segment: num(e.segment) }));
  }

  setExposure(patch: ExposurePatch): Promise<ActionResult<ExposureState>> {
    const body = { locked: patch.locked, exposure_us: patch.exposureUs, gain: patch.gain, mode: patch.mode };
    return this.act("atlas/capture/exposure", "PUT", body, coerceExposure);
  }

  listSessions(): Promise<CaptureSession[] | null> {
    return this.read("atlas/sessions", coerceSessions);
  }

  upload(sessionId: string): Promise<ActionResult<null>> {
    return this.act(`atlas/sessions/${encodeURIComponent(sessionId)}/upload`, "POST", undefined, () => null);
  }

  deleteSession(sessionId: string, force = false): Promise<ActionResult<null>> {
    const q = force ? "?force=true" : "";
    return this.act(`atlas/sessions/${encodeURIComponent(sessionId)}${q}`, "DELETE", undefined, () => null);
  }

  reconstruct(
    sessionId: string,
    preset: ReconstructPreset,
  ): Promise<ActionResult<{ node: string | null; jobId: string | null }>> {
    return this.act(
      `atlas/sessions/${encodeURIComponent(sessionId)}/reconstruct`,
      "POST",
      { preset },
      (e) => ({ node: str(e.node), jobId: str(e.job_id) }),
    );
  }

  getCalib(): Promise<CalibStore | null> {
    return this.read("atlas/calib", coerceCalibStore);
  }

  getCalibRig(version: string): Promise<Rig | null> {
    return this.read(`atlas/calib/${encodeURIComponent(version)}`, (raw) =>
      raw && typeof raw === "object" && Array.isArray((raw as Rig).sensors) ? (raw as Rig) : null,
    );
  }

  /** Push a rig document to the device (it validates and stores it as the
   * agent's active rig). */
  putRig(rig: Rig): Promise<ActionResult<{ rigId: string | null; calibVersion: string | null }>> {
    return this.act("atlas/rig", "PUT", rig, (e) => ({
      rigId: str(e.rig_id),
      calibVersion: str(e.calib_version),
    }));
  }

  /** WHEP offer for one capture camera's preview: the SDP answer and the
   * session location to delete on teardown, or null when refused. */
  async whepOffer(cameraId: string, sdp: string): Promise<{ answer: string; location: string | null } | null> {
    let res: Response;
    try {
      res = await this.agent.fetch(`atlas/capture/whep?camera=${encodeURIComponent(cameraId)}`, {
        method: "POST",
        headers: { "Content-Type": "application/sdp" },
        body: sdp,
        signal: AbortSignal.timeout(8000),
      });
    } catch {
      return null;
    }
    if (!res || (res.status !== 200 && res.status !== 201)) return null;
    return { answer: await res.text(), location: res.headers.get("Location") };
  }

  /** Tear down a WHEP session at the location its offer returned. */
  async whepClose(location: string): Promise<void> {
    const path = location.replace(/^\/+/, "");
    try {
      await this.agent.fetch(path.startsWith("atlas/") ? path : `atlas/capture/${path}`, {
        method: "DELETE",
        signal: AbortSignal.timeout(4000),
      });
    } catch {
      /* the session expires on its own */
    }
  }
}
