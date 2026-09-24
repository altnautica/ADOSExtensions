/**
 * The Atlas capture-control client: defensive coercion of the wire shapes, the
 * `atlas/<path>` routing through the node agent, the snake_case config-patch
 * mapping, and the 503 "capture service down" branch.
 */

import { describe, it, expect, vi, afterEach } from "vitest";
import { callAt as call, json, stubAgent } from "../../helpers/agent";
import {
  AtlasControlClient,
  coerceReadiness,
  coerceCaptureStatus,
  isActiveCaptureState,
} from "../../../src/lib/agent/atlas-control-client";

describe("isActiveCaptureState", () => {
  it("treats capturing / paused / finalizing as active sessions", () => {
    expect(isActiveCaptureState("capturing")).toBe(true);
    expect(isActiveCaptureState("paused")).toBe(true);
    expect(isActiveCaptureState("finalizing")).toBe(true);
  });

  it("treats idle / bagged / unknown as inactive", () => {
    expect(isActiveCaptureState("idle")).toBe(false);
    expect(isActiveCaptureState("bagged")).toBe(false);
    expect(isActiveCaptureState("")).toBe(false);
    expect(isActiveCaptureState("ended")).toBe(false);
    expect(isActiveCaptureState(null)).toBe(false);
  });
});

const READINESS_WIRE = {
  enabled: true,
  profile: "drone",
  capture_profile: "balanced",
  reconstruct_steps: 15000,
  cameras_configured: 6,
  pose_source: "hybrid",
  service_running: true,
  capturing: true,
  state: "capturing",
  session_id: "atlas-123",
  camera_count: 6,
  keyframes: 42,
  ingest_rate_hz: 6.5,
};

const CAPTURE_WIRE = {
  session_id: "s9",
  state: "capturing",
  keyframes: 1,
  vio_health: "good",
  camera_count: 6,
  ingest_rate_hz: 6,
};

describe("coerceReadiness", () => {
  it("maps snake_case wire fields to camelCase", () => {
    expect(coerceReadiness(READINESS_WIRE)).toEqual({
      enabled: true,
      profile: "drone",
      captureProfile: "balanced",
      reconstructSteps: 15000,
      camerasConfigured: 6,
      poseSource: "hybrid",
      serviceRunning: true,
      capturing: true,
      state: "capturing",
      sessionId: "atlas-123",
      cameraCount: 6,
      keyframes: 42,
      ingestRateHz: 6.5,
    });
  });

  it("reads fields the agent did not report as unknown, never as idle", () => {
    expect(coerceReadiness({})).toMatchObject({
      enabled: false,
      capturing: null,
      camerasConfigured: 0,
      poseSource: "local_vio",
      state: null,
      sessionId: null,
      ingestRateHz: null,
    });
  });

  it("returns null for a non-object body", () => {
    expect(coerceReadiness(null)).toBeNull();
    expect(coerceReadiness("nope")).toBeNull();
    expect(coerceReadiness([1, 2])).toBeNull();
  });
});

describe("coerceCaptureStatus", () => {
  it("maps the capture-status wire shape", () => {
    expect(
      coerceCaptureStatus({
        session_id: "s1",
        state: "paused",
        keyframes: 10,
        vio_health: "good",
        camera_count: 4,
        ingest_rate_hz: 3,
      }),
    ).toEqual({
      sessionId: "s1",
      state: "paused",
      keyframes: 10,
      vioHealth: "good",
      cameraCount: 4,
      ingestRateHz: 3,
    });
  });

  it("returns null for a non-object body", () => {
    expect(coerceCaptureStatus(undefined)).toBeNull();
  });
});

describe("AtlasControlClient", () => {
  afterEach(() => {
    vi.restoreAllMocks();
  });

  it("getReadiness GETs atlas/readiness through the node agent", async () => {
    const { agent, fetch } = stubAgent(() => json(200, READINESS_WIRE));
    const r = await new AtlasControlClient(agent).getReadiness();
    expect(r?.capturing).toBe(true);
    const { path, init } = call(fetch);
    expect(path).toBe("atlas/readiness");
    expect(init.method).toBe("GET");
    expect(init.body).toBeUndefined();
  });

  it("getReadiness returns null on a 404", async () => {
    const { agent } = stubAgent(() => json(404, { error: "not found" }));
    expect(await new AtlasControlClient(agent).getReadiness()).toBeNull();
  });

  it("getReadiness returns null on a transport failure", async () => {
    const { agent } = stubAgent(() => Promise.reject(new Error("boom")));
    expect(await new AtlasControlClient(agent).getReadiness()).toBeNull();
  });

  it("getReadiness returns null on a 2xx body that is not JSON", async () => {
    const { agent } = stubAgent(() => Promise.resolve(new Response("<html>", { status: 200 })));
    expect(await new AtlasControlClient(agent).getReadiness()).toBeNull();
  });

  it("setConfig PUTs atlas/config and maps the camelCase patch to snake_case", async () => {
    const { agent, fetch } = stubAgent(() =>
      json(200, { status: "ok", enabled: true, restart: { ados_atlas: true } }),
    );
    const out = await new AtlasControlClient(agent).setConfig({
      enabled: true,
      captureProfile: "fast",
      reconstructSteps: 7000,
    });
    expect(out).toEqual({ ok: true, enabled: true, restart: { ados_atlas: true } });
    const { path, init } = call(fetch);
    expect(path).toBe("atlas/config");
    expect(init.method).toBe("PUT");
    expect(JSON.parse(String(init.body))).toEqual({
      enabled: true,
      capture_profile: "fast",
      reconstruct_steps: 7000,
    });
  });

  it("setConfig sends only the fields the patch names", async () => {
    const { agent, fetch } = stubAgent(() => json(200, { status: "ok", enabled: false }));
    await new AtlasControlClient(agent).setConfig({ enabled: false });
    expect(JSON.parse(String(call(fetch).init.body))).toEqual({ enabled: false });
  });

  it("setConfig reports a failed service restart as a failure with its reason", async () => {
    const { agent } = stubAgent(() =>
      json(502, {
        status: "error",
        enabled: true,
        persisted: true,
        restart: { status: "error", message: "Restart timed out for the capture service" },
      }),
    );
    expect(await new AtlasControlClient(agent).setConfig({ enabled: true })).toEqual({
      ok: false,
      message: "Restart timed out for the capture service",
    });
  });

  it("setConfig treats a 2xx reply without status ok as a failure", async () => {
    const { agent } = stubAgent(() => json(200, { enabled: true }));
    expect(await new AtlasControlClient(agent).setConfig({ enabled: true })).toEqual({
      ok: false,
      message: "HTTP 200",
    });
  });

  it("setConfig's deadline covers the agent's service restart", async () => {
    // The agent restarts the capture service before it answers a config write;
    // a read-sized deadline would abort a write that then landed.
    const timeout = vi.spyOn(AbortSignal, "timeout");
    const { agent } = stubAgent(() =>
      json(200, { status: "ok", enabled: true, restart: { status: "ok" } }),
    );
    await new AtlasControlClient(agent).setConfig({ enabled: true });
    expect(timeout).toHaveBeenCalledTimes(1);
    expect(timeout.mock.calls[0]?.[0]).toBeGreaterThanOrEqual(40_000);
  });

  it("captureStart POSTs atlas/capture/start and returns the coerced status", async () => {
    const { agent, fetch } = stubAgent(() => json(200, CAPTURE_WIRE));
    const r = await new AtlasControlClient(agent).captureStart();
    expect(r.ok && r.status.sessionId).toBe("s9");
    const { path, init } = call(fetch);
    expect(path).toBe("atlas/capture/start");
    expect(init.method).toBe("POST");
  });

  it("routes each lifecycle action to its own path", async () => {
    const { agent, fetch } = stubAgent(() => json(200, CAPTURE_WIRE));
    const c = new AtlasControlClient(agent);
    await c.captureStop();
    await c.capturePause();
    await c.captureResume();
    expect(fetch.mock.calls.map((args) => args[0])).toEqual([
      "atlas/capture/stop",
      "atlas/capture/pause",
      "atlas/capture/resume",
    ]);
  });

  it("capture action reports serviceDown on a 503", async () => {
    const { agent } = stubAgent(() => json(503, { error: "service_unavailable" }));
    expect(await new AtlasControlClient(agent).captureStop()).toEqual({
      ok: false,
      serviceDown: true,
      message: "service_unavailable",
    });
  });

  it("capture action reports a non-503 failure without serviceDown", async () => {
    const { agent } = stubAgent(() => json(500, { message: "boom" }));
    expect(await new AtlasControlClient(agent).capturePause()).toEqual({
      ok: false,
      serviceDown: false,
      message: "boom",
    });
  });

  it("capture action separates a transport failure from a refused one", async () => {
    const { agent } = stubAgent(() => Promise.reject(new Error("unreachable")));
    expect(await new AtlasControlClient(agent).captureStart()).toEqual({
      ok: false,
      serviceDown: false,
      message: "transport_error",
    });
  });

  it("capture action refuses a 2xx body that is not a status object", async () => {
    const { agent } = stubAgent(() => json(200, [1, 2]));
    expect(await new AtlasControlClient(agent).captureResume()).toEqual({
      ok: false,
      serviceDown: false,
      message: "bad_response",
    });
  });
});
