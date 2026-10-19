/**
 * The capture device client: each action hits its `atlas/` route with the
 * snake_case body the device expects, a refusal carries the device's error
 * code, and reads degrade to null.
 */

import { describe, expect, it } from "vitest";

import { CaptureClient } from "../../../src/lib/agent/capture-client";
import { callAt, json, stubAgent } from "../../helpers/agent";

describe("CaptureClient", () => {
  it("starts with operator metadata in snake_case and returns the session id", async () => {
    const { agent, fetch } = stubAgent(() => json(200, { ok: true, session_id: "visor-1-x" }));
    const res = await new CaptureClient(agent).start({
      site: "Warehouse 3",
      floor: 1,
      room: "Dock",
      operatorAnchor: { latDeg: 12.97, lonDeg: 77.59, hEllipsoidM: 920, headingDeg: 90 },
    });
    expect(res).toEqual({ ok: true, value: { sessionId: "visor-1-x" } });
    const { path, init } = callAt(fetch);
    expect(path).toBe("atlas/capture/start");
    expect(JSON.parse(String(init.body))).toEqual({
      site: "Warehouse 3",
      floor: 1,
      room: "Dock",
      operator_anchor: { lat_deg: 12.97, lon_deg: 77.59, h_ellipsoid_m: 920, heading_deg: 90 },
    });
  });

  it("carries the device's error code on a refusal", async () => {
    const full = stubAgent(() => json(409, { error: "E_STORAGE_FULL" }));
    expect(await new CaptureClient(full.agent).start()).toEqual({
      ok: false,
      code: "E_STORAGE_FULL",
      message: "E_STORAGE_FULL",
    });
    const down = stubAgent(() => json(503, {}));
    expect(await new CaptureClient(down.agent).stop()).toMatchObject({ ok: false, code: "service_unavailable" });
    const gone = stubAgent(() => Promise.reject(new Error("net")));
    expect(await new CaptureClient(gone.agent).stop()).toMatchObject({ ok: false, code: "transport_error" });
  });

  it("marks a segment with room and floor", async () => {
    const { agent, fetch } = stubAgent(() => json(200, { ok: true, segment: 3 }));
    expect(await new CaptureClient(agent).mark("Kitchen", 0)).toEqual({ ok: true, value: { segment: 3 } });
    expect(callAt(fetch).path).toBe("atlas/capture/mark");
    expect(JSON.parse(String(callAt(fetch).init.body))).toEqual({ room: "Kitchen", floor: 0 });
  });

  it("puts the exposure lock and reads the applied exposure back", async () => {
    const { agent, fetch } = stubAgent(() =>
      json(200, { locked: true, exposure_us: 4000, gain: 2, mode: "walking", applied: true }),
    );
    const res = await new CaptureClient(agent).setExposure({ locked: true, mode: "walking" });
    expect(res).toEqual({
      ok: true,
      value: { locked: true, exposureUs: 4000, gain: 2, mode: "walking", applied: true, reason: null },
    });
    expect(callAt(fetch).init.method).toBe("PUT");
  });

  it("deletes a session only with force when it is not uploaded", async () => {
    const { agent, fetch } = stubAgent(() => json(200, {}));
    await new CaptureClient(agent).deleteSession("a b", true);
    expect(callAt(fetch).path).toBe("atlas/sessions/a%20b?force=true");
    expect(callAt(fetch).init.method).toBe("DELETE");
  });

  it("pushes a rig document to the device", async () => {
    const { agent, fetch } = stubAgent(() => json(200, { ok: true, rig_id: "r1", calib_version: "v2" }));
    const rig = { contract: "ados.capture/1" as const, rig_id: "r1", calib_version: "v2", sensors: [] };
    expect(await new CaptureClient(agent).putRig(rig)).toEqual({ ok: true, value: { rigId: "r1", calibVersion: "v2" } });
    expect(callAt(fetch).path).toBe("atlas/rig");
    expect(JSON.parse(String(callAt(fetch).init.body))).toEqual(rig);
  });

  it("reads null state and sessions when the device does not answer", async () => {
    const { agent } = stubAgent(() => json(404, {}));
    const c = new CaptureClient(agent);
    expect(await c.getState()).toBeNull();
    expect(await c.listSessions()).toBeNull();
    expect(await c.getCalib()).toBeNull();
  });

  it("negotiates a WHEP preview through the device and tears it down", async () => {
    const { agent, fetch } = stubAgent((path) =>
      path.startsWith("atlas/capture/whep")
        ? Promise.resolve(new Response("v=0 answer", { status: 201, headers: { Location: "whep/s1?camera=cam_fl" } }))
        : Promise.resolve(new Response(null, { status: 200 })),
    );
    const c = new CaptureClient(agent);
    expect(await c.whepOffer("cam_fl", "v=0 offer")).toEqual({ answer: "v=0 answer", location: "whep/s1?camera=cam_fl" });
    expect(callAt(fetch).path).toBe("atlas/capture/whep?camera=cam_fl");
    expect(callAt(fetch).init.body).toBe("v=0 offer");
    await c.whepClose("whep/s1?camera=cam_fl");
    expect(callAt(fetch, 1).path).toBe("atlas/capture/whep/s1?camera=cam_fl");
    expect(callAt(fetch, 1).init.method).toBe("DELETE");
  });
});
